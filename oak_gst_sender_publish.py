#!/usr/bin/env python3
"""
OAK Camera → GStreamer RTP/UDP sender for ROCK 5B.

Captures RGB frames from OAK (DepthAI v3 API), pushes into GStreamer
via appsrc, encodes with Rockchip MPP H.264 encoder, sends over UDP/RTP.

Dependencies:
    pip install depthai>=3.0
    GStreamer 1.x with rockchipmpp plugin (mpph264enc)

Usage:
    python3 oak_gst_sender.py --host 192.168.1.100 --port 5600
"""

import argparse
import signal
import time

import depthai as dai
import gi
import rclpy
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp, GLib

# ─── Configuration ───────────────────────────────────────────────────────────

DEFAULT_HOST = "192.168.1.100"
DEFAULT_PORT = 5600
CAM_WIDTH = 1920
CAM_HEIGHT = 1080
CAM_FPS = 60
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 400
DEPTH_FPS = 30

DEFAULT_IMAGE_TOPIC = "/oak/rgb/image_raw"
DEFAULT_DEPTH_TOPIC = "/oak/stereo/depth"
RGB_FRAME_ID = "oak_rgb_camera_optical_frame"
DEPTH_FRAME_ID = "oak_depth_camera_optical_frame"


def build_gst_pipeline(host: str, port: int, width: int, height: int, fps: int) -> str:
    """
    GStreamer pipeline string.
    appsrc receives NV12 frames from DepthAI.
    mpph264enc — аппаратный H.264 энкодер Rockchip MPP.
    """
    return (
        f"appsrc name=source is-live=true block=true do-timestamp=true "
        f"  format=time "
        f'  caps=video/x-raw,format=NV12,width={width},height={height},framerate={fps}/1 '
        f"! mpph264enc "
        f"    bps={8_000_000} "
        f"    bps-max={10_000_000} "
        f"    rc-mode=cbr "
        f"    gop={fps} "
        f"    header-mode=1 "
        f"    profile=high "
        f"! h264parse config-interval=1 "
        f"! rtph264pay mtu=1400 pt=96 "
        f"! udpsink host={host} port={port} sync=false async=false"
    )


def build_depthai_pipeline(width: int, height: int, fps: int):
    """
    DepthAI v3 API:
    - Camera().build() вместо setBoardSocket/setSize
    - requestOutput() вместо прямого доступа к .isp/.video
    - createOutputQueue() вместо XLinkOut
    """
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.Camera).build(
        boardSocket=dai.CameraBoardSocket.CAM_A,
        sensorFps=float(fps),
    )

    # requestOutput возвращает выход нужного размера и формата.
    # NV12 — нативный формат ISP, идеален для аппаратного энкодера.
    video_out = cam.requestOutput(
        size=(width, height),
        type=dai.ImgFrame.Type.NV12,
    )
    image_out = cam.requestOutput(
        size=(width, height),
        type=dai.ImgFrame.Type.BGR888i,
        fps=float(fps),
    )

    left = pipeline.create(dai.node.Camera).build(
        boardSocket=dai.CameraBoardSocket.CAM_B,
        sensorFps=float(DEPTH_FPS),
    )
    right = pipeline.create(dai.node.Camera).build(
        boardSocket=dai.CameraBoardSocket.CAM_C,
        sensorFps=float(DEPTH_FPS),
    )

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

    left.requestOutput(
        size=(DEPTH_WIDTH, DEPTH_HEIGHT),
        type=dai.ImgFrame.Type.GRAY8,
        fps=float(DEPTH_FPS),
    ).link(stereo.left)
    right.requestOutput(
        size=(DEPTH_WIDTH, DEPTH_HEIGHT),
        type=dai.ImgFrame.Type.GRAY8,
        fps=float(DEPTH_FPS),
    ).link(stereo.right)

    video_queue = video_out.createOutputQueue(maxSize=2, blocking=False)
    image_queue = image_out.createOutputQueue(maxSize=2, blocking=False)
    depth_queue = stereo.depth.createOutputQueue(maxSize=2, blocking=False)

    return pipeline, video_queue, image_queue, depth_queue


def make_sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=2,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
    )


def get_frame_size(frame_msg, default_width: int, default_height: int) -> tuple[int, int]:
    width = default_width
    height = default_height
    if hasattr(frame_msg, "getWidth"):
        width = int(frame_msg.getWidth())
    if hasattr(frame_msg, "getHeight"):
        height = int(frame_msg.getHeight())
    return width, height


def build_image_msg(frame_msg, stamp, frame_id: str) -> Image:
    width, height = get_frame_size(frame_msg, CAM_WIDTH, CAM_HEIGHT)
    data = bytes(frame_msg.getData())

    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = width * 3
    msg.data = data
    return msg


def build_depth_msg(frame_msg, stamp, frame_id: str) -> Image:
    width, height = get_frame_size(frame_msg, DEPTH_WIDTH, DEPTH_HEIGHT)
    data = bytes(frame_msg.getData())

    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.encoding = "16UC1"
    msg.is_bigendian = 0
    msg.step = width * 2
    msg.data = data
    return msg


class OakGstSender:
    """Captures from OAK (v3 API), pushes frames into GStreamer appsrc."""

    def __init__(self, host: str, port: int, image_topic: str, depth_topic: str):
        self.host = host
        self.port = port
        self.image_topic = image_topic
        self.depth_topic = depth_topic
        self.running = False
        self._gst_pipeline = None
        self._appsrc = None
        self._dai_pipeline = None
        self._video_queue = None
        self._image_queue = None
        self._depth_queue = None
        self._ros_node = None
        self._image_pub = None
        self._depth_pub = None
        self._frame_count = 0
        self._image_count = 0
        self._depth_count = 0
        self._last_stats_frame_count = 0
        self._start_time = 0.0

    def start(self):
        Gst.init(None)
        if not rclpy.ok():
            rclpy.init()

        self._ros_node = rclpy.create_node("oak_gst_sender_publish")
        qos = make_sensor_qos()
        self._image_pub = self._ros_node.create_publisher(
            Image, self.image_topic, qos
        )
        self._depth_pub = self._ros_node.create_publisher(
            Image, self.depth_topic, qos
        )
        print(f"[ROS2] Publishing RGB image: {self.image_topic} (bgr8)")
        print(f"[ROS2] Publishing depth: {self.depth_topic} (16UC1, mm)")

        # ── GStreamer pipeline ──
        pipeline_str = build_gst_pipeline(
            self.host, self.port, CAM_WIDTH, CAM_HEIGHT, CAM_FPS
        )
        print(f"[GStreamer] Pipeline:\n  {pipeline_str}")

        self._gst_pipeline = Gst.parse_launch(pipeline_str)
        self._appsrc = self._gst_pipeline.get_by_name("source")

        bus = self._gst_pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_gst_error)
        bus.connect("message::eos", self._on_gst_eos)

        self._gst_pipeline.set_state(Gst.State.PLAYING)
        print("[GStreamer] Pipeline PLAYING")

        # ── DepthAI v3 ──
        (
            self._dai_pipeline,
            self._video_queue,
            self._image_queue,
            self._depth_queue,
        ) = build_depthai_pipeline(CAM_WIDTH, CAM_HEIGHT, CAM_FPS)
        self._dai_pipeline.start()
        print("[DepthAI] Pipeline started, capturing...")

        self.running = True
        self._start_time = time.monotonic()
        self._frame_count = 0
        self._image_count = 0
        self._depth_count = 0
        self._last_stats_frame_count = 0

        # ── Capture loop ──
        try:
            while self.running and self._dai_pipeline.isRunning():
                rclpy.spin_once(self._ros_node, timeout_sec=0.0)

                frame_msg = self._video_queue.tryGet()
                image_msg = self._image_queue.tryGet()
                depth_msg = self._depth_queue.tryGet()

                if frame_msg is not None:
                    # getData() returns raw bytes (NV12 from isp output)
                    frame_data = frame_msg.getData()

                    # Wrap in GStreamer buffer
                    buf = Gst.Buffer.new_allocate(None, len(frame_data), None)
                    buf.fill(0, bytes(frame_data))

                    # Timestamp (PTS)
                    elapsed_ns = int(
                        (time.monotonic() - self._start_time) * Gst.SECOND
                    )
                    buf.pts = elapsed_ns
                    buf.duration = Gst.SECOND // CAM_FPS

                    # Push to GStreamer
                    ret = self._appsrc.emit("push-buffer", buf)
                    if ret != Gst.FlowReturn.OK:
                        print(f"[GStreamer] appsrc push-buffer returned {ret}")
                        break

                    self._frame_count += 1

                now = self._ros_node.get_clock().now().to_msg()
                if image_msg is not None:
                    self._image_pub.publish(
                        build_image_msg(image_msg, now, RGB_FRAME_ID)
                    )
                    self._image_count += 1

                if depth_msg is not None:
                    self._depth_pub.publish(
                        build_depth_msg(depth_msg, now, DEPTH_FRAME_ID)
                    )
                    self._depth_count += 1

                if frame_msg is None and image_msg is None and depth_msg is None:
                    time.sleep(0.001)

                if (
                    self._frame_count
                    and self._frame_count % (CAM_FPS * 5) == 0
                    and self._frame_count != self._last_stats_frame_count
                ):
                    self._last_stats_frame_count = self._frame_count
                    elapsed = time.monotonic() - self._start_time
                    print(
                        f"[Stats] Sent {self._frame_count} frames, "
                        f"published {self._image_count} RGB / "
                        f"{self._depth_count} depth, "
                        f"avg gst {self._frame_count / elapsed:.1f} fps"
                    )

        except KeyboardInterrupt:
            print("\n[Ctrl+C] Stopping...")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        if self._appsrc:
            self._appsrc.emit("end-of-stream")
        if self._gst_pipeline:
            self._gst_pipeline.set_state(Gst.State.NULL)
            print("[GStreamer] Pipeline stopped")
        if self._dai_pipeline:
            self._dai_pipeline.stop()
            print("[DepthAI] Pipeline stopped")
        if self._ros_node:
            self._ros_node.destroy_node()
            self._ros_node = None
        if rclpy.ok():
            rclpy.shutdown()

    def _on_gst_error(self, bus, msg):
        err, dbg = msg.parse_error()
        print(f"[GStreamer ERROR] {err.message}")
        if dbg:
            print(f"  Debug: {dbg}")
        self.running = False

    def _on_gst_eos(self, bus, msg):
        print("[GStreamer] End of stream")
        self.running = False


def main():
    parser = argparse.ArgumentParser(
        description="OAK Camera → GStreamer RTP/UDP sender (ROCK 5B, DepthAI v3)"
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help=f"Destination IP (default: {DEFAULT_HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"Destination UDP port (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--image-topic",
        default=DEFAULT_IMAGE_TOPIC,
        help=f"ROS2 RGB image topic (default: {DEFAULT_IMAGE_TOPIC})",
    )
    parser.add_argument(
        "--depth-topic",
        default=DEFAULT_DEPTH_TOPIC,
        help=f"ROS2 depth topic (default: {DEFAULT_DEPTH_TOPIC})",
    )
    args = parser.parse_args()

    sender = OakGstSender(args.host, args.port, args.image_topic, args.depth_topic)

    def sig_handler(signum, frame):
        sender.running = False

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    sender.start()


if __name__ == "__main__":
    main()
