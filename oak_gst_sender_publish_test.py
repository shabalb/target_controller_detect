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

import cv2
import depthai as dai
import gi
import numpy as np
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
ROS_IMAGE_WIDTH = 640
ROS_IMAGE_HEIGHT = 480
ROS_IMAGE_FPS = 3
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 400
DEPTH_FPS = 30

DEFAULT_IMAGE_TOPIC = "/oak/rgb/image_raw"
DEFAULT_DEPTH_TOPIC = "/oak/stereo/depth"
RGB_FRAME_ID = "oak_rgb_camera_optical_frame"
DEPTH_FRAME_ID = "oak_depth_camera_optical_frame"


def ensure_depth_output_size(width: int, height: int) -> None:
    if width % 16 != 0:
        raise ValueError(f"DEPTH_WIDTH must be multiple of 16, got {width}")
    if height <= 0:
        raise ValueError(f"DEPTH_HEIGHT must be positive, got {height}")


def build_gst_pipeline(host: str, port: int, width: int, height: int, fps: int) -> str:
    """
    GStreamer pipeline string.
    appsrc receives NV12 frames from DepthAI.
    mpph264enc — аппаратный H.264 энкодер Rockchip MPP.
    """
    return (
        f"appsrc name=source is-live=true block=false do-timestamp=true "
        f"  max-buffers=2 leaky-type=downstream "
        f"  format=time "
        f'  caps=video/x-raw,format=NV12,width={width},height={height},framerate={fps}/1 '
        f"! mpph264enc "
        f"    bps={4_000_000} "
        f"    bps-max={6_000_000} "
        f"    rc-mode=cbr "
        f"    gop={fps} "
        f"    header-mode=1 "
        f"    profile=high "
        f"! h264parse config-interval=1 "
        f"! rtph264pay mtu=1400 pt=96 "
        f"! udpsink host={host} port={port} sync=false async=false"
    )


def build_depthai_pipeline(
    video_width: int,
    video_height: int,
    video_fps: int,
):
    """
    DepthAI v3 API:
    - Camera().build() вместо setBoardSocket/setSize
    - requestOutput() вместо прямого доступа к .isp/.video
    - createOutputQueue() вместо XLinkOut
    """
    ensure_depth_output_size(DEPTH_WIDTH, DEPTH_HEIGHT)

    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.Camera).build(
        boardSocket=dai.CameraBoardSocket.CAM_A,
        sensorFps=float(video_fps),
    )

    # requestOutput возвращает выход нужного размера и формата.
    # NV12 — нативный формат ISP, идеален для аппаратного энкодера.
    video_out = cam.requestOutput(
        size=(video_width, video_height),
        type=dai.ImgFrame.Type.NV12,
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
    preset_mode = dai.node.StereoDepth.PresetMode
    if hasattr(preset_mode, "HIGH_DENSITY"):
        stereo_preset = preset_mode.HIGH_DENSITY
    else:
        stereo_preset = preset_mode.FAST_DENSITY
    stereo.setDefaultProfilePreset(stereo_preset)
    stereo.setLeftRightCheck(True)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(DEPTH_WIDTH, DEPTH_HEIGHT)

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
    depth_queue = stereo.depth.createOutputQueue(maxSize=2, blocking=False)

    return pipeline, video_queue, depth_queue


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


def build_image_msg(
    frame_msg,
    stamp,
    frame_id: str,
    image_width: int,
    image_height: int,
) -> Image:
    width, height = get_frame_size(frame_msg, CAM_WIDTH, CAM_HEIGHT)
    nv12 = np.frombuffer(bytes(frame_msg.getData()), dtype=np.uint8)
    expected_size = width * height * 3 // 2
    if nv12.size < expected_size:
        raise ValueError(
            f"NV12 frame is too small: {nv12.size} bytes, expected {expected_size}"
        )

    nv12 = nv12[:expected_size].reshape((height * 3 // 2, width))
    bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
    if bgr.shape[1] != image_width or bgr.shape[0] != image_height:
        bgr = cv2.resize(
            bgr, (image_width, image_height), interpolation=cv2.INTER_AREA
        )

    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(bgr.shape[0])
    msg.width = int(bgr.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = bgr.tobytes()
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

    def __init__(
        self,
        host: str,
        port: int,
        image_topic: str,
        depth_topic: str,
        image_width: int,
        image_height: int,
        image_fps: int,
    ):
        self.host = host
        self.port = port
        self.image_topic = image_topic
        self.depth_topic = depth_topic
        self.image_width = image_width
        self.image_height = image_height
        self.image_fps = image_fps
        self.running = False
        self._gst_pipeline = None
        self._gst_bus = None
        self._appsrc = None
        self._dai_pipeline = None
        self._video_queue = None
        self._depth_queue = None
        self._ros_node = None
        self._image_pub = None
        self._depth_pub = None
        self._frame_count = 0
        self._image_count = 0
        self._depth_count = 0
        self._last_stats_frame_count = 0
        self._last_status_time = 0.0
        self._last_image_publish_time = 0.0
        self._reported_video_format = False
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
        print(
            f"[ROS2] Publishing RGB image: {self.image_topic} "
            f"(bgr8, {self.image_width}x{self.image_height}@{self.image_fps})"
        )
        print(f"[ROS2] Publishing depth: {self.depth_topic} (16UC1, mm)")

        # ── GStreamer pipeline ──
        pipeline_str = build_gst_pipeline(
            self.host, self.port, CAM_WIDTH, CAM_HEIGHT, CAM_FPS
        )
        print(f"[GStreamer] Pipeline:\n  {pipeline_str}")

        self._gst_pipeline = Gst.parse_launch(pipeline_str)
        self._appsrc = self._gst_pipeline.get_by_name("source")

        self._gst_bus = self._gst_pipeline.get_bus()
        self._gst_bus.add_signal_watch()
        self._gst_bus.connect("message::error", self._on_gst_error)
        self._gst_bus.connect("message::eos", self._on_gst_eos)

        self._gst_pipeline.set_state(Gst.State.PLAYING)
        print("[GStreamer] Pipeline PLAYING")

        # ── DepthAI v3 ──
        (
            self._dai_pipeline,
            self._video_queue,
            self._depth_queue,
        ) = build_depthai_pipeline(
            CAM_WIDTH,
            CAM_HEIGHT,
            CAM_FPS,
        )
        self._dai_pipeline.start()
        print("[DepthAI] Pipeline started, capturing...")

        self.running = True
        self._start_time = time.monotonic()
        self._frame_count = 0
        self._image_count = 0
        self._depth_count = 0
        self._last_stats_frame_count = 0
        self._last_status_time = self._start_time
        self._last_image_publish_time = 0.0
        self._reported_video_format = False

        # ── Capture loop ──
        try:
            while self.running and self._dai_pipeline.isRunning():
                rclpy.spin_once(self._ros_node, timeout_sec=0.0)
                self._poll_gst_bus()

                frame_msg = self._video_queue.tryGet()
                depth_msg = self._depth_queue.tryGet()

                if frame_msg is not None:
                    # getData() returns raw bytes (NV12 from isp output)
                    frame_data = frame_msg.getData()
                    if not self._reported_video_format:
                        width, height = get_frame_size(frame_msg, CAM_WIDTH, CAM_HEIGHT)
                        print(
                            f"[DepthAI] GST video frame: "
                            f"{width}x{height}, {len(frame_data)} bytes"
                        )
                        self._reported_video_format = True

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

                    monotonic_now = time.monotonic()
                    image_period = 1.0 / max(float(self.image_fps), 1.0)
                    if monotonic_now - self._last_image_publish_time >= image_period:
                        now = self._ros_node.get_clock().now().to_msg()
                        self._image_pub.publish(
                            build_image_msg(
                                frame_msg,
                                now,
                                RGB_FRAME_ID,
                                self.image_width,
                                self.image_height,
                            )
                        )
                        self._image_count += 1
                        self._last_image_publish_time = monotonic_now
                else:
                    now = self._ros_node.get_clock().now().to_msg()

                if depth_msg is not None:
                    self._depth_pub.publish(
                        build_depth_msg(depth_msg, now, DEPTH_FRAME_ID)
                    )
                    self._depth_count += 1

                if frame_msg is None and depth_msg is None:
                    time.sleep(0.001)

                self._print_periodic_status()

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
        if self._gst_bus:
            self._gst_bus.remove_signal_watch()
            self._gst_bus = None
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

    def _poll_gst_bus(self):
        if not self._gst_bus:
            return

        message_types = (
            Gst.MessageType.ERROR | Gst.MessageType.WARNING | Gst.MessageType.EOS
        )
        while True:
            msg = self._gst_bus.pop_filtered(message_types)
            if msg is None:
                break

            if msg.type == Gst.MessageType.ERROR:
                err, dbg = msg.parse_error()
                print(f"[GStreamer ERROR] {err.message}")
                if dbg:
                    print(f"  Debug: {dbg}")
                self.running = False
                break
            if msg.type == Gst.MessageType.WARNING:
                warn, dbg = msg.parse_warning()
                print(f"[GStreamer WARNING] {warn.message}")
                if dbg:
                    print(f"  Debug: {dbg}")
            elif msg.type == Gst.MessageType.EOS:
                print("[GStreamer] End of stream")
                self.running = False
                break

    def _print_periodic_status(self):
        now = time.monotonic()
        if now - self._last_status_time < 2.0:
            return

        elapsed = max(now - self._start_time, 1e-6)
        print(
            f"[Status] gst={self._frame_count} "
            f"({self._frame_count / elapsed:.1f} fps), "
            f"rgb={self._image_count}, depth={self._depth_count}, "
            f"udp={self.host}:{self.port}"
        )
        if self._frame_count == 0 and (self._image_count or self._depth_count):
            print("[Status] ROS frames are flowing, but GST video queue has no frames")
        self._last_status_time = now


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
    parser.add_argument(
        "--image-width",
        type=int,
        default=ROS_IMAGE_WIDTH,
        help=f"ROS2 RGB image width (default: {ROS_IMAGE_WIDTH})",
    )
    parser.add_argument(
        "--image-height",
        type=int,
        default=ROS_IMAGE_HEIGHT,
        help=f"ROS2 RGB image height (default: {ROS_IMAGE_HEIGHT})",
    )
    parser.add_argument(
        "--image-fps",
        type=int,
        default=ROS_IMAGE_FPS,
        help=f"ROS2 RGB image FPS (default: {ROS_IMAGE_FPS})",
    )
    args = parser.parse_args()

    sender = OakGstSender(
        args.host,
        args.port,
        args.image_topic,
        args.depth_topic,
        args.image_width,
        args.image_height,
        args.image_fps,
    )

    def sig_handler(signum, frame):
        sender.running = False

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    sender.start()


if __name__ == "__main__":
    main()
