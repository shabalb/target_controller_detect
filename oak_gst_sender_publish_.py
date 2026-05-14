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
CAM_FPS = 30
ROS_IMAGE_WIDTH = 640
ROS_IMAGE_HEIGHT = 480
ROS_IMAGE_FPS = 10
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
    image_width: int,
    image_height: int,
    image_fps: int,
    enable_gst: bool,
    enable_image: bool,
    enable_depth: bool,
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

    video_queue = None
    if enable_gst:
        # requestOutput возвращает выход нужного размера и формата.
        # NV12 — нативный формат ISP, идеален для аппаратного энкодера.
        video_out = cam.requestOutput(
            size=(video_width, video_height),
            type=dai.ImgFrame.Type.NV12,
        )
        video_queue = video_out.createOutputQueue(maxSize=2, blocking=False)

    image_queue = None
    image_encoding = "bgr8"
    image_source_format = "nv12"
    if enable_image:
        (
            image_type,
            image_encoding,
            image_source_format,
            image_type_name,
        ) = choose_ros_image_type()
        image_out = cam.requestOutput(
            size=(image_width, image_height),
            type=image_type,
            fps=float(image_fps),
        )
        image_queue = image_out.createOutputQueue(maxSize=2, blocking=False)
        print(
            f"[DepthAI] ROS image output: "
            f"{image_type_name} -> {image_encoding} ({image_source_format})"
        )

    depth_queue = None
    if enable_depth:
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
        depth_queue = stereo.depth.createOutputQueue(maxSize=2, blocking=False)

    return (
        pipeline,
        video_queue,
        image_queue,
        depth_queue,
        image_encoding,
        image_source_format,
    )


def choose_ros_image_type():
    if hasattr(dai.ImgFrame.Type, "NV12"):
        return dai.ImgFrame.Type.NV12, "bgr8", "nv12", "NV12"

    for type_name, encoding in (("BGR888i", "bgr8"), ("RGB888i", "rgb8")):
        if hasattr(dai.ImgFrame.Type, type_name):
            return (
                getattr(dai.ImgFrame.Type, type_name),
                encoding,
                "interleaved",
                type_name,
            )

    raise RuntimeError(
        "DepthAI build does not expose BGR888i/RGB888i or NV12 output types. "
        "Cannot publish color ROS images without OpenCV/RGA conversion."
    )


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
    encoding: str,
    source_format: str,
) -> Image:
    width, height = get_frame_size(frame_msg, ROS_IMAGE_WIDTH, ROS_IMAGE_HEIGHT)
    data = bytes(frame_msg.getData())
    if source_format == "interleaved":
        expected_size = width * height * 3
        if len(data) < expected_size:
            raise ValueError(
                f"Color frame is too small: {len(data)} bytes, expected {expected_size}"
            )
        image_data = data[:expected_size]
    elif source_format == "nv12":
        image_data = nv12_to_bgr_bytes(data, width, height)
        encoding = "bgr8"
    else:
        raise ValueError(f"Unsupported ROS image source format: {source_format}")

    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = width * 3
    msg.data = image_data
    return msg


def nv12_to_bgr_bytes(data: bytes, width: int, height: int) -> bytes:
    expected_size = width * height * 3 // 2
    if len(data) < expected_size:
        raise ValueError(
            f"NV12 frame is too small: {len(data)} bytes, expected {expected_size}"
        )

    nv12 = np.frombuffer(data[:expected_size], dtype=np.uint8)
    y = nv12[: width * height].reshape((height, width)).astype(np.int16)
    uv = nv12[width * height :].reshape((height // 2, width // 2, 2)).astype(np.int16)
    u = uv[:, :, 0].repeat(2, axis=0).repeat(2, axis=1)[:height, :width] - 128
    v = uv[:, :, 1].repeat(2, axis=0).repeat(2, axis=1)[:height, :width] - 128
    c = y - 16

    r = (298 * c + 409 * v + 128) >> 8
    g = (298 * c - 100 * u - 208 * v + 128) >> 8
    b = (298 * c + 516 * u + 128) >> 8
    bgr = np.dstack(
        (
            np.clip(b, 0, 255),
            np.clip(g, 0, 255),
            np.clip(r, 0, 255),
        )
    ).astype(np.uint8)
    return bgr.tobytes()


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
        enable_gst: bool,
        enable_image: bool,
        enable_depth: bool,
    ):
        self.host = host
        self.port = port
        self.image_topic = image_topic
        self.depth_topic = depth_topic
        self.image_width = image_width
        self.image_height = image_height
        self.image_fps = image_fps
        self.enable_gst = enable_gst
        self.enable_image = enable_image
        self.enable_depth = enable_depth
        self.running = False
        self._gst_pipeline = None
        self._gst_bus = None
        self._appsrc = None
        self._dai_pipeline = None
        self._video_queue = None
        self._image_queue = None
        self._depth_queue = None
        self._image_encoding = "bgr8"
        self._image_source_format = "interleaved"
        self._ros_node = None
        self._image_pub = None
        self._depth_pub = None
        self._frame_count = 0
        self._image_count = 0
        self._depth_count = 0
        self._last_stats_frame_count = 0
        self._last_status_time = 0.0
        self._last_status_frame_count = 0
        self._last_status_image_count = 0
        self._last_status_depth_count = 0
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
        if self.enable_image:
            print(
                f"[ROS2] Publishing RGB image: {self.image_topic} "
                f"(color, {self.image_width}x{self.image_height}@{self.image_fps})"
            )
        else:
            print("[ROS2] RGB image publishing disabled")
        if self.enable_depth:
            print(f"[ROS2] Publishing depth: {self.depth_topic} (16UC1, mm)")
        else:
            print("[ROS2] Depth publishing disabled")

        # ── GStreamer pipeline ──
        if self.enable_gst:
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
        else:
            print("[GStreamer] Disabled")

        # ── DepthAI v3 ──
        (
            self._dai_pipeline,
            self._video_queue,
            self._image_queue,
            self._depth_queue,
            self._image_encoding,
            self._image_source_format,
        ) = build_depthai_pipeline(
            CAM_WIDTH,
            CAM_HEIGHT,
            CAM_FPS,
            self.image_width,
            self.image_height,
            self.image_fps,
            self.enable_gst,
            self.enable_image,
            self.enable_depth,
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
        self._last_status_frame_count = 0
        self._last_status_image_count = 0
        self._last_status_depth_count = 0
        self._last_image_publish_time = 0.0
        self._reported_video_format = False

        # ── Capture loop ──
        try:
            while self.running and self._dai_pipeline.isRunning():
                rclpy.spin_once(self._ros_node, timeout_sec=0.0)
                self._poll_gst_bus()

                frame_msg = self._video_queue.tryGet() if self._video_queue else None
                image_msg = self._image_queue.tryGet() if self._image_queue else None
                depth_msg = self._depth_queue.tryGet() if self._depth_queue else None

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

                if image_msg is not None:
                    now = self._ros_node.get_clock().now().to_msg()
                    self._image_pub.publish(
                        build_image_msg(
                            image_msg,
                            now,
                            RGB_FRAME_ID,
                            self._image_encoding,
                            self._image_source_format,
                        )
                    )
                    self._image_count += 1
                    self._last_image_publish_time = time.monotonic()

                if depth_msg is not None:
                    now = self._ros_node.get_clock().now().to_msg()
                    self._depth_pub.publish(
                        build_depth_msg(depth_msg, now, DEPTH_FRAME_ID)
                    )
                    self._depth_count += 1

                if frame_msg is None and image_msg is None and depth_msg is None:
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
        dt = now - self._last_status_time
        if dt < 2.0:
            return

        elapsed = max(now - self._start_time, 1e-6)
        gst_now = (self._frame_count - self._last_status_frame_count) / max(dt, 1e-6)
        rgb_now = (self._image_count - self._last_status_image_count) / max(dt, 1e-6)
        depth_now = (self._depth_count - self._last_status_depth_count) / max(dt, 1e-6)
        print(
            f"[Status] gst={self._frame_count} "
            f"({gst_now:.1f} now, {self._frame_count / elapsed:.1f} avg), "
            f"rgb={self._image_count} ({rgb_now:.1f} now), "
            f"depth={self._depth_count} ({depth_now:.1f} now), "
            f"udp={self.host}:{self.port}"
        )
        if self.enable_gst and gst_now == 0.0 and self._frame_count:
            print("[Status] GST frames stopped arriving from DepthAI")
        if self.enable_image and rgb_now == 0.0 and self._image_count:
            print("[Status] RGB frames stopped arriving from DepthAI")
        if self.enable_depth and depth_now == 0.0 and self._depth_count:
            print("[Status] Depth frames stopped arriving from DepthAI")
        self._last_status_frame_count = self._frame_count
        self._last_status_image_count = self._image_count
        self._last_status_depth_count = self._depth_count
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
    parser.add_argument(
        "--no-gst",
        action="store_true",
        help="Disable GStreamer/RTP output for diagnostics",
    )
    parser.add_argument(
        "--no-image",
        action="store_true",
        help="Disable ROS2 RGB image output for diagnostics",
    )
    parser.add_argument(
        "--no-depth",
        action="store_true",
        help="Disable ROS2 depth output for diagnostics",
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
        not args.no_gst,
        not args.no_image,
        not args.no_depth,
    )

    def sig_handler(signum, frame):
        sender.running = False

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    sender.start()


if __name__ == "__main__":
    main()
