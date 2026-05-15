#!/usr/bin/env python3
"""
OAK Camera -> GStreamer RTP/UDP sender + ROS2 Image/PointCloud2 publisher.

One DepthAI pipeline is used for three parallel outputs:
  * high-rate RGB NV12 frames to GStreamer/RTP/UDP;
  * lower-rate ROS2 BGR Image messages for detectors;
  * aligned stereo depth converted to organized ROS2 PointCloud2.
"""

import argparse
import math
import signal
import threading
import time

import cv2
import depthai as dai
import gi
import numpy as np
import rclpy
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField

gi.require_version("Gst", "1.0")
from gi.repository import Gst


DEFAULT_HOST = "192.168.1.100"
DEFAULT_PORT = 5600

CAM_WIDTH = 1920
CAM_HEIGHT = 1080
CAM_FPS = 30

ROS_IMAGE_WIDTH = 960
ROS_IMAGE_HEIGHT = 540
ROS_IMAGE_FPS = 10

DEPTH_WIDTH = 640
DEPTH_HEIGHT = 400
DEPTH_FPS = 30
DEFAULT_CAMERA_FOV = 1.255

DEFAULT_IMAGE_TOPIC = "/oak/rgb/image_raw"
DEFAULT_CLOUD_TOPIC = "/oak/stereo/points"
RGB_FRAME_ID = "oak_rgb_camera_optical_frame"
DEPTH_FRAME_ID = "oak_depth_camera_optical_frame"


def make_sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=2,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
    )


def choose_stereo_preset():
    preset_mode = dai.node.StereoDepth.PresetMode
    for name in ("HIGH_DENSITY", "FAST_DENSITY", "DEFAULT"):
        if hasattr(preset_mode, name):
            return getattr(preset_mode, name), name
    raise RuntimeError("DepthAI StereoDepth.PresetMode has no supported preset")


def choose_h264_encoder(preferred: str) -> tuple[str, str]:
    factories = {
        "mpp": (
            "mpph264enc",
            "mpph264enc bps=4000000 bps-max=6000000 rc-mode=cbr "
            "gop={fps} header-mode=1 profile=high",
        ),
        "v4l2": (
            "v4l2h264enc",
            "v4l2h264enc extra-controls=\"controls,video_bitrate=4000000\"",
        ),
        "x264": (
            "x264enc",
            "videoconvert ! video/x-raw,format=I420 "
            "! x264enc bitrate=4000 speed-preset=ultrafast tune=zerolatency "
            "key-int-max={fps}",
        ),
    }

    order = [preferred] if preferred != "auto" else ["mpp", "v4l2", "x264"]
    for name in order:
        if name not in factories:
            raise ValueError(f"Unknown encoder '{preferred}', expected auto/mpp/v4l2/x264")
        factory_name, pipeline_part = factories[name]
        if Gst.ElementFactory.find(factory_name):
            return name, pipeline_part

    tried = ", ".join(factories[name][0] for name in order)
    raise RuntimeError(f"No usable H.264 GStreamer encoder found. Tried: {tried}")


def build_gst_pipeline(
    host: str,
    port: int,
    width: int,
    height: int,
    fps: int,
    encoder: str,
) -> tuple[str, str]:
    encoder_name, encoder_part = choose_h264_encoder(encoder)
    return (
        "appsrc name=source is-live=true block=false do-timestamp=true "
        "max-buffers=2 leaky-type=downstream format=time "
        f"caps=video/x-raw,format=NV12,width={width},height={height},framerate={fps}/1 "
        "! queue max-size-buffers=2 leaky=downstream "
        f"! {encoder_part.format(fps=fps)} "
        "! h264parse config-interval=1 "
        "! rtph264pay mtu=1400 pt=96 "
        f"! udpsink host={host} port={port} sync=false async=false",
        encoder_name,
    )


def build_depthai_pipeline(video_width: int, video_height: int, video_fps: int):
    if DEPTH_WIDTH % 16 != 0:
        raise ValueError(f"DEPTH_WIDTH must be a multiple of 16, got {DEPTH_WIDTH}")

    pipeline = dai.Pipeline()

    rgb = pipeline.create(dai.node.Camera).build(
        boardSocket=dai.CameraBoardSocket.CAM_A,
        sensorFps=float(video_fps),
    )
    video_out = rgb.requestOutput(
        size=(video_width, video_height),
        type=dai.ImgFrame.Type.NV12,
        fps=float(video_fps),
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
    stereo_preset, preset_name = choose_stereo_preset()
    stereo.setDefaultProfilePreset(stereo_preset)
    stereo.setLeftRightCheck(True)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(DEPTH_WIDTH, DEPTH_HEIGHT)
    print(f"[DepthAI] Stereo preset: {preset_name}")

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


def get_frame_size(frame_msg, default_width: int, default_height: int) -> tuple[int, int]:
    width = int(frame_msg.getWidth()) if hasattr(frame_msg, "getWidth") else default_width
    height = int(frame_msg.getHeight()) if hasattr(frame_msg, "getHeight") else default_height
    return width, height


def build_bgr_image_msg(
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
        raise ValueError(f"NV12 frame has {nv12.size} bytes, expected {expected_size}")

    nv12 = nv12[:expected_size].reshape((height * 3 // 2, width))
    bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
    if bgr.shape[1] != image_width or bgr.shape[0] != image_height:
        bgr = cv2.resize(bgr, (image_width, image_height), interpolation=cv2.INTER_AREA)

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


def depth_to_meters(frame_msg) -> tuple[np.ndarray, int, int]:
    width, height = get_frame_size(frame_msg, DEPTH_WIDTH, DEPTH_HEIGHT)
    raw = np.frombuffer(bytes(frame_msg.getData()), dtype=np.uint16)
    expected = width * height
    if raw.size < expected:
        raise ValueError(f"Depth frame has {raw.size} samples, expected {expected}")
    depth_mm = raw[:expected].reshape((height, width))
    return depth_mm.astype(np.float32) * 0.001, width, height


def build_point_cloud_msg(
    frame_msg,
    stamp,
    frame_id: str,
    horizontal_fov: float,
) -> PointCloud2:
    depth_m, width, height = depth_to_meters(frame_msg)

    fx = width / (2.0 * math.tan(horizontal_fov / 2.0))
    vertical_fov = 2.0 * math.atan(math.tan(horizontal_fov / 2.0) * height / width)
    fy = height / (2.0 * math.tan(vertical_fov / 2.0))
    cx = (width - 1) * 0.5
    cy = (height - 1) * 0.5

    u = np.arange(width, dtype=np.float32)[None, :]
    v = np.arange(height, dtype=np.float32)[:, None]
    valid = np.isfinite(depth_m) & (depth_m > 0.05) & (depth_m < 20.0)

    z = np.where(valid, depth_m, np.nan).astype(np.float32)
    x = ((u - cx) * z / fx).astype(np.float32)
    y = ((v - cy) * z / fy).astype(np.float32)
    cloud = np.dstack((x, y, z)).astype(np.float32, copy=False)

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * width
    msg.is_dense = False
    msg.data = cloud.tobytes()
    return msg


class OakGstSender:
    def __init__(
        self,
        host: str,
        port: int,
        image_topic: str,
        cloud_topic: str,
        image_width: int,
        image_height: int,
        image_fps: int,
        camera_fov: float,
        encoder: str,
    ):
        self.host = host
        self.port = port
        self.image_topic = image_topic
        self.cloud_topic = cloud_topic
        self.image_width = image_width
        self.image_height = image_height
        self.image_fps = image_fps
        self.camera_fov = camera_fov
        self.encoder = encoder

        self.running = False
        self._gst_pipeline = None
        self._gst_bus = None
        self._appsrc = None
        self._dai_pipeline = None
        self._video_queue = None
        self._depth_queue = None
        self._ros_node = None
        self._image_pub = None
        self._cloud_pub = None
        self._video_thread = None
        self._cloud_thread = None
        self._frame_count = 0
        self._image_count = 0
        self._cloud_count = 0
        self._last_status_time = 0.0
        self._last_image_publish_time = 0.0
        self._start_time = 0.0
        self._reported_video_format = False
        self._reported_depth_format = False

    def start(self):
        Gst.init(None)
        if not rclpy.ok():
            rclpy.init()

        self._ros_node = rclpy.create_node("oak_gst_sender_pub_1")
        qos = make_sensor_qos()
        self._image_pub = self._ros_node.create_publisher(Image, self.image_topic, qos)
        self._cloud_pub = self._ros_node.create_publisher(PointCloud2, self.cloud_topic, qos)
        print(
            f"[ROS2] Publishing image: {self.image_topic} "
            f"(bgr8, {self.image_width}x{self.image_height}@{self.image_fps})"
        )
        print(
            f"[ROS2] Publishing cloud: {self.cloud_topic} "
            f"({DEPTH_WIDTH}x{DEPTH_HEIGHT}, xyz float32, frame={DEPTH_FRAME_ID})"
        )

        pipeline_str, encoder_name = build_gst_pipeline(
            self.host,
            self.port,
            CAM_WIDTH,
            CAM_HEIGHT,
            CAM_FPS,
            self.encoder,
        )
        print(f"[GStreamer] Encoder: {encoder_name}")
        print(f"[GStreamer] Pipeline:\n  {pipeline_str}")
        self._gst_pipeline = Gst.parse_launch(pipeline_str)
        self._appsrc = self._gst_pipeline.get_by_name("source")
        self._gst_bus = self._gst_pipeline.get_bus()
        self._gst_pipeline.set_state(Gst.State.PLAYING)
        print("[GStreamer] Pipeline PLAYING")

        self._dai_pipeline, self._video_queue, self._depth_queue = build_depthai_pipeline(
            CAM_WIDTH,
            CAM_HEIGHT,
            CAM_FPS,
        )
        self._dai_pipeline.start()
        print("[DepthAI] Pipeline started, capturing...")

        self.running = True
        self._start_time = time.monotonic()
        self._last_status_time = self._start_time

        self._video_thread = threading.Thread(target=self._video_loop, name="oak-video", daemon=True)
        self._cloud_thread = threading.Thread(target=self._cloud_loop, name="oak-cloud", daemon=True)
        self._video_thread.start()
        self._cloud_thread.start()

        try:
            while self.running and self._dai_pipeline.isRunning():
                rclpy.spin_once(self._ros_node, timeout_sec=0.0)
                self._poll_gst_bus()
                self._print_periodic_status()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\n[Ctrl+C] Stopping...")
        finally:
            self.stop()

    def stop(self):
        self.running = False

        for worker in (self._video_thread, self._cloud_thread):
            if worker and worker.is_alive() and worker is not threading.current_thread():
                worker.join(timeout=1.0)

        if self._appsrc:
            self._appsrc.emit("end-of-stream")
            self._appsrc = None
        if self._gst_pipeline:
            self._gst_pipeline.set_state(Gst.State.NULL)
            self._gst_pipeline = None
            print("[GStreamer] Pipeline stopped")
        self._gst_bus = None

        if self._dai_pipeline:
            self._dai_pipeline.stop()
            self._dai_pipeline = None
            print("[DepthAI] Pipeline stopped")

        if self._ros_node:
            self._ros_node.destroy_node()
            self._ros_node = None
        if rclpy.ok():
            rclpy.shutdown()

    def _video_loop(self):
        while self.running and self._dai_pipeline and self._dai_pipeline.isRunning():
            frame_msg = self._video_queue.tryGet()
            if frame_msg is None:
                time.sleep(0.001)
                continue

            frame_data = bytes(frame_msg.getData())
            if not self._reported_video_format:
                width, height = get_frame_size(frame_msg, CAM_WIDTH, CAM_HEIGHT)
                print(f"[DepthAI] Video frame: {width}x{height}, {len(frame_data)} bytes")
                self._reported_video_format = True

            buf = Gst.Buffer.new_allocate(None, len(frame_data), None)
            buf.fill(0, frame_data)
            elapsed_ns = int((time.monotonic() - self._start_time) * Gst.SECOND)
            buf.pts = elapsed_ns
            buf.dts = elapsed_ns
            buf.duration = Gst.SECOND // CAM_FPS

            ret = self._appsrc.emit("push-buffer", buf)
            if ret != Gst.FlowReturn.OK:
                print(f"[GStreamer] appsrc push-buffer returned {ret}")
                self.running = False
                break
            self._frame_count += 1

            now_monotonic = time.monotonic()
            image_period = 1.0 / max(float(self.image_fps), 1.0)
            if now_monotonic - self._last_image_publish_time >= image_period:
                stamp = self._ros_node.get_clock().now().to_msg()
                try:
                    self._image_pub.publish(
                        build_bgr_image_msg(
                            frame_msg,
                            stamp,
                            RGB_FRAME_ID,
                            self.image_width,
                            self.image_height,
                        )
                    )
                    self._image_count += 1
                    self._last_image_publish_time = now_monotonic
                except Exception as exc:
                    print(f"[ROS2 ERROR] image publish failed: {exc}")

    def _cloud_loop(self):
        while self.running and self._dai_pipeline and self._dai_pipeline.isRunning():
            depth_msg = self._depth_queue.tryGet()
            if depth_msg is None:
                time.sleep(0.001)
                continue

            if not self._reported_depth_format:
                width, height = get_frame_size(depth_msg, DEPTH_WIDTH, DEPTH_HEIGHT)
                print(f"[DepthAI] Depth frame: {width}x{height}")
                self._reported_depth_format = True

            stamp = self._ros_node.get_clock().now().to_msg()
            try:
                self._cloud_pub.publish(
                    build_point_cloud_msg(depth_msg, stamp, DEPTH_FRAME_ID, self.camera_fov)
                )
                self._cloud_count += 1
            except Exception as exc:
                print(f"[ROS2 ERROR] cloud publish failed: {exc}")

    def _poll_gst_bus(self):
        if not self._gst_bus:
            return

        message_types = Gst.MessageType.ERROR | Gst.MessageType.WARNING | Gst.MessageType.EOS
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
            f"[Status] gst={self._frame_count} ({self._frame_count / elapsed:.1f} fps), "
            f"rgb={self._image_count}, cloud={self._cloud_count}, udp={self.host}:{self.port}"
        )
        if self._frame_count == 0 and (self._image_count or self._cloud_count):
            print("[Status] ROS is publishing, but no frames reached GStreamer appsrc yet")
        self._last_status_time = now


def main():
    parser = argparse.ArgumentParser(
        description="OAK -> GStreamer RTP/UDP sender + ROS2 Image/PointCloud2 publisher"
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"Destination IP (default: {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"Destination UDP port (default: {DEFAULT_PORT})")
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC, help=f"ROS2 image topic (default: {DEFAULT_IMAGE_TOPIC})")
    parser.add_argument("--cloud-topic", default=DEFAULT_CLOUD_TOPIC, help=f"ROS2 PointCloud2 topic (default: {DEFAULT_CLOUD_TOPIC})")
    parser.add_argument("--image-width", type=int, default=ROS_IMAGE_WIDTH, help=f"ROS2 image width (default: {ROS_IMAGE_WIDTH})")
    parser.add_argument("--image-height", type=int, default=ROS_IMAGE_HEIGHT, help=f"ROS2 image height (default: {ROS_IMAGE_HEIGHT})")
    parser.add_argument("--image-fps", type=int, default=ROS_IMAGE_FPS, help=f"ROS2 image FPS (default: {ROS_IMAGE_FPS})")
    parser.add_argument("--camera-fov", type=float, default=DEFAULT_CAMERA_FOV, help=f"Horizontal FOV in radians for PointCloud2 projection (default: {DEFAULT_CAMERA_FOV})")
    parser.add_argument("--encoder", choices=("auto", "mpp", "v4l2", "x264"), default="auto", help="H.264 encoder to use (default: auto)")
    args = parser.parse_args()

    sender = OakGstSender(
        args.host,
        args.port,
        args.image_topic,
        args.cloud_topic,
        args.image_width,
        args.image_height,
        args.image_fps,
        args.camera_fov,
        args.encoder,
    )

    def sig_handler(signum, frame):
        sender.running = False

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)
    sender.start()


if __name__ == "__main__":
    main()
