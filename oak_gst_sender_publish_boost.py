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
import atexit
import queue
import signal
import struct
import threading
import time
from multiprocessing import shared_memory

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
CAM_WIDTH = 960
CAM_HEIGHT = 540
CAM_FPS = 5
ROS_IMAGE_WIDTH = 640
ROS_IMAGE_HEIGHT = 480
ROS_IMAGE_FPS = 3
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 400
DEPTH_FPS = 30
SHM_VIDEO_WIDTH = 960
SHM_VIDEO_HEIGHT = 540
SHM_FPS = 5
SHM_DEPTH_WIDTH = 640
SHM_DEPTH_HEIGHT = 480
SHM_DEPTH_FPS = 20


DEFAULT_IMAGE_TOPIC = "/oak/rgb/image_raw"
DEFAULT_DEPTH_TOPIC = "/oak/stereo/depth"
RGB_FRAME_ID = "oak_rgb_camera_optical_frame"
DEPTH_FRAME_ID = "oak_depth_camera_optical_frame"

SHM_MAGIC = b"OAKSHM1\0"
SHM_HEADER_FORMAT = "<8sIIIIIIIIQQ"
SHM_HEADER_SIZE = 4096
SHM_ENCODING_SIZE = 16
SHM_DEFAULT_SLOTS = 4


class ShmImageRing:
    """
    POSIX shared-memory ring buffer.

    C++ readers can open the same segment with boost::interprocess::shared_memory_object.
    Layout:
      header at offset 0, slot sequence counters after header,
      then fixed-size image slots.
    """

    def __init__(
        self,
        name: str,
        width: int,
        height: int,
        step: int,
        encoding: str,
        slots: int,
        frame_bytes: int,
    ):
        self.name = name
        self.width = width
        self.height = height
        self.step = step
        self.encoding = encoding
        self.slots = max(int(slots), 1)
        self.frame_bytes = int(frame_bytes)
        self.seq = 0
        self.write_index = 0
        self._slot_seq_offset = SHM_HEADER_SIZE
        self._data_offset = self._slot_seq_offset + self.slots * 8
        self._total_size = self._data_offset + self.slots * self.frame_bytes

        self._unlink_existing()
        self._shm = shared_memory.SharedMemory(
            name=self.name, create=True, size=self._total_size
        )
        atexit.register(self.close)
        self._write_header()
        print(
            f"[SHM] Created {self.name}: {self.width}x{self.height} "
            f"{self.encoding}, step={self.step}, slots={self.slots}, "
            f"frame_bytes={self.frame_bytes}"
        )

    def close(self):
        shm = getattr(self, "_shm", None)
        if shm is None:
            return
        self._shm = None
        try:
            shm.close()
        except FileNotFoundError:
            pass
        try:
            shm.unlink()
        except FileNotFoundError:
            pass

    def write(self, frame_data) -> None:
        data = bytes(frame_data)
        if len(data) > self.frame_bytes:
            raise ValueError(
                f"Frame for {self.name} is too large: {len(data)} > {self.frame_bytes}"
            )

        slot = self.write_index
        self.seq += 1
        odd_seq = self.seq * 2 + 1
        even_seq = self.seq * 2 + 2
        self._write_slot_seq(slot, odd_seq)

        offset = self._data_offset + slot * self.frame_bytes
        self._shm.buf[offset : offset + len(data)] = data
        if len(data) < self.frame_bytes:
            self._shm.buf[offset + len(data) : offset + self.frame_bytes] = b"\0" * (
                self.frame_bytes - len(data)
            )

        self._write_slot_seq(slot, even_seq)
        self._write_header(latest_slot=slot, latest_seq=even_seq, data_size=len(data))
        self.write_index = (self.write_index + 1) % self.slots

    def _write_header(self, latest_slot: int = 0, latest_seq: int = 0, data_size=None):
        encoding_bytes = self.encoding.encode("ascii", errors="ignore")[
            :SHM_ENCODING_SIZE
        ]
        encoding_bytes = encoding_bytes + b"\0" * (SHM_ENCODING_SIZE - len(encoding_bytes))
        data_size = self.frame_bytes if data_size is None else int(data_size)
        struct.pack_into(
            SHM_HEADER_FORMAT,
            self._shm.buf,
            0,
            SHM_MAGIC,
            1,
            self.width,
            self.height,
            self.step,
            self.frame_bytes,
            self.slots,
            self._data_offset,
            data_size,
            latest_slot,
            latest_seq,
        )
        self._shm.buf[struct.calcsize(SHM_HEADER_FORMAT) : struct.calcsize(SHM_HEADER_FORMAT) + SHM_ENCODING_SIZE] = encoding_bytes

    def _write_slot_seq(self, slot: int, seq: int):
        struct.pack_into("<Q", self._shm.buf, self._slot_seq_offset + slot * 8, seq)

    def _unlink_existing(self):
        try:
            old = shared_memory.SharedMemory(name=self.name, create=False)
        except FileNotFoundError:
            return
        old.close()
        try:
            old.unlink()
        except FileNotFoundError:
            pass


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


def build_depthai_pipeline(
    video_width: int,
    video_height: int,
    video_fps: int,
    enable_depth: bool,
    depth_fps: int,
    depth_preset_name: str,
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

    video_queue = video_out.createOutputQueue(maxSize=2, blocking=False)
    depth_queue = None

    if enable_depth:
        left = pipeline.create(dai.node.Camera).build(
            boardSocket=dai.CameraBoardSocket.CAM_B,
            sensorFps=float(depth_fps),
        )
        right = pipeline.create(dai.node.Camera).build(
            boardSocket=dai.CameraBoardSocket.CAM_C,
            sensorFps=float(depth_fps),
        )

        stereo = pipeline.create(dai.node.StereoDepth)
        preset_mode = dai.node.StereoDepth.PresetMode
        if depth_preset_name == "high" and hasattr(preset_mode, "HIGH_DENSITY"):
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
            fps=float(depth_fps),
        ).link(stereo.left)
        right.requestOutput(
            size=(DEPTH_WIDTH, DEPTH_HEIGHT),
            type=dai.ImgFrame.Type.GRAY8,
            fps=float(depth_fps),
        ).link(stereo.right)

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
        enable_image_publish: bool,
        enable_depth: bool,
        enable_depth_publish: bool,
        depth_fps: int,
        depth_publish_fps: int,
        depth_preset_name: str,
        enable_shm: bool,
        shm_prefix: str,
        shm_slots: int,
    ):
        self.host = host
        self.port = port
        self.image_topic = image_topic
        self.depth_topic = depth_topic
        self.image_width = image_width
        self.image_height = image_height
        self.image_fps = image_fps
        self.enable_image_publish = enable_image_publish
        self.enable_depth = enable_depth
        self.enable_depth_publish = enable_depth_publish
        self.depth_fps = depth_fps
        self.depth_publish_fps = depth_publish_fps
        self.depth_preset_name = depth_preset_name
        self.enable_shm = enable_shm
        self.shm_prefix = shm_prefix
        self.shm_slots = shm_slots
        self._ros_enabled = enable_image_publish or (
            enable_depth and enable_depth_publish
        )
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
        self._rgb_shm = None
        self._depth_shm = None
        self._video_thread = None
        self._image_thread = None
        self._depth_thread = None
        self._image_frame_queue = queue.Queue(maxsize=1)
        self._frame_count = 0
        self._image_count = 0
        self._depth_count = 0
        self._depth_publish_count = 0
        self._last_stats_frame_count = 0
        self._last_status_time = 0.0
        self._last_status_frame_count = 0
        self._last_status_image_count = 0
        self._last_status_depth_count = 0
        self._last_status_depth_publish_count = 0
        self._last_image_publish_time = 0.0
        self._last_image_offer_time = 0.0
        self._last_depth_publish_time = 0.0
        self._reported_video_format = False
        self._reported_depth_format = False
        self._start_time = 0.0

    def start(self):
        Gst.init(None)

        if self._ros_enabled:
            if not rclpy.ok():
                rclpy.init()

            self._ros_node = rclpy.create_node("oak_gst_sender_publish")
            qos = make_sensor_qos()
            if self.enable_image_publish:
                self._image_pub = self._ros_node.create_publisher(
                    Image, self.image_topic, qos
                )
            if self.enable_depth and self.enable_depth_publish:
                self._depth_pub = self._ros_node.create_publisher(
                    Image, self.depth_topic, qos
                )
        else:
            print("[ROS2] All publishers disabled; ROS node will not be created")

        if self.enable_image_publish:
            print(
                f"[ROS2] Publishing RGB image: {self.image_topic} "
                f"(bgr8, {self.image_width}x{self.image_height}@{self.image_fps})"
            )
        else:
            print("[ROS2] RGB image publishing disabled")
        if self.enable_depth and self.enable_depth_publish:
            print(
                f"[ROS2] Publishing depth: {self.depth_topic} "
                f"(16UC1, mm, target {self.depth_publish_fps} fps)"
            )
        elif self.enable_depth:
            print("[ROS2] Depth pipeline enabled, ROS depth publishing disabled")
        else:
            print("[DepthAI] Stereo depth pipeline disabled")

        if self.enable_shm:
            rgb_frame_bytes = CAM_WIDTH * CAM_HEIGHT * 3 // 2
            self._rgb_shm = ShmImageRing(
                f"{self.shm_prefix}_rgb",
                CAM_WIDTH,
                CAM_HEIGHT,
                CAM_WIDTH,
                "nv12",
                self.shm_slots,
                rgb_frame_bytes,
            )
            if self.enable_depth:
                depth_frame_bytes = DEPTH_WIDTH * DEPTH_HEIGHT * 2
                self._depth_shm = ShmImageRing(
                    f"{self.shm_prefix}_depth",
                    DEPTH_WIDTH,
                    DEPTH_HEIGHT,
                    DEPTH_WIDTH * 2,
                    "16UC1",
                    self.shm_slots,
                    depth_frame_bytes,
                )
        else:
            print("[SHM] Shared-memory image output disabled")

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
            self.enable_depth,
            self.depth_fps,
            self.depth_preset_name,
        )
        self._dai_pipeline.start()
        print("[DepthAI] Pipeline started, capturing...")

        self.running = True
        self._start_time = time.monotonic()
        self._frame_count = 0
        self._image_count = 0
        self._depth_count = 0
        self._depth_publish_count = 0
        self._last_stats_frame_count = 0
        self._last_status_time = self._start_time
        self._last_status_frame_count = 0
        self._last_status_image_count = 0
        self._last_status_depth_count = 0
        self._last_status_depth_publish_count = 0
        self._last_image_publish_time = 0.0
        self._last_image_offer_time = 0.0
        self._last_depth_publish_time = 0.0
        self._reported_video_format = False
        self._reported_depth_format = False
        self._image_frame_queue = queue.Queue(maxsize=1)

        self._video_thread = threading.Thread(
            target=self._video_loop,
            name="oak-gst-video",
            daemon=True,
        )
        self._video_thread.start()
        if self.enable_image_publish:
            self._image_thread = threading.Thread(
                target=self._image_loop,
                name="oak-ros-image",
                daemon=True,
            )
            self._image_thread.start()
        if self._depth_queue is not None:
            self._depth_thread = threading.Thread(
                target=self._depth_loop,
                name="oak-ros-depth",
                daemon=True,
            )
            self._depth_thread.start()

        # ── Supervisor loop ──
        try:
            while self.running and self._dai_pipeline.isRunning():
                if self._ros_node is not None:
                    rclpy.spin_once(self._ros_node, timeout_sec=0.0)
                self._poll_gst_bus()
                self._print_periodic_status()
                time.sleep(0.005)

        except KeyboardInterrupt:
            print("\n[Ctrl+C] Stopping...")
        finally:
            self.stop()

    def stop(self):
        self.running = False

        for worker in (self._video_thread, self._image_thread, self._depth_thread):
            if worker and worker.is_alive() and worker is not threading.current_thread():
                worker.join(timeout=1.0)

        if self._appsrc:
            self._appsrc.emit("end-of-stream")
            self._appsrc = None
        if self._gst_pipeline:
            self._gst_pipeline.set_state(Gst.State.NULL)
            self._gst_pipeline = None
            print("[GStreamer] Pipeline stopped")
        if self._gst_bus:
            self._gst_bus.remove_signal_watch()
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
        if self._rgb_shm:
            self._rgb_shm.close()
            self._rgb_shm = None
        if self._depth_shm:
            self._depth_shm.close()
            self._depth_shm = None

    def _video_loop(self):
        while self.running and self._dai_pipeline and self._dai_pipeline.isRunning():
            frame_msg = self._video_queue.tryGet()
            if frame_msg is None:
                time.sleep(0.001)
                continue

            # getData() returns raw bytes (NV12 from isp output)
            frame_data = frame_msg.getData()
            if not self._reported_video_format:
                width, height = get_frame_size(frame_msg, CAM_WIDTH, CAM_HEIGHT)
                print(
                    f"[DepthAI] GST video frame: "
                    f"{width}x{height}, {len(frame_data)} bytes"
                )
                self._reported_video_format = True

            buf = Gst.Buffer.new_allocate(None, len(frame_data), None)
            buf.fill(0, bytes(frame_data))
            elapsed_ns = int((time.monotonic() - self._start_time) * Gst.SECOND)
            buf.pts = elapsed_ns
            buf.duration = Gst.SECOND // CAM_FPS

            ret = self._appsrc.emit("push-buffer", buf)
            if ret != Gst.FlowReturn.OK:
                print(f"[GStreamer] appsrc push-buffer returned {ret}")
                self.running = False
                break

            self._frame_count += 1
            if self._rgb_shm is not None:
                try:
                    self._rgb_shm.write(frame_data)
                except Exception as exc:
                    print(f"[SHM ERROR] RGB write failed: {exc}")
                    self._rgb_shm = None
            if self.enable_image_publish:
                self._offer_image_frame(frame_msg)

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
                    f"{self._depth_publish_count} depth, "
                    f"avg gst {self._frame_count / max(elapsed, 1e-6):.1f} fps"
                )

    def _offer_image_frame(self, frame_msg):
        monotonic_now = time.monotonic()
        image_period = 1.0 / max(float(self.image_fps), 1.0)
        if monotonic_now - self._last_image_offer_time < image_period:
            return

        try:
            self._image_frame_queue.put_nowait(frame_msg)
            self._last_image_offer_time = monotonic_now
        except queue.Full:
            try:
                self._image_frame_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._image_frame_queue.put_nowait(frame_msg)
                self._last_image_offer_time = monotonic_now
            except queue.Full:
                pass

    def _image_loop(self):
        while self.running and self._dai_pipeline and self._dai_pipeline.isRunning():
            try:
                frame_msg = self._image_frame_queue.get(timeout=0.05)
            except queue.Empty:
                continue

            stamp = self._ros_node.get_clock().now().to_msg()
            try:
                self._image_pub.publish(
                    build_image_msg(
                        frame_msg,
                        stamp,
                        RGB_FRAME_ID,
                        self.image_width,
                        self.image_height,
                    )
                )
                self._image_count += 1
                self._last_image_publish_time = time.monotonic()
            except Exception as exc:
                print(f"[ROS2 ERROR] image publish failed: {exc}")

    def _depth_loop(self):
        while self.running and self._dai_pipeline and self._dai_pipeline.isRunning():
            depth_msg = self._depth_queue.tryGet()
            if depth_msg is None:
                time.sleep(0.001)
                continue

            if not self._reported_depth_format:
                width, height = get_frame_size(depth_msg, DEPTH_WIDTH, DEPTH_HEIGHT)
                print(f"[DepthAI] Depth frame: {width}x{height}")
                self._reported_depth_format = True

            self._depth_count += 1
            if self._depth_shm is not None:
                try:
                    self._depth_shm.write(depth_msg.getData())
                except Exception as exc:
                    print(f"[SHM ERROR] Depth write failed: {exc}")
                    self._depth_shm = None
            if not self.enable_depth_publish:
                continue

            monotonic_now = time.monotonic()
            publish_period = 1.0 / max(float(self.depth_publish_fps), 1.0)
            if monotonic_now - self._last_depth_publish_time < publish_period:
                continue

            stamp = self._ros_node.get_clock().now().to_msg()
            try:
                self._depth_pub.publish(
                    build_depth_msg(depth_msg, stamp, DEPTH_FRAME_ID)
                )
                self._depth_publish_count += 1
                self._last_depth_publish_time = monotonic_now
            except Exception as exc:
                print(f"[ROS2 ERROR] depth publish failed: {exc}")

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
        depth_pub_now = (
            self._depth_publish_count - self._last_status_depth_publish_count
        ) / max(dt, 1e-6)
        print(
            f"[Status] gst={self._frame_count} "
            f"({gst_now:.1f} now, {self._frame_count / elapsed:.1f} avg), "
            f"rgb={self._image_count} ({rgb_now:.1f} now), "
            f"depth_in={self._depth_count} ({depth_now:.1f} now), "
            f"depth_pub={self._depth_publish_count} ({depth_pub_now:.1f} now), "
            f"udp={self.host}:{self.port}"
        )
        if self._frame_count == 0 and (self._image_count or self._depth_count):
            print("[Status] ROS frames are flowing, but GST video queue has no frames")
        self._last_status_frame_count = self._frame_count
        self._last_status_image_count = self._image_count
        self._last_status_depth_count = self._depth_count
        self._last_status_depth_publish_count = self._depth_publish_count
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
        "--no-image-publish",
        action="store_true",
        help="Deprecated compatibility flag; ROS2 RGB publishing is disabled by default",
    )
    parser.add_argument(
        "--publish-ros-image",
        action="store_true",
        help="Enable ROS2 RGB image publishing in addition to shared memory",
    )
    parser.add_argument(
        "--no-depth",
        action="store_true",
        help="Disable DepthAI stereo depth pipeline for diagnostics",
    )
    parser.add_argument(
        "--no-depth-publish",
        action="store_true",
        help="Deprecated compatibility flag; ROS2 depth publishing is disabled by default",
    )
    parser.add_argument(
        "--publish-ros-depth",
        action="store_true",
        help="Enable ROS2 depth image publishing in addition to shared memory",
    )
    parser.add_argument(
        "--depth-fps",
        type=int,
        default=DEPTH_FPS,
        help=f"DepthAI stereo camera FPS (default: {DEPTH_FPS})",
    )
    parser.add_argument(
        "--depth-publish-fps",
        type=int,
        default=DEPTH_FPS,
        help=f"ROS2 depth publish FPS limit (default: {DEPTH_FPS})",
    )
    parser.add_argument(
        "--depth-preset",
        choices=("high", "fast"),
        default="high",
        help="StereoDepth preset: high uses HIGH_DENSITY when available, fast uses FAST_DENSITY",
    )
    parser.add_argument(
        "--no-shm",
        action="store_true",
        help="Disable shared-memory RGB/depth output",
    )
    parser.add_argument(
        "--shm-prefix",
        default="oak",
        help="Shared-memory segment prefix (default: oak -> oak_rgb/oak_depth)",
    )
    parser.add_argument(
        "--shm-slots",
        type=int,
        default=SHM_DEFAULT_SLOTS,
        help=f"Shared-memory ring slots (default: {SHM_DEFAULT_SLOTS})",
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
        args.publish_ros_image and not args.no_image_publish,
        not args.no_depth,
        args.publish_ros_depth and not args.no_depth_publish,
        args.depth_fps,
        args.depth_publish_fps,
        args.depth_preset,
        not args.no_shm,
        args.shm_prefix,
        args.shm_slots,
    )

    def sig_handler(signum, frame):
        sender.running = False

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    sender.start()


if __name__ == "__main__":
    main()
