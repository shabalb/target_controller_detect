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
import sys
import time
import signal

import depthai as dai
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp, GLib

# ─── Configuration ───────────────────────────────────────────────────────────

DEFAULT_HOST = "192.168.1.100"
DEFAULT_PORT = 5600
CAM_WIDTH = 1920
CAM_HEIGHT = 1080
CAM_FPS = 60


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

    video_queue = video_out.createOutputQueue(maxSize=2, blocking=False)

    return pipeline, video_queue


class OakGstSender:
    """Captures from OAK (v3 API), pushes frames into GStreamer appsrc."""

    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.running = False
        self._gst_pipeline = None
        self._appsrc = None
        self._dai_pipeline = None
        self._video_queue = None
        self._frame_count = 0
        self._start_time = 0.0

    def start(self):
        Gst.init(None)

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
        self._dai_pipeline, self._video_queue = build_depthai_pipeline(
            CAM_WIDTH, CAM_HEIGHT, CAM_FPS
        )
        self._dai_pipeline.start()
        print("[DepthAI] Pipeline started, capturing...")

        self.running = True
        self._start_time = time.monotonic()
        self._frame_count = 0

        # ── Capture loop ──
        try:
            while self.running and self._dai_pipeline.isRunning():
                frame_msg = self._video_queue.tryGet()
                if frame_msg is None:
                    time.sleep(0.001)
                    continue

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
                if self._frame_count % (CAM_FPS * 5) == 0:
                    elapsed = time.monotonic() - self._start_time
                    print(
                        f"[Stats] Sent {self._frame_count} frames, "
                        f"avg {self._frame_count / elapsed:.1f} fps"
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
    args = parser.parse_args()

    sender = OakGstSender(args.host, args.port)

    def sig_handler(signum, frame):
        sender.running = False

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    sender.start()


if __name__ == "__main__":
    main()
