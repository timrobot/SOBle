#!/usr/bin/env python3
"""
Raspberry Pi camera → hardware H.264 → RTP/UDP (GStreamer / PyGObject).

Target: Pi Zero 2 W (32-bit), libcamera via libcamerasrc, 1280×720 @ 30 fps.

Requires on the Pi:
  sudo apt-get install -y \
    python3-gi gstreamer1.0-tools \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-libcamera

Receive on a host (example):
  gst-launch-1.0 -v udpsrc port=5000 caps=application/x-rtp,media=video,encoding-name=H264,payload=96 \
    ! rtph264depay ! h264parse ! avdec_h264 ! autovideosink sync=false

  # or ffplay:
  ffplay -fflags nobuffer -flags low_delay -framedrop \
    -f h264 udp://0.0.0.0:5000?overrun_nonfatal=1&fifo_size=50000000
"""

from __future__ import annotations

import os
import signal
import sys
import multiprocessing as mp
import time

import gi

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402


def build_pipeline(
    host: str,
    port: int,
    width: int = 1280,
    height: int = 720,
    fps: int = 30,
    bitrate: int = 2_500_000,
) -> str:
    """GStreamer pipeline string: libcamerasrc → v4l2h264enc → RTP/UDP.

    - Profile: Main (h264_profile=4) for good compression/compat
    - Level: 4.1 (h264_level=13) — standard for 720p/1080p hardware decode
    - Bitrate: ~2.5 Mbps default for 720p30 over Wi‑Fi
    - Keyframe period: ~0.5s (I-frame every fps/2 frames) for lower RTP/UDP latency
    """
    # Shorter GOP (half-second) for faster recovery on loss / join.
    keyint = max(1, fps // 2)
    return (
        "libcamerasrc ! "
        f"video/x-raw,width={width},height={height},framerate={fps}/1 ! "
        "v4l2h264enc extra-controls="
        f'"controls, h264_profile=4, h264_level=13, '
        f"video_bitrate_mode=0, video_bitrate={bitrate}, "
        f"repeat_sequence_header=1, h264_i_frame_period={keyint}\" ! "
        "rtph264pay config-interval=1 pt=96 ! "
        f"udpsink host={host} port={port} sync=false"
    )

def start_camera_stream(host: str, port: int, width: int, height: int, fps: int, bitrate: int) -> int:
    Gst.init(None)
    pipeline_str = build_pipeline(host, port, width, height, fps, bitrate)
    print(f"Pipeline:\n  {pipeline_str}\n", flush=True)
    print(f"RTP/UDP → {host}:{port}", flush=True)

    pipeline = Gst.parse_launch(pipeline_str)
    bus = pipeline.get_bus()
    loop = GLib.MainLoop()

    def on_message(_bus: Gst.Bus, message: Gst.Message) -> bool:
        t = message.type
        if t == Gst.MessageType.EOS:
            print("EOS", flush=True)
            loop.quit()
        elif t == Gst.MessageType.ERROR:
            err, dbg = message.parse_error()
            print(f"GStreamer error: {err}", flush=True)
            if dbg:
                print(dbg, flush=True)
            loop.quit()
        elif t == Gst.MessageType.WARNING:
            warn, dbg = message.parse_warning()
            print(f"Warning: {warn}", flush=True)
            if dbg:
                print(dbg, flush=True)
        return True

    bus.add_signal_watch()
    bus.connect("message", on_message)

    def shutdown(*_args: object) -> None:
        print("Stopping...", flush=True)
        pipeline.set_state(Gst.State.NULL)
        loop.quit()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        print("Failed to set pipeline to PLAYING", file=sys.stderr)
        return 1

    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)
        bus.remove_signal_watch()

    return 0

def run_camera_stream(host: str, port: int, width: int, height: int, fps: int, bitrate: int) -> mp.Process:
    process = mp.Process(target=start_camera_stream, args=(host, port, width, height, fps, bitrate))
    process.start()
    return process

def stop_camera_stream(process: mp.Process) -> None:
    process.terminate()
    process.join()

if __name__ == "__main__":
    process = run_camera_stream("255.255.255.255", 5000, 1280, 720, 30, 4000000)
    time.sleep(10)
    stop_camera_stream(process)
