"""Host-side RTP/H.264 receiver → shared BGR frame buffer (GStreamer / PyGObject)."""

from __future__ import annotations

import multiprocessing as mp
import numpy as np
import platform
import signal
import sys
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from multiprocessing.synchronize import Event as MpEvent

# Match raspi/camera_stream.py sender (1280×720)
STREAM_WIDTH = 1280
STREAM_HEIGHT = 720
STREAM_CHANNELS = 3
STREAM_FRAME_BYTES = STREAM_WIDTH * STREAM_HEIGHT * STREAM_CHANNELS
STREAM_BUFFER_COUNT = 2
SHARED_FRAME_BYTES = STREAM_FRAME_BYTES * STREAM_BUFFER_COUNT

_RTP_CAPS = (
    "application/x-rtp,media=video,encoding-name=H264,payload=96"
)


_DECODER_CACHE: str | None = None


def _gst_element_available(name: str) -> bool:
    """True if GStreamer can instantiate this element (Gst API, else gst-inspect-1.0)."""
    try:
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst  # noqa: E402

        if not Gst.is_initialized():
            Gst.init(None)
        return Gst.ElementFactory.find(name) is not None
    except Exception:
        return _gst_element_available_inspect(name)


def _gst_element_available_inspect(name: str) -> bool:
    import shutil
    import subprocess

    gst_inspect = shutil.which("gst-inspect-1.0")
    if gst_inspect is None:
        return False
    try:
        return (
            subprocess.run(
                [gst_inspect, name],
                capture_output=True,
                timeout=5,
                check=False,
            ).returncode
            == 0
        )
    except (OSError, subprocess.TimeoutExpired):
        return False


def _pick_decoder(candidates: list[str]) -> str:
    for element in candidates:
        if _gst_element_available(element):
            return element
    return "avdec_h264"


def _decoder_element() -> str:
    """Pick the best installed H.264 decoder for this OS (probed via GStreamer)."""
    global _DECODER_CACHE
    if _DECODER_CACHE is not None:
        return _DECODER_CACHE

    system = platform.system()
    if system == "Darwin":
        _DECODER_CACHE = _pick_decoder(["vtdec_h264", "avdec_h264"])
    elif system in ("Windows", "Linux"):
        _DECODER_CACHE = _pick_decoder(["nvh264dec", "avdec_h264"])
    else:
        _DECODER_CACHE = _pick_decoder(["avdec_h264"])
    return _DECODER_CACHE


def build_receive_pipeline(port: int = 5000) -> str:
    """UDP/RTP H.264 → decoder → BGR appsink."""
    decoder = _decoder_element()
    return (
        # 1. buffer-size=2000000 expands the OS network stack UDP cache to 2MB to handle bursts.
        f"udpsrc port={int(port)} buffer-size=2000000 caps=\"{_RTP_CAPS}\" ! "
        # 2. rtpjitterbuffer collects, sorts, and reorders out-of-order wireless packets.
        "rtpjitterbuffer latency=50 drop-on-latency=true ! "
        "rtph264depay ! "
        "h264parse ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        f"{decoder} ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false"
    )


def _receive_stream_worker(
    port: int,
    frame_buf: mp.Array,
    write_slot: mp.Value,
    read_slot: mp.Value,
    frame_ready: MpEvent,
    frame_seq: mp.Value,
    frame_lock: mp.Lock,
    stop: MpEvent,
) -> None:
    import gi

    gi.require_version("Gst", "1.0")
    from gi.repository import GLib, Gst  # noqa: E402

    Gst.init(None)
    pipeline = Gst.parse_launch(build_receive_pipeline(port))
    appsink = pipeline.get_by_name("sink")
    if appsink is None:
        print("camera_stream: appsink not found", file=sys.stderr)
        return

    def on_new_sample(_sink: Gst.Element) -> Gst.FlowReturn:
        if stop.is_set():
            return Gst.FlowReturn.EOS
        sample = _sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR
        buffer = sample.get_buffer()
        ok, map_info = buffer.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR
        try:
            n = map_info.size
            if n > STREAM_FRAME_BYTES:
                return Gst.FlowReturn.OK

            with frame_lock:
                slot = int(write_slot.value) % STREAM_BUFFER_COUNT
                offset = slot * STREAM_FRAME_BYTES
                obj = frame_buf.get_obj()
                dest_arr = np.frombuffer(obj, dtype=np.uint8, offset=offset, count=n)
                src_arr = np.frombuffer(map_info.data, dtype=np.uint8, count=n)
                np.copyto(dest_arr, src_arr)
                read_slot.value = slot
                write_slot.value = 1 - slot
                frame_seq.value += 1
            frame_ready.set()
        finally:
            buffer.unmap(map_info)
        return Gst.FlowReturn.OK

    appsink.connect("new-sample", on_new_sample)
    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_message(_bus: Gst.Bus, message: Gst.Message) -> bool:
        if message.type in (Gst.MessageType.EOS, Gst.MessageType.ERROR):
            loop.quit()
        return True

    bus.connect("message", on_message)

    def shutdown(*_args: object) -> None:
        pipeline.set_state(Gst.State.NULL)
        loop.quit()

    def poll_stop() -> bool:
        if stop.is_set():
            shutdown()
            return False
        return True

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    GLib.timeout_add(100, poll_stop)
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    finally:
        bus.remove_signal_watch()
        pipeline.set_state(Gst.State.NULL)


def run_receive_stream(
    port: int,
    frame_buf: mp.Array,
    write_slot: mp.Value,
    read_slot: mp.Value,
    frame_ready: MpEvent,
    frame_seq: mp.Value,
    frame_lock: mp.Lock,
    stop: MpEvent,
    *,
    mp_context: mp.context.BaseContext | None = None,
) -> mp.Process:
    """Start the GStreamer receive loop in a child process."""
    ctx = mp_context or mp
    proc = ctx.Process(
        target=_receive_stream_worker,
        args=(
            port,
            frame_buf,
            write_slot,
            read_slot,
            frame_ready,
            frame_seq,
            frame_lock,
            stop,
        ),
        daemon=True,
    )
    proc.start()
    return proc


def stop_receive_stream(process: mp.Process | None, stop: MpEvent | None) -> None:
    if stop is not None:
        stop.set()
    if process is None:
        return
    process.join(timeout=3.0)
    if process.is_alive():
        process.terminate()
        process.join(timeout=1.0)
