#!/usr/bin/env python3
import asyncio
import json
import multiprocessing
import websockets
import gi

# We do not call Gst.init() at the module top level anymore.
# Because we are using the 'spawn' method, initialization must occur
# entirely inside the isolated child process space.

PIPELINE_DESC = """
  libcamerasrc af-mode=continuous ! 
  video/x-raw,width=1152,height=648,framerate=30/1,format=NV12 ! 
  v4l2h264enc device=/dev/video11 capture-io-mode=dmabuf output-io-mode=dmabuf extra-controls="controls,video_bitrate=2000000,video_h264_profile=0" ! 
  video/x-h264,level=(string)4,stream-format=(string)byte-stream ! 
  h264parse ! 
  queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=2 ! 
  rtph264pay config-interval=1 pt=96 aggregate-mode=zero-latency mtu=1200 ! 
  application/x-rtp,media=video,encoding-name=H264,payload=96 ! 
  webrtcbin name=sendrecv
"""

class WebRTCSession:
    """Manages the lifecycle of a single GStreamer WebRTC stream session[cite: 3]."""
    def __init__(self, ws, loop, Gst, GstWebRTC, GstSdp):
        self.ws = ws
        self.loop = loop
        self.Gst = Gst
        self.GstWebRTC = GstWebRTC
        self.GstSdp = GstSdp
        
        print("Starting a new GStreamer streaming instance...")
        self.pipe = self.Gst.parse_launch(PIPELINE_DESC)
        self.webrtc = self.pipe.get_by_name('sendrecv')
        self.webrtc.connect('on-ice-candidate', self.on_ice_candidate)
        self.pipe.set_state(self.Gst.State.PLAYING)

    def on_ice_candidate(self, _, mlineindex, candidate):
        try:
            msg = json.dumps({'candidate': {'candidate': candidate, 'sdpMLineIndex': mlineindex}})
            print(f"[WS] Sending ICE candidate to client", flush=True)
            asyncio.run_coroutine_threadsafe(self.ws.send(msg), self.loop)
        except Exception as e:
            print(f"[WS] Error sending ICE candidate: {e}", flush=True)

    def process_message(self, data):
        try:
            if 'answer' in data:
                print(f"[WS] Received answer from client", flush=True)
                sdp_text = data['answer']['sdp']
                res, sdp_msg = self.GstSdp.SDPMessage.new()
                self.GstSdp.sdp_message_parse_buffer(bytes(sdp_text.encode()), sdp_msg)
                answer = self.GstWebRTC.WebRTCSessionDescription.new(self.GstWebRTC.WebRTCSDPType.ANSWER, sdp_msg)
                promise = self.Gst.Promise.new()
                self.webrtc.emit('set-remote-description', answer, promise)
                promise.interrupt() 
                print(f"[WS] Answer processed", flush=True)
            elif 'candidate' in data:
                print(f"[WS] Received ICE candidate from client", flush=True)
                cand = data['candidate']['candidate']
                mline = data['candidate']['sdpMLineIndex']
                self.webrtc.emit('add-ice-candidate', mline, cand)
            elif data.get('request') == 'offer':
                print(f"[WS] Received offer request from client", flush=True)
                promise = self.Gst.Promise.new_with_change_func(self.on_offer_created, None)
                self.webrtc.emit('create-offer', None, promise)
                print(f"[WS] Offer creation in progress...", flush=True)
        except Exception as e:
            print(f"[WS] Error in process_message: {e}", flush=True)
            raise

    def on_offer_created(self, promise, _):
        try:
            print(f"[WS] Offer creation callback fired", flush=True)
            reply = promise.get_reply()
            offer = reply.get_value('offer')
            self.webrtc.emit('set-local-description', offer, None)
            text = offer.sdp.as_text()
            msg = json.dumps({'offer': {'type': 'offer', 'sdp': text}})
            print(f"[WS] Sending offer to client ({len(msg)} bytes)", flush=True)
            asyncio.run_coroutine_threadsafe(self.ws.send(msg), self.loop)
            print(f"[WS] Offer sent", flush=True)
        except Exception as e:
            print(f"[WS] Error in on_offer_created: {e}", flush=True)
            raise

    def stop(self):
        print("Tearing down GStreamer streaming instance...")
        self.pipe.set_state(self.Gst.State.NULL)


def _worker_process_entry(host, port):
    """Target function executed entirely within the spawned process context."""
    print(f"[WS] Worker process started, host={host}, port={port}", flush=True)
    
    # Strict dynamic runtime imports to safely isolate GStreamer symbols from parent process space
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstWebRTC', '1.0')
    gi.require_version('GstSdp', '1.0')
    from gi.repository import Gst, GstWebRTC, GstSdp, GLib
    import threading

    print(f"[WS] GStreamer libraries loaded", flush=True)
    Gst.init(None)
    print(f"[WS] GStreamer initialized", flush=True)

    # 1. Run GStreamer core event loops inside a tracking worker thread
    gst_loop = GLib.MainLoop()
    gst_thread = threading.Thread(target=gst_loop.run, daemon=True)
    gst_thread.start()
    print(f"[WS] GStreamer event loop started", flush=True)

    # 2. Run Async WebSocket handler execution cycles
    async def _handle_connection(ws):
        loop = asyncio.get_running_loop()
        print(f"[WS] New client connected from {ws.remote_address}", flush=True)
        session = WebRTCSession(ws, loop, Gst, GstWebRTC, GstSdp)
        try:
            async for message in ws:
                try:
                    data = json.loads(message)
                    session.process_message(data)
                except json.JSONDecodeError as e:
                    print(f"[WS] JSON decode error: {e}", flush=True)
                except Exception as e:
                    print(f"[WS] Error processing message: {e}", flush=True)
        except websockets.exceptions.ConnectionClosed:
            print(f"[WS] Client disconnected from {ws.remote_address}", flush=True)
        except Exception as e:
            print(f"[WS] Unexpected error in connection handler: {e}", flush=True)
        finally:
            try:
                session.stop()
            except Exception as e:
                print(f"[WS] Error stopping session: {e}", flush=True)

    async def run_server():
        print(f"[WS] Signaling server listening on ws://{host}:{port}", flush=True)
        try:
            async with websockets.serve(_handle_connection, host, port):
                print(f"[WS] Server ready to accept connections on port {port}", flush=True)
                await asyncio.Future()  # block forever
        except OSError as e:
            print(f"[WS] FATAL: Failed to bind to {host}:{port} - {e}", flush=True)
            raise
        except Exception as e:
            print(f"[WS] FATAL: Unexpected error in run_server - {e}", flush=True)
            raise

    try:
        print(f"[WS] Starting asyncio event loop", flush=True)
        asyncio.run(run_server())
    except KeyboardInterrupt:
        print("[WS] KeyboardInterrupt received", flush=True)
    except Exception as e:
        print(f"[WS] Asyncio error: {e}", flush=True)
    finally:
        print("[WS] Shutting down...", flush=True)
        gst_loop.quit()
        gst_thread.join(timeout=1.0)
        print("[WS] Worker process exiting", flush=True)


class WebRTCStreamer:
    """The external engine wrapper providing clean multiprocessing lifecycles."""
    def __init__(self, host="0.0.0.0", port=8765):
        self.host = host
        self.port = port
        self._process = None

    def startStreamTask(self):
        """Spawns an isolated background process to run the signaling and camera pipeline."""
        if self._process and self._process.is_alive():
            print("WebRTCStreamer is already running.")
            return

        # Explicitly enforce clean 'spawn' behavior regardless of system environment defaults
        ctx = multiprocessing.get_context('spawn')
        
        self._process = ctx.Process(
            target=_worker_process_entry,
            args=(self.host, self.port),
            daemon=True
        )
        self._process.start()
        print(f"WebRTCStreamer process spawned successfully (PID: {self._process.pid}).")

    def stopStreamTask(self):
        """Gracefully terminates the background process and instantly drops camera locks."""
        if not self._process:
            return

        print("Stopping WebRTCStreamer task...")
        if self._process.is_alive():
            self._process.terminate()  # Sends standard SIGTERM signallers
            self._process.join(timeout=2.0)
            
            # Forced cleanup fallback if process stays locked in memory blocks
            if self._process.is_alive():
                print("Process did not respond to SIGTERM, invoking kill...")
                self._process.kill()
                self._process.join()

        self._process = None
        print("WebRTCStreamer stopped cleanly. Camera hardware released.")


if __name__ == "__main__":
    import time
    streamer = WebRTCStreamer()
    streamer.startStreamTask()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        streamer.stopStreamTask()