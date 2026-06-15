#!/usr/bin/env python3
"""BLE → Pi WiFi → RTP camera stream (UDP 5000), display with OpenCV."""

import time

import cv2
from soble import SO101Platform

if __name__ == "__main__":
    platform = SO101Platform("Capybara", log_state=False)

    def on_frame(frame) -> None:
        cv2.imshow("SO101 camera", frame)
        cv2.waitKey(1)

    platform.start()

    try:
        host = platform.enableCameraStreamMode(onFrameCallback=on_frame)
        print(f"Stream requested to {host}:5000 — press Q or Esc to quit")

        while True:
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        platform.stop()
        print("Stopped.")
