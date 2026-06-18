#!/usr/bin/env python3
"""H.264 camera stream, display with OpenCV. Configure WiFi on the Pi manually."""

import cv2
from soble import SO101Platform

if __name__ == "__main__":
    platform = SO101Platform("Capybara", log_state=False)

    host = platform.videoCapture()
    print(f"Stream requested to {host}:5000 — press Q or Esc to quit")

    try:
        while True:
            frame = platform.imread()
            if frame is not None:
                cv2.imshow("SO101 camera", frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        cv2.destroyAllWindows()
