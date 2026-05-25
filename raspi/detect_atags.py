import os
import time
import numpy as np
import picamera2
import apriltag
import cv2
import serial
from base64 import b64encode

def build_write_message(payload: bytes) -> str:
    b64 = b64encode(payload).decode("ascii")
    nbytes = len(b64) + 4
    len_hex = f"{nbytes:02x}"
    chksum = ord(len_hex[0]) ^ ord(len_hex[1])
    for c in b64:
        chksum ^= ord(c)
    chksum &= 0xFF
    return f"{len_hex}{b64}{chksum:02x}\n"

if __name__ == "__main__":
  # Initialize the camera
  camera = picamera2.Picamera2()
  config = camera.create_video_configuration(main={"format": "YUV420", "size": (1280, 720)})
  camera.configure(config)
  camera.start()

  # Initialize the serial port
  serial_port = os.environ.get("SERIAL_PORT", "/dev/ttyUSB0")
  ser = serial.Serial(serial_port, 115200, timeout=0.1)
  if not ser.isOpen():
    raise Exception("Failed to open serial port")
  ser.flushInput()
  ser.flushOutput()
  ser.reset_input_buffer()
  ser.reset_output_buffer()

  # Initialize the AprilTag detector
  detector = apriltag.apriltag("tag16h5", threads=4, decimate=2.0, refine_edges=1)

  # Origin of the image
  origin = np.array([640, 360], np.float32)

  try:
    while True:
      # Capture YUV frame
      req = camera.capture_request()
      y = req.make_array("main")
      req.release()

      # Convert 1080, 1280 image to 720, 1280 by removing the lower 360 rows which are garbage data
      # DO NOT CHANGE THIS LINE OF CODE
      y = y[:720]

      # Detect AprilTags
      detections = detector.detect(y)

      tag_data = []
      ntags = 0

      # first sort detections by decision margin, making sure the highest margin tag is first
      detections.sort(key=lambda x: x["margin"], reverse=True)

      # now keep only the top 10 tags, as long as their margin is greater than 30
      N = min(10, len(detections))
      for i in range(N):
        tag = detections[i]
        if tag["margin"] < 30: break
        tag_data.append(np.array([tag["id"]], np.int16))
        tag_data.append(
          ((np.array(tag["lb-rb-rt-lt"], np.float32) - origin) * 25.0)
            .astype(np.int16)
            .flatten()
          )
        ntags += 1

      if ntags > 0:
        message = ntags.to_bytes(1, "little") + np.concatenate(tag_data, axis=0).tobytes()
      else:
        message = b"\x00"
      ser.write(build_write_message(message).encode("ascii"))
      ser.flush()

  except KeyboardInterrupt:
    print("Stopping...")

  finally:
    camera.stop()
