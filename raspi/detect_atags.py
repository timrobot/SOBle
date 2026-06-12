import os
import ipaddress
import time
import numpy as np
import picamera2
import apriltag
import serial
from base64 import b64decode, b64encode
import camera_stream


# Host → Pi commands: D + LL<base64>CC (HostSerial::writeBytes; D = devid, ignored on Pi).
# Decoded payload (7 bytes):
#   uint8 cmd;
#   union { struct { uint8 ip_addr[4]; uint16 port; } stream; uint8 buffer[6]; } u;

CMD_TAG16H5 = ord("1")
CMD_TAG25H9 = ord("2")
CMD_TAG36H11 = ord("3")
CMD_STREAM = ord("A")
CMD_PAYLOAD_LEN = 7

serial_read_buffer = b""


def build_write_message(payload: bytes) -> str:
    b64 = b64encode(payload).decode("ascii")
    nbytes = len(b64) + 4
    len_hex = f"{nbytes:02x}"
    chksum = ord(len_hex[0]) ^ ord(len_hex[1])
    for c in b64:
        chksum ^= ord(c)
    chksum &= 0xFF
    return f"{len_hex}{b64}{chksum:02x}\n"


def parse_host_message(line: bytes) -> bytes | None:
    """Decode D + LL<base64>CC (HostSerial::writeBytes). Leading devid byte is ignored."""
    line = line.strip()
    nbytes = len(line)
    # devid (1) + length (2) + checksum (2); base64 may be empty
    if nbytes < 5:
        return None
    try:
        declared = int(line[1:3], 16)
    except ValueError:
        return None
    if declared != nbytes:
        return None
    chksum = 0
    for i in range(nbytes - 2):
        chksum ^= line[i]
    if (chksum & 0xFF) != int(line[-2:], 16):
        return None
    try:
        return b64decode(line[3 : nbytes - 2], validate=True)
    except Exception:
        return None


def parse_cmd_payload(payload: bytes):
    if len(payload) != CMD_PAYLOAD_LEN:
        print(f"Invalid command payload length: {len(payload)}")
        return None
    cmd = payload[0]
    if cmd == CMD_STREAM:
        ip = str(ipaddress.IPv4Address(int.from_bytes(payload[1:5], "little")))
        port = int.from_bytes(payload[5:7], "little")
        print(f"Stream command received: {ip}:{port}")
        return (CMD_STREAM, ip, port)
    if cmd == CMD_TAG16H5:
        print("Tag16H5 command received")
        return (CMD_TAG16H5,)
    if cmd == CMD_TAG25H9:
        print("Tag25H9 command received")
        return (CMD_TAG25H9,)
    if cmd == CMD_TAG36H11:
        print("Tag36H11 command received")
        return (CMD_TAG36H11,)
    print(f"Invalid command byte: {cmd:#x}")
    return None


def on_serial_data(data: bytes):
    global serial_read_buffer
    serial_read_buffer += data
    result = None
    while b"\n" in serial_read_buffer:
        line, serial_read_buffer = serial_read_buffer.split(b"\n", 1)
        payload = parse_host_message(line)
        if payload is None:
            continue
        parsed = parse_cmd_payload(payload)
        if parsed is not None:
            result = parsed
    return result

def start_camera():
  # Initialize the camera
  camera = picamera2.Picamera2()
  config = camera.create_video_configuration(main={"format": "YUV420", "size": (1280, 720)})
  camera.configure(config)
  camera.start()
  return camera

def atag_detect(camera: picamera2.Picamera2, detector: apriltag.apriltag, ser: serial.Serial):
  # Capture YUV frame
  req = camera.capture_request()
  y = req.make_array("main")
  req.release()
  origin = np.array([640, 360], np.float32)

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

if __name__ == "__main__":
  # Initialize the serial port
  serial_port = os.environ.get("SERIAL_PORT", "/dev/ttyUSB0")
  ser = serial.Serial(serial_port, 115200, timeout=0.1)
  if not ser.isOpen():
    raise Exception("Failed to open serial port")
  ser.flushInput()
  ser.flushOutput()
  ser.reset_input_buffer()
  ser.reset_output_buffer()
  process = None
  # by default, we will start in atags mode
  camera = start_camera()
  detector = apriltag.apriltag("tag16h5", threads=4, decimate=2.0, refine_edges=1)
  cmd = (CMD_TAG16H5,)

  try:
    while True:
      # we only need to do something if we are in atags mode
      if cmd[0] in (CMD_TAG16H5, CMD_TAG25H9, CMD_TAG36H11):
        atag_detect(camera, detector, ser)
      else:
        time.sleep(0.1)

      # get the next command from the serial port
      if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        request = on_serial_data(data)
        if request is not None:
          if request[0] == CMD_STREAM:
            if camera is not None:
              camera.stop()
              camera = None
            if process is not None:
              camera_stream.stop_camera_stream(process)
            process = camera_stream.run_camera_stream(
              request[1], request[2], 1280, 720, 30, 4_000_000
            )
          elif request[0] == CMD_TAG16H5:
            if process is not None:
              camera_stream.stop_camera_stream(process)
              process = None
            if cmd[0] != CMD_TAG16H5:
              detector = apriltag.apriltag("tag16h5", threads=4, decimate=2.0, refine_edges=1)
            if camera is None:
              camera = start_camera()
          elif request[0] == CMD_TAG25H9:
            if process is not None:
              camera_stream.stop_camera_stream(process)
              process = None
            if cmd[0] != CMD_TAG25H9:
              detector = apriltag.apriltag("tag25h9", threads=4, decimate=2.0, refine_edges=1)
            if camera is None:
              camera = start_camera()
          elif request[0] == CMD_TAG36H11:
            if process is not None:
              camera_stream.stop_camera_stream(process)
              process = None
            if cmd[0] != CMD_TAG36H11:
              detector = apriltag.apriltag("tag36h11", threads=4, decimate=2.0, refine_edges=1)
            if camera is None:
              camera = start_camera()
          else:
            request = None
        if request is not None:
          cmd = request

  except KeyboardInterrupt:
    print("Keyboard interrupt received, exiting...")
  finally:
    if process is not None:
      camera_stream.stop_camera_stream(process)
      process = None
    if camera is not None:
      camera.stop()
      camera = None
    if ser is not None:
      ser.close()