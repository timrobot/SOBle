import os
import time
import numpy as np
import picamera2
import apriltag
from libcamera import controls
import serial
from base64 import b64decode, b64encode


# Host → Pi commands: D + LL<base64>CC (HostSerial::writeBytes; D = devid, ignored on Pi).
# 7-byte payloads: uint8 cmd (+ padding). Tag family cmds: 1/2/3.

CMD_TAG16H5 = ord("1")
CMD_TAG25H9 = ord("2")
CMD_TAG36H11 = ord("3")
CMD_PAYLOAD_LEN = 7


serial_read_buffer = b""

def pack_status_byte(ntags: int) -> bytes:
  return bytes([min(max(ntags, 0), 10) & 0x1F])

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
    if len(payload) < 1:
        print("Invalid command payload: empty")
        return None
    cmd = payload[0]
    if len(payload) != CMD_PAYLOAD_LEN:
        print(f"Invalid command payload length: {len(payload)}")
        return None
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


def poll_serial(ser: serial.Serial):
  global serial_read_buffer
  if ser.in_waiting <= 0:
    return None
  serial_read_buffer += ser.read(ser.in_waiting)
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


def handle_request(request, *, camera, detector, cmd):
  """Apply one host command. Returns (camera, detector, cmd)."""
  if request[0] not in (CMD_TAG16H5, CMD_TAG25H9, CMD_TAG36H11):
    return camera, detector, cmd

  tag_family = {
    CMD_TAG16H5: "TAG16H5",
    CMD_TAG25H9: "TAG25H9",
    CMD_TAG36H11: "TAG36H11",
  }.get(request[0], "UNKNOWN")
  print(f"[TAGS] Switching to {tag_family} mode...", flush=True)

  if request[0] == CMD_TAG16H5 and cmd[0] != CMD_TAG16H5:
    detector = apriltag.apriltag("tag16h5", threads=4, decimate=2.0, refine_edges=1)
  elif request[0] == CMD_TAG25H9 and cmd[0] != CMD_TAG25H9:
    detector = apriltag.apriltag("tag25h9", threads=4, decimate=2.0, refine_edges=1)
  elif request[0] == CMD_TAG36H11 and cmd[0] != CMD_TAG36H11:
    detector = apriltag.apriltag("tag36h11", threads=4, decimate=2.0, refine_edges=1)

  if camera is None:
    camera = start_camera()

  return camera, detector, request


def release_camera(camera: picamera2.Picamera2 | None) -> None:
  """Stop and close picamera2."""
  if camera is None:
    return
  try:
    camera.stop()
  except Exception:
    pass
  try:
    camera.close()
  except Exception:
    pass


def start_camera():
  # Initialize the camera (imx708: continuous AF)
  camera = picamera2.Picamera2()
  config = camera.create_video_configuration(
    main={"format": "YUV420", "size": (1280, 720)},
    controls={"AfMode": controls.AfModeEnum.Continuous},
  )
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
  detections = list(detector.detect(y))

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
    message = pack_status_byte(ntags) + np.concatenate(tag_data, axis=0).tobytes()
  else:
    message = pack_status_byte(0)
  ser.write(build_write_message(message).encode("ascii"))
  ser.flush()

if __name__ == "__main__":
  # Initialize the serial port
  serial_port = os.environ.get("SERIAL_PORT", "/dev/ttyACM0")
  ser = serial.Serial(serial_port, 115200, timeout=0.1)
  if not ser.isOpen():
    raise Exception("Failed to open serial port")
  ser.flushInput()
  ser.flushOutput()
  ser.reset_input_buffer()
  ser.reset_output_buffer()

  camera = start_camera()
  detector = apriltag.apriltag("tag16h5", threads=4, decimate=2.0, refine_edges=1)
  cmd = (CMD_TAG16H5,)

  try:
    while True:
      request = poll_serial(ser)
      if request is not None:
        camera, detector, cmd = handle_request(
          request,
          camera=camera,
          detector=detector,
          cmd=cmd,
        )

      atag_detect(camera, detector, ser)

  except KeyboardInterrupt:
    print("Keyboard interrupt received, exiting...")
  finally:
    if camera is not None:
      release_camera(camera)
      camera = None
    if ser is not None:
      ser.close()
