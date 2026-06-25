import os
import ipaddress
import subprocess
import time
import numpy as np
import picamera2
import apriltag
from libcamera import controls
import serial
from base64 import b64decode, b64encode
import camera_stream


# Host → Pi commands: D + LL<base64>CC (HostSerial::writeBytes; D = devid, ignored on Pi).
# 7-byte payloads: uint8 cmd + union (ip[4] + port for stream; tag family for 1/2/3).

CMD_TAG16H5 = ord("1")
CMD_TAG25H9 = ord("2")
CMD_TAG36H11 = ord("3")
CMD_STREAM = ord("A")
CMD_PAYLOAD_LEN = 7


serial_read_buffer = b""

def pack_status_byte(ntags: int, wifi_connected: bool) -> bytes:
  value = min(max(ntags, 0), 10) & 0x1F
  if wifi_connected:
    value |= 0x80
  return bytes([value])

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
    if cmd == CMD_STREAM:
        ip = str(ipaddress.IPv4Address(int.from_bytes(payload[1:5], "little")))
        port = int.from_bytes(payload[5:7], "little")
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


def handle_request(request, *, camera, detector, cmd, process, wifi_connected):
  """Apply one host command. Returns (camera, detector, cmd, process, wifi_connected)."""
  if request[0] == CMD_STREAM:
    release_camera(camera)
    camera = None
    wifi_connected = get_wifi_connected()
    if not wifi_connected:
      print("WiFi not connected, cannot start camera stream", flush=True)
      return camera, detector, cmd, process, wifi_connected
    if process is not None:
      camera_stream.stop_camera_stream(process)
      process = None
    time.sleep(0.5)  # libcamera needs a moment after picamera2 releases the sensor
    process = camera_stream.run_camera_stream(
      request[1], request[2], 1280, 720, 30, 4_000_000
    )
  elif request[0] == CMD_TAG16H5:
    if process is not None:
      camera_stream.stop_camera_stream(process)
      process = None
      time.sleep(0.5)
    if cmd[0] != CMD_TAG16H5:
      detector = apriltag.apriltag("tag16h5", threads=4, decimate=2.0, refine_edges=1)
    if camera is None:
      camera = start_camera()
  elif request[0] == CMD_TAG25H9:
    if process is not None:
      camera_stream.stop_camera_stream(process)
      process = None
      time.sleep(0.5)
    if cmd[0] != CMD_TAG25H9:
      detector = apriltag.apriltag("tag25h9", threads=4, decimate=2.0, refine_edges=1)
    if camera is None:
      camera = start_camera()
  elif request[0] == CMD_TAG36H11:
    if process is not None:
      camera_stream.stop_camera_stream(process)
      process = None
      time.sleep(0.5)
    if cmd[0] != CMD_TAG36H11:
      detector = apriltag.apriltag("tag36h11", threads=4, decimate=2.0, refine_edges=1)
    if camera is None:
      camera = start_camera()
  if request[0] in (CMD_STREAM, CMD_TAG16H5, CMD_TAG25H9, CMD_TAG36H11):
    cmd = request
  return camera, detector, cmd, process, wifi_connected


def release_camera(camera: picamera2.Picamera2 | None) -> None:
  """Stop and close picamera2 so libcamerasrc can open the sensor."""
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
  # Initialize the camera (imx708: continuous AF matches libcamerasrc af-mode=continuous)
  camera = picamera2.Picamera2()
  config = camera.create_video_configuration(
    main={"format": "YUV420", "size": (1280, 720)},
    controls={"AfMode": controls.AfModeEnum.Continuous},
  )
  camera.configure(config)
  camera.start()
  return camera

def atag_detect(camera: picamera2.Picamera2, detector: apriltag.apriltag, ser: serial.Serial, wifi_connected: bool):
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
    message = pack_status_byte(ntags, wifi_connected) + np.concatenate(tag_data, axis=0).tobytes()
  else:
    message = pack_status_byte(0, wifi_connected)
  ser.write(build_write_message(message).encode("ascii"))
  ser.flush()

def get_wifi_connected() -> bool:
  # query whether or not wifi is already up and running
  state = subprocess.run(
    ["nmcli", "-t", "-f", "STATE", "general"],
    capture_output=True,
    text=True,
    timeout=10,
    check=False,
  )
  if "connected" in (state.stdout or "").lower():
    return True
  return False

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
  process = None
  # by default, we will start in atags mode
  camera = start_camera()
  detector = apriltag.apriltag("tag16h5", threads=4, decimate=2.0, refine_edges=1)
  cmd = (CMD_TAG16H5,)
  wifi_connected = get_wifi_connected()
  last_serial_ms = time.time()
  last_wifi_check_ms = time.time()

  try:
    while True:
      if time.time() - last_wifi_check_ms > 1.0:
        last_wifi_check_ms = time.time()
        wifi_connected = get_wifi_connected()

      request = poll_serial(ser)
      if request is not None:
        camera, detector, cmd, process, wifi_connected = handle_request(
          request,
          camera=camera,
          detector=detector,
          cmd=cmd,
          process=process,
          wifi_connected=wifi_connected,
        )

      # we only need to do something if we are in atags mode
      if cmd[0] in (CMD_TAG16H5, CMD_TAG25H9, CMD_TAG36H11):
        atag_detect(camera, detector, ser, wifi_connected)
        last_serial_ms = time.time()
      else:
        # send a heartbeat to the ESP32 to keep the connection alive if in streaming mode
        if time.time() - last_serial_ms > 0.1: # send every 100ms so that ESP32 doesn't timeout
          last_serial_ms = time.time()
          if wifi_connected:
            ser.write(build_write_message(pack_status_byte(0, True)).encode("ascii"))
            ser.flush()
          else:
            ser.write(build_write_message(pack_status_byte(0, False)).encode("ascii"))
            ser.flush()

      request = poll_serial(ser)
      if request is not None:
        camera, detector, cmd, process, wifi_connected = handle_request(
          request,
          camera=camera,
          detector=detector,
          cmd=cmd,
          process=process,
          wifi_connected=wifi_connected,
        )

  except KeyboardInterrupt:
    print("Keyboard interrupt received, exiting...")
  finally:
    if process is not None:
      camera_stream.stop_camera_stream(process)
      process = None
    if camera is not None:
      release_camera(camera)
      camera = None
    if ser is not None:
      ser.close()