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
# WiFi payload: 'W' + user_len + pass_len + user bytes + pass bytes (no NULs).

CMD_TAG16H5 = ord("1")
CMD_TAG25H9 = ord("2")
CMD_TAG36H11 = ord("3")
CMD_STREAM = ord("A")
CMD_WIFI = ord("W")
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
    if len(payload) < 1:
        print("Invalid command payload: empty")
        return None
    cmd = payload[0]
    if cmd == CMD_WIFI:
        if len(payload) < 3:
            print(f"Invalid WiFi command payload length: {len(payload)}")
            return None
        user_len = payload[1]
        pass_len = payload[2]
        need = 3 + user_len + pass_len
        if len(payload) != need:
            print(
                f"Invalid WiFi command payload length: {len(payload)} "
                f"(expected {need} for user_len={user_len} pass_len={pass_len})"
            )
            return None
        user = payload[3 : 3 + user_len].decode("utf-8", errors="replace")
        password = payload[3 + user_len : need].decode("utf-8", errors="replace")
        return (CMD_WIFI, user, password)
    if len(payload) != CMD_PAYLOAD_LEN:
        print(f"Invalid command payload length: {len(payload)}")
        return None
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
  # Initialize the camera (imx708: continuous AF matches libcamerasrc af-mode=continuous)
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
    message = ntags.to_bytes(1, "little") + np.concatenate(tag_data, axis=0).tobytes()
  else:
    message = b"\x00"
  ser.write(build_write_message(message).encode("ascii"))
  ser.flush()

def connect_wifi(ssid: str, password: str) -> bool:
  """Connect wlan via NetworkManager (nmcli). Returns True if connected."""
  if not ssid or not password:
    print("WiFi connect skipped: SSID and password are required")
    return False

  print(f"Connecting to WiFi SSID {ssid!r}...")
  subprocess.run(
    ["sudo", "nmcli", "device", "wifi", "rescan"],
    capture_output=True,
    text=True,
    timeout=30,
    check=False,
  )
  time.sleep(2)

  cmd = [
    "sudo",
    "nmcli",
    "device",
    "wifi",
    "connect",
    ssid,
    "password",
    password,
  ]

  try:
    result = subprocess.run(
      cmd,
      capture_output=True,
      text=True,
      timeout=90,
      check=False,
    )
  except subprocess.TimeoutExpired:
    print("WiFi connect timed out")
    return False

  if result.returncode != 0:
    err = (result.stderr or result.stdout or "").strip()
    print(f"WiFi connect failed ({result.returncode}): {err}")
    return False

  deadline = time.time() + 30
  while time.time() < deadline:
    state = subprocess.run(
      ["nmcli", "-t", "-f", "STATE", "general"],
      capture_output=True,
      text=True,
      timeout=10,
      check=False,
    )
    if "connected" in (state.stdout or "").lower():
      print(f"WiFi connected ({ssid!r})")
      return True
    time.sleep(1)

  print(f"WiFi connect command ok but state not connected ({ssid!r})")
  return False

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
  wifi_connected = False
  last_serial_ms = time.time()

  try:
    while True:
      # we only need to do something if we are in atags mode
      if cmd[0] in (CMD_TAG16H5, CMD_TAG25H9, CMD_TAG36H11):
        atag_detect(camera, detector, ser)
      else:
        time.sleep(0.05)

      # get the next command from the serial port
      if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        request = on_serial_data(data)
        if request is not None:
          if request[0] == CMD_WIFI:
            wifi_connected = connect_wifi(request[1], request[2])
            if wifi_connected:
              last_serial_ms = time.time()
              ser.write(build_write_message(b"\xFF").encode("ascii")) # arbitrary number to indicate wifi connection
              ser.flush()
          elif request[0] == CMD_STREAM:
            if camera is not None:
              camera.stop()
              camera = None
            if not wifi_connected:
              wifi_connected = get_wifi_connected()
              if not wifi_connected:
                print("WiFi not connected, cannot start camera stream")
                continue
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
          if request[0] in (CMD_STREAM, CMD_TAG16H5, CMD_TAG25H9, CMD_TAG36H11):
            cmd = request

      if wifi_connected and time.time() - last_serial_ms > 1.0:
        last_serial_ms = time.time()
        ser.write(build_write_message(b"\xFF").encode("ascii")) # arbitrary number to indicate wifi connection
        ser.flush()

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