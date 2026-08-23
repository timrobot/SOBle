// BLE robot control (So101-Platform protocol), STS bus, QMI8658 IMU, ST7789 LCD

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>
#include <string.h>
#include <vector>

#include "STSDriver.h"
#include "HostSerial.h"
#include "WS_QMI8658.h"
#include "Madgwick.h"
#include "IMUCircleQueue.h"
#include "QuaternionUtil.h"

// Waveshare ESP32-S3-LCD-1.3 display + IMU
Arduino_DataBus *bus = new Arduino_ESP32SPI(38 /* DC */, 39 /* CS */, 40 /* SCK */, 41 /* MOSI */,
                                              GFX_NOT_DEFINED /* MISO */, FSPI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, 42 /* RST */, 1 /* rotation: 90° CW */, true /* IPS */, 240,
                                        240, 0, 0, 0, 0);

// Pi USB (Type-C → CH343P USB-UART bridge → ESP32 UART0). Not USB CDC Serial.
#define RASPI_RX 44  // U0RXD ← CH343 TX
#define RASPI_TX 43  // U0TXD → CH343 RX
#define RASPI_BAUD 115200

// STS serial bus on the 16-pin header (see Waveshare schematic GPIO_OUT / H2+H1).
// ESP32 TX → STS RX, ESP32 RX ← STS TX
#define ARM_TX 1
#define ARM_RX 2
#define ARM_BAUD 1000000

// BLE GATT
static const char *kBleAdj[] = {"Soft", "Tiny", "Wee", "Warm", "Chub", "Calm", "Big"};
static const char *kBleNoun[] = {"Capybara", "Pika", "Panda", "Seal", "Koala", "Hedgehog"};
static char bleName[32];
static const char *kService = "4fafc201-1fb5-459e-8fcc-c5c9c331d914";
static const char *kChar = "beb5483e-36e1-4688-b7f2-e6a6a6d74324";

static void chooseBleName() {
  const size_t nAdj = sizeof(kBleAdj) / sizeof(kBleAdj[0]);
  const size_t nNoun = sizeof(kBleNoun) / sizeof(kBleNoun[0]);
  const char *adj = kBleAdj[esp_random() % nAdj];
  const char *noun = kBleNoun[esp_random() % nNoun];
  snprintf(bleName, sizeof(bleName), "%s%s", adj, noun);
}

BLECharacteristic *bleCh;
static bool bleClientConnected = false;

unsigned long prevBleTxMs = 0;
unsigned long prevBleRxMs = 0;
unsigned long prevSTSWriteMs = 0;
unsigned long prevSTSReadMs = 0;
const long STS_TX_INTERVAL = 10;
const long STS_RX_INTERVAL = STS_TX_INTERVAL;
const long BLE_RX_TIMEOUT = 250;
const long BLE_TX_INTERVAL = 40;

#pragma pack(push, 1)
struct RobotCommand {
  uint8_t cmd;
  int8_t left;
  int8_t right;
  uint8_t arm[9];
  uint8_t enabled; // bit0=J1, bit1=J2, ...
};

struct AprilTagInfo {
  uint16_t tag_id;
  int16_t tag_corners[8];
};

struct RobotState {
  uint8_t wheelEnc[3];
  uint8_t armPos[9];
  int16_t quat[4];
  uint8_t ntags;
  AprilTagInfo tags[10];
};
#pragma pack(pop)

static_assert(sizeof(RobotCommand) == 13, "RobotCommand layout must match BLE host");
static constexpr uint8_t kArmEnableMask = 0x3F; // bit0=J1 .. bit5=J6

static constexpr uint8_t CMD_ACTUATORS = '0';
static constexpr uint8_t CMD_TAG16H5 = '1';
static constexpr uint8_t CMD_TAG25H9 = '2';
static constexpr uint8_t CMD_TAG36H11 = '3';
static constexpr uint8_t CMD_STREAM = 'A';
static constexpr size_t RASPI_CMD_PAYLOAD_LEN = 7;

RobotCommand targets;
RobotState st;

HardwareSerial RaspiSerial(0);
HostSerial hostSerial(RaspiSerial, 1);
static uint8_t raspi_alive = 0;
static uint8_t wifi_connected = 0;
static uint8_t disp_raspi_alive = 0xFF;
static uint8_t disp_wifi_connected = 0xFF;
static uint8_t disp_sts_online_mask = 0xFF;

static constexpr int16_t kDispStatusPiY = 100;
static constexpr int16_t kDispStatusWifiY = 124;
static constexpr int16_t kDispStatusStsY = 148;
static constexpr int16_t kDispStatusLineH = 20;

STSDriver sts(2, ARM_RX, ARM_TX, ARM_BAUD);
static bool stsHaltSent = false;
static bool wheelVelocityPrimed = false;
static uint8_t s_armTorqueMask = 0; // which arm joints currently have torque enabled

/** Sketch-level STS topology (may change at runtime). */
static constexpr uint8_t kMaxServos = 8;
static constexpr uint8_t kArmJointCount = 6;
static constexpr uint8_t kWheelCount = 2;
static std::vector<uint8_t> gAllServoIds = {1, 2, 3, 4, 5, 6, 7, 8};
static std::vector<uint8_t> gArmIds = {1, 2, 3, 4, 5, 6};
static std::vector<uint8_t> gWheelIds = {7, 8};
/** IDs that currently answer SYNC_READ (subset of gAllServoIds). */
static std::vector<uint8_t> gOnlineIds;
/** bit0 = ID1 … bit7 = ID8 */
static uint8_t gOnlineMask = 0;

static uint16_t gArmRawPos[kArmJointCount];
static uint16_t gWheelRawPos[kWheelCount] = {0, 0};

static constexpr float MADGWICK_BETA = 0.06f;
Madgwick imuFilter(MADGWICK_BETA);
static uint32_t imuMicrosPrev = 0;

static IMUCircleQueue imuCal;
static float gQuat0[4] = {1.f, 0.f, 0.f, 0.f};

struct CbServer : BLEServerCallbacks {
  void onConnect(BLEServer *) {
    bleClientConnected = true;
    prevBleRxMs = 0;
  }

  void onDisconnect(BLEServer *pServer) {
    bleClientConnected = false;
    prevBleRxMs = 0;
    stsHaltSent = false;
    BLEDevice::startAdvertising();
    if (pServer != nullptr && pServer->getAdvertising() != nullptr) {
      pServer->getAdvertising()->start();
    }
  }
};

struct CbChar : BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    const size_t n = c->getLength();
    if (n == 0) {
      return;
    }
    const uint8_t *buf = c->getData();

    switch (buf[0]) {
      case CMD_ACTUATORS:
        if (n >= sizeof(RobotCommand)) {
          memcpy(&targets, buf, sizeof(RobotCommand));
        } else if (n >= 3) {
          targets.cmd = CMD_ACTUATORS;
          targets.left = (int8_t)buf[1];
          targets.right = (int8_t)buf[2];
        } else {
          break;
        }
        // Host teleop is symmetric; left wheel STS mount is reversed vs right.
        targets.left = (int8_t)constrain(-(int)targets.left, -125, 125);
        targets.right = (int8_t)constrain((int)targets.right, -125, 125);
        prevBleRxMs = millis();
        break;

      case CMD_TAG16H5:
      case CMD_TAG25H9:
      case CMD_TAG36H11:
      case CMD_STREAM:
        if (n >= RASPI_CMD_PAYLOAD_LEN) {
          hostSerial.writeBytes((void *)buf, RASPI_CMD_PAYLOAD_LEN);
        }
        break;

      default:
        break;
    }
  }
};

static CbServer cbServer;
static CbChar cbChar;

static void rebuildOnlineIdsFromMask() {
  gOnlineIds.clear();
  for (uint8_t id : gAllServoIds) {
    if (id >= 1 && id <= kMaxServos && (gOnlineMask & (1u << (id - 1)))) {
      gOnlineIds.push_back(id);
    }
  }
}

static bool isServoOnline(uint8_t id) {
  return id >= 1 && id <= kMaxServos && (gOnlineMask & (1u << (id - 1))) != 0;
}

static void setServoOnline(uint8_t id, bool online) {
  if (id < 1 || id > kMaxServos) {
    return;
  }
  const uint8_t bit = (uint8_t)(1u << (id - 1));
  if (online) {
    gOnlineMask = (uint8_t)(gOnlineMask | bit);
  } else {
    gOnlineMask = (uint8_t)(gOnlineMask & ~bit);
  }
}

static std::vector<uint8_t> filterOnline(const std::vector<uint8_t> &ids) {
  std::vector<uint8_t> out;
  out.reserve(ids.size());
  for (uint8_t id : ids) {
    if (isServoOnline(id)) {
      out.push_back(id);
    }
  }
  return out;
}

static void applyReadPositions(const std::vector<uint8_t> &ids, const std::vector<int16_t> &pos) {
  const size_t n = ids.size() < pos.size() ? ids.size() : pos.size();
  for (size_t i = 0; i < n; i++) {
    const uint8_t id = ids[i];
    const uint16_t raw = (uint16_t)(pos[i] & 0x0FFF);
    if (id >= 1 && id <= kArmJointCount) {
      gArmRawPos[id - 1] = raw;
    } else if (id == 7) {
      gWheelRawPos[0] = raw;
    } else if (id == 8) {
      gWheelRawPos[1] = raw;
    }
  }
}

static void packArm12(const uint16_t *in, uint8_t packed[9]) {
  memset(packed, 0, 9);
  int byteidx = 0;
  bool insert2 = true;
  for (int i = 0; i < kArmJointCount; i++) {
    const uint16_t v = in[i];
    if (insert2) {
      packed[byteidx++] = (uint8_t)(v & 0xFF);
      packed[byteidx] = (uint8_t)((v >> 8) & 0x0F);
    } else {
      packed[byteidx++] = (uint8_t)((v & 0x0F) << 4) | packed[byteidx];
      packed[byteidx++] = (uint8_t)((v >> 4) & 0xFF);
    }
    insert2 = !insert2;
  }
}

static void packWheelEnc12(uint16_t left, uint16_t right, uint8_t out[3]) {
  out[0] = (uint8_t)left;
  out[1] = (uint8_t)(((left >> 8) & 0x0F) | ((right & 0x0F) << 4));
  out[2] = (uint8_t)(right >> 4);
}

/** Refresh online mask + packed encoders from a bus-wide SYNC_READ scan. */
static void refreshServoPresenceAndEncoders() {
  std::vector<uint8_t> foundIds;
  std::vector<int16_t> foundPos;
  gOnlineMask = 0;
  gOnlineIds.clear();
  if (!sts.scanAllServosOnline(gAllServoIds, foundIds, foundPos)) {
    return;
  }
  for (uint8_t id : foundIds) {
    setServoOnline(id, true);
  }
  rebuildOnlineIdsFromMask();
  applyReadPositions(foundIds, foundPos);
  packArm12(gArmRawPos, st.armPos);
  packWheelEnc12(gWheelRawPos[0], gWheelRawPos[1], st.wheelEnc);
}

static void unpackArm12(const uint8_t packed[9], uint16_t *out) {
  int byteidx = 0;
  bool extract2 = true;
  uint16_t a, b = 0;
  for (int i = 0; i < kArmJointCount; i++) {
    a = packed[byteidx++];
    if (extract2) {
      b = packed[byteidx++];
      out[i] = ((b & 0x0F) << 8) | a;
      b >>= 4;
    } else {
      out[i] = (a << 4) | b;
    }
    extract2 = !extract2;
  }
}

static void driveArmJointsFromTargets() {
  unpackArm12(targets.arm, gArmRawPos);

  const uint8_t wantMask = targets.enabled & kArmEnableMask;
  const uint8_t engageMask = (uint8_t)(wantMask & ~s_armTorqueMask);

  std::vector<uint8_t> releaseIds;
  std::vector<uint8_t> engageIds;
  std::vector<uint8_t> driveIds;
  std::vector<int16_t> drivePos;
  releaseIds.reserve(kArmJointCount);
  engageIds.reserve(kArmJointCount);
  driveIds.reserve(kArmJointCount);
  drivePos.reserve(kArmJointCount);

  for (uint8_t i = 0; i < kArmJointCount; i++) {
    const uint8_t bit = (uint8_t)(1u << i);
    const uint8_t id = (uint8_t)(i + 1);
    if (!isServoOnline(id)) {
      continue;
    }
    if (engageMask & bit) {
      engageIds.push_back(id);
    }
    if (wantMask & bit) {
      driveIds.push_back(id);
      drivePos.push_back((int16_t)gArmRawPos[i]);
    } else {
      releaseIds.push_back(id);
    }
  }

  if (!releaseIds.empty()) {
    sts.releaseTorque(releaseIds);
  }
  if (!engageIds.empty()) {
    sts.engageTorque(engageIds);
  }
  if (!driveIds.empty()) {
    sts.setAngles(driveIds, drivePos);
  }

  s_armTorqueMask = wantMask;
}

static int16_t wheelSpeedToSts(int8_t speed) {
  // static constexpr int16_t kStsMaxWheelSpeed = 3400;
  // static constexpr int8_t kWheelCmdScale = 27; // ±125 BLE cmd → ±3375 STS (~±3400 max)
  int16_t s = constrain((int16_t)speed * 27, -3400, 3400);
  if (s >= 0) return s;
  return (int16_t)((uint16_t)((uint16_t)(-s) | 0x8000u));
}

static void driveWheelVelocityFromTargets() {
  const std::vector<uint8_t> onlineWheels = filterOnline(gWheelIds);
  if (onlineWheels.empty()) {
    return;
  }
  if (!wheelVelocityPrimed && (targets.left != 0 || targets.right != 0)) {
    // setup() may run before servo power is up; re-enter velocity mode on first drive.
    sts.enableVelocityMode(onlineWheels);
    wheelVelocityPrimed = true;
  }
  std::vector<int16_t> wheelSpeed;
  wheelSpeed.reserve(onlineWheels.size());
  for (uint8_t id : onlineWheels) {
    if (id == 7) {
      wheelSpeed.push_back(wheelSpeedToSts(targets.left));
    } else if (id == 8) {
      wheelSpeed.push_back(wheelSpeedToSts(targets.right));
    }
  }
  if (wheelSpeed.size() == onlineWheels.size()) {
    sts.setSpeed(onlineWheels, wheelSpeed);
  }
}

static bool isBleActive(unsigned long now) {
  if (prevBleRxMs == 0) {
    return false;
  }
  return now - prevBleRxMs < BLE_RX_TIMEOUT;
}

static bool sampleIMU(float q[4]) {
  float ax, ay, az, gx, gy, gz, tempC;
  if (!QMI8658_read(ax, ay, az, gx, gy, gz, tempC)) {
    return false;
  }

  const uint32_t nowUs = micros();
  float dt = (nowUs - imuMicrosPrev) * 1e-6f;
  imuMicrosPrev = nowUs;
  if (dt <= 0.f || dt > 0.2f) {
    return false;
  }

  imuFilter.updateIMU(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az, dt);
  q[0] = imuFilter.q0();
  q[1] = imuFilter.q1();
  q[2] = imuFilter.q2();
  q[3] = imuFilter.q3();
  return true;
}

static void calibrateIMUZero() {
  imuCal.reset();
  while (!imuCal.isFull()) {
    float q[4];
    if (sampleIMU(q)) {
      imuCal.push(q[0], q[1], q[2], q[3]);
    }
    delay(5);
  }
  if (!imuCal.averageQuat(gQuat0)) {
    gQuat0[0] = 1.f;
    gQuat0[1] = gQuat0[2] = gQuat0[3] = 0.f;
  }
}

static void drawStatusLine(int16_t y, const char *label, bool ok) {
  gfx->fillRect(8, y, 224, kDispStatusLineH, RGB565_BLACK);
  gfx->setTextSize(2);
  gfx->setCursor(8, y);
  if (ok) {
    gfx->setTextColor(RGB565(0, 255, 0), RGB565_BLACK);
    gfx->print(label);
    gfx->println(" online");
  } else {
    gfx->setTextColor(RGB565_DARKGREY, RGB565_BLACK);
    gfx->print(label);
    gfx->println(" offline");
  }
}

static void drawStsStatusLine(int16_t y, uint8_t onlineMask) {
  gfx->fillRect(8, y, 224, kDispStatusLineH * 2, RGB565_BLACK);
  gfx->setTextSize(2);
  gfx->setCursor(8, y);
  const bool any = onlineMask != 0;
  if (any) {
    gfx->setTextColor(RGB565(0, 255, 0), RGB565_BLACK);
    gfx->println("STS found");
  } else {
    gfx->setTextColor(RGB565_DARKGREY, RGB565_BLACK);
    gfx->println("STS missing");
  }

  gfx->setTextSize(1);
  gfx->setCursor(8, y + kDispStatusLineH);
  if (!any) {
    gfx->setTextColor(RGB565_DARKGREY, RGB565_BLACK);
    gfx->print("-");
    return;
  }
  gfx->setTextColor(RGB565(0, 255, 0), RGB565_BLACK);
  bool first = true;
  for (uint8_t id = 1; id <= kMaxServos; id++) {
    if (onlineMask & (1u << (id - 1))) {
      if (!first) {
        gfx->print(' ');
      }
      gfx->print(id);
      first = false;
    }
  }
}

static void updateStatusDisplayIfChanged() {
  if (raspi_alive != disp_raspi_alive) {
    disp_raspi_alive = raspi_alive;
    drawStatusLine(kDispStatusPiY, "Pi", raspi_alive != 0);
  }
  if (wifi_connected != disp_wifi_connected) {
    disp_wifi_connected = wifi_connected;
    drawStatusLine(kDispStatusWifiY, "WiFi", wifi_connected != 0);
  }
  if (gOnlineMask != disp_sts_online_mask) {
    disp_sts_online_mask = gOnlineMask;
    drawStsStatusLine(kDispStatusStsY, gOnlineMask);
  }
}

static void initDisplay() {
  gfx->fillScreen(RGB565_BLACK);

  gfx->setTextColor(RGB565_CYAN, RGB565_BLACK);
  gfx->setTextSize(2);
  gfx->setCursor(8, 8);
  gfx->println("BLE Name");

  gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
  gfx->setTextSize(3);
  gfx->setCursor(8, 32);
  gfx->println(bleName);
}

void setup() {
  chooseBleName();

  gfx->begin(40000000);
  initDisplay();
  updateStatusDisplayIfChanged();

  hostSerial.begin(RASPI_BAUD, RASPI_RX, RASPI_TX);

  memset(&targets, 0, sizeof(targets));
  memset(&st, 0, sizeof(st));

  QMI8658_Init();
  imuMicrosPrev = micros();
  calibrateIMUZero();
  st.quat[0] = 1000;
  st.quat[1] = st.quat[2] = st.quat[3] = 0;

  BLEDevice::init(bleName);
  BLEDevice::setMTU(247);

  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(&cbServer);

  BLEService *svc = srv->createService(kService);
  bleCh = svc->createCharacteristic(kChar,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR |
    BLECharacteristic::PROPERTY_NOTIFY);
  bleCh->addDescriptor(new BLE2902());
  bleCh->setCallbacks(&cbChar);
  bleCh->setValue("ok");

  svc->start();

  BLEAdvertising *a = BLEDevice::getAdvertising();
  a->addServiceUUID(kService);
  a->setScanResponse(true);
  a->setMinPreferred(0x06);
  a->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  sts.begin();
  refreshServoPresenceAndEncoders();
  updateStatusDisplayIfChanged();
  {
    const std::vector<uint8_t> onlineWheels = filterOnline(gWheelIds);
    if (!onlineWheels.empty()) {
      sts.enableVelocityMode(onlineWheels);
      wheelVelocityPrimed = true;
    }
  }
}

void loop() {
  float qNow[4] = {imuFilter.q0(), imuFilter.q1(), imuFilter.q2(), imuFilter.q3()};
  if (sampleIMU(qNow)) {
    imuCal.push(qNow[0], qNow[1], qNow[2], qNow[3]);
  }

  const unsigned long currentMs = millis();
  if (bleClientConnected && currentMs - prevBleTxMs >= BLE_TX_INTERVAL) {
    prevBleTxMs = currentMs;
    float qBle[4];
    if (!imuCal.weightedAverageQuat(qBle)) {
      qBle[0] = qNow[0];
      qBle[1] = qNow[1];
      qBle[2] = qNow[2];
      qBle[3] = qNow[3];
    }
    float qRel[4];
    quat_diff(gQuat0, qBle, qRel);
    st.quat[0] = quat_unit_to_milli(qRel[0]);
    st.quat[1] = quat_unit_to_milli(qRel[1]);
    st.quat[2] = quat_unit_to_milli(qRel[2]);
    st.quat[3] = quat_unit_to_milli(qRel[3]);
    bleCh->setValue((uint8_t *)&st, sizeof(st));
    bleCh->notify();
  }

  message_t *msg = hostSerial.readMessage();
  bool is_ipv4 = false;
  if (msg == STIMEOUT) {
    st.ntags = 0;
    memset(st.tags, 0, 10 * sizeof(AprilTagInfo));
    raspi_alive = 0;
    wifi_connected = 0;
  } else if (msg != nullptr && msg->length >= 1) {
    raspi_alive = 1;
    wifi_connected = (msg->data[0] & 0x80) >> 7; // first bit is wifi_connected status
    is_ipv4 = (msg->data[0] & 0x20) >> 5; // third bit is flag if this is ipv4
    if (is_ipv4) {
      if (msg->length == 5) {
        // we have an ipv4 address
        memcpy(st.tags, &msg->data[1], 4);
      }
      st.ntags = 0xE0; // 11100000
    } else {
      st.ntags = msg->data[0] & 0x1F; // the last 5 bits are the ntags count
      if (st.ntags > 10) {
        st.ntags = 10;
      }
      const size_t need = 1 + (size_t)st.ntags * sizeof(AprilTagInfo);
      if (msg->length >= (int)need && st.ntags > 0) {
        memcpy(st.tags, &msg->data[1], st.ntags * sizeof(AprilTagInfo));
      } else if (st.ntags > 0) {
        st.ntags = 0;
      }
      if (st.ntags < 10) {
        memset(&st.tags[st.ntags], 0, (10 - st.ntags) * sizeof(AprilTagInfo));
      }
      st.ntags = st.ntags | (raspi_alive << 7) | (wifi_connected << 6); // restructure st.ntags for transmission
    }
  }
  updateStatusDisplayIfChanged();

  if (isBleActive(currentMs)) {
    stsHaltSent = false;
    if (currentMs - prevSTSWriteMs >= STS_TX_INTERVAL) {
      prevSTSWriteMs = currentMs;
      driveArmJointsFromTargets();
      driveWheelVelocityFromTargets();
    }
  } else if (prevBleRxMs != 0 && !stsHaltSent) {
    const std::vector<uint8_t> online = filterOnline(gAllServoIds);
    if (!online.empty()) {
      sts.halt(online);
    }
    if (s_armTorqueMask != 0) {
      const std::vector<uint8_t> onlineArms = filterOnline(gArmIds);
      if (!onlineArms.empty()) {
        sts.releaseTorque(onlineArms);
      }
      s_armTorqueMask = 0;
    }
    stsHaltSent = true;
  }

  if (currentMs - prevSTSReadMs >= STS_RX_INTERVAL) {
    prevSTSReadMs = currentMs;
    refreshServoPresenceAndEncoders();
    updateStatusDisplayIfChanged();
  }
}
