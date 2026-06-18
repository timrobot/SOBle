// BLE robot control (So101-Platform protocol), STS bus, QMI8658 IMU, ST7789 LCD

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>
#include <string.h>

#include "STSDriver.h"
#include "HostSerial.h"
#include "WS_QMI8658.h"
#include "Madgwick.h"

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
// ESP32 RX ← STS TX, ESP32 TX → STS RX
#define ARM_RX 1   // ← STS TX
#define ARM_TX 2   // → STS RX
#define ARM_BAUD 1000000

// BLE GATT
static const char *kName = "Capybara";
static const char *kService = "4fafc201-1fb5-459e-8fcc-c5c9c331d914";
static const char *kChar = "beb5483e-36e1-4688-b7f2-e6a6a6d74324";

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

static constexpr int16_t kDispStatusPiY = 100;
static constexpr int16_t kDispStatusWifiY = 124;
static constexpr int16_t kDispStatusLineH = 20;

STSDriver sts(2, ARM_RX, ARM_TX, ARM_BAUD);
static bool stsHaltSent = false;
static bool wheelVelocityPrimed = false;
static uint8_t s_armTorqueMask = 0; // which arm joints currently have torque enabled
static uint16_t gArmRawPos[STSDriver::kArmJointCount];
static int16_t gServoReadPos[STSDriver::kMaxServos];
static uint16_t gWheelRawPos[2] = {0, 0};

static constexpr float MADGWICK_BETA = 0.06f;
Madgwick imuFilter(MADGWICK_BETA);
static uint32_t imuMicrosPrev = 0;

struct CbServer : BLEServerCallbacks {
  void onConnect(BLEServer *) { bleClientConnected = true; }

  void onDisconnect(BLEServer *) {
    bleClientConnected = false;
    BLEDevice::startAdvertising();
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
        targets.left = (int8_t)constrain((int)targets.left, -125, 125);
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

static void packArm12(const uint16_t in[STSDriver::kArmJointCount], uint8_t packed[9]) {
  memset(packed, 0, 9);
  int byteidx = 0;
  bool insert2 = true;
  for (int i = 0; i < STSDriver::kArmJointCount; i++) {
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

static void unpackArm12(const uint8_t packed[9], uint16_t out[STSDriver::kArmJointCount]) {
  int byteidx = 0;
  bool extract2 = true;
  uint16_t a, b = 0;
  for (int i = 0; i < STSDriver::kArmJointCount; i++) {
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

static inline int16_t quatUnitToMilli(float q) {
  return constrain((long)lroundf(q * 1000.0f), -32768, 32767);
}

static void driveArmJointsFromTargets() {
  unpackArm12(targets.arm, gArmRawPos);

  const uint8_t wantMask = targets.enabled & kArmEnableMask;
  const uint8_t engageMask = (uint8_t)(wantMask & ~s_armTorqueMask);

  uint8_t releaseIds[STSDriver::kArmJointCount];
  uint8_t engageIds[STSDriver::kArmJointCount];
  uint8_t driveIds[STSDriver::kArmJointCount];
  int16_t drivePos[STSDriver::kArmJointCount];
  uint8_t nRelease = 0;
  uint8_t nEngage = 0;
  uint8_t nDrive = 0;

  for (uint8_t i = 0; i < STSDriver::kArmJointCount; i++) {
    const uint8_t bit = (uint8_t)(1u << i);
    const uint8_t id = (uint8_t)(i + 1);
    if (engageMask & bit) {
      engageIds[nEngage++] = id;
    }
    if (wantMask & bit) {
      driveIds[nDrive] = id;
      drivePos[nDrive] = (int16_t)gArmRawPos[i];
      nDrive++;
    } else {
      releaseIds[nRelease++] = id;
    }
  }

  if (nRelease > 0) {
    sts.releaseTorque(releaseIds, nRelease);
  }
  if (nEngage > 0) {
    sts.engageTorque(engageIds, nEngage);
  }
  if (nDrive > 0) {
    sts.setAngles(driveIds, nDrive, drivePos);
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
  if (!wheelVelocityPrimed && (targets.left != 0 || targets.right != 0)) {
    // setup() may run before servo power is up; re-enter velocity mode on first drive.
    sts.enableVelocityMode({7, 8});
    wheelVelocityPrimed = true;
  }
  const int16_t wheelSpeed[2] = {wheelSpeedToSts(targets.left), wheelSpeedToSts(targets.right)};
  sts.setSpeed({7, 8}, wheelSpeed);
}

// prevBleRxMs is updated in BLE onWrite (async); use signed delta so millis() wrap
// or a stale snapshot cannot make bleActive false while the host is still streaming.
static bool isBleActive(unsigned long now) {
  if (prevBleRxMs == 0) {
    return false;
  }
  return (long)(now - prevBleRxMs) < BLE_RX_TIMEOUT;
}

static void updateIMU() {
  float ax, ay, az, gx, gy, gz, tempC;
  if (!QMI8658_read(ax, ay, az, gx, gy, gz, tempC)) {
    return;
  }

  const uint32_t nowUs = micros();
  float dt = (nowUs - imuMicrosPrev) * 1e-6f;
  imuMicrosPrev = nowUs;
  if (dt <= 0.f || dt > 0.2f) {
    return;
  }

  imuFilter.updateIMU(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az, dt);
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

static void updateStatusDisplayIfChanged() {
  if (raspi_alive != disp_raspi_alive) {
    disp_raspi_alive = raspi_alive;
    drawStatusLine(kDispStatusPiY, "Pi", raspi_alive != 0);
  }
  if (wifi_connected != disp_wifi_connected) {
    disp_wifi_connected = wifi_connected;
    drawStatusLine(kDispStatusWifiY, "WiFi", wifi_connected != 0);
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
  gfx->println(kName);
}

void setup() {
  gfx->begin(40000000);
  initDisplay();
  updateStatusDisplayIfChanged();

  hostSerial.begin(RASPI_BAUD, RASPI_RX, RASPI_TX);

  memset(&targets, 0, sizeof(targets));
  memset(&st, 0, sizeof(st));

  QMI8658_Init();
  imuMicrosPrev = micros();
  st.quat[0] = 1000;
  st.quat[1] = st.quat[2] = st.quat[3] = 0;

  BLEDevice::init(kName);
  BLEDevice::setMTU(512);

  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(&cbServer);

  BLEService *svc = srv->createService(kService);
  bleCh = svc->createCharacteristic(
      kChar, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
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
  sts.enableVelocityMode({7, 8});
}

void loop() {
  updateIMU();
  message_t *msg = hostSerial.readMessage();
  if (msg == STIMEOUT) {
    st.ntags = 0;
    memset(st.tags, 0, 10 * sizeof(AprilTagInfo));
    raspi_alive = 0;
    wifi_connected = 0;
  } else if (msg != nullptr && msg->length >= 1) {
    raspi_alive = 1;
    wifi_connected = (msg->data[0] & 0x80) >> 7; // first bit is wifi_connected status
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
  updateStatusDisplayIfChanged();

  const unsigned long currentMs = millis();
  const bool bleActive = isBleActive(currentMs);
  if (bleActive) {
    stsHaltSent = false;
    if (currentMs - prevSTSWriteMs >= STS_TX_INTERVAL) {
      prevSTSWriteMs = currentMs;
      driveArmJointsFromTargets();
      driveWheelVelocityFromTargets();
    }
  } else if (prevBleRxMs != 0 && !stsHaltSent) {
    sts.halt({1, 2, 3, 4, 5, 6, 7, 8});
    if (s_armTorqueMask != 0) {
      const uint8_t allArmIds[STSDriver::kArmJointCount] = {1, 2, 3, 4, 5, 6};
      sts.releaseTorque(allArmIds, STSDriver::kArmJointCount);
      s_armTorqueMask = 0;
    }
    stsHaltSent = true;
  }

  if (currentMs - prevSTSReadMs >= STS_RX_INTERVAL) {
    prevSTSReadMs = currentMs;
    if (sts.readAngles({1, 2, 3, 4, 5, 6, 7, 8}, gServoReadPos)) {
      for (uint8_t i = 0; i < STSDriver::kArmJointCount; i++) {
        gArmRawPos[i] = (uint16_t)(gServoReadPos[i] & 0x0FFF);
      }
      packArm12(gArmRawPos, st.armPos);

      gWheelRawPos[0] = (uint16_t)(gServoReadPos[6] & 0x0FFF); // STS 7 = left
      gWheelRawPos[1] = (uint16_t)(gServoReadPos[7] & 0x0FFF); // STS 8 = right
      packWheelEnc12(gWheelRawPos[0], gWheelRawPos[1], st.wheelEnc);
    }
  }

  const unsigned long notifyNow = millis();
  if (bleClientConnected && notifyNow - prevBleTxMs >= BLE_TX_INTERVAL) {
    prevBleTxMs = notifyNow;
    st.quat[0] = quatUnitToMilli(imuFilter.q0());
    st.quat[1] = quatUnitToMilli(imuFilter.q1());
    st.quat[2] = quatUnitToMilli(imuFilter.q2());
    st.quat[3] = quatUnitToMilli(imuFilter.q3());
    bleCh->setValue((uint8_t *)&st, sizeof(st));
    bleCh->notify();
  }
}
