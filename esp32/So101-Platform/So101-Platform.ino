#include "So101Arm.h"
#include "HostSerial.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <string.h>
#include <math.h>
#include <U8g2lib.h>
#include "Madgwick.h"

// Pins for ESP32 WROOM
#define WIRE_SCL0 22
#define WIRE_SDA0 21
#define WIRE_SCL1 32
#define WIRE_SDA1 33
#define ARM_RX 17 // connect to SO101 RX
#define ARM_TX 16 // connect to SO101 TX
#define ARM_BAUD 1000000
#define SERVO_LEFT 26
#define SERVO_RIGHT 14
#define AS5600_ADDR 0x36
#define OLED_SCL 13
#define OLED_SDA 5

// Wire0 (21/22): MPU6050 + left AS5600. Wire1 (32/33): right AS5600.

// BLE GATT (central connects by service/characteristic UUID).
static const char *kName = "Capybara"; // Specify unique name like Capybara, Meerkat, Platypus, Hedgehog, Aardvark
static const char *kService = "4fafc201-1fb5-459e-8fcc-c5c9c331d914";
static const char *kChar = "beb5483e-36e1-4688-b7f2-e6a6a6d74324";

BLECharacteristic *bleCh;

unsigned long prevBleTxMs = 0;
unsigned long prevArmWriteMs = 0;
unsigned long prevArmReadMs = 0;
const long SO_TX_INTERVAL = 10; // 100Hz arm SYNC_WRITE
const long SO_RX_INTERVAL = SO_TX_INTERVAL; // 100Hz arm SYNC_READ (independent of writes)
const long BLE_RX_TIMEOUT = 250; // timeout after last BLE command
const long BLE_TX_INTERVAL = 40; // 25Hz robot state notify

static uint32_t lastCommandMs = 0;

#pragma pack(push, 1)
struct RobotCommand {
  int8_t left; // -125 to 125
  int8_t right; // -125 to 125
  uint8_t arm[9]; // STS goal position (u16), each position takes up 12bits - 12*6 = 72bits = 9bytes
};

struct AprilTagInfo {
  uint16_t tag_id;
  int16_t tag_corners[8]; // (corners - origin) * 25
};

struct RobotState {
  uint8_t enc[3]; // encoders are 12 bit - 2 encoders = 24bits = 3bytes
  uint8_t armPos[9]; // each position has a range [0, 4095] = 72bits for 6 = 9bytes
  int16_t quat[4]; // w,x,y,z * 1000 (Madgwick q0..q3); identity [1000,0,0,0]
  uint8_t ntags;
  AprilTagInfo tags[10]; // max 10 tags
};
#pragma pack(pop)

RobotCommand targets;
RobotState st;

HostSerial hostSerial(Serial, 1);
So101Arm arm(2, ARM_RX, ARM_TX, ARM_BAUD);
static bool armHaltSent = false;
static int16_t gJointPos[So101Arm::kMotorCount];
static uint16_t gJointRawPos[So101Arm::kMotorCount];

static uint16_t encRaw[2] = {0, 0};

Servo servoLeft;
Servo servoRight;

// Bit‑bang I2C constructor
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(
  U8G2_R0,        // rotation
  OLED_SCL,       // SCL
  OLED_SDA,       // SDA
  U8X8_PIN_NONE   // reset
);

static inline void updateDisplay() {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 10, "Bluetooth Name:");

  display.setFont(u8g2_font_10x20_tf);
  display.drawStr(0, 40, kName);

  display.sendBuffer();
}

static constexpr float MADGWICK_BETA = 0.06f;
// Madgwick::updateIMU expects accel in g, gyro in rad/s (Adafruit reports m/s² and rad/s).
static constexpr float MS2_TO_G = 1.0f / 9.80665f;
Adafruit_MPU6050 mpu;
Madgwick imuFilter(MADGWICK_BETA);
static uint32_t imuMicrosPrev = 0;
static bool imuReady = false;

struct CbServer : BLEServerCallbacks {
  void onDisconnect(BLEServer *) { BLEDevice::startAdvertising(); }
};

struct CbChar : BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    String v = c->getValue();
    const size_t n = v.length();
    if (n != sizeof(RobotCommand)) {
      return;
    }
    memcpy(&targets, v.c_str(), sizeof(targets));
    targets.left = (int8_t)constrain((int)targets.left, -125, 125);
    targets.right = (int8_t)constrain((int)targets.right, -125, 125);
    lastCommandMs = millis();
  }
};

static CbServer cbServer;
static CbChar cbChar;

static void packArm12(const uint16_t in[So101Arm::kMotorCount], uint8_t packed[9]) {
  memset(packed, 0, 9);
  int byteidx = 0;
  bool insert2 = true;
  for (int i = 0; i < So101Arm::kMotorCount; i++) {
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

static void unpackArm12(const uint8_t packed[9], uint16_t out[So101Arm::kMotorCount]) {
  int byteidx = 0;
  bool extract2 = true;
  uint16_t a, b = 0;
  for (int i = 0; i < So101Arm::kMotorCount; i++) {
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

// 2 x 12-bit LSB-first (same layout as first 3 bytes of packArm12).
static inline void packEnc12(uint16_t left, uint16_t right, uint8_t out[3]) {
  out[0] = (uint8_t)left;
  out[1] = (uint8_t)(((left >> 8) & 0x0F) | ((right & 0x0F) << 4));
  out[2] = (uint8_t)(right >> 4);
}

void setup() {
  hostSerial.begin(115200);
  delay(200);

  memset(&targets, 0, sizeof(targets));
  memset(&st, 0, sizeof(st));

  BLEDevice::init(kName);
  // Large enough for RobotState notify (AprilTag list + headers); central must negotiate up.
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

  Wire.begin(WIRE_SDA0, WIRE_SCL0);
  Wire.setClock(400000);
  Wire1.begin(WIRE_SDA1, WIRE_SCL1);
  Wire1.setClock(400000);

  if (mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire)) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
    imuMicrosPrev = micros();
    imuReady = true;
    st.quat[0] = 1000;
    st.quat[1] = st.quat[2] = st.quat[3] = 0;
  } else {
    st.quat[0] = 1000;
    st.quat[1] = st.quat[2] = st.quat[3] = 0;
  }

  servoLeft.attach(SERVO_LEFT);
  servoRight.attach(SERVO_RIGHT);
  servoLeft.writeMicroseconds((int)(targets.left * 4) + 1500);
  servoRight.writeMicroseconds((int)(targets.right * 4) + 1500);

  arm.begin();

  display.begin();
  updateDisplay();
  delay(50);
}

static uint16_t getAS5600Reading(TwoWire &bus) {
  bus.beginTransmission(AS5600_ADDR);
  bus.write(0x0E); // RAW ANGLE register (high byte)
  bus.endTransmission();

  bus.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
  if (bus.available() >= 2) {
    uint16_t rawAngle = (uint16_t)bus.read() << 8;
    rawAngle |= bus.read();
    return rawAngle;
  }
  return (uint16_t)-1;
}

/** Left AS5600 + MPU on Wire0, right AS5600 on Wire1. */
static void pollWheelEncoders() {
  const uint16_t left = getAS5600Reading(Wire);
  if (left != (uint16_t)-1) {
    encRaw[0] = left & 0x0FFF;
  }
  const uint16_t right = getAS5600Reading(Wire1);
  if (right != (uint16_t)-1) {
    encRaw[1] = right & 0x0FFF;
  }
}

static inline int16_t quatUnitToMilli(float q) {
  return constrain((long)lroundf(q * 1000.0f), -32768, 32767);
}

static void updateIMU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  const uint32_t nowUs = micros();
  float dt = (nowUs - imuMicrosPrev) * 1e-6f;
  imuMicrosPrev = nowUs;
  if (dt > 0.f && dt <= 0.2f) {
    const float ax = a.acceleration.x * MS2_TO_G;
    const float ay = a.acceleration.y * MS2_TO_G;
    const float az = a.acceleration.z * MS2_TO_G;
    const float gx = g.gyro.x;
    const float gy = g.gyro.y;
    const float gz = g.gyro.z;
    imuFilter.updateIMU(gx, gy, gz, ax, ay, az, dt);
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // Update sensors
  if (imuReady) updateIMU();
  pollWheelEncoders();

  // Pi -> USB serial HostSerial payload from detect_atags.py (same as test-serial-pi.ino loop)
  message_t *msg = hostSerial.readMessage();
  if (msg == STIMEOUT) {
    st.ntags = 0;
    memset(st.tags, 0, 10 * sizeof(struct AprilTagInfo));
  } else if (msg != nullptr && msg->length >= 1) {
    st.ntags = msg->data[0];
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
  }

  // Write motor commands
  const bool bleActive = (lastCommandMs != 0) && (currentMillis - lastCommandMs < BLE_RX_TIMEOUT);
  if (bleActive) {
    armHaltSent = false;
    if (currentMillis - prevArmWriteMs >= SO_TX_INTERVAL) {
      prevArmWriteMs = currentMillis;
      unpackArm12(targets.arm, gJointRawPos);
      for (uint8_t i = 0; i < So101Arm::kMotorCount; i++) {
        gJointPos[i] = (int16_t)gJointRawPos[i];
      }
      arm.setAngles(gJointPos);
    }
  } else {
    targets.left = 0;
    targets.right = 0;
    if (lastCommandMs != 0 && !armHaltSent) {
      arm.halt();
      armHaltSent = true;
    }
  }

  if (currentMillis - prevArmReadMs >= SO_RX_INTERVAL) {
    prevArmReadMs = currentMillis;
    if (arm.readAngles(gJointPos)) {
      for (uint8_t i = 0; i < So101Arm::kMotorCount; i++) {
        gJointRawPos[i] = (uint16_t)(gJointPos[i] & 0x0FFF);
      }
      packArm12(gJointRawPos, st.armPos);
    }
  }

  servoLeft.writeMicroseconds((int)(targets.left * 4) + 1500);
  servoRight.writeMicroseconds((int)(targets.right * 4) + 1500);

  if (currentMillis - prevBleTxMs >= BLE_TX_INTERVAL) {
    prevBleTxMs = currentMillis;

    packEnc12(encRaw[0], encRaw[1], st.enc);

    if (imuReady) {
      st.quat[0] = quatUnitToMilli(imuFilter.q0());
      st.quat[1] = quatUnitToMilli(imuFilter.q1());
      st.quat[2] = quatUnitToMilli(imuFilter.q2());
      st.quat[3] = quatUnitToMilli(imuFilter.q3());
    } else {
      st.quat[0] = 1000;
      st.quat[1] = st.quat[2] = st.quat[3] = 0;
    }
    bleCh->setValue((uint8_t *)&st, sizeof(st));
    bleCh->notify();
  }
}
