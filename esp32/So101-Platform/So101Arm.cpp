#include "So101Arm.h"
#include <string.h>

static const uint8_t INST_SYNC_READ = 0x82;
static const uint8_t INST_SYNC_WRITE = 0x83;
static const uint8_t REG_PRESENT_POSITION = 0x38;
static const uint8_t REG_GOAL_POSITION = 0x2A;
static const uint8_t REG_GOAL_SPEED_L = 46;
static const uint8_t ARM_DATA_LEN = 2;
static const uint8_t RESP_FRAME_LEN = 8;

static const size_t kExpectRx = So101Arm::kMotorCount * RESP_FRAME_LEN;
static const uint32_t kRxIdleUs = 400;
static const uint32_t kRxMaxUs = 8000;

const uint8_t So101Arm::kIds[kMotorCount] = {1, 2, 3, 4, 5, 6};

So101Arm::So101Arm(uint8_t uartNum, int rxPin, int txPin, uint32_t baud)
    : _serial(uartNum),
      _rxPin(rxPin),
      _txPin(txPin),
      _baud(baud),
      _syncReadTxLen(0),
      _syncWriteTxLen(0),
      _presentValid(false),
      _halted(false) {
  memset(_present, 0, sizeof(_present));
}

void So101Arm::begin() {
  _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
  _syncReadTxLen = buildSyncReadPacket(_syncReadTx);
  while (_serial.available() > 0) {
    _serial.read();
  }
}

uint8_t So101Arm::checksum(const uint8_t *bodyFromId, size_t n) {
  uint16_t sum = 0;
  for (size_t i = 0; i < n; i++) {
    sum += bodyFromId[i];
  }
  return (uint8_t)((~sum) & 0xFF);
}

size_t So101Arm::buildSyncReadPacket(uint8_t *out) {
  const uint8_t lenField = (uint8_t)(kMotorCount + 4);
  size_t idx = 0;
  out[idx++] = 0xFF;
  out[idx++] = 0xFF;
  out[idx++] = 0xFE;
  out[idx++] = lenField;
  out[idx++] = INST_SYNC_READ;
  out[idx++] = REG_PRESENT_POSITION;
  out[idx++] = ARM_DATA_LEN;
  for (uint8_t i = 0; i < kMotorCount; i++) {
    out[idx++] = kIds[i];
  }
  out[idx++] = checksum(&out[2], idx - 2);
  return idx;
}

int So101Arm::parseStatusFrame(const uint8_t *frame) {
  if (frame[0] != 0xFF || frame[1] != 0xFF) {
    return -1;
  }
  uint16_t sum = 0;
  for (size_t i = 2; i < 7; i++) {
    sum += frame[i];
  }
  if ((uint8_t)((~sum) & 0xFF) != frame[7]) {
    return -1;
  }
  return (int)((frame[6] << 8) | frame[5]);
}

int So101Arm::findPosition(const uint8_t *buf, size_t buflen, uint8_t id) {
  for (size_t i = 0; i + RESP_FRAME_LEN <= buflen; i++) {
    if (buf[i] == 0xFF && buf[i + 1] == 0xFF && buf[i + 2] == id) {
      return parseStatusFrame(&buf[i]);
    }
  }
  return -1;
}

bool So101Arm::readAngles(int16_t out[kMotorCount]) {
  while (_serial.available() > 0) {
    _serial.read();
  }
  _serial.write(_syncReadTx, _syncReadTxLen);
  _serial.flush();

  size_t n = 0;
  const uint32_t t0 = micros();
  uint32_t lastByteUs = 0;
  while ((uint32_t)(micros() - t0) < kRxMaxUs) {
    while (_serial.available() > 0 && n < kExpectRx) {
      _rxBuf[n++] = (uint8_t)_serial.read();
      lastByteUs = micros();
    }
    if (n >= kExpectRx) {
      break;
    }
    if (n > 0 && lastByteUs > 0 && (uint32_t)(micros() - lastByteUs) > kRxIdleUs) {
      break;
    }
  }
  if (n < kExpectRx) {
    return false;
  }

  for (uint8_t i = 0; i < kMotorCount; i++) {
    const int pos = findPosition(_rxBuf, n, kIds[i]);
    if (pos < 0) {
      return false;
    }
    _present[i] = (int16_t)pos;
    out[i] = _present[i];
  }
  _presentValid = true;
  return true;
}

size_t So101Arm::buildSyncWritePositionPacket(uint8_t *out, const uint16_t positions[kMotorCount]) {
  const uint8_t dataLenPerServo = 4;
  const uint8_t lenField = (uint8_t)(4 + 5 * kMotorCount);
  size_t idx = 0;

  out[idx++] = 0xFF;
  out[idx++] = 0xFF;
  out[idx++] = 0xFE;
  out[idx++] = lenField;
  out[idx++] = INST_SYNC_WRITE;
  out[idx++] = REG_GOAL_POSITION;
  out[idx++] = dataLenPerServo;

  for (uint8_t i = 0; i < kMotorCount; i++) {
    const uint16_t pos = positions[i];
    out[idx++] = kIds[i];
    out[idx++] = pos & 0xFF;
    out[idx++] = (pos >> 8) & 0xFF;
    out[idx++] = 0;
    out[idx++] = 0;
  }
  out[idx++] = checksum(&out[2], idx - 2);
  return idx;
}

size_t So101Arm::buildSyncWriteSpeedPacket(uint8_t *out, uint16_t speed) {
  const uint8_t dataLenPerServo = 2;
  const uint8_t lenField = (uint8_t)(4 + 3 * kMotorCount);
  size_t idx = 0;

  out[idx++] = 0xFF;
  out[idx++] = 0xFF;
  out[idx++] = 0xFE;
  out[idx++] = lenField;
  out[idx++] = INST_SYNC_WRITE;
  out[idx++] = REG_GOAL_SPEED_L;
  out[idx++] = dataLenPerServo;

  for (uint8_t i = 0; i < kMotorCount; i++) {
    out[idx++] = kIds[i];
    out[idx++] = speed & 0xFF;
    out[idx++] = (speed >> 8) & 0xFF;
  }
  out[idx++] = checksum(&out[2], idx - 2);
  return idx;
}

void So101Arm::syncWriteRaw(const uint8_t *packet, size_t len) {
  while (_serial.available() > 0) {
    _serial.read();
  }
  _serial.write(packet, len);
  _serial.flush();
}

void So101Arm::halt() {
  _halted = true;
  _syncWriteTxLen = buildSyncWriteSpeedPacket(_syncWriteTx, 0);
  syncWriteRaw(_syncWriteTx, _syncWriteTxLen);
}

void So101Arm::setAngles(const int16_t raw[kMotorCount]) {
  _halted = false;
  uint16_t goals[kMotorCount];
  for (uint8_t i = 0; i < kMotorCount; i++) {
    goals[i] = (uint16_t)(raw[i] & 0x0FFF);
  }
  _syncWriteTxLen = buildSyncWritePositionPacket(_syncWriteTx, goals);
  syncWriteRaw(_syncWriteTx, _syncWriteTxLen);
}
