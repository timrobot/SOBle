#include "STSDriver.h"
#include <string.h>

static const uint8_t INST_SYNC_READ = 0x82;
static const uint8_t INST_SYNC_WRITE = 0x83;
static const uint8_t REG_TORQUE_ENABLE = 40;
static const uint8_t REG_ACCELERATION = 41;
static const uint8_t REG_MODE = 33;
static const uint8_t MODE_VELOCITY = 1;
static const uint8_t REG_PRESENT_POSITION = 0x38;
static const uint8_t REG_GOAL_POSITION = 0x2A;
static const uint8_t REG_GOAL_SPEED_L = 46;
static const uint8_t ARM_DATA_LEN = 2;
static const uint8_t RESP_FRAME_LEN = 8;

static const uint32_t kRxIdleUs = 400;
static const uint32_t kRxMaxUs = 8000;
static constexpr uint8_t kWheelAcceleration = 200;

STSDriver::STSDriver(uint8_t uartNum, int rxPin, int txPin, uint32_t baud)
    : _serial(uartNum),
      _rxPin(rxPin),
      _txPin(txPin),
      _baud(baud),
      _syncWriteTxLen(0),
      _presentValid(false),
      _halted(false),
      _wheelVelocityModeReady(false) {}

void STSDriver::begin() {
  _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
  while (_serial.available() > 0) {
    _serial.read();
  }
}

uint8_t STSDriver::listSize(std::initializer_list<uint8_t> ids) {
  return (uint8_t)ids.size();
}

uint8_t STSDriver::checksum(const uint8_t *bodyFromId, size_t n) {
  uint16_t sum = 0;
  for (size_t i = 0; i < n; i++) {
    sum += bodyFromId[i];
  }
  return (uint8_t)((~sum) & 0xFF);
}

size_t STSDriver::buildSyncReadPacket(uint8_t *out, const uint8_t *ids, uint8_t count) {
  const uint8_t lenField = (uint8_t)(count + 4);
  size_t idx = 0;
  out[idx++] = 0xFF;
  out[idx++] = 0xFF;
  out[idx++] = 0xFE;
  out[idx++] = lenField;
  out[idx++] = INST_SYNC_READ;
  out[idx++] = REG_PRESENT_POSITION;
  out[idx++] = ARM_DATA_LEN;
  for (uint8_t i = 0; i < count; i++) {
    out[idx++] = ids[i];
  }
  out[idx++] = checksum(&out[2], idx - 2);
  return idx;
}

int STSDriver::parseStatusFrame(const uint8_t *frame) {
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

int STSDriver::findPosition(const uint8_t *buf, size_t buflen, uint8_t id) {
  for (size_t i = 0; i + RESP_FRAME_LEN <= buflen; i++) {
    if (buf[i] == 0xFF && buf[i + 1] == 0xFF && buf[i + 2] == id) {
      return parseStatusFrame(&buf[i]);
    }
  }
  return -1;
}

bool STSDriver::readAngles(std::initializer_list<uint8_t> ids, int16_t *out) {
  const uint8_t count = listSize(ids);
  if (count == 0 || count > kMaxServos || out == nullptr) {
    return false;
  }

  uint8_t idList[kMaxServos];
  uint8_t idx = 0;
  for (uint8_t id : ids) {
    idList[idx++] = id;
  }

  uint8_t tx[32];
  const size_t txLen = buildSyncReadPacket(tx, idList, count);
  const size_t expectRx = (size_t)count * RESP_FRAME_LEN;

  while (_serial.available() > 0) {
    _serial.read();
  }
  _serial.write(tx, txLen);
  _serial.flush();

  size_t n = 0;
  const uint32_t t0 = micros();
  uint32_t lastByteUs = 0;
  while ((uint32_t)(micros() - t0) < kRxMaxUs) {
    while (_serial.available() > 0 && n < expectRx) {
      _rxBuf[n++] = (uint8_t)_serial.read();
      lastByteUs = micros();
    }
    if (n >= expectRx) {
      break;
    }
    if (n > 0 && lastByteUs > 0 && (uint32_t)(micros() - lastByteUs) > kRxIdleUs) {
      break;
    }
  }
  if (n < expectRx) {
    return false;
  }

  for (uint8_t i = 0; i < count; i++) {
    const int pos = findPosition(_rxBuf, n, idList[i]);
    if (pos < 0) {
      return false;
    }
    out[i] = (int16_t)pos;
  }
  _presentValid = true;
  return true;
}

size_t STSDriver::buildSyncWritePositionPacket(uint8_t *out, const uint8_t *ids, uint8_t count,
                                               const uint16_t *positions) {
  const uint8_t dataLenPerServo = 4;
  const uint8_t lenField = (uint8_t)(4 + 5 * count);
  size_t idx = 0;

  out[idx++] = 0xFF;
  out[idx++] = 0xFF;
  out[idx++] = 0xFE;
  out[idx++] = lenField;
  out[idx++] = INST_SYNC_WRITE;
  out[idx++] = REG_GOAL_POSITION;
  out[idx++] = dataLenPerServo;

  for (uint8_t i = 0; i < count; i++) {
    const uint16_t pos = positions[i];
    out[idx++] = ids[i];
    out[idx++] = pos & 0xFF;
    out[idx++] = (pos >> 8) & 0xFF;
    out[idx++] = 0;
    out[idx++] = 0;
  }
  out[idx++] = checksum(&out[2], idx - 2);
  return idx;
}

size_t STSDriver::buildSyncWriteBytePacket(uint8_t *out, const uint8_t *ids, uint8_t count,
                                           uint8_t reg, const uint8_t *values) {
  const uint8_t dataLenPerServo = 1;
  const uint8_t lenField = (uint8_t)(4 + 2 * count);
  size_t idx = 0;

  out[idx++] = 0xFF;
  out[idx++] = 0xFF;
  out[idx++] = 0xFE;
  out[idx++] = lenField;
  out[idx++] = INST_SYNC_WRITE;
  out[idx++] = reg;
  out[idx++] = dataLenPerServo;

  for (uint8_t i = 0; i < count; i++) {
    out[idx++] = ids[i];
    out[idx++] = values[i];
  }
  out[idx++] = checksum(&out[2], idx - 2);
  return idx;
}

size_t STSDriver::buildSyncWriteSpeedPacket(uint8_t *out, const uint8_t *ids, uint8_t count,
                                            const uint16_t *speeds) {
  const uint8_t dataLenPerServo = 2;
  const uint8_t lenField = (uint8_t)(4 + 3 * count);
  size_t idx = 0;

  out[idx++] = 0xFF;
  out[idx++] = 0xFF;
  out[idx++] = 0xFE;
  out[idx++] = lenField;
  out[idx++] = INST_SYNC_WRITE;
  out[idx++] = REG_GOAL_SPEED_L;
  out[idx++] = dataLenPerServo;

  for (uint8_t i = 0; i < count; i++) {
    const uint16_t speed = speeds[i];
    out[idx++] = ids[i];
    out[idx++] = speed & 0xFF;
    out[idx++] = (speed >> 8) & 0xFF;
  }
  out[idx++] = checksum(&out[2], idx - 2);
  return idx;
}

void STSDriver::syncWriteRaw(const uint8_t *packet, size_t len) {
  while (_serial.available() > 0) {
    _serial.read();
  }
  _serial.write(packet, len);
  _serial.flush();
}

void STSDriver::halt(std::initializer_list<uint8_t> ids) {
  const uint8_t count = listSize(ids);
  if (count == 0 || count > kMaxServos) {
    return;
  }

  uint8_t idList[kMaxServos];
  uint16_t zero[kMaxServos] = {0};
  uint8_t idx = 0;
  for (uint8_t id : ids) {
    idList[idx++] = id;
  }

  _halted = true;
  _syncWriteTxLen = buildSyncWriteSpeedPacket(_syncWriteTx, idList, count, zero);
  syncWriteRaw(_syncWriteTx, _syncWriteTxLen);
}

void STSDriver::setAngles(const uint8_t *ids, uint8_t count, const int16_t *raw) {
  if (count == 0 || count > kMaxServos || ids == nullptr || raw == nullptr) {
    return;
  }

  uint16_t goals[kMaxServos];
  for (uint8_t i = 0; i < count; i++) {
    goals[i] = (uint16_t)(raw[i] & 0x0FFF);
  }

  _halted = false;
  _syncWriteTxLen = buildSyncWritePositionPacket(_syncWriteTx, ids, count, goals);
  syncWriteRaw(_syncWriteTx, _syncWriteTxLen);
}

void STSDriver::setAngles(std::initializer_list<uint8_t> ids, const int16_t *raw) {
  const uint8_t count = listSize(ids);
  if (count == 0 || count > kMaxServos || raw == nullptr) {
    return;
  }

  uint8_t idList[kMaxServos];
  uint8_t idx = 0;
  for (uint8_t id : ids) {
    idList[idx++] = id;
  }
  setAngles(idList, count, raw);
}

void STSDriver::releaseTorque(const uint8_t *ids, uint8_t count) {
  if (count == 0 || count > kMaxServos || ids == nullptr) {
    return;
  }

  uint8_t off[kMaxServos] = {0};
  _syncWriteTxLen = buildSyncWriteBytePacket(_syncWriteTx, ids, count, REG_TORQUE_ENABLE, off);
  syncWriteRaw(_syncWriteTx, _syncWriteTxLen);
}

void STSDriver::engageTorque(const uint8_t *ids, uint8_t count) {
  if (count == 0 || count > kMaxServos || ids == nullptr) {
    return;
  }

  uint8_t on[kMaxServos];
  for (uint8_t i = 0; i < count; i++) {
    on[i] = 1;
  }
  _syncWriteTxLen = buildSyncWriteBytePacket(_syncWriteTx, ids, count, REG_TORQUE_ENABLE, on);
  syncWriteRaw(_syncWriteTx, _syncWriteTxLen);
}

void STSDriver::enableVelocityMode(const uint8_t *ids, uint8_t count) {
  if (count == 0 || count > kMaxServos || ids == nullptr) {
    return;
  }

  uint8_t mode[kMaxServos];
  for (uint8_t i = 0; i < count; i++) {
    mode[i] = MODE_VELOCITY;
  }

  releaseTorque(ids, count);
  delay(20);
  _syncWriteTxLen = buildSyncWriteBytePacket(_syncWriteTx, ids, count, REG_MODE, mode);
  syncWriteRaw(_syncWriteTx, _syncWriteTxLen);
  delay(20);

  uint8_t accel[kMaxServos];
  for (uint8_t i = 0; i < count; i++) {
    accel[i] = kWheelAcceleration;
  }
  _syncWriteTxLen = buildSyncWriteBytePacket(_syncWriteTx, ids, count, REG_ACCELERATION, accel);
  syncWriteRaw(_syncWriteTx, _syncWriteTxLen);

  engageTorque(ids, count);
  _wheelVelocityModeReady = true;
}

void STSDriver::enableVelocityMode(std::initializer_list<uint8_t> ids) {
  const uint8_t count = listSize(ids);
  if (count == 0 || count > kMaxServos) {
    return;
  }

  uint8_t idList[kMaxServos];
  uint8_t idx = 0;
  for (uint8_t id : ids) {
    idList[idx++] = id;
  }
  enableVelocityMode(idList, count);
}

void STSDriver::setSpeed(std::initializer_list<uint8_t> ids, const int16_t *raw) {
  const uint8_t count = listSize(ids);
  if (count == 0 || count > kMaxServos || raw == nullptr) {
    return;
  }

  uint8_t idList[kMaxServos];
  uint16_t speeds[kMaxServos];
  uint8_t idx = 0;
  for (uint8_t id : ids) {
    idList[idx] = id;
    speeds[idx] = (uint16_t)raw[idx];
    idx++;
  }

  if (!_wheelVelocityModeReady) {
    enableVelocityMode(idList, count);
  }

  _halted = false;
  _syncWriteTxLen = buildSyncWriteSpeedPacket(_syncWriteTx, idList, count, speeds);
  syncWriteRaw(_syncWriteTx, _syncWriteTxLen);
}
