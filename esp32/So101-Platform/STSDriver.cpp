#include "STSDriver.h"
#include <string.h>

static const uint8_t INST_PING = 0x01;
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
static const uint8_t PING_RESP_LEN = 6;

static const uint32_t kRxIdleUs = 400;
static const uint32_t kRxMaxUs = 8000;
static const uint32_t kPingRxMaxUs = 2000;
static constexpr uint8_t kWheelAcceleration = 0;
/** Practical FeeTech SYNC_* packet limit (IDs + framing). */
static constexpr size_t kMaxIdsPerPacket = 20;

STSDriver::STSDriver(uint8_t uartNum, int rxPin, int txPin, uint32_t baud)
    : _serial(uartNum),
      _rxPin(rxPin),
      _txPin(txPin),
      _baud(baud),
      _presentValid(false),
      _halted(false),
      _wheelVelocityModeReady(false) {}

void STSDriver::begin() {
  _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
  while (_serial.available() > 0) {
    _serial.read();
  }
}

bool STSDriver::idsUsable(const std::vector<uint8_t> &ids) const {
  return !ids.empty() && ids.size() <= kMaxIdsPerPacket;
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

bool STSDriver::readAngles(const std::vector<uint8_t> &ids, std::vector<int16_t> &out) {
  if (!idsUsable(ids)) {
    return false;
  }
  const uint8_t count = (uint8_t)ids.size();
  const size_t expectRx = (size_t)count * RESP_FRAME_LEN;

  std::vector<uint8_t> tx(32 + count);
  const size_t txLen = buildSyncReadPacket(tx.data(), ids.data(), count);
  _rxBuf.assign(expectRx, 0);

  while (_serial.available() > 0) {
    _serial.read();
  }
  _serial.write(tx.data(), txLen);
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

  out.resize(count);
  for (uint8_t i = 0; i < count; i++) {
    const int pos = findPosition(_rxBuf.data(), n, ids[i]);
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

void STSDriver::halt(const std::vector<uint8_t> &ids) {
  if (!idsUsable(ids)) {
    return;
  }
  const uint8_t count = (uint8_t)ids.size();
  std::vector<uint16_t> zero(count, 0);
  _syncWriteTx.resize(8 + 3 * count);
  _halted = true;
  const size_t len = buildSyncWriteSpeedPacket(_syncWriteTx.data(), ids.data(), count, zero.data());
  syncWriteRaw(_syncWriteTx.data(), len);
}

void STSDriver::setAngles(const std::vector<uint8_t> &ids, const std::vector<int16_t> &raw) {
  if (!idsUsable(ids) || raw.size() != ids.size()) {
    return;
  }
  const uint8_t count = (uint8_t)ids.size();
  std::vector<uint16_t> goals(count);
  for (uint8_t i = 0; i < count; i++) {
    goals[i] = (uint16_t)(raw[i] & 0x0FFF);
  }

  _syncWriteTx.resize(8 + 5 * count);
  _halted = false;
  const size_t len =
      buildSyncWritePositionPacket(_syncWriteTx.data(), ids.data(), count, goals.data());
  syncWriteRaw(_syncWriteTx.data(), len);
}

void STSDriver::releaseTorque(const std::vector<uint8_t> &ids) {
  if (!idsUsable(ids)) {
    return;
  }
  const uint8_t count = (uint8_t)ids.size();
  std::vector<uint8_t> off(count, 0);
  _syncWriteTx.resize(8 + 2 * count);
  const size_t len =
      buildSyncWriteBytePacket(_syncWriteTx.data(), ids.data(), count, REG_TORQUE_ENABLE, off.data());
  syncWriteRaw(_syncWriteTx.data(), len);
}

void STSDriver::engageTorque(const std::vector<uint8_t> &ids) {
  if (!idsUsable(ids)) {
    return;
  }
  const uint8_t count = (uint8_t)ids.size();
  std::vector<uint8_t> on(count, 1);
  _syncWriteTx.resize(8 + 2 * count);
  const size_t len =
      buildSyncWriteBytePacket(_syncWriteTx.data(), ids.data(), count, REG_TORQUE_ENABLE, on.data());
  syncWriteRaw(_syncWriteTx.data(), len);
}

bool STSDriver::ping(uint8_t id) {
  // TX: FF FF ID 02 01 CHK
  uint8_t tx[6];
  tx[0] = 0xFF;
  tx[1] = 0xFF;
  tx[2] = id;
  tx[3] = 0x02;
  tx[4] = INST_PING;
  tx[5] = checksum(&tx[2], 3);

  while (_serial.available() > 0) {
    _serial.read();
  }
  _serial.write(tx, sizeof(tx));
  _serial.flush();

  uint8_t rx[PING_RESP_LEN];
  size_t n = 0;
  const uint32_t t0 = micros();
  uint32_t lastByteUs = 0;
  while ((uint32_t)(micros() - t0) < kPingRxMaxUs) {
    while (_serial.available() > 0 && n < PING_RESP_LEN) {
      rx[n++] = (uint8_t)_serial.read();
      lastByteUs = micros();
    }
    if (n >= PING_RESP_LEN) {
      break;
    }
    if (n > 0 && lastByteUs > 0 && (uint32_t)(micros() - lastByteUs) > kRxIdleUs) {
      break;
    }
  }
  if (n < PING_RESP_LEN) {
    return false;
  }
  // RX: FF FF ID 02 ERR CHK
  if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id || rx[3] != 0x02) {
    return false;
  }
  return checksum(&rx[2], 3) == rx[5];
}

void STSDriver::enableVelocityMode(const std::vector<uint8_t> &ids) {
  if (!idsUsable(ids)) {
    return;
  }
  const uint8_t count = (uint8_t)ids.size();
  std::vector<uint8_t> mode(count, MODE_VELOCITY);

  releaseTorque(ids);
  delay(20);
  _syncWriteTx.resize(8 + 2 * count);
  size_t len =
      buildSyncWriteBytePacket(_syncWriteTx.data(), ids.data(), count, REG_MODE, mode.data());
  syncWriteRaw(_syncWriteTx.data(), len);
  delay(20);

  std::vector<uint8_t> accel(count, kWheelAcceleration);
  len = buildSyncWriteBytePacket(_syncWriteTx.data(), ids.data(), count, REG_ACCELERATION,
                                 accel.data());
  syncWriteRaw(_syncWriteTx.data(), len);

  engageTorque(ids);
  _wheelVelocityModeReady = true;
}

void STSDriver::setSpeed(const std::vector<uint8_t> &ids, const std::vector<int16_t> &raw) {
  if (!idsUsable(ids) || raw.size() != ids.size()) {
    return;
  }
  const uint8_t count = (uint8_t)ids.size();
  std::vector<uint16_t> speeds(count);
  for (uint8_t i = 0; i < count; i++) {
    speeds[i] = (uint16_t)raw[i];
  }

  if (!_wheelVelocityModeReady) {
    enableVelocityMode(ids);
  }

  _syncWriteTx.resize(8 + 3 * count);
  _halted = false;
  const size_t len =
      buildSyncWriteSpeedPacket(_syncWriteTx.data(), ids.data(), count, speeds.data());
  syncWriteRaw(_syncWriteTx.data(), len);
}
