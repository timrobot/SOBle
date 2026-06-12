#ifndef STS_DRIVER_H
#define STS_DRIVER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <initializer_list>

class STSDriver {
public:
  static constexpr uint8_t kMaxServos = 8;
  /** Arm joints (goal position); IDs 1–6. */
  static constexpr uint8_t kArmJointCount = 6;
  /** Wheel / speed servos; IDs 7–8. */
  static constexpr uint8_t kWheelCount = 2;

  STSDriver(uint8_t uartNum, int rxPin, int txPin, uint32_t baud = 1000000);

  void begin();

  /**
   * SYNC_READ present position for the listed IDs (e.g. {1,2,3,4,5,6,7,8}).
   * out[i] matches the i-th ID in ids. Returns false on bus error.
   */
  bool readAngles(std::initializer_list<uint8_t> ids, int16_t *out);

  /**
   * SYNC_WRITE goal position for the listed IDs (e.g. {1,2,3,4,5,6}).
   * raw[i] is the goal for the i-th ID in ids (12-bit STS ticks). Clears halt.
   */
  void setAngles(std::initializer_list<uint8_t> ids, const int16_t *raw);
  void setAngles(const uint8_t *ids, uint8_t count, const int16_t *raw);

  /**
   * STS Mode=1 (constant speed / wheel). GOAL_SPEED is ignored in position mode.
   * Torque is cycled off briefly while writing the mode register.
   */
  void enableVelocityMode(std::initializer_list<uint8_t> ids);

  /**
   * SYNC_WRITE goal speed for the listed IDs (e.g. {7,8}).
   * raw[i] is the speed for the i-th ID in ids. Ensures velocity mode on first call.
   */
  void setSpeed(std::initializer_list<uint8_t> ids, const int16_t *raw);

  /** SYNC_WRITE goal speed 0 on the listed IDs (e.g. {1,2,3,4,5,6,7,8}). */
  void halt(std::initializer_list<uint8_t> ids);

  /** Torque off (STS Torque_Enable=0) so joints can be moved by hand. */
  void releaseTorque(const uint8_t *ids, uint8_t count);
  /** Torque on (STS Torque_Enable=1) before position commands. */
  void engageTorque(const uint8_t *ids, uint8_t count);

  bool isHalted() const { return _halted; }

  bool hasValidPresent() const { return _presentValid; }

private:
  static uint8_t checksum(const uint8_t *bodyFromId, size_t n);
  size_t buildSyncReadPacket(uint8_t *out, const uint8_t *ids, uint8_t count);
  size_t buildSyncWritePositionPacket(uint8_t *out, const uint8_t *ids, uint8_t count,
                                      const uint16_t *positions);
  size_t buildSyncWriteSpeedPacket(uint8_t *out, const uint8_t *ids, uint8_t count,
                                   const uint16_t *speeds);
  size_t buildSyncWriteBytePacket(uint8_t *out, const uint8_t *ids, uint8_t count, uint8_t reg,
                                  const uint8_t *values);
  int parseStatusFrame(const uint8_t *frame);
  int findPosition(const uint8_t *buf, size_t buflen, uint8_t id);
  void syncWriteRaw(const uint8_t *packet, size_t len);
  void enableVelocityMode(const uint8_t *ids, uint8_t count);
  static uint8_t listSize(std::initializer_list<uint8_t> ids);

  HardwareSerial _serial;
  int _rxPin;
  int _txPin;
  uint32_t _baud;

  uint8_t _syncWriteTx[80];
  size_t _syncWriteTxLen;
  uint8_t _rxBuf[72];
  bool _presentValid;
  bool _halted;
  bool _wheelVelocityModeReady;
};

#endif
