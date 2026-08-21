#ifndef STS_DRIVER_H
#define STS_DRIVER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <vector>

class STSDriver {
public:
  STSDriver(uint8_t uartNum, int rxPin, int txPin, uint32_t baud = 1000000);

  void begin();

  /**
   * SYNC_READ present position for ``ids``.
   * On success, ``out`` is resized to ``ids.size()`` with out[i] for ids[i].
   * Returns false on bus error or missing reply for any ID.
   */
  bool readAngles(const std::vector<uint8_t> &ids, std::vector<int16_t> &out);

  /**
   * SYNC_WRITE goal position for ``ids``.
   * raw[i] is the goal for ids[i] (12-bit STS ticks). Clears halt.
   */
  void setAngles(const std::vector<uint8_t> &ids, const std::vector<int16_t> &raw);

  /**
   * STS Mode=1 (constant speed / wheel). GOAL_SPEED is ignored in position mode.
   * Torque is cycled off briefly while writing the mode register.
   */
  void enableVelocityMode(const std::vector<uint8_t> &ids);

  /**
   * SYNC_WRITE goal speed for ``ids``.
   * raw[i] is the speed for ids[i]. Ensures velocity mode on first call.
   */
  void setSpeed(const std::vector<uint8_t> &ids, const std::vector<int16_t> &raw);

  /** SYNC_WRITE goal speed 0 on ``ids``. */
  void halt(const std::vector<uint8_t> &ids);

  /** Torque off (STS Torque_Enable=0) so joints can be moved by hand. */
  void releaseTorque(const std::vector<uint8_t> &ids);
  /** Torque on (STS Torque_Enable=1) before position commands. */
  void engageTorque(const std::vector<uint8_t> &ids);

  /**
   * FeeTech PING (instruction 0x01) for a single servo ID — not a SYNC_READ.
   * Returns true if a valid status reply is received.
   */
  bool ping(uint8_t id);

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
  bool idsUsable(const std::vector<uint8_t> &ids) const;

  HardwareSerial _serial;
  int _rxPin;
  int _txPin;
  uint32_t _baud;

  std::vector<uint8_t> _syncWriteTx;
  std::vector<uint8_t> _rxBuf;
  bool _presentValid;
  bool _halted;
  bool _wheelVelocityModeReady;
};

#endif
