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
   * Partial replies are OK: ``out`` is resized to ``ids.size()``; ``out[i]`` is the
   * 12-bit position when that ID answered, or -1 if missing.
   * Returns true if at least one ID replied with a valid status frame.
   */
  bool readAngles(const std::vector<uint8_t> &ids, std::vector<int16_t> &out);

  /**
   * SYNC_READ ``candidateIds`` and keep only responders.
   * ``foundIds`` / ``foundPos`` are parallel (cleared then filled).
   * Returns true if at least one servo replied.
   */
  bool scanAllServosOnline(const std::vector<uint8_t> &candidateIds,
                           std::vector<uint8_t> &foundIds,
                           std::vector<int16_t> &foundPos);

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
  /** Validate FF FF … status frame; return present-position raw or -1. */
  int parseStatusFrame(const uint8_t *frame);
  /** Scan ``buf`` byte-by-byte for a valid status frame from ``id``. */
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
