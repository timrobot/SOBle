#ifndef SO101_ARM_H
#define SO101_ARM_H

#include <Arduino.h>
#include <HardwareSerial.h>

class So101Arm {
public:
  static constexpr uint8_t kMotorCount = 6;

  So101Arm(uint8_t uartNum, int rxPin, int txPin, uint32_t baud = 1000000);

  void begin();

  /** SYNC_WRITE goal positions (raw STS ticks, motors 1–6). Clears halt. */
  void setAngles(const int16_t raw[kMotorCount]);

  /** SYNC_WRITE goal speed 0 on all joints; no position writes until setAngles(). */
  void halt();

  bool isHalted() const { return _halted; }

  /** SYNC_READ present positions into out[0..5]. Returns false on bus error. */
  bool readAngles(int16_t out[kMotorCount]);

  bool hasValidPresent() const { return _presentValid; }

private:
  static uint8_t checksum(const uint8_t *bodyFromId, size_t n);
  size_t buildSyncReadPacket(uint8_t *out);
  size_t buildSyncWritePositionPacket(uint8_t *out, const uint16_t positions[kMotorCount]);
  size_t buildSyncWriteSpeedPacket(uint8_t *out, uint16_t speed);
  int parseStatusFrame(const uint8_t *frame);
  int findPosition(const uint8_t *buf, size_t buflen, uint8_t id);
  void syncWriteRaw(const uint8_t *packet, size_t len);

  HardwareSerial _serial;
  int _rxPin;
  int _txPin;
  uint32_t _baud;

  uint8_t _syncReadTx[32];
  size_t _syncReadTxLen;
  uint8_t _syncWriteTx[64];
  size_t _syncWriteTxLen;
  uint8_t _rxBuf[48];
  int16_t _present[kMotorCount];
  bool _presentValid;
  bool _halted;

  static const uint8_t kIds[kMotorCount];
};

#endif
