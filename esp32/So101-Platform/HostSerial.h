#ifndef HOSTSERIAL_H
#define HOSTSERIAL_H

#include "HardwareSerial.h"

#define SMAXBYTES    512
#define SMSGBYTES    256
#define STIMEOUTMS   20 // 50Hz

typedef struct {
  uint8_t data[SMSGBYTES];
  int length;
} message_t;

#define STIMEOUT     (message_t *)(-1)

class HostSerial {
private:
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)  // Teensy 4.x
  usb_serial_class& ser;
#elif defined(ESP32) || defined(ARDUINO_ARCH_ESP32) || __AVR__
  HardwareSerial& ser;
#else
  Serial_& ser;
#endif
  int devid;
  uint32_t _timeout;

  char readbuf[SMAXBYTES];
  int readptr;
  char parsebuf[SMSGBYTES];
  char writebuf[SMSGBYTES];
  message_t msg;
  int writeptr;

  uint32_t lastReadTime;
  uint32_t lastWriteTime;

public:
  HostSerial(
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)  // Teensy 4.x
  usb_serial_class& ser,
#elif defined(ESP32) || defined(ARDUINO_ARCH_ESP32) || __AVR__
  HardwareSerial& ser,
#else
  Serial_& ser,
#endif
    int devid=0,
    int timeout=STIMEOUTMS
  );

  void begin(const int64_t baudrate=115200);
  /**
   * Read message, null if there is nothing, STIMEOUT on timeout
   */
  message_t *readMessage(void);
  void writeBytes(void *data, int size);

  void print(char data);
  void print(int data);
  void print(float data);
  void print(const char data[]);
  void println(char data);
  void println(int data);
  void println(float data);
  void println(const char data[]);
};

#endif // HOSTSERIAL_H
