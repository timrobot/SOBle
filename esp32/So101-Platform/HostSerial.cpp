#include "Arduino.h"
#include "HostSerial.h"
#include "Base64.h"
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32) || __AVR__
#include <HardwareSerial.h>
#endif

#define TOHEX(x) ((x) > 9 ? (x) - 10 + 'a' : (x) + '0')
#define FROMHEX(x) ((x) <= '9' ? (x) - '0' : ((x) >= 'a' ? (x) - 'a' + 10 : (x) - 'A' + 10) )

// Pi heartbeats at 100 ms in stream mode; allow a few missed frames.
static constexpr uint32_t kPiAliveTimeoutMs = 250;

/// Drain UART RX without blocking (never call readBytes — it can wait on timeout).
static int readAvailableNonBlocking(Stream &ser, char *buf, int readptr, int cap) {
  while (ser.available() > 0 && readptr < cap - 1) {
    buf[readptr++] = (char)ser.read();
  }
  buf[readptr] = '\0';
  return readptr;
}

/// Wrapper for integer communication, sends byte arrays as b64
/// Byte formats must be specified beforehand - this includes any calls for RPC or otherwise
/// <param name="ser">the serial interface</param>
/// <param name="devid">the id of this device</param>
/// <param name="header_timeout">the number of milliseconds to wait between each header msg</param>
HostSerial::HostSerial(
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)  // Teensy 4.x
  usb_serial_class& ser,
#elif defined(ESP32) || defined(ARDUINO_ARCH_ESP32) || __AVR__
  Stream& ser,
#else
  Serial_& ser,
#endif
  int device_id, int timeout) :
    ser(ser), devid(device_id), _timeout((uint32_t)timeout),
    readptr(0), writeptr(0), lastReadTime(0), lastWriteTime(0) {
  // b64data CC\n
  // CC = checksum 0-F 0-F
  memset(readbuf, 0, sizeof(readbuf));
  memset(writebuf, 0, sizeof(writebuf));
}

void HostSerial::begin(const int64_t baudrate, int rxPin, int txPin) {
  #if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)  // Teensy 4.x
  ser.begin(baudrate);
  #elif defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  #if ARDUINO_USB_CDC_ON_BOOT
  if (&ser == (Stream *)&Serial) {
    Serial.begin((uint32_t)baudrate);
    lastReadTime = millis();
    return;
  }
  #endif
  if (rxPin >= 0 && txPin >= 0) {
    static_cast<HardwareSerial &>(ser).begin(baudrate, SERIAL_8N1, rxPin, txPin);
  } else {
    static_cast<HardwareSerial &>(ser).begin(baudrate);
  }
  static_cast<HardwareSerial &>(ser).setTimeout(0);
  #elif __AVR__
  static_cast<HardwareSerial &>(ser).begin(baudrate);
  #else
  ser.begin(baudrate);
  #endif
  lastReadTime = millis();
}

message_t *HostSerial::readMessage(void) {
  uint32_t msec = millis();
  int nbytes = 0, i, length = 0;
  uint8_t chksum = 0, chkA, chkB;
  char *ptr = parsebuf, *delim;

  if (ser.available()) {
    readptr = readAvailableNonBlocking(ser, readbuf, readptr, (int)sizeof(readbuf));
    // Floating RX with no Pi can accumulate garbage without '\n' — drop if full.
    if (readptr >= (int)sizeof(readbuf) - 1) {
      readptr = 0;
      readbuf[0] = '\0';
      return nullptr;
    }
    if ((delim = strchr(readbuf, '\n')) == NULL) {
      return nullptr; // partial line, wait for more bytes
    }
    delim[0] = '\0'; // marker
    strcpy(parsebuf, readbuf);
    nbytes = (int)(delim - readbuf);
    readptr -= nbytes + 1;
    if (readptr > 0) {
      memmove((void *)readbuf, (void *)(delim + 1), readptr);
    }
    readbuf[readptr] = '\0';

    if (nbytes >= 4 && nbytes % 4 == 0) {
      for (i = 0; i < nbytes - 2; i++) {
        chksum ^= (*ptr++);
      }
      chkA = (uint8_t)(*ptr++);
      chkA = FROMHEX(chkA);
      chkB = (uint8_t)(*ptr++);
      chkB = FROMHEX(chkB);
      if (chksum != ((chkA << 4) | chkB)) {
        return nullptr;
      }
      length = (FROMHEX(parsebuf[0]) << 4) + FROMHEX(parsebuf[1]);
      if (length != nbytes) {
        return nullptr;
      }

      lastReadTime = msec;
      msg.length = nbytes = decode_base64((uint8_t *)&parsebuf[2], nbytes - 4, msg.data);
    } else {
      msg.length = nbytes = 0;
    }
  }

  // safety turn off command
  if ((msec - lastReadTime) >= kPiAliveTimeoutMs) {
    return STIMEOUT;
  } else if (nbytes == 0) {
    return nullptr;
  } else {
    return &msg;
  }
}

void HostSerial::writeBytes(void *data, int len) {
  uint32_t msec = millis();
  int nbytes, i;
  uint8_t chksum = 0, chkA = 0, chkB = 0, lenA = 0, lenB = 0;
  uint8_t *ptr = (uint8_t *)writebuf;

  if (len >= 0) {
    nbytes = encode_base64((uint8_t *)data, len, (uint8_t *)writebuf);
    for (i = 0; i < nbytes; i++) {
      chksum ^= (*ptr++);
    }
  } else {
    nbytes = 0;
  }

  nbytes += 5; // devid len1 len2 DATA chk1 chk2

  lenA = (uint8_t)((nbytes & 0xF0) >> 4);
  lenA = TOHEX(lenA);
  lenB = (uint8_t)(nbytes & 0x0F);
  lenB = TOHEX(lenB);

  chksum ^= (devid + '0') ^ lenA ^ lenB;

  chkA = (chksum & 0xF0) >> 4;
  chkA = TOHEX(chkA);
  chkB = (chksum & 0x0F);
  chkB = TOHEX(chkB);

  lastWriteTime = msec;
  ser.write(devid + '0');
  ser.write(lenA);
  ser.write(lenB);
  if (nbytes > 5) {
    ser.write(writebuf, nbytes - 5);
  }
  ser.write(chkA);
  ser.write(chkB);
  ser.write('\n');
  ser.flush();
}

void HostSerial::print(char data) { ser.print(data); }
void HostSerial::print(int data) { ser.print(data); }
void HostSerial::print(float data) { ser.print(data); }
void HostSerial::print(const char data[]) { ser.print(data); }
void HostSerial::println(char data) { ser.println(data); }
void HostSerial::println(int data) { ser.println(data); }
void HostSerial::println(float data) { ser.println(data); }
void HostSerial::println(const char data[]) { ser.println(data); }