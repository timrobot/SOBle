#ifndef IMUCIRCLEQUEUE_H
#define IMUCIRCLEQUEUE_H

#include <Arduino.h>

class IMUCircleQueue {
public:
  static constexpr uint8_t kCapacity = 32;
  static constexpr uint8_t kWeightedWindow = 10;

  IMUCircleQueue();

  void reset();
  bool isFull() const;
  void push(float w, float x, float y, float z);
  bool averageQuat(float out[4]) const;
  bool weightedAverageQuat(float out[4]) const;

private:
  float buf[kCapacity][4];
  uint8_t count;
  uint8_t maxlen;
  uint8_t start;
  uint8_t end;
};

#endif
