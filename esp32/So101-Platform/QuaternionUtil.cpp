#include "QuaternionUtil.h"

#include <math.h>

void quat_diff(const float q0[4], const float q1[4], float out[4]) {
  const float inv[4] = {q0[0], -q0[1], -q0[2], -q0[3]};
  const float w1 = inv[0];
  const float x1 = inv[1];
  const float y1 = inv[2];
  const float z1 = inv[3];
  const float w2 = q1[0];
  const float x2 = q1[1];
  const float y2 = q1[2];
  const float z2 = q1[3];
  out[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
  out[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
  out[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
  out[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}

int16_t quat_unit_to_milli(float q) {
  return constrain((long)lroundf(q * 1000.0f), -32768, 32767);
}
