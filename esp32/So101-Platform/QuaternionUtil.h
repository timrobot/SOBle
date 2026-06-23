#ifndef QUATERNIONUTIL_H
#define QUATERNIONUTIL_H

#include <Arduino.h>
#include <stdint.h>

// Delta quaternion that rotates q0 into q1: inv(q0) * q1 (w, x, y, z).
void quat_diff(const float q0[4], const float q1[4], float out[4]);

// Unit quaternion component → milli-units for BLE RobotState packing.
int16_t quat_unit_to_milli(float q);

#endif
