// Madgwick fusion for MPU-style IMU: 6-DOF (IMU) and 9-DOF (MARG).
// IMU: S. O. H. Madgwick, 2010. MARG equations from common open implementation
// (e.g. MadgwickAHRS / x-io), gyro in rad/s, accel in g, mag any consistent unit
// (normalized internally). Quaternion w,x,y,z = q0,q1,q2,q3.

#ifndef MADGWICK_H
#define MADGWICK_H

#include <Arduino.h>
#include <math.h>

class Madgwick {
 public:
  explicit Madgwick(float beta = 0.08f) : beta_(beta), q0_(1.f), q1_(0.f), q2_(0.f), q3_(0.f) {}

  void setBeta(float b) { beta_ = b; }
  float beta() const { return beta_; }

  // Gyro rad/s, accel in g, dt seconds
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);

  // Same + magnetometer (µT, LSB, etc. OK — vector is normalized). If |m|==0, uses IMU only.
  void updateMARG(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my,
                  float mz, float dt);

  float q0() const { return q0_; }
  float q1() const { return q1_; }
  float q2() const { return q2_; }
  float q3() const { return q3_; }

 private:
  static float invSqrt(float x) { return 1.0f / sqrtf(x); }

  float beta_;
  float q0_, q1_, q2_, q3_;
};

inline void Madgwick::updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  if (dt <= 0.f) {
    return;
  }

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2;
  float _8q1, _8q2;
  float q0q0, q1q1, q2q2, q3q3;

  qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
  qDot2 = 0.5f * (q0_ * gx + q2_ * gz - q3_ * gy);
  qDot3 = 0.5f * (q0_ * gy - q1_ * gz + q3_ * gx);
  qDot4 = 0.5f * (q0_ * gz + q1_ * gy - q2_ * gx);

  if (!((ax == 0.f) && (ay == 0.f) && (az == 0.f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    _2q0 = 2.0f * q0_;
    _2q1 = 2.0f * q1_;
    _2q2 = 2.0f * q2_;
    _2q3 = 2.0f * q3_;
    _4q0 = 4.0f * q0_;
    _4q1 = 4.0f * q1_;
    _4q2 = 4.0f * q2_;
    _8q1 = 8.0f * q1_;
    _8q2 = 8.0f * q2_;
    q0q0 = q0_ * q0_;
    q1q1 = q1_ * q1_;
    q2q2 = q2_ * q2_;
    q3q3 = q3_ * q3_;

    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_ - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2_ + _2q0 * ax + _4q2 * q1q1 - _2q1 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3_ - _2q1 * ax + 4.0f * q2q2 * q3_ - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    qDot1 -= beta_ * s0;
    qDot2 -= beta_ * s1;
    qDot3 -= beta_ * s2;
    qDot4 -= beta_ * s3;
  }

  q0_ += qDot1 * dt;
  q1_ += qDot2 * dt;
  q2_ += qDot3 * dt;
  q3_ += qDot4 * dt;

  recipNorm = invSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  q0_ *= recipNorm;
  q1_ *= recipNorm;
  q2_ *= recipNorm;
  q3_ *= recipNorm;
}

inline void Madgwick::updateMARG(float gx, float gy, float gz, float ax, float ay, float az, float mx,
                                 float my, float mz, float dt) {
  if (dt <= 0.f) {
    return;
  }

  if ((mx == 0.f) && (my == 0.f) && (mz == 0.f)) {
    updateIMU(gx, gy, gz, ax, ay, az, dt);
    return;
  }

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx;
  float _2bx, _2bz, _4bx, _4bz;
  float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
  qDot2 = 0.5f * (q0_ * gx + q2_ * gz - q3_ * gy);
  qDot3 = 0.5f * (q0_ * gy - q1_ * gz + q3_ * gx);
  qDot4 = 0.5f * (q0_ * gz + q1_ * gy - q2_ * gx);

  if (!((ax == 0.f) && (ay == 0.f) && (az == 0.f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    if (!(recipNorm > 0.f)) {
      updateIMU(gx, gy, gz, ax, ay, az, dt);
      return;
    }
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    _2q0mx = 2.0f * q0_ * mx;
    _2q0my = 2.0f * q0_ * my;
    _2q0mz = 2.0f * q0_ * mz;
    _2q1mx = 2.0f * q1_ * mx;
    _2q0 = 2.0f * q0_;
    _2q1 = 2.0f * q1_;
    _2q2 = 2.0f * q2_;
    _2q3 = 2.0f * q3_;
    _2q0q2 = 2.0f * q0_ * q2_;
    _2q2q3 = 2.0f * q2_ * q3_;
    q0q0 = q0_ * q0_;
    q0q1 = q0_ * q1_;
    q0q2 = q0_ * q2_;
    q0q3 = q0_ * q3_;
    q1q1 = q1_ * q1_;
    q1q2 = q1_ * q2_;
    q1q3 = q1_ * q3_;
    q2q2 = q2_ * q2_;
    q2q3 = q2_ * q3_;
    q3q3 = q3_ * q3_;

    hx = mx * q0q0 - _2q0my * q3_ + _2q0mz * q2_ + mx * q1q1 + _2q1 * my * q2_ + _2q1 * mz * q3_ - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3_ + my * q0q0 - _2q0mz * q1_ + _2q1mx * q2_ - my * q1q1 + my * q2q2 + _2q2 * mz * q3_ - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2_ + _2q0my * q1_ + mz * q0q0 + _2q1mx * q3_ - mz * q1q1 + _2q2 * my * q3_ - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    if (_2bx < 1e-6f) {
      updateIMU(gx, gy, gz, ax, ay, az, dt);
      return;
    }

    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
         _2bz * q2_ * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q3_ + _2bz * q1_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q2_ * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * q1_ * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         _2bz * q3_ * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q2_ + _2bz * q0_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q3_ - 4.0f * _2bz * q1_) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * q2_ * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         (-_4bx * q2_ - _2bz * q0_) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q1_ + _2bz * q3_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q0_ - 4.0f * _2bz * q2_) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
         (-_4bx * q3_ + _2bz * q1_) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q0_ + _2bz * q2_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q1_ * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    qDot1 -= beta_ * s0;
    qDot2 -= beta_ * s1;
    qDot3 -= beta_ * s2;
    qDot4 -= beta_ * s3;
  }

  q0_ += qDot1 * dt;
  q1_ += qDot2 * dt;
  q2_ += qDot3 * dt;
  q3_ += qDot4 * dt;

  recipNorm = invSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  q0_ *= recipNorm;
  q1_ *= recipNorm;
  q2_ *= recipNorm;
  q3_ *= recipNorm;
}

#endif
