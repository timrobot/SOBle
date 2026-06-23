#include "IMUCircleQueue.h"

#include <math.h>

IMUCircleQueue::IMUCircleQueue() { reset(); }

void IMUCircleQueue::reset() {
  count = 0;
  start = 0;
  end = 0;
  maxlen = kCapacity;
}

bool IMUCircleQueue::isFull() const { return count >= maxlen; }

void IMUCircleQueue::push(float w, float x, float y, float z) {
  buf[end][0] = w;
  buf[end][1] = x;
  buf[end][2] = y;
  buf[end][3] = z;
  end = (uint8_t)((end + 1) % maxlen);
  if (count < maxlen) {
    count++;
  } else {
    start = (uint8_t)((start + 1) % maxlen);
  }
}

bool IMUCircleQueue::averageQuat(float out[4]) const {
  if (count == 0) {
    return false;
  }

  const float *first = buf[start];
  float sum[4] = {0.f, 0.f, 0.f, 0.f};
  for (uint8_t i = 0; i < count; i++) {
    const uint8_t idx = (uint8_t)((start + i) % maxlen);
    float q[4] = {buf[idx][0], buf[idx][1], buf[idx][2], buf[idx][3]};
    const float dot =
        q[0] * first[0] + q[1] * first[1] + q[2] * first[2] + q[3] * first[3];
    if (dot < 0.f) {
      q[0] = -q[0];
      q[1] = -q[1];
      q[2] = -q[2];
      q[3] = -q[3];
    }
    sum[0] += q[0];
    sum[1] += q[1];
    sum[2] += q[2];
    sum[3] += q[3];
  }

  const float invN = 1.f / (float)count;
  out[0] = sum[0] * invN;
  out[1] = sum[1] * invN;
  out[2] = sum[2] * invN;
  out[3] = sum[3] * invN;

  const float norm = sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] +
                           out[3] * out[3]);
  if (norm < 1e-9f) {
    out[0] = first[0];
    out[1] = first[1];
    out[2] = first[2];
    out[3] = first[3];
    return true;
  }
  out[0] /= norm;
  out[1] /= norm;
  out[2] /= norm;
  out[3] /= norm;
  return true;
}

namespace {
constexpr float kWeightedQuatWeights[IMUCircleQueue::kWeightedWindow] = {
    0.01f, 0.02f, 0.04f, 0.05f, 0.08f, 0.11f, 0.15f, 0.17f, 0.18f, 0.19f,
};
}  // namespace

bool IMUCircleQueue::weightedAverageQuat(float out[4]) const {
  if (count < kWeightedWindow) {
    return false;
  }

  float quats[kWeightedWindow][4];
  for (uint8_t i = 0; i < kWeightedWindow; i++) {
    const uint8_t idx =
        (uint8_t)((end + maxlen - kWeightedWindow + i) % maxlen);
    quats[i][0] = buf[idx][0];
    quats[i][1] = buf[idx][1];
    quats[i][2] = buf[idx][2];
    quats[i][3] = buf[idx][3];
  }

  const float *newest = quats[kWeightedWindow - 1];
  float sum[4] = {0.f, 0.f, 0.f, 0.f};
  for (uint8_t i = 0; i < kWeightedWindow; i++) {
    float q[4] = {quats[i][0], quats[i][1], quats[i][2], quats[i][3]};
    const float dot =
        q[0] * newest[0] + q[1] * newest[1] + q[2] * newest[2] + q[3] * newest[3];
    if (dot < 0.f) {
      q[0] = -q[0];
      q[1] = -q[1];
      q[2] = -q[2];
      q[3] = -q[3];
    }
    const float w = kWeightedQuatWeights[i];
    sum[0] += q[0] * w;
    sum[1] += q[1] * w;
    sum[2] += q[2] * w;
    sum[3] += q[3] * w;
  }

  const float norm = sqrtf(sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2] +
                           sum[3] * sum[3]);
  if (norm < 1e-9f) {
    out[0] = newest[0];
    out[1] = newest[1];
    out[2] = newest[2];
    out[3] = newest[3];
    return true;
  }
  out[0] = sum[0] / norm;
  out[1] = sum[1] / norm;
  out[2] = sum[2] / norm;
  out[3] = sum[3] / norm;
  return true;
}
