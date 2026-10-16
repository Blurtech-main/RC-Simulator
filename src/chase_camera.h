#pragma once

#include "flight_math.h"
#include <cmath>

// A chase camera must not inherit every short-period yaw oscillation from the
// aeroplane.  Follow the horizontal flight path with a damped, rate-limited
// heading; pitch and roll remain visible on the aircraft itself.
inline float wrapAngle(float radians) {
  while (radians > PI) radians -= 2.f * PI;
  while (radians < -PI) radians += 2.f * PI;
  return radians;
}

inline float advanceChaseHeading(float current, float target, float dt,
                                 float timeConstant = 1.8f,
                                 float maxRate = 22.f * PI / 180.f) {
  const float error = wrapAngle(target - current);
  const float desiredStep = error * (1.f - std::exp(-dt / std::max(.01f, timeConstant)));
  return wrapAngle(current + clampf(desiredStep, -maxRate * dt, maxRate * dt));
}

inline Quat yawAttitude(float heading) {
  const float half = heading * .5f;
  return {std::cos(half), 0.f, std::sin(half), 0.f};
}

inline Quat relativeToYawCamera(Quat aircraft, float cameraHeading) {
  const Quat camera = yawAttitude(cameraHeading);
  return qnorm(qmul({camera.w, -camera.x, -camera.y, -camera.z}, aircraft));
}
