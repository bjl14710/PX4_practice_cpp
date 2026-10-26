#pragma once

namespace px4p::math {

constexpr float kGravity = 9.80665f;  ///< standard gravity [m/s^2]
constexpr float kPi = 3.14159265f;

constexpr float deg_to_rad(float deg) { return deg * kPi / 180.f; }
constexpr float rad_to_deg(float rad) { return rad * 180.f / kPi; }

}  // namespace px4p::math
