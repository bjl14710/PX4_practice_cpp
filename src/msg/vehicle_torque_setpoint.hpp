#pragma once

#include <cstdint>

#include "lib/mathlib/vector3.hpp"

namespace px4p::msg {

/// Normalized torque + thrust demand (output of mc_rate_control).
/// PX4 publishes torque and thrust as two topics; one is enough here.
struct VehicleTorqueSetpoint {
  uint64_t timestamp{0};   ///< [us]
  math::Vector3f torque;   ///< normalized roll, pitch, yaw torque [-1, 1]
  float thrust{0.f};       ///< normalized collective thrust [0, 1]
};

}  // namespace px4p::msg
