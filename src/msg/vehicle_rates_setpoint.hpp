#pragma once

#include <cstdint>

#include "lib/mathlib/vector3.hpp"

namespace px4p::msg {

/// Desired body angular rates (output of mc_att_control, input to mc_rate_control).
struct VehicleRatesSetpoint {
  uint64_t timestamp{0};   ///< [us]
  math::Vector3f rates;    ///< roll, pitch, yaw rate [rad/s]
  float thrust{0.f};       ///< normalized collective thrust [0, 1]
};

}  // namespace px4p::msg
