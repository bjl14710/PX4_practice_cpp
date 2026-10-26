#pragma once

#include <cstdint>

#include "lib/mathlib/quaternion.hpp"

namespace px4p::msg {

/// Desired attitude and collective thrust (input to mc_att_control).
struct VehicleAttitudeSetpoint {
  uint64_t timestamp{0};  ///< [us]
  math::Quatf q_d;        ///< desired rotation body -> world
  float thrust{0.f};      ///< normalized collective thrust [0, 1]
};

}  // namespace px4p::msg
