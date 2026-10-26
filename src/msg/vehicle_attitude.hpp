#pragma once

#include <cstdint>

#include "lib/mathlib/quaternion.hpp"

namespace px4p::msg {

/// Estimated attitude, published by the attitude estimator.
struct VehicleAttitude {
  uint64_t timestamp{0};  ///< [us]
  math::Quatf q;          ///< rotation body FRD -> world NED
};

}  // namespace px4p::msg
