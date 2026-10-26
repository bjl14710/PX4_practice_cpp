#pragma once

#include <cstdint>

#include "lib/mathlib/vector3.hpp"

namespace px4p::msg {

/// Filtered IMU data published by the sensors module, body frame FRD.
/// PX4 equivalents: sensor_combined / vehicle_angular_velocity.
struct SensorImu {
  uint64_t timestamp{0};          ///< [us]
  math::Vector3f accel_m_s2;
  math::Vector3f gyro_rad_s;
};

}  // namespace px4p::msg
