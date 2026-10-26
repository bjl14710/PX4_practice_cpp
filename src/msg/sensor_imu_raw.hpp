#pragma once

#include <cstdint>

#include "lib/mathlib/vector3.hpp"

namespace px4p::msg {

/// Raw IMU sample straight from the (simulated) sensor, body frame FRD.
/// PX4 splits this into sensor_accel and sensor_gyro.
struct SensorImuRaw {
  uint64_t timestamp{0};          ///< [us]
  math::Vector3f accel_m_s2;      ///< specific force; reads (0, 0, -9.81) at rest
  math::Vector3f gyro_rad_s;      ///< angular rate
};

}  // namespace px4p::msg
