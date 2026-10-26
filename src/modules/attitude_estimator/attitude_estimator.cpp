#include "modules/attitude_estimator/attitude_estimator.hpp"

#include <cmath>

#include "lib/mathlib/constants.hpp"

namespace px4p {

using math::Quatf;
using math::Vector3f;

AttitudeEstimator::AttitudeEstimator(const AttitudeEstimatorParams& params)
    : ScheduledWorkItem("attitude_estimator", kIntervalUs), params_(params) {}

void AttitudeEstimator::initialize_from_accel(const Vector3f& accel) {
  // At rest the accelerometer reads -g along the body's "down" direction,
  // which gives roll and pitch directly. Yaw is unobservable, start at 0.
  const float roll = std::atan2(-accel.y, -accel.z);
  const float pitch = std::atan2(accel.x, std::sqrt(accel.y * accel.y + accel.z * accel.z));
  q_ = Quatf::from_euler(roll, pitch, 0.f);
}

void AttitudeEstimator::Run() {
  msg::SensorImu imu{};
  if (!imu_sub_.update(imu)) {
    return;
  }

  if (!initialized_) {
    initialize_from_accel(imu.accel_m_s2);
    last_imu_timestamp_ = imu.timestamp;
    initialized_ = true;
  } else {
    const float dt = us_to_s(imu.timestamp - last_imu_timestamp_);
    last_imu_timestamp_ = imu.timestamp;
    if (dt <= 0.f || dt > 0.1f) {
      return;  // stale or out-of-order data: don't integrate a huge step
    }

    Vector3f omega = imu.gyro_rad_s - gyro_bias_;

    // Only trust the accelerometer when it measures roughly 1 g; otherwise
    // the vehicle is accelerating and "down" is not where the accel says.
    const float accel_norm = imu.accel_m_s2.norm();
    if (std::fabs(accel_norm - math::kGravity) < params_.accel_trust_band_m_s2) {
      const Vector3f down_measured = (-imu.accel_m_s2).normalized();
      const Vector3f down_predicted = q_.conjugate().rotate({0.f, 0.f, 1.f});
      // The cross product is the rotation axis that turns the prediction
      // toward the measurement; its length grows with the angle between them.
      const Vector3f error = down_measured.cross(down_predicted);
      omega += error * params_.accel_gain;
      gyro_bias_ -= error * (params_.gyro_bias_gain * dt);
    }

    q_ = q_.integrate(omega, dt);
  }

  msg::VehicleAttitude att{};
  att.timestamp = imu.timestamp;
  att.q = q_;
  attitude_pub_.publish(att);
}

}  // namespace px4p
