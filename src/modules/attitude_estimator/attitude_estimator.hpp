#pragma once

// attitude_estimator: complementary filter on gyro + accelerometer.
// PX4 used a filter like this (attitude_estimator_q) before EKF2 took over.
//
// Gyros are smooth but drift; the accelerometer knows which way is down but
// is noisy and fooled by acceleration. So: integrate the gyro, and nudge the
// estimate toward the accelerometer's "down" a little every step.
// There is no magnetometer, so yaw is gyro-only and slowly drifts.

#include "lib/mathlib/quaternion.hpp"
#include "msg/sensor_imu.hpp"
#include "msg/vehicle_attitude.hpp"
#include "platform/work_queue.hpp"
#include "uorb/publication.hpp"
#include "uorb/subscription.hpp"

namespace px4p {

struct AttitudeEstimatorParams {
  float accel_gain{0.2f};            ///< pull toward the accel's "down" (PX4: ATT_W_ACC)
  float gyro_bias_gain{0.1f};        ///< how fast gyro bias is learned (PX4: ATT_W_GYRO_BIAS)
  float accel_trust_band_m_s2{2.0f}; ///< ignore accel if |a| differs from g by more than this
};

class AttitudeEstimator final : public ScheduledWorkItem {
 public:
  static constexpr hrt_abstime kIntervalUs = 4000;  // 250 Hz

  explicit AttitudeEstimator(const AttitudeEstimatorParams& params = {});

  void Run() override;

  const math::Vector3f& gyro_bias() const { return gyro_bias_; }

 private:
  void initialize_from_accel(const math::Vector3f& accel);

  AttitudeEstimatorParams params_;
  uorb::Subscription<msg::SensorImu> imu_sub_;
  uorb::Publication<msg::VehicleAttitude> attitude_pub_;

  math::Quatf q_;
  math::Vector3f gyro_bias_;
  hrt_abstime last_imu_timestamp_{0};
  bool initialized_{false};
};

}  // namespace px4p
