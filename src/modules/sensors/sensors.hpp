#pragma once

// sensors: turns raw IMU samples into filtered ones (PX4: src/modules/sensors).

#include "lib/mathlib/low_pass_filter.hpp"
#include "msg/sensor_imu.hpp"
#include "msg/sensor_imu_raw.hpp"
#include "platform/work_queue.hpp"
#include "uorb/publication.hpp"
#include "uorb/subscription.hpp"

namespace px4p {

struct SensorsParams {
  float gyro_cutoff_hz{80.f};   ///< PX4: IMU_GYRO_CUTOFF
  float accel_cutoff_hz{30.f};  ///< PX4: IMU_ACCEL_CUTOFF
};

class Sensors final : public ScheduledWorkItem {
 public:
  static constexpr hrt_abstime kIntervalUs = 1000;  // match the IMU rate

  explicit Sensors(const SensorsParams& params = {});

  void Run() override;

 private:
  uorb::Subscription<msg::SensorImuRaw> raw_sub_;
  uorb::Publication<msg::SensorImu> imu_pub_;
  math::LowPassFilter<math::Vector3f> gyro_filter_;
  math::LowPassFilter<math::Vector3f> accel_filter_;
};

}  // namespace px4p
