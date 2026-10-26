#include "modules/sensors/sensors.hpp"

namespace px4p {

namespace {
constexpr float kSampleHz = 1e6f / static_cast<float>(Sensors::kIntervalUs);
}  // namespace

Sensors::Sensors(const SensorsParams& params)
    : ScheduledWorkItem("sensors", kIntervalUs),
      gyro_filter_(kSampleHz, params.gyro_cutoff_hz),
      accel_filter_(kSampleHz, params.accel_cutoff_hz) {}

void Sensors::Run() {
  msg::SensorImuRaw raw{};
  if (!raw_sub_.update(raw)) {
    return;
  }

  msg::SensorImu imu{};
  imu.timestamp = raw.timestamp;
  imu.gyro_rad_s = gyro_filter_.apply(raw.gyro_rad_s);
  imu.accel_m_s2 = accel_filter_.apply(raw.accel_m_s2);
  imu_pub_.publish(imu);
}

}  // namespace px4p
