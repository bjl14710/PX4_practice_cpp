#include "modules/mc_rate_control/mc_rate_control.hpp"

namespace px4p {

McRateControl::McRateControl(const RateControlParams& params)
    : ScheduledWorkItem("mc_rate_control", kIntervalUs),
      params_(params),
      pids_{math::Pid{params.roll}, math::Pid{params.pitch}, math::Pid{params.yaw}} {}

void McRateControl::Run() {
  msg::SensorImu imu{};
  if (!imu_sub_.update(imu)) {
    return;
  }
  rates_sp_sub_.update(rates_sp_);
  status_sub_.update(status_);

  const float dt = last_imu_timestamp_ == 0 ? 0.f : us_to_s(imu.timestamp - last_imu_timestamp_);
  last_imu_timestamp_ = imu.timestamp;

  msg::VehicleTorqueSetpoint torque_sp{};
  torque_sp.timestamp = hrt_absolute_time();

  if (status_.arming_state != msg::VehicleStatus::ArmingState::kArmed) {
    for (math::Pid& pid : pids_) {
      pid.reset();
    }
    torque_sp_pub_.publish(torque_sp);  // zero torque, zero thrust
    return;
  }

  const bool integrate = rates_sp_.thrust >= params_.min_thrust_for_integration;
  for (int i = 0; i < 3; ++i) {
    torque_sp.torque[i] =
        pids_[static_cast<size_t>(i)].update(rates_sp_.rates[i], imu.gyro_rad_s[i], dt, integrate);
  }
  torque_sp.thrust = rates_sp_.thrust;
  torque_sp_pub_.publish(torque_sp);
}

}  // namespace px4p
