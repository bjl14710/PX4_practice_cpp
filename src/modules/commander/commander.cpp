#include "modules/commander/commander.hpp"

#include <cmath>

namespace px4p {

using ArmingState = msg::VehicleStatus::ArmingState;
using Command = msg::VehicleCommand::Command;

Commander::Commander(const CommanderParams& params)
    : ScheduledWorkItem("commander", kIntervalUs), params_(params) {}

std::string_view Commander::preflight_check() const {
  const hrt_abstime now = hrt_absolute_time();
  if (!have_imu_ || now - imu_.timestamp > params_.imu_timeout_us) {
    return "IMU data missing or stale";
  }
  if (!have_attitude_) {
    return "no attitude estimate";
  }
  // The body z axis expressed in the world frame; its z component is cos(tilt).
  const float cos_tilt = attitude_.q.rotate({0.f, 0.f, 1.f}).z;
  if (cos_tilt < std::cos(params_.max_arm_tilt_rad)) {
    return "vehicle tilted too far";
  }
  return {};
}

void Commander::Run() {
  if (imu_sub_.update(imu_)) {
    have_imu_ = true;
  }
  if (attitude_sub_.update(attitude_)) {
    have_attitude_ = true;
  }

  msg::VehicleCommand cmd{};
  if (command_sub_.update(cmd)) {
    if (cmd.command == Command::kArm && status_.arming_state == ArmingState::kDisarmed) {
      last_rejection_ = preflight_check();
      if (last_rejection_.empty()) {
        status_.arming_state = ArmingState::kArmed;
      }
    } else if (cmd.command == Command::kDisarm) {
      status_.arming_state = ArmingState::kDisarmed;
    }
  }

  // Publish every cycle, not just on change, so late subscribers catch up.
  status_.timestamp = hrt_absolute_time();
  status_pub_.publish(status_);
}

}  // namespace px4p
