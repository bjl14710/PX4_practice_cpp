#pragma once

// commander: owns the vehicle state machine (PX4: src/modules/commander).
//
//   DISARMED --[ARM command && preflight checks pass]--> ARMED
//   ARMED    --[DISARM command]------------------------> DISARMED
//
// Preflight checks keep us from spinning motors when the estimate is bad.

#include <string_view>

#include "msg/sensor_imu.hpp"
#include "msg/vehicle_attitude.hpp"
#include "msg/vehicle_command.hpp"
#include "msg/vehicle_status.hpp"
#include "platform/work_queue.hpp"
#include "uorb/publication.hpp"
#include "uorb/subscription.hpp"

namespace px4p {

struct CommanderParams {
  hrt_abstime imu_timeout_us{100'000};  ///< IMU older than this = unhealthy
  float max_arm_tilt_rad{0.5236f};      ///< refuse to arm if tilted > 30 deg
};

class Commander final : public ScheduledWorkItem {
 public:
  static constexpr hrt_abstime kIntervalUs = 20'000;  // 50 Hz

  explicit Commander(const CommanderParams& params = {});

  void Run() override;

  msg::VehicleStatus::ArmingState arming_state() const { return status_.arming_state; }

  /// Why the last ARM command was refused (empty if it wasn't).
  std::string_view last_rejection() const { return last_rejection_; }

 private:
  /// Returns an empty string if it is safe to arm, otherwise the reason not to.
  std::string_view preflight_check() const;

  CommanderParams params_;
  uorb::Subscription<msg::VehicleCommand> command_sub_;
  uorb::Subscription<msg::SensorImu> imu_sub_;
  uorb::Subscription<msg::VehicleAttitude> attitude_sub_;
  uorb::Publication<msg::VehicleStatus> status_pub_;

  msg::VehicleStatus status_{};
  msg::SensorImu imu_{};
  msg::VehicleAttitude attitude_{};
  bool have_imu_{false};
  bool have_attitude_{false};
  std::string_view last_rejection_;
};

}  // namespace px4p
