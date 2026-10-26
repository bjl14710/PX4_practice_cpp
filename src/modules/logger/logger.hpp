#pragma once

// logger: writes a CSV row per cycle (a much simpler cousin of PX4's ULog
// logger). Open the CSV in a spreadsheet or any plotting tool.

#include <ostream>

#include "msg/actuator_motors.hpp"
#include "msg/vehicle_attitude.hpp"
#include "msg/vehicle_attitude_setpoint.hpp"
#include "msg/vehicle_status.hpp"
#include "platform/work_queue.hpp"
#include "sim/quadrotor_sim.hpp"
#include "uorb/subscription.hpp"

namespace px4p {

class Logger final : public ScheduledWorkItem {
 public:
  static constexpr hrt_abstime kIntervalUs = 20'000;  // 50 Hz

  /// Ground truth comes straight from the sim: a real logger can't do that,
  /// but it's how we check the estimator.
  Logger(std::ostream& out, const sim::QuadrotorSim& sim);

  void Run() override;

 private:
  std::ostream& out_;
  const sim::QuadrotorSim& sim_;
  bool header_written_{false};

  uorb::Subscription<msg::VehicleAttitude> attitude_sub_;
  uorb::Subscription<msg::VehicleAttitudeSetpoint> setpoint_sub_;
  uorb::Subscription<msg::VehicleStatus> status_sub_;
  uorb::Subscription<msg::ActuatorMotors> motors_sub_;

  msg::VehicleAttitude attitude_{};
  msg::VehicleAttitudeSetpoint setpoint_{};
  msg::VehicleStatus status_{};
  msg::ActuatorMotors motors_{};
};

}  // namespace px4p
