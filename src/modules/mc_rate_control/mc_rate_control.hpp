#pragma once

// mc_rate_control: rate error -> normalized torque (one PID per axis).
// PX4: src/modules/mc_rate_control. This is the innermost, fastest loop.

#include <array>

#include "lib/mathlib/pid.hpp"
#include "msg/sensor_imu.hpp"
#include "msg/vehicle_rates_setpoint.hpp"
#include "msg/vehicle_status.hpp"
#include "msg/vehicle_torque_setpoint.hpp"
#include "platform/work_queue.hpp"
#include "uorb/publication.hpp"
#include "uorb/subscription.hpp"

namespace px4p {

struct RateControlParams {
  // kp, ki, kd, integrator_limit, output_limit (PX4: MC_ROLLRATE_P/I/D, ...)
  math::PidGains roll{0.15f, 0.2f, 0.003f, 0.3f, 1.f};
  math::PidGains pitch{0.15f, 0.2f, 0.003f, 0.3f, 1.f};
  math::PidGains yaw{0.3f, 0.1f, 0.f, 0.3f, 1.f};
  /// Below this thrust we assume we're on the ground and freeze the
  /// integrators, otherwise they wind up while the vehicle can't rotate.
  float min_thrust_for_integration{0.15f};
};

class McRateControl final : public ScheduledWorkItem {
 public:
  static constexpr hrt_abstime kIntervalUs = 2000;  // 500 Hz

  explicit McRateControl(const RateControlParams& params = {});

  void Run() override;

 private:
  RateControlParams params_;
  std::array<math::Pid, 3> pids_;

  uorb::Subscription<msg::SensorImu> imu_sub_;
  uorb::Subscription<msg::VehicleRatesSetpoint> rates_sp_sub_;
  uorb::Subscription<msg::VehicleStatus> status_sub_;
  uorb::Publication<msg::VehicleTorqueSetpoint> torque_sp_pub_;

  msg::VehicleRatesSetpoint rates_sp_{};
  msg::VehicleStatus status_{};
  hrt_abstime last_imu_timestamp_{0};
};

}  // namespace px4p
