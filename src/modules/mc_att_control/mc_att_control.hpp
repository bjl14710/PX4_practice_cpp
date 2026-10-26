#pragma once

// mc_att_control: attitude error -> desired body rates (P controller).
// PX4: src/modules/mc_att_control/AttitudeControl.

#include "lib/mathlib/vector3.hpp"
#include "msg/vehicle_attitude.hpp"
#include "msg/vehicle_attitude_setpoint.hpp"
#include "msg/vehicle_rates_setpoint.hpp"
#include "platform/work_queue.hpp"
#include "uorb/publication.hpp"
#include "uorb/subscription.hpp"

namespace px4p {

struct AttitudeControlParams {
  math::Vector3f gain{6.5f, 6.5f, 2.8f};          ///< PX4: MC_ROLL_P, MC_PITCH_P, MC_YAW_P
  math::Vector3f rate_limit_rad_s{3.8f, 3.8f, 3.5f}; ///< PX4: MC_*RATE_MAX
};

/// Pure control law, kept separate from the module so it can be unit tested.
math::Vector3f attitude_control_update(const math::Quatf& q, const math::Quatf& q_d,
                                       const AttitudeControlParams& params);

class McAttControl final : public ScheduledWorkItem {
 public:
  static constexpr hrt_abstime kIntervalUs = 4000;  // 250 Hz, same as the estimator

  explicit McAttControl(const AttitudeControlParams& params = {});

  void Run() override;

 private:
  AttitudeControlParams params_;
  uorb::Subscription<msg::VehicleAttitude> attitude_sub_;
  uorb::Subscription<msg::VehicleAttitudeSetpoint> setpoint_sub_;
  uorb::Publication<msg::VehicleRatesSetpoint> rates_sp_pub_;
  msg::VehicleAttitudeSetpoint setpoint_{};  ///< level, zero thrust until told otherwise
};

}  // namespace px4p
