#include "modules/mc_att_control/mc_att_control.hpp"

#include <algorithm>

namespace px4p {

using math::Quatf;
using math::Vector3f;

Vector3f attitude_control_update(const Quatf& q, const Quatf& q_d,
                                 const AttitudeControlParams& params) {
  // Error rotation expressed in the body frame: "from where I am to where I
  // want to be". canonical() picks the shorter of the two ways around.
  const Quatf q_err = (q.conjugate() * q_d).canonical();

  // For a rotation of angle a about unit axis u, imag() = u * sin(a/2), so
  // 2 * imag() ~= u * a: a rotation vector, i.e. a per-axis angle error.
  const Vector3f angle_error = q_err.imag() * 2.f;

  Vector3f rates = angle_error.emult(params.gain);
  for (int i = 0; i < 3; ++i) {
    rates[i] = std::clamp(rates[i], -params.rate_limit_rad_s[i], params.rate_limit_rad_s[i]);
  }
  return rates;
}

McAttControl::McAttControl(const AttitudeControlParams& params)
    : ScheduledWorkItem("mc_att_control", kIntervalUs), params_(params) {}

void McAttControl::Run() {
  msg::VehicleAttitude attitude{};
  if (!attitude_sub_.update(attitude)) {
    return;
  }
  setpoint_sub_.update(setpoint_);

  msg::VehicleRatesSetpoint rates_sp{};
  rates_sp.timestamp = hrt_absolute_time();
  rates_sp.rates = attitude_control_update(attitude.q, setpoint_.q_d, params_);
  rates_sp.thrust = setpoint_.thrust;
  rates_sp_pub_.publish(rates_sp);
}

}  // namespace px4p
