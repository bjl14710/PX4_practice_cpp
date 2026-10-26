#include "modules/logger/logger.hpp"

#include "lib/mathlib/constants.hpp"

namespace px4p {

using math::rad_to_deg;

Logger::Logger(std::ostream& out, const sim::QuadrotorSim& sim)
    : ScheduledWorkItem("logger", kIntervalUs), out_(out), sim_(sim) {}

void Logger::Run() {
  if (!header_written_) {
    out_ << "t_s,armed,roll_deg,pitch_deg,yaw_deg,roll_sp_deg,pitch_sp_deg,yaw_sp_deg,"
            "true_roll_deg,true_pitch_deg,true_yaw_deg,altitude_m,thrust_sp,m1,m2,m3,m4\n";
    header_written_ = true;
  }

  attitude_sub_.update(attitude_);
  setpoint_sub_.update(setpoint_);
  status_sub_.update(status_);
  motors_sub_.update(motors_);

  const math::Vector3f est = attitude_.q.to_euler();
  const math::Vector3f sp = setpoint_.q_d.to_euler();
  const math::Vector3f truth = sim_.state().attitude.to_euler();
  const bool armed = status_.arming_state == msg::VehicleStatus::ArmingState::kArmed;

  out_ << us_to_s(hrt_absolute_time()) << ',' << (armed ? 1 : 0) << ',' << rad_to_deg(est.x)
       << ',' << rad_to_deg(est.y) << ',' << rad_to_deg(est.z) << ',' << rad_to_deg(sp.x) << ','
       << rad_to_deg(sp.y) << ',' << rad_to_deg(sp.z) << ',' << rad_to_deg(truth.x) << ','
       << rad_to_deg(truth.y) << ',' << rad_to_deg(truth.z) << ',' << sim_.altitude_m() << ','
       << setpoint_.thrust;
  for (float m : motors_.control) {
    out_ << ',' << m;
  }
  out_ << '\n';
}

}  // namespace px4p
