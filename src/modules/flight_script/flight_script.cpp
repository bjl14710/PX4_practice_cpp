#include "modules/flight_script/flight_script.hpp"

#include <utility>

#include "lib/mathlib/constants.hpp"

namespace px4p {

using math::deg_to_rad;
using Type = ScriptEvent::Type;

FlightScript::FlightScript(std::vector<ScriptEvent> events)
    : ScheduledWorkItem("flight_script", kIntervalUs), events_(std::move(events)) {}

void FlightScript::Run() {
  const hrt_abstime now = hrt_absolute_time();

  while (!finished() && us_to_s(now) >= events_[next_event_].time_s) {
    const ScriptEvent& ev = events_[next_event_++];
    if (ev.type == Type::kAttitude) {
      setpoint_.q_d = math::Quatf::from_euler(deg_to_rad(ev.roll_deg), deg_to_rad(ev.pitch_deg),
                                              deg_to_rad(ev.yaw_deg));
      setpoint_.thrust = ev.thrust;
    } else {
      msg::VehicleCommand cmd{};
      cmd.timestamp = now;
      cmd.command = ev.type == Type::kArm ? msg::VehicleCommand::Command::kArm
                                          : msg::VehicleCommand::Command::kDisarm;
      command_pub_.publish(cmd);
    }
  }

  setpoint_.timestamp = now;
  setpoint_pub_.publish(setpoint_);
}

std::vector<ScriptEvent> demo_flight(float hover_thrust) {
  const float h = hover_thrust;
  // There's no altitude controller yet (that's exercise 1!), so altitude is
  // flown "open loop": a bit above hover to climb, a bit below to stop.
  return {
      {0.5f, Type::kArm},
      {1.0f, Type::kAttitude, 0.f, 0.f, 0.f, h * 1.15f},     // climb
      {2.0f, Type::kAttitude, 0.f, 0.f, 0.f, h * 0.87f},     // slow the climb
      {3.0f, Type::kAttitude, 0.f, 0.f, 0.f, h},             // hover
      {4.0f, Type::kAttitude, 10.f, 0.f, 0.f, h * 1.016f},   // roll right 10 deg
      {6.0f, Type::kAttitude, 0.f, 0.f, 0.f, h},             // level
      {7.0f, Type::kAttitude, 0.f, -10.f, 0.f, h * 1.016f},  // pitch nose-down 10 deg
      {9.0f, Type::kAttitude, 0.f, 0.f, 0.f, h},             // level
      {10.0f, Type::kAttitude, 0.f, 0.f, 90.f, h},           // yaw to 90 deg
      {13.0f, Type::kAttitude, 0.f, 0.f, 90.f, h * 0.9f},    // descend
      {17.0f, Type::kDisarm},
  };
}

}  // namespace px4p
