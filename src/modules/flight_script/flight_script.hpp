#pragma once

// flight_script: a scripted "pilot". Plays back a list of timed events that
// arm/disarm the vehicle and change the attitude setpoint. It stands in for
// an RC transmitter or a ground station until a real flight mode exists.

#include <vector>

#include "msg/vehicle_attitude_setpoint.hpp"
#include "msg/vehicle_command.hpp"
#include "platform/work_queue.hpp"
#include "uorb/publication.hpp"

namespace px4p {

struct ScriptEvent {
  enum class Type { kArm, kDisarm, kAttitude };

  float time_s{0.f};
  Type type{Type::kAttitude};
  // Used only by kAttitude:
  float roll_deg{0.f};
  float pitch_deg{0.f};
  float yaw_deg{0.f};
  float thrust{0.f};
};

class FlightScript final : public ScheduledWorkItem {
 public:
  static constexpr hrt_abstime kIntervalUs = 20'000;  // 50 Hz, like an RC link

  /// Events must be sorted by time_s.
  explicit FlightScript(std::vector<ScriptEvent> events);

  void Run() override;

  bool finished() const { return next_event_ >= events_.size(); }

 private:
  std::vector<ScriptEvent> events_;
  size_t next_event_{0};
  msg::VehicleAttitudeSetpoint setpoint_{};

  uorb::Publication<msg::VehicleCommand> command_pub_;
  uorb::Publication<msg::VehicleAttitudeSetpoint> setpoint_pub_;
};

/// The demo flight used by px4p_sitl: take off, roll, pitch, yaw, land.
std::vector<ScriptEvent> demo_flight(float hover_thrust);

}  // namespace px4p
