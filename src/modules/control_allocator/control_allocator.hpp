#pragma once

// control_allocator: torque + thrust -> individual motor commands (the "mixer").
// PX4: src/modules/control_allocator. Older PX4 used mixer files; the
// coefficients below are PX4's classic Quadrotor X mixer table.

#include <array>

#include "msg/actuator_motors.hpp"
#include "msg/vehicle_status.hpp"
#include "msg/vehicle_torque_setpoint.hpp"
#include "platform/work_queue.hpp"
#include "uorb/publication.hpp"
#include "uorb/subscription.hpp"

namespace px4p {

/// How much each motor responds to roll, pitch and yaw demand.
struct MixerRow {
  float roll;
  float pitch;
  float yaw;
};

// Motor order matches sim::QuadrotorSim (PX4 Quad X numbering).
constexpr std::array<MixerRow, 4> kQuadXMixer{{
    {-0.707107f, +0.707107f, +1.f},  // 1: front right, CCW
    {+0.707107f, -0.707107f, +1.f},  // 2: rear left,   CCW
    {+0.707107f, +0.707107f, -1.f},  // 3: front left,  CW
    {-0.707107f, -0.707107f, -1.f},  // 4: rear right,  CW
}};

/// Pure mixing function, separate from the module so it can be unit tested.
/// Motors can only spin between 0 and 1, so when the demand doesn't fit
/// something has to give. Priority (like PX4): roll/pitch first, then
/// thrust, and yaw last. Losing attitude control is worse than losing altitude.
std::array<float, 4> mix(const math::Vector3f& torque, float thrust);

class ControlAllocator final : public ScheduledWorkItem {
 public:
  static constexpr hrt_abstime kIntervalUs = 2000;  // 500 Hz, same as rate control

  ControlAllocator();

  void Run() override;

 private:
  uorb::Subscription<msg::VehicleTorqueSetpoint> torque_sp_sub_;
  uorb::Subscription<msg::VehicleStatus> status_sub_;
  uorb::Publication<msg::ActuatorMotors> motors_pub_;
  msg::VehicleTorqueSetpoint torque_sp_{};
  msg::VehicleStatus status_{};
};

}  // namespace px4p
