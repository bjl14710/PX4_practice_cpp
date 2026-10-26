// End-to-end SITL tests: the whole stack flying the simulated quadrotor.

#include <gtest/gtest.h>

#include "flight_stack.hpp"
#include "lib/mathlib/constants.hpp"
#include "uorb/publication.hpp"

namespace px4p {
namespace {

using math::deg_to_rad;
using math::rad_to_deg;

class ClosedLoopTest : public ::testing::Test {
 protected:
  void SetUp() override { uorb::reset_all(); }

  void arm() {
    msg::VehicleCommand cmd{};
    cmd.timestamp = hrt_absolute_time();
    cmd.command = msg::VehicleCommand::Command::kArm;
    cmd_pub_.publish(cmd);
  }

  void set_attitude(float roll_deg, float pitch_deg, float yaw_deg, float thrust) {
    msg::VehicleAttitudeSetpoint sp{};
    sp.timestamp = hrt_absolute_time();
    sp.q_d = math::Quatf::from_euler(deg_to_rad(roll_deg), deg_to_rad(pitch_deg),
                                     deg_to_rad(yaw_deg));
    sp.thrust = thrust;
    sp_pub_.publish(sp);
  }

  /// Arms on the ground, climbs for 1 s, then settles into a hover.
  void take_off() {
    stack_.run_until(500'000);  // let the estimator initialize
    arm();
    stack_.run_until(600'000);
    ASSERT_EQ(stack_.commander().arming_state(), msg::VehicleStatus::ArmingState::kArmed);
    const float h = stack_.sim().hover_command();
    set_attitude(0.f, 0.f, 0.f, h * 1.15f);
    stack_.run_until(1'600'000);
    set_attitude(0.f, 0.f, 0.f, h);
    stack_.run_until(2'600'000);
  }

  math::Vector3f true_euler_deg() {
    const math::Vector3f e = stack_.sim().state().attitude.to_euler();
    return {rad_to_deg(e.x), rad_to_deg(e.y), rad_to_deg(e.z)};
  }

  FlightStack stack_;
  uorb::Publication<msg::VehicleCommand> cmd_pub_;
  uorb::Publication<msg::VehicleAttitudeSetpoint> sp_pub_;
};

TEST_F(ClosedLoopTest, MotorsStayOffWhileDisarmed) {
  set_attitude(0.f, 0.f, 0.f, 0.8f);  // thrust requested, but not armed
  stack_.run_until(2'000'000);
  EXPECT_FLOAT_EQ(stack_.sim().altitude_m(), 0.f);
}

TEST_F(ClosedLoopTest, TakesOffAndHoversLevel) {
  take_off();
  EXPECT_GT(stack_.sim().altitude_m(), 0.5f);
  const math::Vector3f e = true_euler_deg();
  EXPECT_NEAR(e.x, 0.f, 0.5f);
  EXPECT_NEAR(e.y, 0.f, 0.5f);
}

TEST_F(ClosedLoopTest, TracksRollStep) {
  take_off();
  const float h = stack_.sim().hover_command();
  set_attitude(10.f, 0.f, 0.f, h);
  stack_.run_until(stack_.now() + 500'000);
  // Within 1.5 deg after 0.5 s. Not exact: the accel-only estimator is
  // pulled toward level while the vehicle is tilted (see lesson notes).
  EXPECT_NEAR(true_euler_deg().x, 10.f, 1.5f);
}

TEST_F(ClosedLoopTest, TracksYawStep) {
  take_off();
  const float h = stack_.sim().hover_command();
  set_attitude(0.f, 0.f, 45.f, h);
  stack_.run_until(stack_.now() + 2'000'000);
  EXPECT_NEAR(true_euler_deg().z, 45.f, 2.f);
  EXPECT_GT(stack_.sim().altitude_m(), 0.5f);  // yaw didn't cost altitude
}

}  // namespace
}  // namespace px4p
