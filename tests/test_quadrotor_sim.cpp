#include <gtest/gtest.h>

#include "sim/quadrotor_sim.hpp"

namespace px4p::sim {
namespace {

constexpr float kDt = 0.001f;

std::array<float, 4> all(float u) { return {u, u, u, u}; }

/// A sim already airborne at 10 m, so the ground doesn't interfere.
QuadrotorSim airborne_sim() {
  QuadrotorSim sim;
  QuadrotorState s;
  s.position_ned = {0.f, 0.f, -10.f};
  const float hover_thrust = sim.hover_command() * sim.params().max_thrust_per_motor_n;
  s.motor_thrust_n = {hover_thrust, hover_thrust, hover_thrust, hover_thrust};
  sim.set_state(s);
  return sim;
}

TEST(QuadrotorSimTest, RestsOnGroundWithMotorsOff) {
  QuadrotorSim sim;
  for (int i = 0; i < 1000; ++i) {
    sim.step(all(0.f), kDt);
  }
  EXPECT_FLOAT_EQ(sim.altitude_m(), 0.f);
  // An accelerometer at rest reads 1 g "up" (negative z in FRD).
  EXPECT_NEAR(sim.specific_force_body().z, -kGravity, 1e-4f);
}

TEST(QuadrotorSimTest, HoverCommandHoldsAltitude) {
  QuadrotorSim sim = airborne_sim();
  for (int i = 0; i < 2000; ++i) {
    sim.step(all(sim.hover_command()), kDt);
  }
  EXPECT_NEAR(sim.altitude_m(), 10.f, 1e-3f);
}

TEST(QuadrotorSimTest, FullThrottleClimbs) {
  QuadrotorSim sim;
  for (int i = 0; i < 1000; ++i) {
    sim.step(all(1.f), kDt);
  }
  EXPECT_GT(sim.altitude_m(), 1.f);
}

TEST(QuadrotorSimTest, LeftMotorsHarderRollsRight) {
  // Motors 2 and 3 are on the left. More thrust there pushes the left side
  // up, which is a positive (right-wing-down) roll in FRD.
  QuadrotorSim sim = airborne_sim();
  const float h = sim.hover_command();
  for (int i = 0; i < 100; ++i) {
    sim.step({h - 0.05f, h + 0.05f, h + 0.05f, h - 0.05f}, kDt);
  }
  EXPECT_GT(sim.state().omega_body.x, 0.1f);
  EXPECT_NEAR(sim.state().omega_body.y, 0.f, 1e-3f);
}

TEST(QuadrotorSimTest, CcwMotorsHarderYawsRight) {
  // Motors 1 and 2 spin CCW; their reaction torque turns the body CW seen
  // from above, which is positive yaw in NED/FRD.
  QuadrotorSim sim = airborne_sim();
  const float h = sim.hover_command();
  for (int i = 0; i < 100; ++i) {
    sim.step({h + 0.05f, h + 0.05f, h - 0.05f, h - 0.05f}, kDt);
  }
  EXPECT_GT(sim.state().omega_body.z, 0.01f);
}

}  // namespace
}  // namespace px4p::sim
