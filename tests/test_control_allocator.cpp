#include <gtest/gtest.h>

#include <numeric>

#include "modules/control_allocator/control_allocator.hpp"

namespace px4p {
namespace {

using math::Vector3f;

float sum(const std::array<float, 4>& u) { return std::accumulate(u.begin(), u.end(), 0.f); }

TEST(MixerTest, PureThrustIsEqualOnAllMotors) {
  const auto u = mix({}, 0.6f);
  for (float m : u) {
    EXPECT_FLOAT_EQ(m, 0.6f);
  }
}

TEST(MixerTest, PositiveRollSpeedsUpLeftMotors) {
  const auto u = mix({0.1f, 0.f, 0.f}, 0.5f);
  // Left motors are 2 and 3 (indices 1, 2); right are 1 and 4 (indices 0, 3).
  EXPECT_GT(u[1], u[0]);
  EXPECT_GT(u[2], u[3]);
  EXPECT_NEAR(sum(u), 2.f, 1e-5f);  // roll doesn't change total thrust
}

TEST(MixerTest, PositivePitchSpeedsUpFrontMotors) {
  const auto u = mix({0.f, 0.1f, 0.f}, 0.5f);
  EXPECT_GT(u[0], u[1]);  // 1 front > 2 rear
  EXPECT_GT(u[2], u[3]);  // 3 front > 4 rear
}

TEST(MixerTest, SaturationKeepsRollDifference) {
  // 0.95 thrust + roll would push the left motors past 1. They are all
  // lowered together, so the left/right difference survives.
  const auto unsaturated = mix({0.2f, 0.f, 0.f}, 0.5f);
  const auto saturated = mix({0.2f, 0.f, 0.f}, 0.95f);
  EXPECT_FLOAT_EQ(saturated[1], 1.f);
  EXPECT_NEAR(saturated[1] - saturated[0], unsaturated[1] - unsaturated[0], 1e-5f);
}

TEST(MixerTest, YawIsLimitedToAvailableHeadroom) {
  // A huge yaw demand must not starve two motors and drop the vehicle.
  const auto u = mix({0.f, 0.f, 2.f}, 0.6f);
  for (float m : u) {
    EXPECT_GE(m, 0.f);
    EXPECT_LE(m, 1.f);
  }
  EXPECT_NEAR(sum(u), 4.f * 0.6f, 1e-5f);  // thrust preserved
  EXPECT_FLOAT_EQ(u[0], 1.f);              // CCW motors at the limit
}

}  // namespace
}  // namespace px4p
