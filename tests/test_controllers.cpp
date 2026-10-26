#include <gtest/gtest.h>

#include "lib/mathlib/constants.hpp"
#include "modules/mc_att_control/mc_att_control.hpp"

namespace px4p {
namespace {

using math::deg_to_rad;
using math::Quatf;
using math::Vector3f;

TEST(AttitudeControlTest, NoErrorMeansNoRate) {
  const Quatf q = Quatf::from_euler(0.1f, 0.2f, 0.3f);
  const Vector3f rates = attitude_control_update(q, q, {});
  EXPECT_NEAR(rates.norm(), 0.f, 1e-6f);
}

TEST(AttitudeControlTest, SmallRollErrorGivesProportionalRollRate) {
  AttitudeControlParams params;
  const float error = deg_to_rad(5.f);
  const Vector3f rates =
      attitude_control_update(Quatf{}, Quatf::from_euler(error, 0.f, 0.f), params);
  EXPECT_NEAR(rates.x, params.gain.x * error, 1e-3f);
  EXPECT_NEAR(rates.y, 0.f, 1e-6f);
  EXPECT_NEAR(rates.z, 0.f, 1e-6f);
}

TEST(AttitudeControlTest, LargeErrorIsRateLimited) {
  AttitudeControlParams params;
  const Vector3f rates =
      attitude_control_update(Quatf{}, Quatf::from_euler(0.f, deg_to_rad(90.f), 0.f), params);
  EXPECT_FLOAT_EQ(rates.y, params.rate_limit_rad_s.y);
}

TEST(AttitudeControlTest, TakesTheShortWayAround) {
  // From yaw +170 deg to yaw -170 deg is 20 deg to the right, not 340 to the left.
  const Quatf q = Quatf::from_euler(0.f, 0.f, deg_to_rad(170.f));
  const Quatf q_d = Quatf::from_euler(0.f, 0.f, deg_to_rad(-170.f));
  EXPECT_GT(attitude_control_update(q, q_d, {}).z, 0.f);
}

}  // namespace
}  // namespace px4p
