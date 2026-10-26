#include <gtest/gtest.h>

#include "lib/mathlib/constants.hpp"
#include "lib/mathlib/low_pass_filter.hpp"
#include "lib/mathlib/pid.hpp"
#include "lib/mathlib/quaternion.hpp"

namespace px4p::math {
namespace {

constexpr float kTol = 1e-5f;

void expect_near(const Vector3f& a, const Vector3f& b, float tol = kTol) {
  EXPECT_NEAR(a.x, b.x, tol);
  EXPECT_NEAR(a.y, b.y, tol);
  EXPECT_NEAR(a.z, b.z, tol);
}

TEST(Vector3fTest, CrossProductFollowsRightHandRule) {
  expect_near(Vector3f{1.f, 0.f, 0.f}.cross({0.f, 1.f, 0.f}), {0.f, 0.f, 1.f});
  expect_near(Vector3f{0.f, 1.f, 0.f}.cross({1.f, 0.f, 0.f}), {0.f, 0.f, -1.f});
}

TEST(Vector3fTest, NormalizedZeroVectorStaysZero) {
  expect_near(Vector3f{}.normalized(), {});
}

TEST(QuatfTest, IdentityLeavesVectorsAlone) {
  expect_near(Quatf{}.rotate({1.f, 2.f, 3.f}), {1.f, 2.f, 3.f});
}

TEST(QuatfTest, YawOf90DegreesTurnsNorthIntoEast) {
  // Body x (nose) points north at yaw 0; after +90 deg yaw it points east.
  const Quatf q = Quatf::from_euler(0.f, 0.f, deg_to_rad(90.f));
  expect_near(q.rotate({1.f, 0.f, 0.f}), {0.f, 1.f, 0.f});
}

TEST(QuatfTest, EulerRoundTrip) {
  const Vector3f euler{0.3f, -0.2f, 1.1f};
  expect_near(Quatf::from_euler(euler.x, euler.y, euler.z).to_euler(), euler);
}

TEST(QuatfTest, ConjugateUndoesRotation) {
  const Quatf q = Quatf::from_axis_angle({1.f, 2.f, 3.f}, 0.7f);
  const Vector3f v{0.5f, -1.f, 2.f};
  expect_near(q.conjugate().rotate(q.rotate(v)), v);
}

TEST(QuatfTest, IntegratingConstantRateForOneSecond) {
  Quatf q;
  for (int i = 0; i < 1000; ++i) {
    q = q.integrate({0.f, 0.f, 0.5f}, 0.001f);  // 0.5 rad/s yaw
  }
  EXPECT_NEAR(q.to_euler().z, 0.5f, 1e-4f);
  EXPECT_NEAR(q.norm(), 1.f, 1e-6f);
}

TEST(PidTest, ProportionalOnly) {
  Pid pid({2.f, 0.f, 0.f, 0.f, 10.f});
  EXPECT_FLOAT_EQ(pid.update(1.f, 0.25f, 0.01f), 1.5f);
}

TEST(PidTest, IntegratorIsClamped) {
  Pid pid({0.f, 1.f, 0.f, 0.3f, 10.f});
  for (int i = 0; i < 1000; ++i) {
    pid.update(1.f, 0.f, 0.01f);
  }
  EXPECT_FLOAT_EQ(pid.integral(), 0.3f);
}

TEST(PidTest, FrozenIntegratorDoesNotGrow) {
  Pid pid({0.f, 1.f, 0.f, 1.f, 10.f});
  pid.update(1.f, 0.f, 0.1f, /*integrate=*/false);
  EXPECT_FLOAT_EQ(pid.integral(), 0.f);
}

TEST(PidTest, OutputIsClamped) {
  Pid pid({100.f, 0.f, 0.f, 0.f, 1.f});
  EXPECT_FLOAT_EQ(pid.update(1.f, 0.f, 0.01f), 1.f);
  EXPECT_FLOAT_EQ(pid.update(-1.f, 0.f, 0.01f), -1.f);
}

TEST(PidTest, NoDerivativeKickOnSetpointStep) {
  Pid pid({0.f, 0.f, 1.f, 0.f, 10.f});
  pid.update(0.f, 0.f, 0.01f);
  // The setpoint jumps but the measurement doesn't move: D term stays 0.
  EXPECT_FLOAT_EQ(pid.update(5.f, 0.f, 0.01f), 0.f);
}

TEST(LowPassFilterTest, FirstSampleInitializesState) {
  LowPassFilter<float> lpf(1000.f, 10.f);
  EXPECT_FLOAT_EQ(lpf.apply(3.f), 3.f);
}

TEST(LowPassFilterTest, ConvergesToConstantInput) {
  LowPassFilter<float> lpf(1000.f, 10.f);
  lpf.apply(0.f);
  float y = 0.f;
  for (int i = 0; i < 1000; ++i) {
    y = lpf.apply(1.f);
  }
  EXPECT_NEAR(y, 1.f, 1e-4f);
}

}  // namespace
}  // namespace px4p::math
