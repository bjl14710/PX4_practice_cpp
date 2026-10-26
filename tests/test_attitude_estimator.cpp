#include <gtest/gtest.h>

#include "lib/mathlib/constants.hpp"
#include "modules/attitude_estimator/attitude_estimator.hpp"
#include "uorb/publication.hpp"
#include "uorb/subscription.hpp"

namespace px4p {
namespace {

using math::deg_to_rad;
using math::Quatf;
using math::Vector3f;

class AttitudeEstimatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    uorb::reset_all();
    hrt::set_time(0);
  }

  /// Publishes an IMU sample and runs the estimator once, 4 ms later.
  void feed(const Vector3f& accel, const Vector3f& gyro) {
    now_ += AttitudeEstimator::kIntervalUs;
    hrt::set_time(now_);
    msg::SensorImu imu{};
    imu.timestamp = now_;
    imu.accel_m_s2 = accel;
    imu.gyro_rad_s = gyro;
    imu_pub_.publish(imu);
    estimator_.Run();
  }

  Vector3f estimated_euler() {
    msg::VehicleAttitude att{};
    EXPECT_TRUE(att_sub_.copy(att));
    return att.q.to_euler();
  }

  /// What an accelerometer reads when resting at attitude q.
  static Vector3f accel_at_rest(const Quatf& q) {
    return q.conjugate().rotate({0.f, 0.f, -math::kGravity});
  }

  hrt_abstime now_{0};
  AttitudeEstimator estimator_;
  uorb::Publication<msg::SensorImu> imu_pub_;
  uorb::Subscription<msg::VehicleAttitude> att_sub_;
};

TEST_F(AttitudeEstimatorTest, InitializesRollAndPitchFromGravity) {
  const Quatf truth = Quatf::from_euler(deg_to_rad(20.f), deg_to_rad(-15.f), 0.f);
  feed(accel_at_rest(truth), {});
  const Vector3f e = estimated_euler();
  EXPECT_NEAR(e.x, deg_to_rad(20.f), 1e-4f);
  EXPECT_NEAR(e.y, deg_to_rad(-15.f), 1e-4f);
}

TEST_F(AttitudeEstimatorTest, TracksRotationWithGyro) {
  feed(accel_at_rest({}), {});
  // Yaw at 1 rad/s for 1 s. Yaw doesn't change gravity, so accel stays put.
  for (int i = 0; i < 250; ++i) {
    feed(accel_at_rest({}), {0.f, 0.f, 1.f});
  }
  EXPECT_NEAR(estimated_euler().z, 1.f, 0.01f);
}

TEST_F(AttitudeEstimatorTest, AccelCorrectsGyroBias) {
  // A gyro that always reads 0.02 rad/s of roll while the vehicle is level.
  // Without correction the roll estimate would drift 0.02 rad every second.
  feed(accel_at_rest({}), {});
  for (int i = 0; i < 250 * 60; ++i) {  // 60 s
    feed(accel_at_rest({}), {0.02f, 0.f, 0.f});
  }
  EXPECT_NEAR(estimated_euler().x, 0.f, deg_to_rad(0.5f));
  EXPECT_NEAR(estimator_.gyro_bias().x, 0.02f, 0.002f);
}

}  // namespace
}  // namespace px4p
