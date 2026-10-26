#include <gtest/gtest.h>

#include "lib/mathlib/constants.hpp"
#include "modules/commander/commander.hpp"
#include "uorb/publication.hpp"

namespace px4p {
namespace {

using ArmingState = msg::VehicleStatus::ArmingState;
using Command = msg::VehicleCommand::Command;

class CommanderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    uorb::reset_all();
    hrt::set_time(1'000'000);
  }

  void publish_imu() {
    msg::SensorImu imu{};
    imu.timestamp = hrt_absolute_time();
    imu_pub_.publish(imu);
  }

  void publish_attitude(float roll_rad) {
    msg::VehicleAttitude att{};
    att.timestamp = hrt_absolute_time();
    att.q = math::Quatf::from_euler(roll_rad, 0.f, 0.f);
    att_pub_.publish(att);
  }

  void send(Command c) {
    msg::VehicleCommand cmd{};
    cmd.timestamp = hrt_absolute_time();
    cmd.command = c;
    cmd_pub_.publish(cmd);
    commander_.Run();
  }

  Commander commander_;
  uorb::Publication<msg::SensorImu> imu_pub_;
  uorb::Publication<msg::VehicleAttitude> att_pub_;
  uorb::Publication<msg::VehicleCommand> cmd_pub_;
};

TEST_F(CommanderTest, StartsDisarmed) {
  commander_.Run();
  EXPECT_EQ(commander_.arming_state(), ArmingState::kDisarmed);
}

TEST_F(CommanderTest, RefusesToArmWithoutImu) {
  publish_attitude(0.f);
  send(Command::kArm);
  EXPECT_EQ(commander_.arming_state(), ArmingState::kDisarmed);
  EXPECT_EQ(commander_.last_rejection(), "IMU data missing or stale");
}

TEST_F(CommanderTest, RefusesToArmWithStaleImu) {
  publish_imu();
  publish_attitude(0.f);
  hrt::set_time(hrt_absolute_time() + 500'000);  // IMU now 0.5 s old
  send(Command::kArm);
  EXPECT_EQ(commander_.arming_state(), ArmingState::kDisarmed);
}

TEST_F(CommanderTest, RefusesToArmWhenTilted) {
  publish_imu();
  publish_attitude(math::deg_to_rad(45.f));
  send(Command::kArm);
  EXPECT_EQ(commander_.arming_state(), ArmingState::kDisarmed);
  EXPECT_EQ(commander_.last_rejection(), "vehicle tilted too far");
}

TEST_F(CommanderTest, ArmsAndDisarmsWhenHealthy) {
  publish_imu();
  publish_attitude(0.f);
  send(Command::kArm);
  EXPECT_EQ(commander_.arming_state(), ArmingState::kArmed);
  EXPECT_TRUE(commander_.last_rejection().empty());

  send(Command::kDisarm);
  EXPECT_EQ(commander_.arming_state(), ArmingState::kDisarmed);
}

}  // namespace
}  // namespace px4p
