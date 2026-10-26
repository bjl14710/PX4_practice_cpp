#include <gtest/gtest.h>

#include "uorb/publication.hpp"
#include "uorb/subscription.hpp"

namespace px4p {
namespace {

struct TestMsg {
  int value{0};
};

class UorbTest : public ::testing::Test {
 protected:
  void SetUp() override { uorb::reset_all(); }
};

TEST_F(UorbTest, NothingPublishedMeansNoData) {
  uorb::Subscription<TestMsg> sub;
  TestMsg msg{};
  EXPECT_FALSE(sub.updated());
  EXPECT_FALSE(sub.update(msg));
  EXPECT_FALSE(sub.copy(msg));
}

TEST_F(UorbTest, SubscriberSeesEachSampleOnce) {
  uorb::Publication<TestMsg> pub;
  uorb::Subscription<TestMsg> sub;

  pub.publish(TestMsg{42});
  EXPECT_TRUE(sub.updated());

  TestMsg msg{};
  ASSERT_TRUE(sub.update(msg));
  EXPECT_EQ(msg.value, 42);

  // Same sample again: not "updated", but copy() still returns it.
  EXPECT_FALSE(sub.updated());
  EXPECT_FALSE(sub.update(msg));
  TestMsg again{};
  EXPECT_TRUE(sub.copy(again));
  EXPECT_EQ(again.value, 42);
}

TEST_F(UorbTest, OnlyLatestSampleIsKept) {
  uorb::Publication<TestMsg> pub;
  uorb::Subscription<TestMsg> sub;

  pub.publish(TestMsg{1});
  pub.publish(TestMsg{2});
  pub.publish(TestMsg{3});

  TestMsg msg{};
  ASSERT_TRUE(sub.update(msg));
  EXPECT_EQ(msg.value, 3);  // queue depth is 1: samples 1 and 2 are gone
}

TEST_F(UorbTest, SubscribersAreIndependent) {
  uorb::Publication<TestMsg> pub;
  uorb::Subscription<TestMsg> a;
  uorb::Subscription<TestMsg> b;

  pub.publish(TestMsg{7});
  TestMsg msg{};
  ASSERT_TRUE(a.update(msg));

  EXPECT_FALSE(a.updated());
  EXPECT_TRUE(b.updated());  // b hasn't read it yet
}

TEST_F(UorbTest, ResetAllClearsTopics) {
  uorb::Publication<TestMsg> pub;
  pub.publish(TestMsg{5});
  uorb::reset_all();

  uorb::Subscription<TestMsg> sub;
  TestMsg msg{};
  EXPECT_FALSE(sub.copy(msg));
}

}  // namespace
}  // namespace px4p
