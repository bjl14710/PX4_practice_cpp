#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "platform/work_queue.hpp"

namespace px4p {
namespace {

class Recorder final : public ScheduledWorkItem {
 public:
  Recorder(std::string name, hrt_abstime interval_us, std::vector<std::string>& log)
      : ScheduledWorkItem(std::move(name), interval_us), log_(log) {}

  void Run() override { log_.push_back(name()); }

 private:
  std::vector<std::string>& log_;
};

class WorkQueueTest : public ::testing::Test {
 protected:
  void SetUp() override { hrt::set_time(0); }
};

TEST_F(WorkQueueTest, RunsAtTheConfiguredRate) {
  std::vector<std::string> log;
  Recorder fast("fast", 1000, log);
  Recorder slow("slow", 4000, log);
  WorkQueue wq;
  wq.add(fast);
  wq.add(slow);

  for (hrt_abstime t = 0; t < 8000; t += 1000) {
    wq.run_due(t);
  }

  EXPECT_EQ(std::count(log.begin(), log.end(), "fast"), 8);
  EXPECT_EQ(std::count(log.begin(), log.end(), "slow"), 2);  // t = 0 and t = 4000
}

TEST_F(WorkQueueTest, RunsInInsertionOrder) {
  std::vector<std::string> log;
  Recorder first("first", 1000, log);
  Recorder second("second", 1000, log);
  WorkQueue wq;
  wq.add(first);
  wq.add(second);

  wq.run_due(0);
  ASSERT_EQ(log.size(), 2u);
  EXPECT_EQ(log[0], "first");
  EXPECT_EQ(log[1], "second");
}

TEST_F(WorkQueueTest, SkipsMissedCyclesInsteadOfBursting) {
  std::vector<std::string> log;
  Recorder item("item", 1000, log);
  WorkQueue wq;
  wq.add(item);

  wq.run_due(0);
  wq.run_due(10'000);  // 10 cycles late: run once, not ten times
  EXPECT_EQ(log.size(), 2u);
  wq.run_due(10'500);  // next run is due at 11'000
  EXPECT_EQ(log.size(), 2u);
}

}  // namespace
}  // namespace px4p
