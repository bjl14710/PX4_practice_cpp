#pragma once

// A tiny version of PX4's work queue (platforms/common/px4_work_queue).
//
// In PX4 most modules are not threads. They are ScheduledWorkItems: objects
// with a Run() method that a work queue thread calls at a fixed interval.
// Here a single WorkQueue runs every item on the caller's thread, in the
// order the items were added, whenever their interval has elapsed.

#include <string>
#include <vector>

#include "platform/hrt.hpp"

namespace px4p {

/// Base class for every module. Subclasses implement Run().
class ScheduledWorkItem {
 public:
  ScheduledWorkItem(std::string name, hrt_abstime interval_us);
  virtual ~ScheduledWorkItem() = default;

  // Modules hold subscriptions and references to each other; copying one
  // would silently duplicate that state, so forbid it.
  ScheduledWorkItem(const ScheduledWorkItem&) = delete;
  ScheduledWorkItem& operator=(const ScheduledWorkItem&) = delete;

  /// Called by the WorkQueue once per interval.
  virtual void Run() = 0;

  const std::string& name() const { return name_; }
  hrt_abstime interval_us() const { return interval_us_; }

 private:
  friend class WorkQueue;

  std::string name_;
  hrt_abstime interval_us_;
  hrt_abstime next_run_us_{0};
};

/// Runs scheduled work items in insertion order.
class WorkQueue {
 public:
  /// Adds an item. The queue does not own it; it must outlive the queue.
  void add(ScheduledWorkItem& item);

  /// Runs every item whose next deadline is at or before now_us.
  void run_due(hrt_abstime now_us);

  const std::vector<ScheduledWorkItem*>& items() const { return items_; }

 private:
  std::vector<ScheduledWorkItem*> items_;
};

}  // namespace px4p
