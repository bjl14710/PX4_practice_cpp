#include "platform/work_queue.hpp"

#include <utility>

namespace px4p {

ScheduledWorkItem::ScheduledWorkItem(std::string name, hrt_abstime interval_us)
    : name_(std::move(name)), interval_us_(interval_us) {}

void WorkQueue::add(ScheduledWorkItem& item) {
  item.next_run_us_ = hrt_absolute_time();
  items_.push_back(&item);
}

void WorkQueue::run_due(hrt_abstime now_us) {
  for (ScheduledWorkItem* item : items_) {
    if (now_us < item->next_run_us_) {
      continue;
    }
    item->Run();
    item->next_run_us_ += item->interval_us_;
    // If we fell behind (e.g. a debugger pause), skip the missed cycles
    // instead of running the item many times back-to-back.
    if (item->next_run_us_ <= now_us) {
      item->next_run_us_ = now_us + item->interval_us_;
    }
  }
}

}  // namespace px4p
