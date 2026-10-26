#include <mutex>
#include <vector>

#include "uorb/topic.hpp"

namespace px4p::uorb {

namespace {

// Function-local statics avoid the "static initialization order fiasco":
// a Topic<T> may be constructed during another file's static init, before a
// namespace-scope vector here would exist.
std::vector<TopicBase*>& registry() {
  static std::vector<TopicBase*> topics;
  return topics;
}

std::mutex& registry_mutex() {
  static std::mutex m;
  return m;
}

}  // namespace

void register_topic(TopicBase* topic) {
  std::lock_guard<std::mutex> lock(registry_mutex());
  registry().push_back(topic);
}

void reset_all() {
  std::lock_guard<std::mutex> lock(registry_mutex());
  for (TopicBase* topic : registry()) {
    topic->reset();
  }
}

}  // namespace px4p::uorb
