#pragma once

// uORB-style topics (PX4: platforms/common/uORB).
//
// uORB is PX4's publish/subscribe message bus. Modules never call each other
// directly: the estimator publishes vehicle_attitude, and anything that cares
// subscribes to it. Each topic keeps only its latest sample plus a
// "generation" counter that increments on every publish, so a subscriber can
// tell whether it has already seen the current sample.

#include <cstdint>
#include <mutex>

namespace px4p::uorb {

/// Type-erased handle so every topic can be cleared from one place.
class TopicBase {
 public:
  virtual ~TopicBase() = default;
  virtual void reset() = 0;
};

/// Records a topic so reset_all() can find it. Called by Topic<T>'s constructor.
void register_topic(TopicBase* topic);

/// Clears every topic's data and generation. Tests call this in SetUp().
void reset_all();

/// The single storage slot for message type T (one topic per message type).
template <typename T>
class Topic final : public TopicBase {
 public:
  /// Function-local static: created on first use, thread-safe since C++11.
  static Topic& instance() {
    static Topic topic;
    return topic;
  }

  void publish(const T& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = msg;
    ++generation_;
  }

  /// Copies the latest sample into out. Returns false if nothing was published yet.
  bool copy(T& out, uint64_t* generation = nullptr) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation_ == 0) {
      return false;
    }
    out = data_;
    if (generation != nullptr) {
      *generation = generation_;
    }
    return true;
  }

  /// Number of publishes so far; 0 means never published.
  uint64_t generation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
  }

  void reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = T{};
    generation_ = 0;
  }

 private:
  Topic() { register_topic(this); }

  mutable std::mutex mutex_;
  T data_{};
  uint64_t generation_{0};
};

}  // namespace px4p::uorb
