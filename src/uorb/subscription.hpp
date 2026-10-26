#pragma once

#include <cstdint>

#include "uorb/topic.hpp"

namespace px4p::uorb {

/// Reads messages of type T (PX4: uORB::Subscription<T>).
///
/// Each subscriber remembers the generation it last copied, so two
/// subscribers to the same topic track "new data" independently.
template <typename T>
class Subscription {
 public:
  /// True if a sample newer than the last one this subscriber copied exists.
  bool updated() const {
    const uint64_t gen = Topic<T>::instance().generation();
    return gen != 0 && gen != last_generation_;
  }

  /// Copies only when updated() is true. Returns true if out was written.
  bool update(T& out) {
    if (!updated()) {
      return false;
    }
    return copy(out);
  }

  /// Copies the latest sample (new or not) and marks it as seen.
  /// Returns false if the topic was never published.
  bool copy(T& out) {
    uint64_t gen = 0;
    if (!Topic<T>::instance().copy(out, &gen)) {
      return false;
    }
    last_generation_ = gen;
    return true;
  }

 private:
  uint64_t last_generation_{0};
};

}  // namespace px4p::uorb
