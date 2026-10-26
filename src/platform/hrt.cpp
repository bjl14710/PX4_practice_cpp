#include "platform/hrt.hpp"

#include <atomic>

namespace px4p {

namespace {
std::atomic<hrt_abstime> g_now_us{0};
}  // namespace

hrt_abstime hrt_absolute_time() { return g_now_us.load(std::memory_order_relaxed); }

namespace hrt {

void set_time(hrt_abstime now_us) { g_now_us.store(now_us, std::memory_order_relaxed); }

}  // namespace hrt

}  // namespace px4p
