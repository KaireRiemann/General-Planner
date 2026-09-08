#pragma once

#include <functional>
#include <utility>

namespace math_utils {
// A worker-local hook also covers nested corridor/yaw optimizations, without
// cancelling optimizers belonging to another task or thread.
inline thread_local std::function<bool()> cancellation_probe;
inline bool optimizationCancelled() {
    return cancellation_probe && cancellation_probe();
}
class ScopedCancellation {
public:
    explicit ScopedCancellation(std::function<bool()> probe)
        : previous_(std::exchange(cancellation_probe, std::move(probe))) {}
    ~ScopedCancellation() { cancellation_probe = std::move(previous_); }
    ScopedCancellation(const ScopedCancellation &) = delete;
    ScopedCancellation &operator=(const ScopedCancellation &) = delete;
private:
    std::function<bool()> previous_;
};
} // namespace math_utils
