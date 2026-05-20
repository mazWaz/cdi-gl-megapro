// Single-producer single-consumer lock-free ring buffer.
//
// Designed for ISR → main-task communication on ESP32 (Xtensa LX6):
//   * Producer (ISR) only writes `write_` and reads `read_`.
//   * Consumer (loop) only writes `read_` and reads `write_`.
//   * 32-bit aligned loads/stores on Xtensa are atomic, and `volatile`
//     prevents the compiler from reordering them past the index update.
//   * Capacity is a power of 2 so wrap is a cheap `& (N-1)` mask.
//
// Holds `N-1` entries (one slot reserved to distinguish empty/full).
#pragma once

#include <cstddef>
#include <cstdint>

namespace cdi::util {

template <typename T, size_t N>
class SpscRing {
    static_assert(N >= 2,                   "N must be at least 2");
    static_assert((N & (N - 1)) == 0,       "N must be a power of 2");

public:
    // Push an item from ISR context. Returns false if the ring is full
    // (the new item is dropped — caller may log/count overruns).
    bool pushFromIsr(const T& v) {
        const size_t w    = write_;
        const size_t next = (w + 1) & (N - 1);
        if (next == read_) return false;        // full
        buf_[w] = v;
        write_ = next;                          // publish
        return true;
    }

    // Pop one item from consumer (non-ISR) context. False if empty.
    bool pop(T& out) {
        const size_t r = read_;
        if (r == write_) return false;          // empty
        out = buf_[r];
        read_ = (r + 1) & (N - 1);
        return true;
    }

    // Current occupancy (approximate — may be stale by one slot).
    size_t size() const {
        return (write_ - read_) & (N - 1);
    }

    bool empty() const { return read_ == write_; }

    static constexpr size_t capacity() { return N - 1; }

private:
    T buf_[N];
    volatile size_t write_ = 0;
    volatile size_t read_  = 0;
};

} // namespace cdi::util
