// SpscQueue.h — bounded single-producer / single-consumer lock-free queue.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

template <typename T, size_t Capacity>
class SpscQueue
{
  static_assert(Capacity > 0, "SpscQueue capacity must be non-zero");
  static_assert(std::is_copy_assignable<T>::value,
                "SpscQueue items must be copy assignable");

public:
  // Producer-only operation. A failed push leaves the queue unchanged.
  bool TryPush(const T& item)
  {
    const uint64_t write = write_.load(std::memory_order_relaxed);
    const uint64_t read = read_.load(std::memory_order_acquire);
    if (write - read >= Capacity) return false;

    slots_[static_cast<size_t>(write % Capacity)] = item;
    write_.store(write + 1, std::memory_order_release);
    return true;
  }

  // Consumer-only operation. A failed pop leaves the destination unchanged.
  bool TryPop(T& item)
  {
    const uint64_t read = read_.load(std::memory_order_relaxed);
    const uint64_t write = write_.load(std::memory_order_acquire);
    if (read == write) return false;

    item = slots_[static_cast<size_t>(read % Capacity)];
    read_.store(read + 1, std::memory_order_release);
    return true;
  }

  // Approximate concurrent snapshot, exact when producer/consumer are quiescent.
  size_t Size() const
  {
    // Read first: the consumer cannot advance beyond a write index it has observed.
    const uint64_t read = read_.load(std::memory_order_acquire);
    const uint64_t write = write_.load(std::memory_order_acquire);
    return static_cast<size_t>(write - read);
  }

  static constexpr size_t CapacityValue() { return Capacity; }

  // Call only after producer and consumer have both stopped.
  void Reset()
  {
    read_.store(0, std::memory_order_relaxed);
    write_.store(0, std::memory_order_relaxed);
  }

private:
  std::array<T, Capacity> slots_{};
  alignas(64) std::atomic<uint64_t> read_{0};  // consumer-owned cursor
  alignas(64) std::atomic<uint64_t> write_{0}; // producer-owned cursor
};
