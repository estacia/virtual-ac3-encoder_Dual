// test_spscqueue.cpp — bounded SPSC queue correctness and concurrency tests.
#include "doctest.h"
#include "SpscQueue.h"

#include <atomic>
#include <cstdint>
#include <thread>

TEST_CASE("SPSC queue preserves FIFO order and capacity")
{
  SpscQueue<uint32_t, 3> queue;
  uint32_t value = 99;

  CHECK(queue.Size() == 0);
  CHECK_FALSE(queue.TryPop(value));
  CHECK(queue.TryPush(10));
  CHECK(queue.TryPush(20));
  CHECK(queue.TryPush(30));
  CHECK_FALSE(queue.TryPush(40));
  CHECK(queue.Size() == 3);

  CHECK(queue.TryPop(value));
  CHECK(value == 10);
  CHECK(queue.TryPop(value));
  CHECK(value == 20);
  CHECK(queue.TryPush(40));
  CHECK(queue.TryPop(value));
  CHECK(value == 30);
  CHECK(queue.TryPop(value));
  CHECK(value == 40);
  CHECK_FALSE(queue.TryPop(value));
  CHECK(queue.Size() == 0);
}

TEST_CASE("SPSC queue concurrent transfer preserves every item")
{
  constexpr uint64_t kItems = 1u << 20;
  SpscQueue<uint64_t, 256> queue;
  std::atomic<bool> sequenceOk{true};

  std::thread producer([&] {
    for (uint64_t value = 0; value < kItems; ++value)
    {
      while (!queue.TryPush(value))
        std::this_thread::yield();
    }
  });

  std::thread consumer([&] {
    uint64_t expected = 0;
    uint64_t value = 0;
    while (expected < kItems)
    {
      if (!queue.TryPop(value))
      {
        std::this_thread::yield();
        continue;
      }
      if (value != expected)
        sequenceOk.store(false, std::memory_order_relaxed);
      ++expected;
    }
  });

  producer.join();
  consumer.join();
  CHECK(sequenceOk.load(std::memory_order_relaxed));
  CHECK(queue.Size() == 0);
}

TEST_CASE("SPSC queue reset empties a quiescent queue")
{
  SpscQueue<uint32_t, 4> queue;
  CHECK(queue.TryPush(1));
  CHECK(queue.TryPush(2));
  queue.Reset();
  CHECK(queue.Size() == 0);
  uint32_t value = 0;
  CHECK_FALSE(queue.TryPop(value));
}
