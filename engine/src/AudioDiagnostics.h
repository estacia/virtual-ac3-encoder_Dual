// AudioDiagnostics.h — lock-free counters shared by the audio pipeline threads.
#pragma once

#include <atomic>
#include <cstdint>

struct AudioDiagnostics
{
  std::atomic<uint64_t> captureDroppedFrames{0};
  std::atomic<uint64_t> captureWasapiErrors{0};
  std::atomic<uint64_t> outputUnderrunFrames{0};
  std::atomic<uint64_t> outputUnderrunPackets{0};
  std::atomic<uint64_t> outputWasapiErrors{0};
  std::atomic<uint64_t> eventTimeouts{0};
  std::atomic<uint64_t> driftTrimFrames{0};

  void Reset()
  {
    captureDroppedFrames.store(0, std::memory_order_relaxed);
    captureWasapiErrors.store(0, std::memory_order_relaxed);
    outputUnderrunFrames.store(0, std::memory_order_relaxed);
    outputUnderrunPackets.store(0, std::memory_order_relaxed);
    outputWasapiErrors.store(0, std::memory_order_relaxed);
    eventTimeouts.store(0, std::memory_order_relaxed);
    driftTrimFrames.store(0, std::memory_order_relaxed);
  }
};
