// RingBuffer.h
//
// Single-producer / single-consumer lock-free PCM ring buffer.
// Internally, head/tail are counted in complete audio frames, never bytes.
// This prevents an overflow or trim from splitting an interleaved PCM frame.
#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <vector>

class RingBuffer
{
public:
  explicit RingBuffer(size_t capacityBytes, size_t bytesPerFrame = 1)
      : frameBytes_(bytesPerFrame ? bytesPerFrame : 1)
  {
    const size_t requestedFrames =
        std::max<size_t>(1, (capacityBytes + frameBytes_ - 1) / frameBytes_);
    size_t frames = 1;
    while (frames < requestedFrames && frames <= (static_cast<size_t>(-1) >> 1))
      frames <<= 1;
    capacityFrames_ = frames;
    maskFrames_ = capacityFrames_ - 1;
    buf_.resize(capacityFrames_ * frameBytes_);
  }

  size_t Capacity() const { return buf_.size(); }
  size_t BytesPerFrame() const { return frameBytes_; }

  // Consumer view: bytes ready to read (always a whole-frame multiple).
  size_t BytesAvailable() const
  {
    const uint64_t head = head_.load(std::memory_order_acquire);
    const uint64_t tail = tail_.load(std::memory_order_relaxed);
    return static_cast<size_t>(head - tail) * frameBytes_;
  }

  // Producer view: free bytes (always a whole-frame multiple).
  size_t SpaceAvailable() const
  {
    const uint64_t head = head_.load(std::memory_order_relaxed);
    const uint64_t tail = tail_.load(std::memory_order_acquire);
    return (capacityFrames_ - static_cast<size_t>(head - tail)) * frameBytes_;
  }

  // Producer: append complete frames only. A trailing partial frame is ignored.
  // Returns bytes actually written; a short write is frame-aligned.
  size_t Write(const void* src, size_t n)
  {
    if (!src || frameBytes_ == 0) return 0;
    const size_t requestedFrames = n / frameBytes_;
    if (requestedFrames == 0) return 0;

    const uint64_t head = head_.load(std::memory_order_relaxed);
    const uint64_t tail = tail_.load(std::memory_order_acquire);
    const size_t usedFrames = static_cast<size_t>(head - tail);
    const size_t freeFrames = capacityFrames_ - std::min(usedFrames, capacityFrames_);
    const size_t frames = std::min(requestedFrames, freeFrames);
    if (frames == 0) return 0;

    const size_t offsetFrames = static_cast<size_t>(head) & maskFrames_;
    const size_t firstFrames = std::min(frames, capacityFrames_ - offsetFrames);
    const size_t firstBytes = firstFrames * frameBytes_;
    const size_t totalBytes = frames * frameBytes_;
    std::memcpy(buf_.data() + offsetFrames * frameBytes_, src, firstBytes);
    if (totalBytes > firstBytes)
      std::memcpy(buf_.data(), static_cast<const uint8_t*>(src) + firstBytes,
                  totalBytes - firstBytes);

    head_.store(head + frames, std::memory_order_release);
    return totalBytes;
  }

  // Consumer view: copy complete frames without consuming.
  size_t Peek(void* dst, size_t n) const
  {
    if (!dst || frameBytes_ == 0) return 0;
    const size_t requestedFrames = n / frameBytes_;
    if (requestedFrames == 0) return 0;

    const uint64_t head = head_.load(std::memory_order_acquire);
    const uint64_t tail = tail_.load(std::memory_order_relaxed);
    const size_t frames = std::min(requestedFrames, static_cast<size_t>(head - tail));
    if (frames == 0) return 0;

    const size_t offsetFrames = static_cast<size_t>(tail) & maskFrames_;
    const size_t firstFrames = std::min(frames, capacityFrames_ - offsetFrames);
    const size_t firstBytes = firstFrames * frameBytes_;
    const size_t totalBytes = frames * frameBytes_;
    std::memcpy(dst, buf_.data() + offsetFrames * frameBytes_, firstBytes);
    if (totalBytes > firstBytes)
      std::memcpy(static_cast<uint8_t*>(dst) + firstBytes, buf_.data(),
                  totalBytes - firstBytes);
    return totalBytes;
  }

  // Consumer: advance by complete frames only.
  void Discard(size_t n)
  {
    const size_t requestedFrames = n / frameBytes_;
    const uint64_t head = head_.load(std::memory_order_acquire);
    const uint64_t tail = tail_.load(std::memory_order_relaxed);
    const size_t frames = std::min(requestedFrames, static_cast<size_t>(head - tail));
    if (frames)
      tail_.store(tail + frames, std::memory_order_release);
  }

  size_t Read(void* dst, size_t n)
  {
    const size_t got = Peek(dst, n);
    Discard(got);
    return got;
  }

  // Consumer: drop everything (used on (re)start).
  void Clear()
  {
    tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
  }

private:
  std::vector<uint8_t> buf_;
  size_t frameBytes_ = 1;
  size_t capacityFrames_ = 1;
  size_t maskFrames_ = 0;
  std::atomic<uint64_t> head_{0}; // complete frames written (producer)
  std::atomic<uint64_t> tail_{0}; // complete frames read (consumer)
};
