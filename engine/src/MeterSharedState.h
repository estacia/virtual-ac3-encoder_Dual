// MeterSharedState.h — Lock-free shared memory telemetry between engine.exe and control_center.exe.
#pragma once

#include <cstdint>
#include <windows.h>
#include "AudioDiagnostics.h"

constexpr const wchar_t* kMeterSharedMemName = L"Local\\VirtualAc3EncoderMeters";
constexpr uint32_t kMeterMagic = 0x56414333; // "VAC3"
constexpr uint32_t kMeterVersion = 2;

#pragma pack(push, 8)
struct MeterSharedState
{
  uint32_t magic;         // kMeterMagic
  uint32_t version;       // kMeterVersion
  uint32_t active;        // 1 if engine is running and actively processing audio, 0 if paused/stopped
  uint32_t sampleRate;    // e.g. 48000
  uint32_t channels;      // 6 (5.1) or 2 (stereo)
  uint32_t isDts;         // 1 if DTS, 0 if AC3, 2 if PCM
  uint32_t latencyPreset; // 0=Extreme, 1=Fast, 2=Balanced, 3=Safe, 4=Relaxed
  uint32_t safeFrames;    // buffer safety margin in frames
  float peakLevels[6];    // Linear 0.0f .. 1.0f (FL, FR, C, LFE, SL, SR)
  float rmsLevels[6];     // Linear RMS energy (FL, FR, C, LFE, SL, SR)
  uint64_t frameCounter;  // Monotonically increasing frame counter
  volatile LONG64 captureDroppedFrames;
  volatile LONG64 outputUnderrunFrames;
  volatile LONG64 outputUnderrunPackets;
  volatile LONG64 wasapiErrorCount;
  volatile LONG64 eventTimeoutCount;
  volatile LONG64 driftTrimFrames;
  volatile LONG64 ringFillFrames;
  volatile LONG64 ringPeakFrames;
  uint32_t ringCapacityFrames;
};
#pragma pack(pop)

class MeterSharedMemory
{
public:
  MeterSharedMemory() = default;
  ~MeterSharedMemory() { Close(); }

  // Producer (engine.exe): creates or opens the mapping with write access
  bool OpenProducer()
  {
    Close();
    hMap_ = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0,
                               sizeof(MeterSharedState), kMeterSharedMemName);
    if (!hMap_)
      return false;
    state_ = static_cast<MeterSharedState*>(MapViewOfFile(hMap_, FILE_MAP_ALL_ACCESS, 0, 0,
                                                          sizeof(MeterSharedState)));
    if (!state_)
    {
      CloseHandle(hMap_);
      hMap_ = nullptr;
      return false;
    }
    ZeroMemory(state_, sizeof(MeterSharedState));
    state_->magic = kMeterMagic;
    state_->version = kMeterVersion;
    state_->channels = 6;
    state_->sampleRate = 48000;
    return true;
  }

  // Consumer (control_center.exe): opens existing mapping with read-only access
  bool OpenConsumer()
  {
    Close();
    hMap_ = OpenFileMappingW(FILE_MAP_READ, FALSE, kMeterSharedMemName);
    if (!hMap_)
      return false;
    state_ = static_cast<MeterSharedState*>(MapViewOfFile(hMap_, FILE_MAP_READ, 0, 0,
                                                          sizeof(MeterSharedState)));
    if (!state_)
    {
      CloseHandle(hMap_);
      hMap_ = nullptr;
      return false;
    }
    return (state_->magic == kMeterMagic);
  }

  void Close()
  {
    if (state_)
    {
      UnmapViewOfFile(state_);
      state_ = nullptr;
    }
    if (hMap_)
    {
      CloseHandle(hMap_);
      hMap_ = nullptr;
    }
  }

  MeterSharedState* Get() { return state_; }
  const MeterSharedState* Get() const { return state_; }
  bool IsOpen() const { return state_ != nullptr; }

private:
  HANDLE hMap_ = nullptr;
  MeterSharedState* state_ = nullptr;
};

inline void PublishAudioDiagnostics(MeterSharedState* state, const AudioDiagnostics* diagnostics,
                                    size_t ringUsedBytes, size_t ringCapacityBytes,
                                    size_t bytesPerFrame)
{
  if (!state || !diagnostics) return;
  InterlockedExchange64(&state->captureDroppedFrames,
      static_cast<LONG64>(diagnostics->captureDroppedFrames.load(std::memory_order_relaxed)));
  InterlockedExchange64(&state->outputUnderrunFrames,
      static_cast<LONG64>(diagnostics->outputUnderrunFrames.load(std::memory_order_relaxed)));
  InterlockedExchange64(&state->outputUnderrunPackets,
      static_cast<LONG64>(diagnostics->outputUnderrunPackets.load(std::memory_order_relaxed)));
  InterlockedExchange64(&state->wasapiErrorCount,
      static_cast<LONG64>(diagnostics->captureWasapiErrors.load(std::memory_order_relaxed) +
                          diagnostics->outputWasapiErrors.load(std::memory_order_relaxed)));
  InterlockedExchange64(&state->eventTimeoutCount,
      static_cast<LONG64>(diagnostics->eventTimeouts.load(std::memory_order_relaxed)));
  InterlockedExchange64(&state->driftTrimFrames,
      static_cast<LONG64>(diagnostics->driftTrimFrames.load(std::memory_order_relaxed)));

  const LONG64 fillFrames = bytesPerFrame
      ? static_cast<LONG64>(ringUsedBytes / bytesPerFrame) : 0;
  InterlockedExchange64(&state->ringFillFrames, fillFrames);
  InterlockedExchange(&reinterpret_cast<volatile LONG*>(&state->ringCapacityFrames),
      bytesPerFrame ? static_cast<LONG>(ringCapacityBytes / bytesPerFrame) : 0);
  LONG64 peak = InterlockedCompareExchange64(&state->ringPeakFrames, 0, 0);
  while (fillFrames > peak)
  {
    const LONG64 previous = InterlockedCompareExchange64(&state->ringPeakFrames, fillFrames, peak);
    if (previous == peak) break;
    peak = previous;
  }
}
