// WasapiPassthrough.h
//
// Opens the optical endpoint in EXCLUSIVE, event-driven IEC 61937 mode. A dedicated encoder
// worker is the sole RingBuffer consumer and produces complete AC3/DTS bursts. The render
// thread only dequeues bursts and copies their carrier frames to WASAPI. Clock-drift trimming
// is requested by the render thread and applied by the encoder worker to preserve SPSC access.
// Underruns emit a valid encoded silence burst so the receiver stays locked.
#pragma once

#include "ComUtil.h"
#include "AudioDiagnostics.h"
#include "MeterSharedState.h"
#include "RingBuffer.h"
#include "SpdifEncoder.h"
#include "WasapiCapture.h" // CaptureFormat
#include <array>

#include <audioclient.h>
#include <mmdeviceapi.h>

#include <atomic>
#include <cstdint>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

class WasapiPassthrough
{
public:
  struct Params
  {
    SpdifEncoder::Codec codec = SpdifEncoder::Codec::AC3;
    int64_t  bitRate = 640000;
    int64_t  dtsBitRate = 1536000;
    uint32_t safeFrames = 1536;    // target PCM cushion after the initial AC-3 frame
    uint32_t renderPeriodFrames = 256; // ~5.33 ms at 48 kHz; endpoint may force fallback
    SpdifEncoder::Upmix upmix = SpdifEncoder::Upmix::Off;
    int      surroundWindowSize = 4096; // 1024 (512-sample hop) for low-latency Extreme mode
    bool     upmixSurround = false; // legacy compatibility flag (mapped to upmix if true)
  };

  WasapiPassthrough() = default;
  ~WasapiPassthrough();

  // Non-intrusive capability check: does `dev` accept AC3 / IEC 61937 in exclusive mode at
  // `rate`? Uses IsFormatSupported only (does not seize the device).
  static bool ProbeAc3(IMMDevice* dev, int rate);
  static bool ProbeDts(IMMDevice* dev, int rate);

  // dev    : optical output endpoint (must support AC3 passthrough in exclusive mode)
  // ring   : shared input ring (filled by WasapiCapture), holding capFmt frames
  // capFmt : capture format — defines the encoder's input (channels/layout/sample fmt/rate)
  bool Init(IMMDevice* dev, RingBuffer* ring, const CaptureFormat& capFmt, const Params& p,
            AudioDiagnostics* diagnostics = nullptr);
  bool Start();
  void Stop();
  bool HasFailed() const { return failed_.load(std::memory_order_relaxed); }
  // Thread-safe live trim for the six AC3 output channels, in dB.
  void SetChannelGainsDb(const std::array<float, 6>& gainsDb) { enc_.SetChannelGainsDb(gainsDb); }

private:
  bool InitExclusive(int rate);
  bool FillRenderBuffer(BYTE* out);
  void EncoderThreadProc();
  void UpdateEncoderSnapshot();
  void PublishDiagnostics();
  void ThreadProc();

  ComPtr<IMMDevice>          dev_;
  ComPtr<IAudioClient>       client_;
  ComPtr<IAudioRenderClient> render_;

  RingBuffer*    ring_ = nullptr;
  CaptureFormat  capFmt_;
  size_t         capBytesPerFrame_ = 0;
  Params         params_;
  AudioDiagnostics* diagnostics_ = nullptr;

  SpdifEncoder   enc_;
  int            framesPerPacket_ = 1536;
  size_t         burstBytes_ = 0;
  static constexpr size_t kMaxBurstBytes = SpdifEncoder::kMaxBytesPerPacket; // AC3 capacity; also fits DTS
  static constexpr int kCarrierBytesPerFrame = 4; // 2ch * 16-bit IEC60958

  UINT32 bufferFrames_ = 0; // exclusive buffer size, in carrier frames
  int    burstsPerCycle_ = 0;

  HANDLE dataEvent_ = nullptr;
  HANDLE stopEvent_ = nullptr;
  std::thread thread_;        // short-period WASAPI render thread
  std::thread encoderThread_; // PCM -> AC-3/DTS -> IEC 61937 worker
  std::atomic_bool running_{false};
  std::atomic_bool failed_{false};

  static constexpr size_t kEncodedQueueCapacity = 12;
  mutable std::mutex queueMutex_;
  std::condition_variable queueCv_;
  size_t queueReadIndex_ = 0;
  size_t queueWriteIndex_ = 0;
  size_t queueCount_ = 0;
  std::atomic<uint32_t> encoderPendingFrames_{0};
  // The render thread only requests trims; the encoder worker applies them as the sole
  // RingBuffer consumer. This preserves the ring's single-consumer contract.
  std::atomic<uint64_t> pendingTrimFrames_{0};

  std::atomic<uint32_t> filterStateSnapshot_{0};
  std::atomic<uint32_t> filterHopSnapshot_{0};
  std::atomic<uint32_t> filterFifoSnapshot_{0};
  std::atomic<uint32_t> filterTargetSnapshot_{0};

  // Drift tracking (render thread only).
  uint32_t driftFrameAccumulator_ = 0;
  uint32_t minAvail_ = 0xFFFFFFFFu;
  bool     primed_ = false;
  uint32_t burstOffsetFrames_ = 1536;
  bool     currentBurstHasAudio_ = false;

  // Encoder worker and render thread buffers.
  std::vector<uint8_t> staging_;
  std::vector<uint8_t> silence_;
  std::vector<uint8_t> encoderBurst_;
  std::vector<uint8_t> queuedBursts_;
  std::vector<uint8_t> currentBurst_;
  std::vector<uint8_t> silenceBurst_;
  MeterSharedMemory    sharedMem_;
};
