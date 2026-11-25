// WasapiCapture.h
//
// Captures PCM from a WASAPI endpoint (the virtual cable's recording side) in shared,
// event-driven mode and pushes interleaved samples into a RingBuffer. Runs on its own
// thread; this is the producer / input-clock side of the pipeline.
#pragma once

#include "ComUtil.h"
#include "AudioDiagnostics.h"
#include "RingBuffer.h"

#include <audioclient.h>
#include <mmdeviceapi.h>

#include <atomic>
#include <thread>

struct CaptureFormat
{
  unsigned channels = 0;
  unsigned sampleRate = 0;
  unsigned bits = 0;       // bits per sample in the captured stream
  bool     isFloat = false;
  uint32_t channelMask = 0; // WASAPI speaker mask (0 if unknown)

  size_t bytesPerFrame() const { return static_cast<size_t>(channels) * (bits / 8); }
};

class WasapiCapture
{
public:
  WasapiCapture() = default;
  ~WasapiCapture();

  // Opens `dev` for shared event-driven capture and learns its mix format.
  //   loopback == false: a real capture endpoint; uses IAudioClient3 when available.
  //   loopback == true : a RENDER endpoint captured via event-driven LOOPBACK.
  //                      Uses the Windows audio-engine period; IAudioClient3 custom
  //                      periods are not supported with AUDCLNT_STREAMFLAGS_LOOPBACK.
  //   loopback == false: requests preferredPeriodHns through IAudioClient3 when available.
  //   preferredPeriodHns: requested period in 100ns units (default 50000 = 5ms).
  bool Init(IMMDevice* dev, bool loopback = false, REFERENCE_TIME preferredPeriodHns = 50000,
            AudioDiagnostics* diagnostics = nullptr);
  // Destination ring for captured samples (set before Start, once it can be sized).
  void SetRing(RingBuffer* ring) { ring_ = ring; }
  bool Start();
  void Stop();
  bool HasFailed() const { return failed_.load(std::memory_order_relaxed); }

  const CaptureFormat& Format() const { return fmt_; }

private:
  void ThreadProc();

  RingBuffer*               ring_ = nullptr;
  ComPtr<IAudioClient>      client_;
  ComPtr<IAudioCaptureClient> capture_;
  CaptureFormat             fmt_;

  bool                      loopback_ = false;
  bool                      isEventDriven_ = false;
  REFERENCE_TIME            preferredPeriodHns_ = 50000;
  HANDLE                    dataEvent_ = nullptr;
  HANDLE                    stopEvent_ = nullptr;
  std::thread               thread_;
  std::atomic_bool          running_{false};
  std::atomic_bool          failed_{false};
  AudioDiagnostics*         diagnostics_ = nullptr;
};
