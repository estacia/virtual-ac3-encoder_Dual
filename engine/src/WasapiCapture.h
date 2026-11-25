// WasapiCapture.h
//
// Captures PCM from a WASAPI endpoint (the virtual cable's recording side) in shared,
// event-driven mode and pushes interleaved samples into a RingBuffer. Runs on its own
// thread; this is the producer / input-clock side of the pipeline.
#pragma once

#include "ComUtil.h"
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
  //                      A short polling fallback supports older loopback implementations.
  //   preferredPeriodHns: requested IAudioClient3 period in 100ns units (default 50000 = 5ms).
  //                       Loopback and fallback paths use the Windows audio-engine period.
  bool Init(IMMDevice* dev, bool loopback = false, REFERENCE_TIME preferredPeriodHns = 50000);
  // Destination ring for captured samples (set before Start, once it can be sized).
  void SetRing(RingBuffer* ring) { ring_ = ring; }
  bool Start();
  void Stop();

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
};
