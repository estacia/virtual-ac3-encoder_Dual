// WasapiCapture.cpp — see WasapiCapture.h.
#include "WasapiCapture.h"

#include <mmreg.h>
#include <ksmedia.h>

#include <algorithm>
#include <cstdint>
#include <avrt.h>
#include <vector>

WasapiCapture::~WasapiCapture()
{
  Stop();
  if (dataEvent_) CloseHandle(dataEvent_);
  if (stopEvent_) CloseHandle(stopEvent_);
}

bool WasapiCapture::Init(IMMDevice* dev, bool loopback, REFERENCE_TIME preferredPeriodHns,
                          AudioDiagnostics* diagnostics)
{
  if (!dev)
    return false;

  diagnostics_ = diagnostics;
  failed_.store(false, std::memory_order_relaxed);
  loopback_ = loopback;
  preferredPeriodHns_ = (preferredPeriodHns > 0) ? preferredPeriodHns : 50000;

  HRESULT hr = dev->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr, &client_);
  if (FAILED(hr))
  {
    std::fprintf(stderr, "[WasapiCapture] Activate IAudioClient failed: %s\n", HrStr(hr).c_str());
    return false;
  }

  WAVEFORMATEX* mix = nullptr;
  hr = client_->GetMixFormat(&mix);
  if (FAILED(hr) || !mix)
  {
    std::fprintf(stderr, "[WasapiCapture] GetMixFormat failed: %s\n", HrStr(hr).c_str());
    if (mix) CoTaskMemFree(mix);
    return false;
  }

  fmt_.channels = mix->nChannels;
  fmt_.sampleRate = mix->nSamplesPerSec;
  fmt_.bits = mix->wBitsPerSample;
  fmt_.isFloat = (mix->wFormatTag == WAVE_FORMAT_IEEE_FLOAT);
  if (mix->wFormatTag == WAVE_FORMAT_EXTENSIBLE && mix->cbSize >= 22)
  {
    auto* ext = reinterpret_cast<WAVEFORMATEXTENSIBLE*>(mix);
    fmt_.channelMask = ext->dwChannelMask;
    fmt_.isFloat = (ext->SubFormat == KSDATAFORMAT_SUBTYPE_IEEE_FLOAT);
  }

  // Windows 10 version 1703 and later supports event-driven loopback capture.
  // ThreadProc retains a short polling fallback for older loopback implementations.
  const DWORD flags = AUDCLNT_STREAMFLAGS_EVENTCALLBACK |
                      (loopback_ ? AUDCLNT_STREAMFLAGS_LOOPBACK : 0);
  isEventDriven_ = true;
  bool usedAudioClient3 = false;
  UINT32 selectedPeriodFrames = 0;

  // IAudioClient3::InitializeSharedAudioStream does not support the LOOPBACK
  // stream flag. Use it for direct recording-endpoint capture only; loopback must
  // use IAudioClient::Initialize and the Windows audio-engine period. For direct
  // capture, request 5 ms (10 ms in Safe/Relaxed modes), aligned to the endpoint's
  // supported fundamental period rather than blindly selecting its minimum.
  if (!loopback_)
  {
    ComPtr<IAudioClient3> client3;
    HRESULT queryHr = client_.As(&client3);
    if (SUCCEEDED(queryHr))
    {
      UINT32 defaultPeriod = 0;
      UINT32 fundamentalPeriod = 0;
      UINT32 minPeriod = 0;
      UINT32 maxPeriod = 0;
      hr = client3->GetSharedModeEnginePeriod(
          mix, &defaultPeriod, &fundamentalPeriod, &minPeriod, &maxPeriod);

      if (SUCCEEDED(hr) && fundamentalPeriod > 0 && minPeriod <= maxPeriod)
      {
        const uint64_t targetNumerator =
            static_cast<uint64_t>(preferredPeriodHns_) * fmt_.sampleRate;
        const UINT32 targetFrames = static_cast<UINT32>(
            (targetNumerator + 9999999ULL) / 10000000ULL);

        const UINT32 minAligned =
            ((minPeriod + fundamentalPeriod - 1) / fundamentalPeriod) * fundamentalPeriod;
        const UINT32 maxAligned =
            (maxPeriod / fundamentalPeriod) * fundamentalPeriod;
        UINT32 periodFrames =
            ((targetFrames + fundamentalPeriod - 1) / fundamentalPeriod) * fundamentalPeriod;

        if (minAligned <= maxAligned)
        {
          periodFrames = std::clamp(periodFrames, minAligned, maxAligned);
          hr = client3->InitializeSharedAudioStream(
              AUDCLNT_STREAMFLAGS_EVENTCALLBACK, periodFrames, mix, nullptr);
          if (SUCCEEDED(hr))
          {
            usedAudioClient3 = true;
            selectedPeriodFrames = periodFrames;
          }
        }
        else
        {
          hr = AUDCLNT_E_INVALID_DEVICE_PERIOD;
        }
      }
      else if (SUCCEEDED(hr))
      {
        hr = AUDCLNT_E_INVALID_DEVICE_PERIOD;
      }
    }
    else
    {
      hr = queryHr;
    }

    if (!usedAudioClient3)
    {
      std::fprintf(stderr,
          "[WasapiCapture] IAudioClient3 low-latency initialization unavailable (%s); "
          "using the shared engine period.\n",
          HrStr(hr).c_str());
    }
  }

  // Shared-mode event-driven streams request the engine-managed buffer with (0, 0).
  // Re-activate after a failed IAudioClient3 attempt because Initialize is one-shot.
  if (!usedAudioClient3)
  {
    client_.Reset();
    hr = dev->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr, &client_);
    if (SUCCEEDED(hr))
      hr = client_->Initialize(AUDCLNT_SHAREMODE_SHARED, flags, 0, 0, mix, nullptr);
  }

  CoTaskMemFree(mix);
  if (FAILED(hr))
  {
    std::fprintf(stderr, "[WasapiCapture] shared event-driven Initialize failed: %s\n",
                 HrStr(hr).c_str());
    return false;
  }

  stopEvent_ = CreateEventW(nullptr, TRUE, FALSE, nullptr); // manual-reset
  if (!stopEvent_) return false;
  dataEvent_ = CreateEventW(nullptr, FALSE, FALSE, nullptr);
  if (!dataEvent_) return false;

  hr = client_->SetEventHandle(dataEvent_);
  if (FAILED(hr))
  {
    std::fprintf(stderr, "[WasapiCapture] SetEventHandle failed: %s\n", HrStr(hr).c_str());
    return false;
  }

  hr = client_->GetService(__uuidof(IAudioCaptureClient), &capture_);
  if (FAILED(hr))
  {
    std::fprintf(stderr, "[WasapiCapture] GetService(IAudioCaptureClient) failed: %s\n",
                 HrStr(hr).c_str());
    return false;
  }

  UINT32 bufferFrames = 0;
  client_->GetBufferSize(&bufferFrames);
  const double bufferMs = fmt_.sampleRate
      ? 1000.0 * bufferFrames / fmt_.sampleRate : 0.0;
  const char* mode = loopback_ ? "loopback event-driven"
      : (usedAudioClient3 ? "IAudioClient3 event-driven" : "shared event-driven fallback");
  char periodLabel[64] = "audio-engine default";
  if (usedAudioClient3)
    std::snprintf(periodLabel, sizeof(periodLabel), "%u frames (~%.2f ms)",
                  selectedPeriodFrames, 1000.0 * selectedPeriodFrames / fmt_.sampleRate);

  std::printf("[WasapiCapture] %s: %u ch, %u Hz, %u-bit %s, mask=0x%X; "
              "buffer=%u frames (~%.2f ms), period=%s\n",
              mode, fmt_.channels, fmt_.sampleRate, fmt_.bits,
              fmt_.isFloat ? "float" : "int", fmt_.channelMask,
              bufferFrames, bufferMs, periodLabel);
  return true;
}

bool WasapiCapture::Start()
{
  if (!ring_ || !client_ || !capture_ || !dataEvent_ || !stopEvent_)
  {
    std::fprintf(stderr, "[WasapiCapture] Start requested before successful Init / SetRing\n");
    return false;
  }
  if (running_.exchange(true))
    return true;
  ResetEvent(stopEvent_);
  const HRESULT hr = client_->Start();
  if (FAILED(hr))
  {
    running_.store(false);
    std::fprintf(stderr, "[WasapiCapture] client Start failed: %s\n", HrStr(hr).c_str());
    return false;
  }
  thread_ = std::thread(&WasapiCapture::ThreadProc, this);
  return true;
}

void WasapiCapture::Stop()
{
  const bool wasRunning = running_.exchange(false, std::memory_order_relaxed);
  if (wasRunning && stopEvent_) SetEvent(stopEvent_);
  // Join even when the worker exited because of an error; otherwise its std::thread
  // remains joinable and a later restart/destructor can terminate the process.
  if (thread_.joinable()) thread_.join();
  if (client_) {
    if (wasRunning) client_->Stop();
    client_.Reset();
  }
  capture_.Reset();
  if (dataEvent_) { CloseHandle(dataEvent_); dataEvent_ = nullptr; }
  if (stopEvent_) { CloseHandle(stopEvent_); stopEvent_ = nullptr; }
}

void WasapiCapture::ThreadProc()
{
  DWORD taskIndex = 0;
  HANDLE mmcss = AvSetMmThreadCharacteristicsW(L"Pro Audio", &taskIndex);

  const size_t bpf = fmt_.bytesPerFrame();
  auto recordError = [&]() {
    failed_.store(true, std::memory_order_relaxed);
    if (diagnostics_)
      diagnostics_->captureWasapiErrors.fetch_add(1, std::memory_order_relaxed);
  };

  auto drain = [&]() -> bool {
    for (;;)
    {
      UINT32 packet = 0;
      HRESULT hr = capture_->GetNextPacketSize(&packet);
      if (FAILED(hr))
      {
        std::fprintf(stderr, "[WasapiCapture] GetNextPacketSize failed: %s\n", HrStr(hr).c_str());
        recordError();
        return false;
      }
      if (packet == 0) return true;

      BYTE* data = nullptr;
      UINT32 frames = 0;
      DWORD flags = 0;
      hr = capture_->GetBuffer(&data, &frames, &flags, nullptr, nullptr);
      if (FAILED(hr))
      {
        std::fprintf(stderr, "[WasapiCapture] GetBuffer failed: %s\n", HrStr(hr).c_str());
        recordError();
        return false;
      }

      const size_t bytes = static_cast<size_t>(frames) * bpf;
      size_t written = 0;
      if (flags & AUDCLNT_BUFFERFLAGS_SILENT)
      {
        static thread_local std::vector<uint8_t> zeros;
        if (zeros.size() < bytes) zeros.resize(bytes, 0);
        written = ring_->Write(zeros.data(), bytes);
      }
      else if (data)
      {
        written = ring_->Write(data, bytes);
      }
      else if (bytes > 0)
      {
        // A null data pointer without SILENT is unexpected; preserve stream timing
        // with silence rather than feeding uninitialized memory into the encoder.
        static thread_local std::vector<uint8_t> zeros;
        if (zeros.size() < bytes) zeros.resize(bytes, 0);
        written = ring_->Write(zeros.data(), bytes);
      }

      if (diagnostics_ && bpf > 0 && written < bytes)
        diagnostics_->captureDroppedFrames.fetch_add((bytes - written) / bpf,
                                                      std::memory_order_relaxed);

      hr = capture_->ReleaseBuffer(frames);
      if (FAILED(hr))
      {
        std::fprintf(stderr, "[WasapiCapture] ReleaseBuffer failed: %s\n", HrStr(hr).c_str());
        recordError();
        return false;
      }
    }
  };

  if (isEventDriven_ && dataEvent_)
  {
    HANDLE waits[2] = { dataEvent_, stopEvent_ };
    const DWORD timeoutMs = loopback_ ? 2 : INFINITE;

    while (running_.load(std::memory_order_relaxed))
    {
      const DWORD w = WaitForMultipleObjects(2, waits, FALSE, timeoutMs);
      if (w == WAIT_OBJECT_0 + 1)
        break;
      if (w == WAIT_FAILED)
      {
        std::fprintf(stderr, "[WasapiCapture] WaitForMultipleObjects failed: %lu\n",
                     GetLastError());
        recordError();
        break;
      }
      // Loopback's 2 ms timeout is only a compatibility poll for systems where the
      // event callback is unavailable; on supported systems the data event wakes us.
      if (!drain()) break;
    }
  }
  else
  {
    while (running_.load(std::memory_order_relaxed))
    {
      const DWORD w = WaitForSingleObject(stopEvent_, 5);
      if (w == WAIT_OBJECT_0) break;
      if (w == WAIT_FAILED)
      {
        std::fprintf(stderr, "[WasapiCapture] WaitForSingleObject failed: %lu\n",
                     GetLastError());
        recordError();
        break;
      }
      if (!drain()) break;
    }
  }

  if (mmcss) AvRevertMmThreadCharacteristics(mmcss);
}
