// WasapiPassthrough.cpp — see WasapiPassthrough.h.
#include "WasapiPassthrough.h"

#include <mmreg.h>
#include <ks.h>
#include <ksmedia.h>
#include <avrt.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>

extern "C" {
#include <libavutil/channel_layout.h>
#include <libavutil/samplefmt.h>
}

namespace {
AVSampleFormat MapSampleFmt(const CaptureFormat& f)
{
  if (f.isFloat && f.bits == 32) return AV_SAMPLE_FMT_FLT;
  if (!f.isFloat && f.bits == 16) return AV_SAMPLE_FMT_S16;
  if (!f.isFloat && f.bits == 32) return AV_SAMPLE_FMT_S32;
  return AV_SAMPLE_FMT_NONE;
}
void FillAc3Format(WAVEFORMATEXTENSIBLE_IEC61937& w, int rate, bool extended)
{
  ZeroMemory(&w, sizeof w);
  WAVEFORMATEXTENSIBLE& x = w.FormatExt;
  x.Format.wFormatTag = WAVE_FORMAT_EXTENSIBLE;
  x.Format.nChannels = 2;
  x.Format.nSamplesPerSec = rate;
  x.Format.wBitsPerSample = 16;
  x.Format.nBlockAlign = 4;
  x.Format.nAvgBytesPerSec = rate * 4;
  x.Samples.wValidBitsPerSample = 16;
  x.dwChannelMask = KSAUDIO_SPEAKER_5POINT1;
  x.SubFormat = KSDATAFORMAT_SUBTYPE_IEC61937_DOLBY_DIGITAL;
  if (extended)
  {
    x.Format.cbSize = sizeof(WAVEFORMATEXTENSIBLE_IEC61937) - sizeof(WAVEFORMATEX);
    w.dwEncodedSamplesPerSec = rate;
    w.dwEncodedChannelCount = 6;
    w.dwAverageBytesPerSec = 0;
  }
  else x.Format.cbSize = sizeof(WAVEFORMATEXTENSIBLE) - sizeof(WAVEFORMATEX);
}
void FillDtsFormat(WAVEFORMATEXTENSIBLE_IEC61937& w, int rate, bool extended)
{
  ZeroMemory(&w, sizeof w);
  WAVEFORMATEXTENSIBLE& x = w.FormatExt;
  x.Format.wFormatTag = WAVE_FORMAT_EXTENSIBLE;
  x.Format.nChannels = 2;
  x.Format.nSamplesPerSec = rate;
  x.Format.wBitsPerSample = 16;
  x.Format.nBlockAlign = 4;
  x.Format.nAvgBytesPerSec = rate * 4;
  x.Samples.wValidBitsPerSample = 16;
  x.dwChannelMask = KSAUDIO_SPEAKER_5POINT1;
  x.SubFormat = KSDATAFORMAT_SUBTYPE_IEC61937_DTS;
  if (extended)
  {
    x.Format.cbSize = sizeof(WAVEFORMATEXTENSIBLE_IEC61937) - sizeof(WAVEFORMATEX);
    w.dwEncodedSamplesPerSec = rate;
    w.dwEncodedChannelCount = 6;
    w.dwAverageBytesPerSec = 0;
  }
  else x.Format.cbSize = sizeof(WAVEFORMATEXTENSIBLE) - sizeof(WAVEFORMATEX);
}
} // namespace

WasapiPassthrough::~WasapiPassthrough()
{
  Stop();
  if (dataEvent_) CloseHandle(dataEvent_);
  if (stopEvent_) CloseHandle(stopEvent_);
}

bool WasapiPassthrough::ProbeAc3(IMMDevice* dev, int rate)
{
  ComPtr<IAudioClient> c;
  if (FAILED(dev->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr, &c))) return false;
  WAVEFORMATEXTENSIBLE_IEC61937 w;
  FillAc3Format(w, rate, true);
  if (c->IsFormatSupported(AUDCLNT_SHAREMODE_EXCLUSIVE, &w.FormatExt.Format, nullptr) == S_OK) return true;
  FillAc3Format(w, rate, false);
  return c->IsFormatSupported(AUDCLNT_SHAREMODE_EXCLUSIVE, &w.FormatExt.Format, nullptr) == S_OK;
}

bool WasapiPassthrough::ProbeDts(IMMDevice* dev, int rate)
{
  ComPtr<IAudioClient> c;
  if (FAILED(dev->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr, &c))) return false;
  WAVEFORMATEXTENSIBLE_IEC61937 w;
  FillDtsFormat(w, rate, true);
  if (c->IsFormatSupported(AUDCLNT_SHAREMODE_EXCLUSIVE, &w.FormatExt.Format, nullptr) == S_OK) return true;
  FillDtsFormat(w, rate, false);
  return c->IsFormatSupported(AUDCLNT_SHAREMODE_EXCLUSIVE, &w.FormatExt.Format, nullptr) == S_OK;
}

bool WasapiPassthrough::Init(IMMDevice* dev, RingBuffer* ring, const CaptureFormat& capFmt,
                             const Params& p, AudioDiagnostics* diagnostics)
{
  dev_ = dev;
  ring_ = ring;
  capFmt_ = capFmt;
  params_ = p;
  diagnostics_ = diagnostics;
  failed_.store(false, std::memory_order_relaxed);
  capBytesPerFrame_ = capFmt.bytesPerFrame();

  const int rate = static_cast<int>(capFmt.sampleRate);
  if (params_.codec == SpdifEncoder::Codec::DTS && rate != 48000)
  {
    std::fprintf(stderr, "[WasapiPassthrough] DTS mode currently requires 48000 Hz input.\n");
    return false;
  }
  if (params_.codec == SpdifEncoder::Codec::AC3 && rate != 48000 && rate != 44100 && rate != 32000)
  {
    std::fprintf(stderr, "[WasapiPassthrough] AC3 capture rate %d unsupported; use 48000, 44100 or 32000 Hz.\n", rate);
    return false;
  }

  const AVSampleFormat inFmt = MapSampleFmt(capFmt);
  if (inFmt == AV_SAMPLE_FMT_NONE)
  {
    std::fprintf(stderr, "[WasapiPassthrough] unsupported capture sample format (%u-bit %s)\n",
                 capFmt.bits, capFmt.isFloat ? "float" : "int");
    return false;
  }

  SpdifEncoder::Params ep;
  ep.codec = params_.codec;
  ep.sampleRate = rate;
  ep.bitRate = params_.bitRate;
  ep.dtsBitRate = params_.dtsBitRate;
  ep.inSampleFmt = inFmt;
  ep.surroundWindowSize = params_.surroundWindowSize;
  ep.upmix = (params_.upmix != SpdifEncoder::Upmix::Off) ? params_.upmix
           : (params_.upmixSurround ? SpdifEncoder::Upmix::Surround : SpdifEncoder::Upmix::Off);
  if (capFmt.channelMask) av_channel_layout_from_mask(&ep.inLayout, capFmt.channelMask);
  else av_channel_layout_default(&ep.inLayout, static_cast<int>(capFmt.channels));

  const bool encOk = enc_.Init(ep);
  av_channel_layout_uninit(&ep.inLayout);
  if (!encOk) return false;

  framesPerPacket_ = enc_.FramesPerPacket();
  burstBytes_ = static_cast<size_t>(framesPerPacket_) * kCarrierBytesPerFrame;
  if (framesPerPacket_ <= 0 || burstBytes_ == 0 || burstBytes_ > kMaxBurstBytes)
  {
    std::fprintf(stderr, "[WasapiPassthrough] invalid encoder frame/burst size: %d frames, %zu bytes\n",
                 framesPerPacket_, burstBytes_);
    return false;
  }
  if (!InitExclusive(rate)) return false;

  const size_t packetBytes = static_cast<size_t>(framesPerPacket_) * capBytesPerFrame_;
  staging_.resize(packetBytes);
  silence_.assign(packetBytes, 0);
  encoderBurst_.resize(kMaxBurstBytes);
  queuedBursts_.resize(kEncodedQueueCapacity * kMaxBurstBytes);
  currentBurst_.resize(kMaxBurstBytes);
  silenceBurst_.resize(kMaxBurstBytes);

  // Prepare a valid pause/silence frame. An underrun must never emit malformed
  // all-zero carrier bytes in place of an IEC 61937 burst.
  const int silenceBytes = enc_.EncodePacket(silence_.data(), silenceBurst_.data(),
                                              static_cast<int>(silenceBurst_.size()));
  if (silenceBytes != static_cast<int>(burstBytes_))
  {
    std::fprintf(stderr, "[WasapiPassthrough] failed to prepare valid silence burst (%d bytes).\n",
                 silenceBytes);
    return false;
  }
  burstOffsetFrames_ = static_cast<uint32_t>(framesPerPacket_);
  UpdateEncoderSnapshot();
  sharedMem_.OpenProducer();
  return true;
}

bool WasapiPassthrough::InitExclusive(int rate)
{
  HR_FAIL(dev_->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr, &client_),
          "Activate IAudioClient");

  WAVEFORMATEXTENSIBLE_IEC61937 w;
  const bool dtsMode = params_.codec == SpdifEncoder::Codec::DTS;
  auto fillFormat = [&](bool extended) {
    if (dtsMode) FillDtsFormat(w, rate, extended);
    else FillAc3Format(w, rate, extended);
  };

  bool extended = true;
  fillFormat(true);
  HRESULT hr = client_->IsFormatSupported(AUDCLNT_SHAREMODE_EXCLUSIVE, &w.FormatExt.Format, nullptr);
  if (hr != S_OK)
  {
    fillFormat(false);
    extended = false;
    hr = client_->IsFormatSupported(AUDCLNT_SHAREMODE_EXCLUSIVE, &w.FormatExt.Format, nullptr);
  }
  if (hr != S_OK)
  {
    std::fprintf(stderr, "[WasapiPassthrough] device does not support %s passthrough (%s).\n",
                 dtsMode ? "DTS" : "AC-3", HrStr(hr).c_str());
    return false;
  }
  std::printf("[WasapiPassthrough] %s format accepted (%s IEC61937 format)\n",
              dtsMode ? "DTS" : "AC-3", extended ? "extended" : "plain");

  REFERENCE_TIME defaultPeriod = 0;
  REFERENCE_TIME minimumExclusivePeriod = 0;
  HR_FAIL(client_->GetDevicePeriod(&defaultPeriod, &minimumExclusivePeriod), "GetDevicePeriod");
  if (minimumExclusivePeriod <= 0) minimumExclusivePeriod = defaultPeriod;

  const REFERENCE_TIME burstHns =
      static_cast<REFERENCE_TIME>(10000000.0 * framesPerPacket_ / rate + 0.5);
  auto framesToHns = [rate](UINT32 frames) -> REFERENCE_TIME {
    return static_cast<REFERENCE_TIME>(10000000.0 * frames / rate + 0.5);
  };
  const UINT32 requestedFrames = std::clamp<UINT32>(
      params_.renderPeriodFrames, 64u, static_cast<UINT32>(std::max(64, framesPerPacket_)));
  const REFERENCE_TIME preferredHns = std::max(minimumExclusivePeriod, framesToHns(requestedFrames));
  const REFERENCE_TIME fullBurstHns = std::max(minimumExclusivePeriod, burstHns);
  const REFERENCE_TIME candidates[2] = { preferredHns, fullBurstHns };

  std::printf("[WasapiPassthrough] exclusive periods: shared-default=%.2f ms, minimum=%.2f ms, "
              "preferred=%.2f ms, full-burst=%.2f ms\n",
              defaultPeriod / 10000.0, minimumExclusivePeriod / 10000.0,
              preferredHns / 10000.0, fullBurstHns / 10000.0);

  bool initialized = false;
  bool fullBurstFallback = false;
  REFERENCE_TIME acceptedHns = preferredHns;
  for (int candidate = 0; candidate < 2 && !initialized; ++candidate)
  {
    if (candidate == 1 && candidates[1] == candidates[0]) break;
    REFERENCE_TIME bufferHns = candidates[candidate];
    for (unsigned attempt = 0; attempt < 4; ++attempt)
    {
      if (candidate > 0 || attempt > 0)
      {
        client_.Reset();
        HR_FAIL(dev_->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr, &client_),
                "re-Activate IAudioClient");
      }
      hr = client_->Initialize(AUDCLNT_SHAREMODE_EXCLUSIVE,
                               AUDCLNT_STREAMFLAGS_EVENTCALLBACK | AUDCLNT_STREAMFLAGS_NOPERSIST,
                               bufferHns, bufferHns, &w.FormatExt.Format, nullptr);
      if (SUCCEEDED(hr))
      {
        initialized = true;
        acceptedHns = bufferHns;
        fullBurstFallback = candidate == 1;
        break;
      }
      if (hr != AUDCLNT_E_BUFFER_SIZE_NOT_ALIGNED) break;

      UINT32 alignedFrames = 0;
      const HRESULT sizeHr = client_->GetBufferSize(&alignedFrames);
      if (FAILED(sizeHr) || alignedFrames == 0)
      {
        if (FAILED(sizeHr)) hr = sizeHr;
        break;
      }
      const REFERENCE_TIME nextHns = std::max(bufferHns, framesToHns(alignedFrames));
      if (nextHns == bufferHns) break;
      bufferHns = nextHns;
    }
  }

  if (!initialized)
  {
    if (hr == static_cast<HRESULT>(0x8889000A))
      std::fprintf(stderr, "[WasapiPassthrough] exclusive Initialize: device in use (0x8889000a).\n");
    else
      std::fprintf(stderr, "[WasapiPassthrough] exclusive Initialize failed: %s\n", HrStr(hr).c_str());
    return false;
  }

  HR_FAIL(client_->GetBufferSize(&bufferFrames_), "GetBufferSize");
  if (bufferFrames_ == 0) return false;
  dataEvent_ = CreateEventW(nullptr, FALSE, FALSE, nullptr);
  stopEvent_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
  if (!dataEvent_ || !stopEvent_) return false;
  HR_FAIL(client_->SetEventHandle(dataEvent_), "SetEventHandle");
  HR_FAIL(client_->GetService(__uuidof(IAudioRenderClient), &render_), "GetService(IAudioRenderClient)");

  std::printf("[WasapiPassthrough] render buffer=%u frames (~%.2f ms); %s burst=%d frames spans %.2f cycles%s; "
              "accepted Initialize=%.2f ms\n",
              bufferFrames_, 1000.0 * bufferFrames_ / rate,
              dtsMode ? "DTS" : "AC-3", framesPerPacket_,
              static_cast<double>(framesPerPacket_) / bufferFrames_,
              fullBurstFallback ? " (full-burst fallback)" : "",
              acceptedHns / 10000.0);
  return true;
}

bool WasapiPassthrough::FillRenderBuffer(BYTE* out)
{
  if (!out || currentBurst_.size() < burstBytes_ || silenceBurst_.size() < burstBytes_)
    return false;

  UINT32 outFrames = 0;
  bool hadAudio = false;
  while (outFrames < bufferFrames_)
  {
    if (burstOffsetFrames_ >= static_cast<uint32_t>(framesPerPacket_))
    {
      bool haveEncoded = false;
      {
        std::lock_guard<std::mutex> lock(queueMutex_);
        if (queueCount_ > 0)
        {
          const size_t offset = queueReadIndex_ * kMaxBurstBytes;
          std::memcpy(currentBurst_.data(), queuedBursts_.data() + offset, burstBytes_);
          queueReadIndex_ = (queueReadIndex_ + 1) % kEncodedQueueCapacity;
          --queueCount_;
          encoderPendingFrames_.fetch_sub(static_cast<uint32_t>(framesPerPacket_), std::memory_order_acq_rel);
          haveEncoded = true;
        }
      }
      if (haveEncoded)
      {
        currentBurstHasAudio_ = true;
        primed_ = true;
        queueCv_.notify_one();
      }
      else
      {
        std::memcpy(currentBurst_.data(), silenceBurst_.data(), burstBytes_);
        currentBurstHasAudio_ = false;
        if (primed_ && diagnostics_)
        {
          diagnostics_->outputUnderrunFrames.fetch_add(
              static_cast<uint64_t>(framesPerPacket_), std::memory_order_relaxed);
          diagnostics_->outputUnderrunPackets.fetch_add(1, std::memory_order_relaxed);
        }
      }
      burstOffsetFrames_ = 0;
    }

    const UINT32 burstRemaining = static_cast<UINT32>(framesPerPacket_) - burstOffsetFrames_;
    const UINT32 chunkFrames = std::min(burstRemaining, bufferFrames_ - outFrames);
    const size_t chunkBytes = static_cast<size_t>(chunkFrames) * kCarrierBytesPerFrame;
    std::memcpy(out + static_cast<size_t>(outFrames) * kCarrierBytesPerFrame,
                currentBurst_.data() + static_cast<size_t>(burstOffsetFrames_) * kCarrierBytesPerFrame,
                chunkBytes);
    if (currentBurstHasAudio_) hadAudio = true;
    outFrames += chunkFrames;
    burstOffsetFrames_ += chunkFrames;
  }
  return hadAudio;
}

void WasapiPassthrough::EncoderThreadProc()
{
  DWORD taskIndex = 0;
  HANDLE mmcss = AvSetMmThreadCharacteristicsW(L"Pro Audio", &taskIndex);
  if (mmcss && !AvSetMmThreadPriority(mmcss, AVRT_PRIORITY_HIGH))
    std::fprintf(stderr, "[WasapiPassthrough] encoder MMCSS HIGH failed: %lu\n", GetLastError());

  const size_t primeBytes =
      (static_cast<size_t>(framesPerPacket_) + params_.safeFrames) * capBytesPerFrame_;
  const size_t packetBytes = staging_.size();
  const std::chrono::milliseconds waitSlice(20);
  bool encoderPrimed = false;

  while (running_.load(std::memory_order_relaxed))
  {
    {
      std::unique_lock<std::mutex> lock(queueMutex_);
      queueCv_.wait(lock, [&] {
        return queueCount_ < kEncodedQueueCapacity || !running_.load(std::memory_order_relaxed);
      });
    }
    if (!running_.load(std::memory_order_relaxed)) break;

    size_t available = ring_->BytesAvailable();
    if (!encoderPrimed)
    {
      if (available < primeBytes)
      {
        ring_->WaitForData(primeBytes, waitSlice);
        continue;
      }
      encoderPrimed = true;
    }
    available = ring_->BytesAvailable();
    if (available < packetBytes)
    {
      if (available == 0) encoderPrimed = false;
      ring_->WaitForData(encoderPrimed ? packetBytes : primeBytes, waitSlice);
      continue;
    }
    if (ring_->Read(staging_.data(), packetBytes) != packetBytes) continue;

    encoderPendingFrames_.fetch_add(static_cast<uint32_t>(framesPerPacket_), std::memory_order_acq_rel);
    const int encodedBytes = enc_.EncodePacket(staging_.data(), encoderBurst_.data(),
                                               static_cast<int>(encoderBurst_.size()));
    UpdateEncoderSnapshot();

    if (encodedBytes < 0)
    {
      encoderPendingFrames_.fetch_sub(static_cast<uint32_t>(framesPerPacket_), std::memory_order_acq_rel);
      failed_.store(true, std::memory_order_release);
      running_.store(false, std::memory_order_release);
      if (diagnostics_) diagnostics_->outputWasapiErrors.fetch_add(1, std::memory_order_relaxed);
      std::fprintf(stderr, "[WasapiPassthrough] AC-3/DTS encoder worker failed.\n");
      queueCv_.notify_all();
      ring_->WakeWaiters();
      if (stopEvent_) SetEvent(stopEvent_);
      break;
    }
    if (encodedBytes != static_cast<int>(burstBytes_))
    {
      std::fprintf(stderr, "[WasapiPassthrough] encoded burst mismatch (%d != %zu); using silence.\n",
                   encodedBytes, burstBytes_);
      std::memcpy(encoderBurst_.data(), silenceBurst_.data(), burstBytes_);
    }

    {
      std::lock_guard<std::mutex> lock(queueMutex_);
      if (!running_.load(std::memory_order_relaxed))
      {
        encoderPendingFrames_.fetch_sub(static_cast<uint32_t>(framesPerPacket_), std::memory_order_acq_rel);
        break;
      }
      const size_t offset = queueWriteIndex_ * kMaxBurstBytes;
      std::memcpy(queuedBursts_.data() + offset, encoderBurst_.data(), burstBytes_);
      queueWriteIndex_ = (queueWriteIndex_ + 1) % kEncodedQueueCapacity;
      ++queueCount_;
    }
    queueCv_.notify_all();
  }
  if (mmcss) AvRevertMmThreadCharacteristics(mmcss);
}

bool WasapiPassthrough::Start()
{
  if (!client_ || !render_ || !dataEvent_ || !stopEvent_)
  {
    std::fprintf(stderr, "[WasapiPassthrough] Start requested before successful Init\n");
    return false;
  }
  if (running_.exchange(true, std::memory_order_relaxed)) return true;

  failed_.store(false, std::memory_order_relaxed);
  ResetEvent(stopEvent_);
  minAvail_ = 0xFFFFFFFFu;
  driftFrameAccumulator_ = 0;
  primed_ = false;
  burstOffsetFrames_ = static_cast<uint32_t>(framesPerPacket_);
  currentBurstHasAudio_ = false;
  encoderPendingFrames_.store(0, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(queueMutex_);
    queueReadIndex_ = queueWriteIndex_ = queueCount_ = 0;
  }

  encoderThread_ = std::thread(&WasapiPassthrough::EncoderThreadProc, this);
  const size_t requiredBursts =
      std::max<size_t>(1, (bufferFrames_ + framesPerPacket_ - 1) / framesPerPacket_);
  {
    std::unique_lock<std::mutex> lock(queueMutex_);
    queueCv_.wait_for(lock, std::chrono::milliseconds(120), [&] {
      return queueCount_ >= requiredBursts || !running_.load(std::memory_order_relaxed);
    });
  }
  if (!running_.load(std::memory_order_relaxed))
  {
    if (encoderThread_.joinable()) encoderThread_.join();
    return false;
  }

  BYTE* out = nullptr;
  HRESULT hr = render_->GetBuffer(bufferFrames_, &out);
  if (FAILED(hr) || !out)
  {
    failed_.store(true, std::memory_order_relaxed);
    running_.store(false, std::memory_order_relaxed);
    if (diagnostics_) diagnostics_->outputWasapiErrors.fetch_add(1, std::memory_order_relaxed);
    std::fprintf(stderr, "[WasapiPassthrough] initial GetBuffer failed: %s\n", HrStr(hr).c_str());
    queueCv_.notify_all();
    ring_->WakeWaiters();
    if (encoderThread_.joinable()) encoderThread_.join();
    return false;
  }

  const bool hadAudio = FillRenderBuffer(out);
  hr = render_->ReleaseBuffer(bufferFrames_, 0);
  if (FAILED(hr))
  {
    failed_.store(true, std::memory_order_relaxed);
    running_.store(false, std::memory_order_relaxed);
    if (diagnostics_) diagnostics_->outputWasapiErrors.fetch_add(1, std::memory_order_relaxed);
    std::fprintf(stderr, "[WasapiPassthrough] initial ReleaseBuffer failed: %s\n", HrStr(hr).c_str());
    queueCv_.notify_all();
    ring_->WakeWaiters();
    if (encoderThread_.joinable()) encoderThread_.join();
    return false;
  }

  if (sharedMem_.IsOpen())
  {
    MeterSharedState* ms = sharedMem_.Get();
    if (ms)
    {
      if (hadAudio) enc_.GetChannelLevels(ms->peakLevels, ms->rmsLevels);
      ms->active = 1;
      ms->frameCounter++;
    }
  }

  hr = client_->Start();
  if (FAILED(hr))
  {
    failed_.store(true, std::memory_order_relaxed);
    running_.store(false, std::memory_order_relaxed);
    if (diagnostics_) diagnostics_->outputWasapiErrors.fetch_add(1, std::memory_order_relaxed);
    std::fprintf(stderr, "[WasapiPassthrough] client Start failed: %s\n", HrStr(hr).c_str());
    queueCv_.notify_all();
    ring_->WakeWaiters();
    if (encoderThread_.joinable()) encoderThread_.join();
    return false;
  }

  if (sharedMem_.IsOpen())
  {
    MeterSharedState* ms = sharedMem_.Get();
    if (ms)
    {
      ms->active = 1;
      ms->sampleRate = static_cast<uint32_t>(capFmt_.sampleRate);
      ms->channels = 6;
      ms->isDts = (params_.codec == SpdifEncoder::Codec::DTS ? 1 : 0);
      ms->safeFrames = params_.safeFrames;
    }
  }
  PublishDiagnostics();
  thread_ = std::thread(&WasapiPassthrough::ThreadProc, this);
  return true;
}

void WasapiPassthrough::Stop()
{
  running_.store(false, std::memory_order_relaxed);
  primed_ = false;
  if (sharedMem_.IsOpen())
  {
    MeterSharedState* ms = sharedMem_.Get();
    if (ms)
    {
      ms->active = 0;
      InterlockedExchange(&ms->surroundFilterState, 0);
      InterlockedExchange(&ms->surroundFilterHopFrames, 0);
      InterlockedExchange64(&ms->surroundFilterFifoFrames, 0);
      InterlockedExchange(&ms->surroundFilterTargetFrames, 0);
      for (int i = 0; i < 6; ++i) { ms->peakLevels[i] = 0.0f; ms->rmsLevels[i] = 0.0f; }
    }
  }
  if (stopEvent_) SetEvent(stopEvent_);
  queueCv_.notify_all();
  if (ring_) ring_->WakeWaiters();
  if (thread_.joinable()) thread_.join();
  if (encoderThread_.joinable()) encoderThread_.join();
  if (client_) client_->Stop();
  client_.Reset();
  render_.Reset();
  dev_.Reset();
  if (dataEvent_) { CloseHandle(dataEvent_); dataEvent_ = nullptr; }
  if (stopEvent_) { CloseHandle(stopEvent_); stopEvent_ = nullptr; }
}

void WasapiPassthrough::UpdateEncoderSnapshot()
{
  filterStateSnapshot_.store(static_cast<uint32_t>(enc_.GetFilterStatus()), std::memory_order_relaxed);
  filterHopSnapshot_.store(enc_.FilterHopFrames(), std::memory_order_relaxed);
  filterFifoSnapshot_.store(enc_.FilterFifoFrames(), std::memory_order_relaxed);
  filterTargetSnapshot_.store(enc_.FilterFifoTargetFrames(), std::memory_order_relaxed);
}

void WasapiPassthrough::PublishDiagnostics()
{
  if (!sharedMem_.IsOpen() || !diagnostics_) return;
  MeterSharedState* state = sharedMem_.Get();
  PublishAudioDiagnostics(state, diagnostics_, ring_ ? ring_->BytesAvailable() : 0,
                          ring_ ? ring_->Capacity() : 0, capBytesPerFrame_);
  if (!state) return;
  InterlockedExchange(&state->surroundFilterState,
      static_cast<LONG>(filterStateSnapshot_.load(std::memory_order_relaxed)));
  InterlockedExchange(&state->surroundFilterHopFrames,
      static_cast<LONG>(filterHopSnapshot_.load(std::memory_order_relaxed)));
  InterlockedExchange64(&state->surroundFilterFifoFrames,
      static_cast<LONG64>(filterFifoSnapshot_.load(std::memory_order_relaxed)));
  InterlockedExchange(&state->surroundFilterTargetFrames,
      static_cast<LONG>(filterTargetSnapshot_.load(std::memory_order_relaxed)));
}

void WasapiPassthrough::ThreadProc()
{
  DWORD taskIndex = 0;
  HANDLE mmcss = AvSetMmThreadCharacteristicsW(L"Pro Audio", &taskIndex);
  if (mmcss && !AvSetMmThreadPriority(mmcss, AVRT_PRIORITY_HIGH))
    std::fprintf(stderr, "[WasapiPassthrough] AvSetMmThreadPriority(HIGH) failed: %lu\n",
                 GetLastError());

  const uint32_t desired = static_cast<uint32_t>(framesPerPacket_) + params_.safeFrames;
  HANDLE waits[2] = { dataEvent_, stopEvent_ };
  unsigned consecutiveTimeouts = 0;

  while (running_.load(std::memory_order_relaxed))
  {
    const DWORD wr = WaitForMultipleObjects(2, waits, FALSE, 2000);
    if (wr == WAIT_OBJECT_0 + 1) break;
    if (wr == WAIT_TIMEOUT)
    {
      if (diagnostics_) diagnostics_->eventTimeouts.fetch_add(1, std::memory_order_relaxed);
      if (++consecutiveTimeouts >= 2)
      {
        failed_.store(true, std::memory_order_relaxed);
        if (diagnostics_) diagnostics_->outputWasapiErrors.fetch_add(1, std::memory_order_relaxed);
        std::fprintf(stderr, "[WasapiPassthrough] repeated render event timeouts; requesting restart\n");
        break;
      }
      continue;
    }
    if (wr == WAIT_FAILED)
    {
      failed_.store(true, std::memory_order_relaxed);
      if (diagnostics_) diagnostics_->outputWasapiErrors.fetch_add(1, std::memory_order_relaxed);
      std::fprintf(stderr, "[WasapiPassthrough] render event wait failed: %lu\n", GetLastError());
      break;
    }
    if (wr != WAIT_OBJECT_0)
    {
      failed_.store(true, std::memory_order_relaxed);
      if (diagnostics_) diagnostics_->outputWasapiErrors.fetch_add(1, std::memory_order_relaxed);
      break;
    }
    consecutiveTimeouts = 0;

    if (primed_)
    {
      const uint32_t avail =
          static_cast<uint32_t>(ring_->BytesAvailable() / capBytesPerFrame_) +
          encoderPendingFrames_.load(std::memory_order_acquire);
      if (avail < minAvail_) minAvail_ = avail;

      driftFrameAccumulator_ += bufferFrames_;
      if (driftFrameAccumulator_ >= capFmt_.sampleRate)
      {
        const uint32_t hysteresis = std::max<uint32_t>(
            static_cast<uint32_t>(framesPerPacket_), params_.safeFrames);
        if (minAvail_ != 0xFFFFFFFFu && minAvail_ > desired + hysteresis)
        {
          const uint32_t excess = minAvail_ - (desired + hysteresis / 2);
          const uint32_t maxTrim = std::max<uint32_t>(1, framesPerPacket_ / 8);
          const uint32_t trim = std::min(excess, maxTrim);
          ring_->Discard(static_cast<size_t>(trim) * capBytesPerFrame_);
          if (diagnostics_ && trim)
            diagnostics_->driftTrimFrames.fetch_add(trim, std::memory_order_relaxed);
        }
        minAvail_ = 0xFFFFFFFFu;
        driftFrameAccumulator_ %= capFmt_.sampleRate;
      }
    }
    else
    {
      minAvail_ = 0xFFFFFFFFu;
      driftFrameAccumulator_ = 0;
    }

    BYTE* out = nullptr;
    HRESULT hr = render_->GetBuffer(bufferFrames_, &out);
    if (FAILED(hr) || !out)
    {
      failed_.store(true, std::memory_order_relaxed);
      if (diagnostics_) diagnostics_->outputWasapiErrors.fetch_add(1, std::memory_order_relaxed);
      std::fprintf(stderr, "[WasapiPassthrough] GetBuffer failed: %s\n", HrStr(hr).c_str());
      break;
    }
    const bool hadAudio = FillRenderBuffer(out);
    hr = render_->ReleaseBuffer(bufferFrames_, 0);
    if (FAILED(hr))
    {
      failed_.store(true, std::memory_order_relaxed);
      if (diagnostics_) diagnostics_->outputWasapiErrors.fetch_add(1, std::memory_order_relaxed);
      std::fprintf(stderr, "[WasapiPassthrough] ReleaseBuffer failed: %s\n", HrStr(hr).c_str());
      break;
    }

    if (sharedMem_.IsOpen())
    {
      MeterSharedState* ms = sharedMem_.Get();
      if (ms)
      {
        if (hadAudio) enc_.GetChannelLevels(ms->peakLevels, ms->rmsLevels);
        ms->active = 1;
        ms->frameCounter++;
      }
    }
    PublishDiagnostics();
  }
  if (mmcss) AvRevertMmThreadCharacteristics(mmcss);
}
