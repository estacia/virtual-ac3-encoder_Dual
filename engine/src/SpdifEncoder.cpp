// SpdifEncoder.cpp  — see SpdifEncoder.h for design notes.

#include "SpdifEncoder.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

extern "C" {
#include <libavutil/error.h>
#include <libavutil/samplefmt.h>
}

namespace {

void LogAv(const char* what, int err)
{
  char buf[AV_ERROR_MAX_STRING_SIZE] = {0};
  av_strerror(err, buf, sizeof buf);
  std::fprintf(stderr, "[SpdifEncoder] %s failed: %s (%d)\n", what, buf, err);
}

} // namespace

SpdifEncoder::SpdifEncoder()
{
  for (auto& gain : channelGainLinear_)
    gain.store(1.0f, std::memory_order_relaxed);
  for (size_t i = 0; i < 6; ++i)
  {
    peakLevels_[i].store(0.0f, std::memory_order_relaxed);
    rmsLevels_[i].store(0.0f, std::memory_order_relaxed);
  }
}

void SpdifEncoder::GetChannelLevels(float outPeaks[6], float outRms[6]) const
{
  for (size_t i = 0; i < 6; ++i)
  {
    outPeaks[i] = peakLevels_[i].load(std::memory_order_relaxed);
    outRms[i] = rmsLevels_[i].load(std::memory_order_relaxed);
  }
}

SpdifEncoder::FilterStatus SpdifEncoder::GetFilterStatus() const
{
  if (speakerFoldDownActive_)
    return FilterStatus::SpeakerLayout4_0FoldDown;
  if (timeDomain_)
  {
    if (!timeDomainMatrixDetected_)
      return FilterStatus::TimeDomainWaiting;
    return timeDomainMatrixApplied_
        ? FilterStatus::TimeDomainMatrix
        : FilterStatus::TimeDomainNativeBypass;
  }
  if (useFilter_)
    return FilterStatus::StereoUpmix;
  if (!dynamicUpmix_)
    return FilterStatus::Disabled;
  if (dynamicFilterBypassed_)
    return FilterStatus::NativeBypass;
  if (dynamicFilterNeedsWarmup_ || !dynamicFilterPrimed_)
    return FilterStatus::Warming;
  if (dynamicFilterActive_)
    return FilterStatus::DynamicStereo;
  return FilterStatus::HoldPrewarm;
}

uint32_t SpdifEncoder::FilterFifoFrames() const
{
  return fifo_ ? static_cast<uint32_t>(std::max(0, av_audio_fifo_size(fifo_))) : 0;
}

uint32_t SpdifEncoder::FilterFifoTargetFrames() const
{
  if (!useFilter_ && !dynamicUpmix_)
    return 0;
  const int minimum = std::max(framesPerPacket_, surroundHopFrames_);
  return static_cast<uint32_t>(minimum + framesPerPacket_);
}

uint32_t SpdifEncoder::FilterHopFrames() const
{
  return (useFilter_ || dynamicUpmix_)
      ? static_cast<uint32_t>(surroundHopFrames_) : 0;
}

void SpdifEncoder::SetChannelGainsDb(const std::array<float, 6>& gainsDb)
{
  for (size_t i = 0; i < channelGainLinear_.size(); ++i)
  {
    const float db = std::clamp(gainsDb[i], -60.0f, 0.0f);
    const float linear = (db <= -60.0f) ? 0.0f : std::pow(10.0f, db / 20.0f);
    channelGainLinear_[i].store(linear, std::memory_order_relaxed);
  }
}

void SpdifEncoder::ApplyTimeDomainUpmix(AVFrame* target)
{
  if (!target || !target->extended_data || !codecCtx_ || codecCtx_->ch_layout.nb_channels < 6)
    return;
  float* fl  = reinterpret_cast<float*>(target->extended_data[0]);
  float* fr  = reinterpret_cast<float*>(target->extended_data[1]);
  float* fc  = reinterpret_cast<float*>(target->extended_data[2]);
  float* lfe = reinterpret_cast<float*>(target->extended_data[3]);
  float* sl  = reinterpret_cast<float*>(target->extended_data[4]);
  float* sr  = reinterpret_cast<float*>(target->extended_data[5]);
  if (!fl || !fr || !fc || !lfe || !sl || !sr)
    return;

  // The filter state persists across 256-sample calls.
  const float alpha = 1.0f - std::exp(-2.0f * 3.14159265358979323846f *
                                      120.0f / static_cast<float>(sampleRate_));
  constexpr float kMinus3dB = 0.70710678f;
  for (int i = 0; i < target->nb_samples; ++i)
  {
    const float left = fl[i], right = fr[i];
    const float mid = 0.5f * (left + right);
    const float side = 0.5f * (left - right);
    fc[i] = mid * kMinus3dB;
    sl[i] = side * kMinus3dB;
    sr[i] = -side * kMinus3dB;
    lfeFilterState_ += alpha * (mid - lfeFilterState_);
    lfe[i] = lfeFilterState_ * 0.31622777f;
  }
}

void SpdifEncoder::ApplyCenterLfeFoldDown(AVFrame* target)
{
  if (!target || !target->extended_data || !codecCtx_ ||
      codecCtx_->ch_layout.nb_channels < 6)
    return;

  float* fl  = reinterpret_cast<float*>(target->extended_data[0]);
  float* fr  = reinterpret_cast<float*>(target->extended_data[1]);
  float* fc  = reinterpret_cast<float*>(target->extended_data[2]);
  float* lfe = reinterpret_cast<float*>(target->extended_data[3]);
  if (!fl || !fr || !fc || !lfe)
    return;

  // ITU-R BS.775-4 phantom-center fold-down: C is sent to L/R at -3 dB each.
  constexpr float kCenterToFront = 0.70710678f;
  // ITU-R BS.775-4 LFE redirection: +10 dB reproduction compensation, then
  // -6 dB to each of the two front speakers. Net coefficient per front = +4 dB.
  constexpr float kLfeToEachFront = 1.58489319f;
  constexpr float kPeakCeiling = 0.95f;

  float peak = 0.0f;
  for (int i = 0; i < target->nb_samples; ++i)
  {
    fl[i] += kCenterToFront * fc[i] + kLfeToEachFront * lfe[i];
    fr[i] += kCenterToFront * fc[i] + kLfeToEachFront * lfe[i];
    peak = std::max(peak, std::max(std::fabs(fl[i]), std::fabs(fr[i])));
  }

  // These channels no longer exist in a 4.0 speaker layout. Clearing them avoids
  // duplicate playback on a downstream device that still advertises a 5.1 bitstream.
  std::fill_n(fc, target->nb_samples, 0.0f);
  std::fill_n(lfe, target->nb_samples, 0.0f);

  // Prevent summed center/LFE peaks from overloading the AC-3/DTS encoder. Attack
  // is immediate; release is smoothed over 50 ms to avoid rapid block-to-block gain
  // jumps. No look-ahead, queue, or additional buffering is introduced.
  const float targetGain = (peak > kPeakCeiling) ? (kPeakCeiling / peak) : 1.0f;
  const float release = 1.0f - std::exp(
      -static_cast<float>(target->nb_samples) /
      (0.050f * static_cast<float>(std::max(1, sampleRate_))));
  for (int i = 0; i < target->nb_samples; ++i)
  {
    if (targetGain < foldDownLimiterGain_)
      foldDownLimiterGain_ = targetGain;
    else
      foldDownLimiterGain_ += release * (targetGain - foldDownLimiterGain_);
    fl[i] *= foldDownLimiterGain_;
    fr[i] *= foldDownLimiterGain_;
  }

  speakerFoldDownActive_ = true;
}

void SpdifEncoder::ApplyChannelGainsAndComputeLevels(AVFrame* target)
{
  if (!target || !target->extended_data || !codecCtx_)
    return;
  const int channels = std::min(6, codecCtx_->ch_layout.nb_channels);
  for (int ch = 0; ch < channels; ++ch)
  {
    float* samples = reinterpret_cast<float*>(target->extended_data[ch]);
    if (!samples) continue;

    const float gain = channelGainLinear_[static_cast<size_t>(ch)].load(std::memory_order_relaxed);
    float peak = 0.0f;
    double sumSq = 0.0;
    if (gain <= 0.0f)
    {
      std::fill_n(samples, target->nb_samples, 0.0f);
    }
    else
    {
      const bool applyGain = gain < 0.999999f;
      for (int i = 0; i < target->nb_samples; ++i)
      {
        float sample = samples[i];
        if (applyGain)
        {
          sample *= gain;
          samples[i] = sample;
        }
        const float magnitude = std::fabs(sample);
        peak = std::max(peak, magnitude);
        sumSq += static_cast<double>(magnitude) * magnitude;
      }
    }

    const float rms = target->nb_samples > 0
        ? static_cast<float>(std::sqrt(sumSq / target->nb_samples)) : 0.0f;
    peakLevels_[static_cast<size_t>(ch)].store(peak, std::memory_order_relaxed);
    rmsLevels_[static_cast<size_t>(ch)].store(rms, std::memory_order_relaxed);
  }
}

SpdifEncoder::~SpdifEncoder()
{
  Close();
}

bool SpdifEncoder::Init(const Params& p)
{
  Close();

  av_channel_layout_copy(&inLayout_, &p.inLayout);
  surroundWindowSize_ = std::clamp(p.surroundWindowSize, 1024, 65536);
  surroundHopFrames_ = std::max(1, surroundWindowSize_ / 2);
  inSampleFmt_ = p.inSampleFmt;
  sampleRate_ = p.sampleRate;
  nextPts_ = 0;

  useDts_ = (p.codec == Codec::DTS);
  const AVCodecID codecId = useDts_ ? AV_CODEC_ID_DTS : AV_CODEC_ID_AC3;
  const AVCodec* codec = avcodec_find_encoder(codecId);
  if (!codec)
  {
    std::fprintf(stderr, "[SpdifEncoder] %s encoder not found in this FFmpeg build\n",
                 useDts_ ? "DTS" : "AC3");
    return false;
  }

  codecCtx_ = avcodec_alloc_context3(codec);
  if (!codecCtx_)
    return false;

  codecCtx_->bit_rate = useDts_ ? p.dtsBitRate : p.bitRate;
  codecCtx_->sample_rate = p.sampleRate;
  codecCtx_->sample_fmt = useDts_ ? AV_SAMPLE_FMT_S32 : AV_SAMPLE_FMT_FLTP;
  codecCtx_->time_base = av_make_q(1, p.sampleRate);
  if (useDts_)
    codecCtx_->strict_std_compliance = FF_COMPLIANCE_EXPERIMENTAL;

  // The encoder (and S/PDIF) carry 5.1.
  AVChannelLayout enc51back = AV_CHANNEL_LAYOUT_5POINT1_BACK;
  AVChannelLayout enc51side = AV_CHANNEL_LAYOUT_5POINT1;
  av_channel_layout_copy(&codecCtx_->ch_layout, useDts_ ? &enc51side : &enc51back);

  int rc = avcodec_open2(codecCtx_, codec, nullptr);
  if (rc < 0)
  {
    av_channel_layout_uninit(&codecCtx_->ch_layout);
    av_channel_layout_copy(&codecCtx_->ch_layout, useDts_ ? &enc51back : &enc51side);
    rc = avcodec_open2(codecCtx_, codec, nullptr);
    if (rc < 0) { LogAv("avcodec_open2", rc); return false; }
  }

  framesPerPacket_ = codecCtx_->frame_size; // 1536 for AC3, 512 for DTS
  if (framesPerPacket_ <= 0 || (framesPerPacket_ % kPcmChunkFrames) != 0)
  {
    std::fprintf(stderr, "[SpdifEncoder] codec frame size %d is not divisible by the %d-sample DSP quantum\n",
                 framesPerPacket_, kPcmChunkFrames);
    return false;
  }
  processedFrames_ = 0;

  // Scratch frame holding the planar-float 5.1 samples. Both the swr path and the
  // surround-filter path fill this.
  frame_ = av_frame_alloc();
  if (!frame_) return false;
  frame_->format = AV_SAMPLE_FMT_FLTP;
  av_channel_layout_copy(&frame_->ch_layout, &codecCtx_->ch_layout);
  frame_->sample_rate = sampleRate_;
  frame_->nb_samples = framesPerPacket_;
  rc = av_frame_get_buffer(frame_, 0);
  if (rc < 0) { LogAv("av_frame_get_buffer", rc); return false; }

  chunkFrame_ = av_frame_alloc();
  if (!chunkFrame_) return false;
  chunkFrame_->format = AV_SAMPLE_FMT_FLTP;
  av_channel_layout_copy(&chunkFrame_->ch_layout, &codecCtx_->ch_layout);
  chunkFrame_->sample_rate = sampleRate_;
  chunkFrame_->nb_samples = kPcmChunkFrames;
  rc = av_frame_get_buffer(chunkFrame_, 0);
  if (rc < 0) { LogAv("av_frame_get_buffer(chunk)", rc); return false; }

  if (useDts_)
  {
    dtsFrame_ = av_frame_alloc();
    if (!dtsFrame_) return false;
    dtsFrame_->format = AV_SAMPLE_FMT_S32;
    av_channel_layout_copy(&dtsFrame_->ch_layout, &codecCtx_->ch_layout);
    dtsFrame_->sample_rate = sampleRate_;
    dtsFrame_->nb_samples = framesPerPacket_;
    rc = av_frame_get_buffer(dtsFrame_, 0);
    if (rc < 0) { LogAv("av_frame_get_buffer(DTS)", rc); return false; }

    rc = swr_alloc_set_opts2(&dtsS32Swr_,
                             &codecCtx_->ch_layout, AV_SAMPLE_FMT_S32, sampleRate_,
                             &codecCtx_->ch_layout, AV_SAMPLE_FMT_FLTP, sampleRate_,
                             0, nullptr);
    if (rc < 0 || !dtsS32Swr_) { LogAv("swr_alloc_set_opts2(DTS)", rc); return false; }
    rc = swr_init(dtsS32Swr_);
    if (rc < 0) { LogAv("swr_init(DTS)", rc); return false; }
  }

  // Input conditioning: either the `surround` upmix filter (stereo->5.1) or plain swr
  // convert/downmix. When input is already multichannel but upmix is requested, use
  // "dynamic" mode: keep both paths live and detect stereo content at runtime.
  upmixMode_ = p.upmix;
  dualStereo_ = (p.upmix == Upmix::DualStereo);
  // Dual Stereo intentionally bypasses the FFT-based surround filter and uses only
  // direct planar channel copies after the normal PCM -> 5.1 sample-format conversion.
  const bool isTimeDomain = (p.upmix == Upmix::TimeDomain ||
                             p.upmix == Upmix::TimeDomainNative51);
  const bool wantUpmix = (p.upmix != Upmix::Off && !dualStereo_ && !isTimeDomain);
  alwaysUpmix_ = (p.upmix == Upmix::Always || p.upmix == Upmix::AlwaysFront);
  timeDomain_ = isTimeDomain;
  timeDomainNative51_ = (p.upmix == Upmix::TimeDomainNative51);
  timeDomainMatrixDetected_ = false;
  timeDomainMatrixApplied_ = false;
  foldDownCenterLfeToFront_ = p.foldDownCenterLfeToFront &&
      (p.upmix == Upmix::TimeDomainNative51 || p.upmix == Upmix::DualStereo);
  speakerFoldDownActive_ = false;
  foldDownLimiterGain_ = 1.0f;
  lfeFilterState_ = 0.0f;
  useFilter_ = (wantUpmix && inLayout_.nb_channels <= 2);
  dynamicUpmix_ = (wantUpmix && inLayout_.nb_channels > 2);

  if (useFilter_)
  {
    av_channel_layout_copy(&filterInLayout_, &inLayout_);
    if (!BuildFilterGraph())
    {
      std::fprintf(stderr, "[SpdifEncoder] surround upmix unavailable; using plain upmix/downmix\n");
      useFilter_ = false;
      av_channel_layout_uninit(&filterInLayout_);
    }
  }
  if (dynamicUpmix_)
  {
    AVChannelLayout stereo = AV_CHANNEL_LAYOUT_STEREO;
    av_channel_layout_copy(&filterInLayout_, &stereo);
    if (!BuildFilterGraph())
    {
      std::fprintf(stderr, "[SpdifEncoder] dynamic upmix unavailable; multichannel passthrough only\n");
      dynamicUpmix_ = false;
      av_channel_layout_uninit(&filterInLayout_);
    }
    else
    {
      const int bps = av_get_bytes_per_sample(inSampleFmt_);
      stereoExtract_.resize(static_cast<size_t>(kPcmChunkFrames) * 2 * bps);
    }
  }
  if (!useFilter_)
  {
    // For ordinary 2-channel Dual Stereo input, convert only FL/FR to planar float.
    // The unused Center/LFE planes are zeroed and the rear planes are copied directly,
    // avoiding an unnecessary six-channel swresample matrix on every packet.
    if (dualStereo_ && inLayout_.nb_channels == 2)
    {
      AVChannelLayout stereo = AV_CHANNEL_LAYOUT_STEREO;
      rc = swr_alloc_set_opts2(&dualStereoSwr_, &stereo, AV_SAMPLE_FMT_FLTP, sampleRate_,
                               &inLayout_, inSampleFmt_, sampleRate_, 0, nullptr);
      if (rc < 0 || !dualStereoSwr_) { LogAv("swr_alloc_set_opts2(DualStereo)", rc); return false; }
      rc = swr_init(dualStereoSwr_);
      if (rc < 0) { LogAv("swr_init(DualStereo)", rc); return false; }
    }
    else
    {
      rc = swr_alloc_set_opts2(&swr_, &codecCtx_->ch_layout, AV_SAMPLE_FMT_FLTP, sampleRate_,
                               &inLayout_, inSampleFmt_, sampleRate_, 0, nullptr);
      if (rc < 0 || !swr_) { LogAv("swr_alloc_set_opts2", rc); return false; }
      rc = swr_init(swr_);
      if (rc < 0) { LogAv("swr_init", rc); return false; }
    }
  }

  pkt_ = av_packet_alloc();
  if (!pkt_) return false;

  rc = avformat_alloc_output_context2(&muxer_, nullptr, "spdif", nullptr);
  if (rc < 0 || !muxer_) { LogAv("avformat_alloc_output_context2(spdif)", rc); return false; }

  AVStream* st = avformat_new_stream(muxer_, nullptr);
  if (!st) { std::fprintf(stderr, "[SpdifEncoder] avformat_new_stream failed\n"); return false; }
  st->id = 0;
  st->time_base = codecCtx_->time_base;
  rc = avcodec_parameters_from_context(st->codecpar, codecCtx_);
  if (rc < 0) { LogAv("avcodec_parameters_from_context", rc); return false; }

  unsigned char* avioBuf = static_cast<unsigned char*>(av_malloc(kMaxBytesPerPacket));
  if (!avioBuf) return false;
  muxer_->pb = avio_alloc_context(avioBuf, kMaxBytesPerPacket, /*write_flag=*/1, this,
                                  /*read=*/nullptr, &WritePacketThunk, /*seek=*/nullptr);
  if (!muxer_->pb) { av_free(avioBuf); return false; }
  muxer_->flags |= AVFMT_FLAG_CUSTOM_IO;

  rc = avformat_write_header(muxer_, nullptr);
  if (rc < 0) { LogAv("avformat_write_header(spdif)", rc); return false; }

  if (dynamicUpmix_)
  {
    dynamicFilterBypassed_ = false;
    dynamicFilterNeedsWarmup_ = false;
    dynamicFilterWarmupFrames_ = 0;
    dynamicFilterActive_ = false;
    dynamicFilterPrimed_ = fifo_ &&
        av_audio_fifo_size(fifo_) >= std::max(framesPerPacket_, surroundHopFrames_);
  }

  return true;
}

void SpdifEncoder::Close()
{
  if (muxer_)
  {
    if (muxer_->pb)
      av_write_trailer(muxer_);
    if (muxer_->pb)
    {
      av_freep(&muxer_->pb->buffer);
      avio_context_free(&muxer_->pb);
    }
    avformat_free_context(muxer_);
    muxer_ = nullptr;
  }
  if (graph_)     avfilter_graph_free(&graph_);
  fsrc_ = fsink_ = nullptr;
  if (filterInputFrame_) av_frame_free(&filterInputFrame_);
  filterInputPts_ = 0;
  surroundWindowSize_ = 4096;
  surroundHopFrames_ = kSurroundFilterHopFrames;
  if (fifo_)      { av_audio_fifo_free(fifo_); fifo_ = nullptr; }
  processedFrames_ = 0;
  if (filtFrame_) av_frame_free(&filtFrame_);
  if (chunkFrame_) av_frame_free(&chunkFrame_);
  useFilter_ = false;
  useDts_ = false;
  timeDomain_ = false;
  timeDomainNative51_ = false;
  timeDomainMatrixDetected_ = false;
  timeDomainMatrixApplied_ = false;
  foldDownCenterLfeToFront_ = false;
  speakerFoldDownActive_ = false;
  foldDownLimiterGain_ = 1.0f;
  lfeFilterState_ = 0.0f;
  dynamicUpmix_ = false;
  alwaysUpmix_ = false;
  dualStereo_ = false;
  dynamicFilterBypassed_ = false;
  dynamicFilterNeedsWarmup_ = false;
  dynamicFilterPrimed_ = false;
  dynamicFilterActive_ = false;
  dynamicFilterWarmupFrames_ = 0;
  filterInputFramesProcessed_ = 0;
  upmixMode_ = Upmix::Off;
  effectivelyStereo_ = true;
  multichannelHoldFrames_ = 0;
  av_channel_layout_uninit(&filterInLayout_);
  stereoExtract_.clear();
  if (dtsS32Swr_) swr_free(&dtsS32Swr_);
  if (dualStereoSwr_) swr_free(&dualStereoSwr_);
  if (swr_)       swr_free(&swr_);
  if (dtsFrame_)  av_frame_free(&dtsFrame_);
  if (frame_)     av_frame_free(&frame_);
  if (pkt_)       av_packet_free(&pkt_);
  if (codecCtx_)  avcodec_free_context(&codecCtx_);
  av_channel_layout_uninit(&inLayout_);
  framesPerPacket_ = 0;
  processedFrames_ = 0;
  nextPts_ = 0;
}

bool SpdifEncoder::BuildFilterGraph()
{
  graph_ = avfilter_graph_alloc();
  if (!graph_) return false;

  char inDesc[64] = {0}, outDesc[64] = {0};
  av_channel_layout_describe(&filterInLayout_, inDesc, sizeof inDesc);
  av_channel_layout_describe(&codecCtx_->ch_layout, outDesc, sizeof outDesc);

  const AVFilter* abuffer = avfilter_get_by_name("abuffer");
  const AVFilter* surround = avfilter_get_by_name("surround");
  const AVFilter* aformat = avfilter_get_by_name("aformat");
  const AVFilter* abuffersink = avfilter_get_by_name("abuffersink");
  if (!abuffer || !surround || !aformat || !abuffersink)
  {
    std::fprintf(stderr, "[SpdifEncoder] a required avfilter is missing (surround/aformat/abuffer)\n");
    return false;
  }

  char srcArgs[256];
  std::snprintf(srcArgs, sizeof srcArgs,
                "sample_rate=%d:sample_fmt=%s:channel_layout=%s:time_base=1/%d",
                sampleRate_, av_get_sample_fmt_name(inSampleFmt_), inDesc, sampleRate_);
  char surArgs[256];
  const bool isFront31 = (upmixMode_ == Upmix::FrontStage || upmixMode_ == Upmix::AlwaysFront);
  const bool isFront30 = (upmixMode_ == Upmix::Front30);
  const bool isSub21 = (upmixMode_ == Upmix::StereoSub21);
  if (isFront30)
  {
    std::snprintf(surArgs, sizeof surArgs,
                  "chl_in=%s:chl_out=%s:fc_out=1.0:lfe_out=0.0:bl_out=0:br_out=0:sl_out=0:sr_out=0:bc_out=0:win_size=%d:overlap=0.5",
                  inDesc, outDesc, surroundWindowSize_);
  }
  else if (isFront31 || isSub21)
  {
    std::snprintf(surArgs, sizeof surArgs,
                  "chl_in=%s:chl_out=%s:fc_out=1.0:lfe_out=1.2:lfe_low=140:bl_out=0:br_out=0:sl_out=0:sr_out=0:bc_out=0:win_size=%d:overlap=0.5",
                  inDesc, outDesc, surroundWindowSize_);
  }
  else
  {
    std::snprintf(surArgs, sizeof surArgs,
                  "chl_in=%s:chl_out=%s:fc_out=1.0:lfe_out=1.2:lfe_low=140:win_size=%d:overlap=0.5",
                  inDesc, outDesc, surroundWindowSize_);
  }
  char fmtArgs[256];
  std::snprintf(fmtArgs, sizeof fmtArgs, "sample_fmts=fltp:sample_rates=%d:channel_layouts=%s",
                sampleRate_, outDesc);

  AVFilterContext* surCtx = nullptr;
  AVFilterContext* fmtCtx = nullptr;
  int rc;
  if ((rc = avfilter_graph_create_filter(&fsrc_, abuffer, "in", srcArgs, nullptr, graph_)) < 0)
  { LogAv("create abuffer", rc); return false; }
  if ((rc = avfilter_graph_create_filter(&surCtx, surround, "surround", surArgs, nullptr, graph_)) < 0)
  { LogAv("create surround", rc); return false; }
  if ((rc = avfilter_graph_create_filter(&fmtCtx, aformat, "aformat", fmtArgs, nullptr, graph_)) < 0)
  { LogAv("create aformat", rc); return false; }
  if ((rc = avfilter_graph_create_filter(&fsink_, abuffersink, "out", nullptr, nullptr, graph_)) < 0)
  { LogAv("create abuffersink", rc); return false; }

  if ((rc = avfilter_link(fsrc_, 0, surCtx, 0)) < 0 ||
      (rc = avfilter_link(surCtx, 0, fmtCtx, 0)) < 0 ||
      (rc = avfilter_link(fmtCtx, 0, fsink_, 0)) < 0)
  { LogAv("avfilter_link", rc); return false; }

  if ((rc = avfilter_graph_config(graph_, nullptr)) < 0)
  { LogAv("avfilter_graph_config", rc); return false; }

  fifo_ = av_audio_fifo_alloc(AV_SAMPLE_FMT_FLTP, codecCtx_->ch_layout.nb_channels, sampleRate_);
  filtFrame_ = av_frame_alloc();
  filterInputFrame_ = av_frame_alloc();
  if (!fifo_ || !filtFrame_ || !filterInputFrame_) return false;

  filterInputFrame_->format = inSampleFmt_;
  filterInputFrame_->sample_rate = sampleRate_;
  filterInputFrame_->nb_samples = kPcmChunkFrames;
  rc = av_channel_layout_copy(&filterInputFrame_->ch_layout, &filterInLayout_);
  if (rc < 0) { LogAv("av_channel_layout_copy(filter input)", rc); return false; }
  rc = av_frame_get_buffer(filterInputFrame_, 0);
  if (rc < 0) { LogAv("av_frame_get_buffer(filter input)", rc); return false; }
  filterInputPts_ = 0;

  // Prime the FIFO past the 2048-sample FFT hop of FFmpeg's `surround` filter so
  // both 1536-sample AC3 and 512-sample DTS packets never underrun between FFT hops.
  const int primeTarget = std::max(framesPerPacket_, surroundHopFrames_);
  const int inBytes =
      kPcmChunkFrames * filterInLayout_.nb_channels * av_get_bytes_per_sample(inSampleFmt_);
  std::vector<uint8_t> silence(static_cast<size_t>(inBytes), 0);
  // Keep the same maximum priming budget as the old codec-sized loop, but feed the
  // filter in 128-sample quanta to match the smaller preallocated input frame.
  const int maxPrimeBlocks = 64 * (framesPerPacket_ / kPcmChunkFrames);
  for (int guard = 0; guard < maxPrimeBlocks && av_audio_fifo_size(fifo_) < primeTarget; ++guard)
    if (!FeedFilterFrames(silence.data(), kPcmChunkFrames))
      break;

  const char* modeName = isFront30 ? "3.0 front stage" : (isSub21 ? "2.1 stereo+sub" : (isFront31 ? "3.1 front stage (Center+Sub)" : "surround"));
  std::printf("[SpdifEncoder] %s upmix enabled%s (%s -> %s; window=%d, hop=%d frames / %.2f ms)\n",
              modeName,
              dynamicUpmix_ ? " (dynamic: auto-detect stereo in multichannel)" : "",
              inDesc, outDesc, surroundWindowSize_, surroundHopFrames_,
              1000.0 * surroundHopFrames_ / sampleRate_);
  return true;
}

bool SpdifEncoder::FeedFilterFrames(const uint8_t* in, int inputFrames)
{
  if (!in || inputFrames <= 0 || inputFrames > kPcmChunkFrames ||
      !filterInputFrame_ || !fsrc_ || !fsink_)
    return false;

  const int inBytes =
      inputFrames * filterInLayout_.nb_channels * av_get_bytes_per_sample(inSampleFmt_);
  int rc = av_frame_make_writable(filterInputFrame_);
  if (rc < 0) { LogAv("av_frame_make_writable(filter input)", rc); return false; }
  filterInputFrame_->nb_samples = inputFrames;
  std::memcpy(filterInputFrame_->data[0], in, static_cast<size_t>(inBytes));
  filterInputFrame_->pts = filterInputPts_;

  rc = av_buffersrc_add_frame_flags(fsrc_, filterInputFrame_, AV_BUFFERSRC_FLAG_KEEP_REF);
  if (rc < 0) { LogAv("av_buffersrc_add_frame_flags", rc); return false; }
  filterInputPts_ += inputFrames;
  filterInputFramesProcessed_ += static_cast<uint64_t>(inputFrames);

  for (;;)
  {
    rc = av_buffersink_get_frame(fsink_, filtFrame_);
    if (rc == AVERROR(EAGAIN) || rc == AVERROR_EOF) break;
    if (rc < 0) { LogAv("av_buffersink_get_frame", rc); return false; }
    const int expected = filtFrame_->nb_samples;
    const int written = av_audio_fifo_write(
        fifo_, reinterpret_cast<void**>(filtFrame_->extended_data), expected);
    av_frame_unref(filtFrame_);
    if (written != expected)
    {
      if (written < 0) LogAv("av_audio_fifo_write", written);
      else std::fprintf(stderr, "[SpdifEncoder] short filter FIFO write (%d/%d)\n", written, expected);
      return false;
    }
  }
  return true;
}

int SpdifEncoder::EncodePacket(const uint8_t* in, uint8_t* outBuf, int outSize)
{
  if (!codecCtx_ || !in || outSize < kMaxBytesPerPacket ||
      framesPerPacket_ <= 0 || (framesPerPacket_ % kPcmChunkFrames) != 0)
    return -1;

  const int bytesPerSample = av_get_bytes_per_sample(inSampleFmt_);
  if (bytesPerSample <= 0 || InChannels() <= 0)
    return -1;
  const size_t inputFrameBytes = static_cast<size_t>(bytesPerSample) * InChannels();

  int encodedBytes = 0;
  for (int offset = 0; offset < framesPerPacket_; offset += kPcmChunkFrames)
  {
    const int n = EncodeChunk(
        in + static_cast<size_t>(offset) * inputFrameBytes,
        kPcmChunkFrames, outBuf, outSize);
    if (n < 0) return n;
    if (n > 0) encodedBytes = n;
  }
  return encodedBytes;
}

int SpdifEncoder::EncodeChunk(const uint8_t* in, int inputFrames,
                              uint8_t* outBuf, int outSize)
{
  if (!codecCtx_ || !in || !chunkFrame_ || !frame_ ||
      inputFrames != kPcmChunkFrames || outSize < kMaxBytesPerPacket)
    return -1;

  bool usedUpmixFilter = false;
  bool haveProcessedChunk = false;

  auto convertDirect = [&](SwrContext* converter) -> bool {
    if (!converter || av_frame_make_writable(chunkFrame_) < 0) return false;
    const uint8_t* inputPlanes[1] = { in };
    const int got = swr_convert(converter, chunkFrame_->extended_data,
                                inputFrames, inputPlanes, inputFrames);
    if (got < 0) { LogAv("swr_convert(chunk)", got); return false; }
    if (got < inputFrames)
    {
      const size_t tailBytes = static_cast<size_t>(inputFrames - got) * sizeof(float);
      for (int ch = 0; ch < codecCtx_->ch_layout.nb_channels; ++ch)
        std::memset(chunkFrame_->extended_data[ch] + static_cast<size_t>(got) * sizeof(float),
                    0, tailBytes);
    }
    return true;
  };

  auto readFilterChunk = [&]() -> bool {
    if (!fifo_ || av_audio_fifo_size(fifo_) < inputFrames) return false;
    if (av_frame_make_writable(chunkFrame_) < 0) return false;
    const int got = av_audio_fifo_read(
        fifo_, reinterpret_cast<void**>(chunkFrame_->extended_data), inputFrames);
    return got == inputFrames;
  };

  if (useFilter_)
  {
    if (!FeedFilterFrames(in, inputFrames)) return -1;
    if (!readFilterChunk()) return 0;
    usedUpmixFilter = true;
    haveProcessedChunk = true;
  }
  else if (dynamicUpmix_)
  {
    bool hasDiscreteActivity = false;
    const bool stereoSource = alwaysUpmix_ ||
        IsEffectivelyStereo(in, inputFrames, &hasDiscreteActivity);

    if (!alwaysUpmix_ && hasDiscreteActivity)
    {
      if (!dynamicFilterBypassed_)
      {
        av_audio_fifo_reset(fifo_);
        dynamicFilterNeedsWarmup_ = true;
        dynamicFilterWarmupFrames_ = surroundHopFrames_;
        dynamicFilterPrimed_ = false;
      }
      dynamicFilterBypassed_ = true;
      dynamicFilterActive_ = false;
    }
    else
    {
      dynamicFilterBypassed_ = false;
      ExtractStereo(in, stereoExtract_.data(), inputFrames);
      if (!FeedFilterFrames(stereoExtract_.data(), inputFrames)) return -1;

      if (dynamicFilterNeedsWarmup_)
      {
        dynamicFilterWarmupFrames_ = std::max(0, dynamicFilterWarmupFrames_ - inputFrames);
        av_audio_fifo_reset(fifo_);
        dynamicFilterPrimed_ = false;
        if (dynamicFilterWarmupFrames_ == 0) dynamicFilterNeedsWarmup_ = false;
      }
      else
      {
        const int maxActiveFifo = std::max(inputFrames, surroundHopFrames_) + framesPerPacket_;
        const int excess = av_audio_fifo_size(fifo_) - maxActiveFifo;
        if (excess > 0) av_audio_fifo_drain(fifo_, excess);
        if (av_audio_fifo_size(fifo_) >= inputFrames) dynamicFilterPrimed_ = true;
      }
    }

    if (stereoSource && !dynamicFilterBypassed_ && !dynamicFilterNeedsWarmup_ &&
        dynamicFilterPrimed_ && readFilterChunk())
    {
      dynamicFilterActive_ = true;
      usedUpmixFilter = true;
      haveProcessedChunk = true;
    }
    else
    {
      dynamicFilterActive_ = false;
      if (!convertDirect(swr_)) return -1;
      haveProcessedChunk = true;
    }
  }
  else if (timeDomain_)
  {
    if (!convertDirect(swr_)) return -1;
    // Both Time-domain modes inspect actual channel activity, not just endpoint layout.
    // This allows stereo carried by a 5.1/7.1 virtual endpoint to be matrix-upmixed,
    // while native multichannel content remains untouched whenever discrete channels
    // (Center/LFE/Surrounds) are active.
    // The Native 5.1 Untouched variant uses a lower -80 dBFS RMS threshold (vs -66 dB)
    // so quiet discrete channels still protect genuine 5.1/7.1 material.
    const double nativeProtectionThreshold = timeDomainNative51_ ? 0.0001 : 0.0005;
    const bool stereoSource = (inLayout_.nb_channels <= 2) ||
        IsEffectivelyStereo(in, inputFrames, nullptr, nativeProtectionThreshold);
    timeDomainMatrixDetected_ = true;
    timeDomainMatrixApplied_ = stereoSource;
    if (stereoSource) ApplyTimeDomainUpmix(chunkFrame_);
    haveProcessedChunk = true;
  }
  else if (dualStereoSwr_)
  {
    if (av_frame_make_writable(chunkFrame_) < 0) return -1;
    const uint8_t* inputPlanes[1] = { in };
    uint8_t* stereoPlanes[2] = {
      chunkFrame_->extended_data[0], chunkFrame_->extended_data[1]
    };
    const int got = swr_convert(dualStereoSwr_, stereoPlanes, inputFrames,
                                inputPlanes, inputFrames);
    if (got < 0) { LogAv("swr_convert(DualStereo chunk)", got); return -1; }

    float* fl  = reinterpret_cast<float*>(chunkFrame_->extended_data[0]);
    float* fr  = reinterpret_cast<float*>(chunkFrame_->extended_data[1]);
    float* fc  = reinterpret_cast<float*>(chunkFrame_->extended_data[2]);
    float* lfe = reinterpret_cast<float*>(chunkFrame_->extended_data[3]);
    float* sl  = reinterpret_cast<float*>(chunkFrame_->extended_data[4]);
    float* sr  = reinterpret_cast<float*>(chunkFrame_->extended_data[5]);
    if (!fl || !fr || !fc || !lfe || !sl || !sr) return -1;
    const size_t convertedBytes = static_cast<size_t>(got) * sizeof(float);
    std::memcpy(sl, fl, convertedBytes);
    std::memcpy(sr, fr, convertedBytes);
    if (got < inputFrames)
    {
      const size_t tailBytes = static_cast<size_t>(inputFrames - got) * sizeof(float);
      std::memset(fl + got, 0, tailBytes);
      std::memset(fr + got, 0, tailBytes);
      std::memset(sl + got, 0, tailBytes);
      std::memset(sr + got, 0, tailBytes);
    }
    std::memset(fc, 0, static_cast<size_t>(inputFrames) * sizeof(float));
    std::memset(lfe, 0, static_cast<size_t>(inputFrames) * sizeof(float));
    haveProcessedChunk = true;
  }
  else
  {
    if (!convertDirect(swr_)) return -1;
    const bool stereoSource = dualStereo_ && IsEffectivelyStereo(in, inputFrames);
    if (stereoSource && codecCtx_->ch_layout.nb_channels >= 6)
    {
      float* fl  = reinterpret_cast<float*>(chunkFrame_->extended_data[0]);
      float* fr  = reinterpret_cast<float*>(chunkFrame_->extended_data[1]);
      float* fc  = reinterpret_cast<float*>(chunkFrame_->extended_data[2]);
      float* lfe = reinterpret_cast<float*>(chunkFrame_->extended_data[3]);
      float* sl  = reinterpret_cast<float*>(chunkFrame_->extended_data[4]);
      float* sr  = reinterpret_cast<float*>(chunkFrame_->extended_data[5]);
      if (!fl || !fr || !fc || !lfe || !sl || !sr) return -1;
      const size_t bytes = static_cast<size_t>(inputFrames) * sizeof(float);
      std::memcpy(sl, fl, bytes);
      std::memcpy(sr, fr, bytes);
      std::memset(fc, 0, bytes);
      std::memset(lfe, 0, bytes);
    }
    haveProcessedChunk = true;
  }

  if (!haveProcessedChunk) return 0;

  if (usedUpmixFilter && codecCtx_->ch_layout.nb_channels >= 6)
  {
    if (upmixMode_ == Upmix::FrontStage || upmixMode_ == Upmix::AlwaysFront)
    {
      for (int ch : { 4, 5 })
      {
        float* s = reinterpret_cast<float*>(chunkFrame_->extended_data[ch]);
        if (s) std::fill_n(s, inputFrames, 0.0f);
      }
    }
    else if (upmixMode_ == Upmix::Front30)
    {
      for (int ch : { 3, 4, 5 })
      {
        float* s = reinterpret_cast<float*>(chunkFrame_->extended_data[ch]);
        if (s) std::fill_n(s, inputFrames, 0.0f);
      }
    }
    else if (upmixMode_ == Upmix::StereoSub21)
    {
      float* fl = reinterpret_cast<float*>(chunkFrame_->extended_data[0]);
      float* fr = reinterpret_cast<float*>(chunkFrame_->extended_data[1]);
      float* fc = reinterpret_cast<float*>(chunkFrame_->extended_data[2]);
      if (fl && fr && fc)
        for (int i = 0; i < inputFrames; ++i)
        {
          fl[i] += 0.70710678f * fc[i];
          fr[i] += 0.70710678f * fc[i];
        }
      for (int ch : { 2, 4, 5 })
      {
        float* s = reinterpret_cast<float*>(chunkFrame_->extended_data[ch]);
        if (s) std::fill_n(s, inputFrames, 0.0f);
      }
    }
  }

  ApplyChannelGainsAndComputeLevels(chunkFrame_);

  if (processedFrames_ < 0 || processedFrames_ + inputFrames > framesPerPacket_)
    return -1;
  if (processedFrames_ == 0 && av_frame_make_writable(frame_) < 0)
    return -1;

  const size_t sampleBytes = static_cast<size_t>(inputFrames) * sizeof(float);
  const size_t dstOffsetBytes = static_cast<size_t>(processedFrames_) * sizeof(float);
  const int outputChannels = codecCtx_->ch_layout.nb_channels;
  for (int ch = 0; ch < outputChannels; ++ch)
  {
    std::memcpy(frame_->extended_data[ch] + dstOffsetBytes,
                chunkFrame_->extended_data[ch], sampleBytes);
  }

  processedFrames_ += inputFrames;
  if (processedFrames_ < framesPerPacket_)
    return 0;
  if (processedFrames_ != framesPerPacket_)
    return -1;

  frame_->nb_samples = framesPerPacket_;
  frame_->pts = nextPts_;
  nextPts_ += framesPerPacket_;
  processedFrames_ = 0;
  return EncodeFrameToBurst(outBuf, outSize);
}

int SpdifEncoder::EncodeFrameToBurst(uint8_t* outBuf, int outSize)
{
  AVFrame* encodeFrame = frame_;
  if (useDts_)
  {
    if (av_frame_make_writable(dtsFrame_) < 0)
      return -1;

    const uint8_t* inData[AV_NUM_DATA_POINTERS] = {};
    for (int ch = 0; ch < codecCtx_->ch_layout.nb_channels; ++ch)
      inData[ch] = frame_->extended_data[ch];

    const int converted = swr_convert(dtsS32Swr_, dtsFrame_->data,
                                      dtsFrame_->nb_samples, inData, frame_->nb_samples);
    if (converted < 0)
    {
      LogAv("swr_convert(DTS)", converted);
      return -1;
    }
    if (converted != framesPerPacket_)
    {
      std::fprintf(stderr, "[SpdifEncoder] unexpected DTS sample count: %d\n", converted);
      return -1;
    }
    dtsFrame_->pts = frame_->pts;
    encodeFrame = dtsFrame_;
  }

  int rc = avcodec_send_frame(codecCtx_, encodeFrame);
  if (rc < 0) { LogAv("avcodec_send_frame", rc); return -1; }

  rc = avcodec_receive_packet(codecCtx_, pkt_);
  if (rc == AVERROR(EAGAIN))
    return 0;
  if (rc < 0) { LogAv("avcodec_receive_packet", rc); return -1; }

  pkt_->stream_index = 0;
  writeDst_ = outBuf;
  writeCap_ = outSize;
  writeLen_ = 0;

  rc = av_write_frame(muxer_, pkt_);
  av_packet_unref(pkt_);
  if (rc < 0) { LogAv("av_write_frame(spdif)", rc); return -1; }
  avio_flush(muxer_->pb);

  int n = writeLen_;
  writeDst_ = nullptr;
  writeCap_ = 0;
  writeLen_ = 0;
  return n;
}

int SpdifEncoder::WritePacketThunk(void* opaque, const uint8_t* buf, int buf_size)
{
  return static_cast<SpdifEncoder*>(opaque)->OnWritePacket(buf, buf_size);
}

bool SpdifEncoder::IsEffectivelyStereo(const uint8_t* in, int inputFrames,
                                       bool* hasDiscreteActivity,
                                       double discreteRmsThreshold) const
{
  if (hasDiscreteActivity)
    *hasDiscreteActivity = false;
  const int ch = inLayout_.nb_channels;
  if (ch <= 2)
    return true;
  if (!in || inputFrames <= 0)
    return true;

  // On a 5.1 / 7.1 endpoint, stereo applications (music players, browsers, Discord, stereo games)
  // only populate Front Left (ch 0) and Front Right (ch 1); Windows leaves all discrete
  // non-stereo channels (Center = ch 2, LFE = ch 3, Surrounds = ch 4..ch-1) at digital zero.
  // If ANY discrete multichannel channel (Center, LFE, or Rear/Side Surrounds, c >= 2) carries
  // signal above the selected RMS threshold, the source is treated as native 5.1/7.1 and
  // passed through untouched. The strict Time-domain mode supplies a lower -80 dBFS threshold.
  double discreteSumSq = 0.0;
  double frontSumSq = 0.0;
  const int discreteChannels = ch - 2;

  if (inSampleFmt_ == AV_SAMPLE_FMT_FLT)
  {
    const float* f = reinterpret_cast<const float*>(in);
    for (int i = 0; i < inputFrames; ++i)
    {
      const double fl = f[i * ch + 0];
      const double fr = f[i * ch + 1];
      frontSumSq += fl * fl + fr * fr;
      for (int c = 2; c < ch; ++c)
      {
        const double v = f[i * ch + c];
        discreteSumSq += v * v;
      }
    }
  }
  else if (inSampleFmt_ == AV_SAMPLE_FMT_S16)
  {
    const int16_t* s = reinterpret_cast<const int16_t*>(in);
    for (int i = 0; i < inputFrames; ++i)
    {
      const double fl = s[i * ch + 0] / 32768.0;
      const double fr = s[i * ch + 1] / 32768.0;
      frontSumSq += fl * fl + fr * fr;
      for (int c = 2; c < ch; ++c)
      {
        const double v = s[i * ch + c] / 32768.0;
        discreteSumSq += v * v;
      }
    }
  }
  else if (inSampleFmt_ == AV_SAMPLE_FMT_S32)
  {
    const int32_t* s = reinterpret_cast<const int32_t*>(in);
    for (int i = 0; i < inputFrames; ++i)
    {
      const double fl = s[i * ch + 0] / 2147483648.0;
      const double fr = s[i * ch + 1] / 2147483648.0;
      frontSumSq += fl * fl + fr * fr;
      for (int c = 2; c < ch; ++c)
      {
        const double v = s[i * ch + c] / 2147483648.0;
        discreteSumSq += v * v;
      }
    }
  }

  const double discreteRms = std::sqrt(discreteSumSq / (inputFrames * discreteChannels));
  const double frontRms = std::sqrt(frontSumSq / (inputFrames * 2));
  constexpr double kFrontActivityThreshold = 0.0005; // ~ -66 dBFS
  const bool discreteActive = discreteRms >= std::max(0.0, discreteRmsThreshold);
  if (hasDiscreteActivity)
    *hasDiscreteActivity = discreteActive;

  if (discreteActive)
  {
    // Active Center, LFE, or Surround audio detected -> native 5.1 passthrough.
    // Hold native 5.1 mode for ~3 seconds of active front-only audio so brief front-only moments
    // in a 5.1 movie, game, or speaker test never flip back into the stereo upmixer.
    multichannelHoldFrames_ = sampleRate_ * 3;
    effectivelyStereo_ = false;
    return false;
  }

  if (multichannelHoldFrames_ > 0)
  {
    if (frontRms >= kFrontActivityThreshold)
      multichannelHoldFrames_ = std::max(0, multichannelHoldFrames_ - inputFrames);
    if (multichannelHoldFrames_ > 0)
    {
      effectivelyStereo_ = false;
      return false;
    }
  }

  effectivelyStereo_ = true;
  return true; // Only FL/FR carry audio -> Upmix to 5.1/3.1!
}

void SpdifEncoder::ExtractStereo(const uint8_t* in, uint8_t* out, int inputFrames) const
{
  const int ch = inLayout_.nb_channels;
  const int bps = av_get_bytes_per_sample(inSampleFmt_);
  const int inStride = ch * bps;
  const int outStride = 2 * bps;
  for (int i = 0; i < inputFrames; ++i)
    std::memcpy(out + i * outStride, in + i * inStride, static_cast<size_t>(outStride));
}

int SpdifEncoder::OnWritePacket(const uint8_t* buf, int buf_size)
{
  int n = buf_size;
  if (writeLen_ + n > writeCap_)
    n = writeCap_ - writeLen_;
  if (n > 0)
  {
    std::memcpy(writeDst_ + writeLen_, buf, static_cast<size_t>(n));
    writeLen_ += n;
  }
  return buf_size;
}
