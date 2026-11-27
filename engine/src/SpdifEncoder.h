// SpdifEncoder.h
//
// Real-time PCM -> AC3 (Dolby Digital) or DTS Core -> IEC 61937 (S/PDIF) burst encoder.
//
// Ported from SoundPusher's SPDIFAudioEncoder (MIT, Daniel Vollmer) and Kodi's
// CAEEncoderFFmpeg (GPL), modernized for FFmpeg 7.x / 8.x:
//   - new AVChannelLayout API (instead of the removed channel_layout/channels fields)
//   - avcodec_send_frame / avcodec_receive_packet (instead of codec->encode2)
//   - swr_alloc_set_opts2
//
// Design (from SoundPusher): swresample converts/downmixes the interleaved input to the
// encoder's required format; libavcodec encodes one packet; libavformat's "spdif" muxer
// wraps it into an IEC 61937 burst via a custom AVIO write callback.
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavutil/audio_fifo.h>
#include <libavutil/channel_layout.h>
#include <libswresample/swresample.h>
}

class SpdifEncoder
{
public:
  enum class Codec { AC3, DTS };

  // State reported by the live stream diagnostics panel.
  enum class FilterStatus : uint32_t
  {
    Disabled = 0,
    StereoUpmix = 1,
    DynamicStereo = 2,
    Warming = 3,
    NativeBypass = 4,
    HoldPrewarm = 5,
    TimeDomainWaiting = 6,
    TimeDomainMatrix = 7,
    TimeDomainNativeBypass = 8,
    SpeakerLayout4_0FoldDown = 9,
  };

  static constexpr int kSurroundFilterHopFrames = 2048;
  // Input-side DSP quantum. AC-3/DTS codec frame sizes remain unchanged.

  SpdifEncoder();
  ~SpdifEncoder();
  SpdifEncoder(const SpdifEncoder&) = delete;
  SpdifEncoder& operator=(const SpdifEncoder&) = delete;

  // Stereo->5.1 upmix mode.
  // - Off: swr default rematrix (front channels only)
  // - Surround: auto-detects non-surround sources and upmixes to all 5.1 speakers (Center + Sub + Rears)
  // - FrontStage: auto-detects non-surround sources and upmixes to 3.1 (Center + Sub/LFE), keeping rears silent
  // - Front30: auto-detects non-surround sources and upmixes to 3.0 (Center + Front L/R, no Sub/Rears)
  // - StereoSub21: auto-detects non-surround sources and upmixes to 2.1 (Front L/R + Sub/LFE, Center & rears silent)
  // - Always: forces 5.1 surround upmixing unconditionally on all inputs
  // - AlwaysFront: forces 3.1 upmixing (Center + Sub/LFE) unconditionally on all inputs
  // - DualStereo: copies FL->SL and FR->SR for stereo content; native multichannel passes through.
  // - In 4.0 speaker layout for DualStereo / TimeDomainNative51, native Center/LFE are folded
  //   into FL/FR and cleared; stereo-derived Center/LFE are omitted to avoid double counting.
  enum class Upmix
  {
    Off,
    Surround,     // Dynamic 5.1 (FL+FR+FC+LFE+SL+SR)
    FrontStage,   // Dynamic 3.1 (FL+FR+FC+LFE, rears silent)
    Front30,      // Dynamic 3.0 (FL+FR+FC, LFE & rears silent)
    StereoSub21,  // Dynamic 2.1 (FL+FR+LFE, Center & rears silent)
    Always,       // Forced 5.1 on all audio
    AlwaysFront,  // Forced 3.1 on all audio
    DualStereo,   // Direct stereo copy: FL->SL, FR->SR; Center/LFE silent; native 5.1 untouched
    TimeDomain,   // Center/difference-rear + one-pole LFE; auto-detects stereo in multichannel input
    TimeDomainNative51, // Content-aware matrix for stereo-in-5.1; native discrete channels are preserved
  };

  struct Params
  {
    Codec codec = Codec::AC3;
    int sampleRate = 48000;                         // AC3: 48000 / 44100 / 32000; DTS: 48000
    int64_t bitRate = 640000;                       // AC3-over-optical maximum
    int64_t dtsBitRate = 1536000;                   // DTS: 1536000 (full-rate) or 768000 (half-rate)
    AVSampleFormat inSampleFmt = AV_SAMPLE_FMT_FLT; // interleaved input from WASAPI/WAV
    AVChannelLayout inLayout{};                     // caller-owned; copied in Init().
                                                    // Any layout; downmixed to 5.1 by swr.
    Upmix upmix = Upmix::Off;                       // stereo->5.1 upmix mode
    bool foldDownCenterLfeToFront = false;          // 4.0 layout: fold native Center + LFE into FL/FR
    int surroundWindowSize = 4096;                  // FFmpeg surround FFT window (1024 for Extreme)
  };

  // Open the encoder. Returns false (and logs to stderr) on failure.
  bool Init(const Params& p);
  // Per-output-channel trim. Values are clamped to -60..0 dB; -60 dB is digital mute.
  void SetChannelGainsDb(const std::array<float, 6>& gainsDb);
  void Close();
  bool IsOpen() const { return codecCtx_ != nullptr; }

  // Thread-safe access to latest output frame peak and RMS levels (0.0 .. 1.0)
  void GetChannelLevels(float outPeaks[6], float outRms[6]) const;

  // Input sample-frames consumed per EncodePacket() call (AC3 == 1536, DTS == 512).
  int FramesPerPacket() const { return framesPerPacket_; }
  // Channels expected in the interleaved input (== inLayout.nb_channels).
  int InChannels() const { return inLayout_.nb_channels; }

  FilterStatus GetFilterStatus() const;
  uint32_t FilterFifoFrames() const;
  uint32_t FilterFifoTargetFrames() const;
  uint32_t FilterHopFrames() const;
  uint64_t FilterProcessedFrames() const { return filterInputFramesProcessed_; }

  // An IEC 61937 AC3 burst is 6144 bytes; DTS burst is 2048 bytes.
  static constexpr int kMaxBytesPerPacket = 6144;

  static constexpr int kPcmChunkFrames = 128;

  // Encode exactly FramesPerPacket() interleaved input frames into one IEC 61937 burst.
  //   in      : FramesPerPacket() * InChannels() samples in Params::inSampleFmt
  //   outBuf  : destination for the S/PDIF burst
  //   outSize : capacity of outBuf (must be >= kMaxBytesPerPacket)
  // Returns bytes written, 0 if the encoder produced no packet, <0 on error.
  int EncodePacket(const uint8_t* in, uint8_t* outBuf, int outSize);
  // Process one 256-sample PCM block. Returns 0 until a full codec frame is ready.
  int EncodeChunk(const uint8_t* in, int inputFrames, uint8_t* outBuf, int outSize);

private:
  // FFmpeg >= 6.1 AVIO write callback signature uses const uint8_t*.
  static int WritePacketThunk(void* opaque, const uint8_t* buf, int buf_size);
  int OnWritePacket(const uint8_t* buf, int buf_size);

  bool BuildFilterGraph();              // surround-upmix path
  bool FeedFilterFrames(const uint8_t* in, int inputFrames);
  int  EncodeFrameToBurst(uint8_t* outBuf, int outSize);
  void ApplyChannelGainsAndComputeLevels(AVFrame* target);
  void ApplyTimeDomainUpmix(AVFrame* target);
  void ApplyCenterLfeFoldDown(AVFrame* target);

  // Dynamic upmix helpers: detect stereo content in a multichannel stream and extract FL+FR.
  // Detect stereo content inside a multichannel endpoint. The strict Native 5.1
  // mode supplies a lower threshold so quiet discrete channels still protect
  // native multichannel material from being matrix-upmixed.
  bool IsEffectivelyStereo(const uint8_t* in, int inputFrames,
                           bool* hasDiscreteActivity = nullptr,
                           double discreteRmsThreshold = 0.0005) const;
  void ExtractStereo(const uint8_t* in, uint8_t* out, int inputFrames) const;

  mutable std::array<std::atomic<float>, 6> peakLevels_{};
  mutable std::array<std::atomic<float>, 6> rmsLevels_{};

  AVCodecContext*  codecCtx_ = nullptr;
  AVFormatContext* muxer_    = nullptr;
  SwrContext*      swr_      = nullptr;
  SwrContext*      dualStereoSwr_ = nullptr; // two-channel conversion fast path for Dual Stereo Copy
  SwrContext*      dtsS32Swr_ = nullptr; // FLTP to packed S32 for DTS
  AVFrame*         frame_    = nullptr;  // complete codec-sized planar-float frame
  AVFrame*         chunkFrame_ = nullptr; // one 128-sample DSP work frame
  int              processedFrames_ = 0; // frames accumulated in frame_ for the next codec packet
  AVFrame*         dtsFrame_  = nullptr;  // packed S32 frame for the DTS encoder
  AVPacket*        pkt_      = nullptr;

  // surround-upmix pipeline (used when upmix == Surround and input is <= 2ch)
  bool             useFilter_ = false;
  bool             useDts_ = false;
  bool             timeDomain_ = false;
  bool             timeDomainNative51_ = false; // content-aware native-channel protection with lower detection threshold
  bool             timeDomainMatrixDetected_ = false;
  bool             timeDomainMatrixApplied_ = false;
  bool             foldDownCenterLfeToFront_ = false;
  bool             speakerFoldDownActive_ = false;
  float            foldDownLimiterGain_ = 1.0f;
  float            lfeFilterState_ = 0.0f;
  AVFilterGraph*   graph_     = nullptr;
  AVFilterContext* fsrc_      = nullptr; // abuffer
  AVFilterContext* fsink_     = nullptr; // abuffersink
  AVAudioFifo*     fifo_      = nullptr; // accumulates filtered 5.1 fltp samples
  AVFrame*         filtFrame_ = nullptr; // scratch for draining the sink
  AVFrame*         filterInputFrame_ = nullptr; // reusable, preallocated filter input frame
  int64_t          filterInputPts_ = 0;
  int              surroundWindowSize_ = 4096;
  int              surroundHopFrames_ = kSurroundFilterHopFrames;

  // Dynamic upmix: input is multichannel (e.g. 5.1 cable) but we detect at runtime
  // whether the content is actually stereo (non-FL/FR channels silent) and upmix it.
  // The FFT graph handles front-only content; it is bypassed while discrete 5.1
  // channels are active and pre-warmed during the existing native-layout hold.
  bool             dynamicUpmix_ = false;
  bool             alwaysUpmix_ = false;
  bool             dualStereo_ = false; // direct L/R copy to rear channels for stereo sources
  bool             dynamicFilterBypassed_ = false;
  bool             dynamicFilterNeedsWarmup_ = false;
  bool             dynamicFilterPrimed_ = false;
  bool             dynamicFilterActive_ = false;
  int              dynamicFilterWarmupFrames_ = 0;
  uint64_t         filterInputFramesProcessed_ = 0;
  Upmix            upmixMode_ = Upmix::Off;
  mutable bool     effectivelyStereo_ = true;
  mutable int      multichannelHoldFrames_ = 0; // hysteresis hold for native 5.1 streams
  AVChannelLayout  filterInLayout_{}; // layout fed to the filter graph (stereo in dynamic mode)
  std::vector<uint8_t> stereoExtract_; // buffer for extracted FL+FR frames

  AVChannelLayout  inLayout_{};
  AVSampleFormat   inSampleFmt_ = AV_SAMPLE_FMT_FLT;
  int              sampleRate_ = 48000;
  int              framesPerPacket_ = 0;
  std::array<std::atomic<float>, 6> channelGainLinear_;
  int64_t          nextPts_ = 0;

  // Scratch state used by OnWritePacket() during a single EncodePacket() call.
  uint8_t* writeDst_ = nullptr;
  int      writeCap_ = 0;
  int      writeLen_ = 0;
};
