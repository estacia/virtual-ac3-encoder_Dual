# Dolby Control Center (`virtual-ac3-encoder`)

[![CI](https://github.com/PatalJunior/virtual-ac3-encoder/actions/workflows/ci.yml/badge.svg)](https://github.com/PatalJunior/virtual-ac3-encoder/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

A real-time Windows 10/11 software implementation of **Dolby Digital Live (DDL)** and **DTS Connect** — a virtual surround audio processor and **DirectX 11 Control Center** that captures stereo or 5.1 PCM audio, encodes it in real time to **DTS 5.1**, **Dolby Digital (AC-3) 5.1**, or **PCM Stereo 2.0**, wraps it as an **IEC 61937 / S-PDIF** bitstream, and streams it over a single **Toslink optical** cable to your AV receiver or soundbar.

> **Why this exists:** Optical (S/PDIF / Toslink) only has enough bandwidth for uncompressed **2.0 stereo PCM** *or* a compressed **5.1 bitstream** (Dolby Digital / DTS) — never uncompressed 5.1 PCM. Most modern motherboards and USB DACs (Realtek, etc.) omit Dolby Digital Live and DTS Interactive licensing, leaving PC games, movies, and music stuck in 2.0 stereo over optical. This project performs the real-time 5.1 encoding and stereo-to-surround upmixing in software with ultra-low latency.

---

## Key Features

| Feature | Details |
| :--- | :--- |
| **Multi-Format Digital Output** | **DTS Core 5.1** (`1536 kbps` full-rate / `768 kbps` half-rate), **Dolby Digital AC-3 5.1** (`640 kbps` / `448 kbps`), and **PCM Stereo 2.0** over **Toslink Optical (S/PDIF)** or **HDMI ARC**. |
| **Smart Multi-Layout Upmixing (`5.1` / `3.1` / `3.0` / `2.1`)** | **Dynamic Surround 5.1**, **Time-domain Matrix 5.1** (Center + difference rears + one-pole LFE; no FFT/FIFO), and **Time-domain Matrix + Native 5.1 Untouched** (matrix only for 2-channel input; all multichannel input bypasses the matrix), plus **Dual Stereo Copy**, **Front Stage 3.1**, **Front Stage 3.0**, **Stereo + Sub 2.1**, **Always 5.1 / 3.1**, or **Off**. |
| **Dynamic Surround Filter Diagnostics** | Reports FFT filter state, active hop size, and live FIFO occupancy. Extreme uses FFmpeg's minimum 1024-sample window (512-sample hop); other presets retain 4096. Optional Time-domain Matrix mode bypasses the FFT/FIFO entirely. |
| **Dolby Control Center (DX11 GUI)** | Hardware-accelerated **Dear ImGui + DirectX 11** dark studio UI with **60 FPS 6-channel VU meters** (peak hold + dB readouts), per-speaker level trims (`-60 dB` to `0 dB` + mute), live hot-reload (zero audio interruption when changing modes/trims), and system tray integration. |
| **5-Tier Latency Presets** | **Extreme**, **Fast**, **Balanced** (default), **Safe**, and **Relaxed** select progressively larger pre-roll cushions. AC-3 Extreme uses a 128-frame input safety cushion and a 1024-sample Surround FFT window. An optional custom SafeFrames control allows 32–1536 samples. Exclusive S/PDIF first attempts a low-latency period of at least 128 frames (~2.67 ms at 48 kHz), then retries the configured 256-frame period and finally a one-burst buffer when needed. Complete AC-3 bursts are streamed over successive render buffers, and encoding runs on a worker thread ahead of the render deadline. |
| **128-Sample Incremental DSP** | The capture ring is consumed in **128-sample blocks** (~2.67 ms at 48 kHz). Each block is conditioned immediately and copied directly into its codec-frame staging buffer; the intermediate processed-PCM FIFO is eliminated. Only a complete codec frame (**1536 samples AC-3 / 512 samples DTS**) is passed to the codec. This overlaps DSP work with continued PCM arrival without changing AC-3 framing or IEC 61937 burst format. |
| **Flexible Audio Capture** | Event-driven loopback capture on Windows 10 1703+ (with a short compatibility poll on older Windows); works out-of-the-box with **Secure Boot ON** via **[VB-CABLE](https://vb-audio.com/Cable/)** (WASAPI loopback or recording capture), or with the included **WDK WaveRT Virtual 5.1 Driver** (`driver/`). |

---

## System Architecture & Audio Flowcharts

### 1. End-to-End Real-Time Pipeline

```mermaid
flowchart LR
    subgraph Sources["Windows Audio Sources"]
        App["Games / Movies / Music\n(Stereo 2.0 or Discrete 5.1 PCM)"]
        Cable["Virtual Endpoint\n(VB-CABLE Input or WaveRT 5.1 Driver)"]
        App --> Cable
    end

    subgraph Engine["Real-Time Engine (engine.exe)"]
        Cap["WasapiCapture\n(Loopback or Event Capture)"]
        Ring["Lock-Free SPSC RingBuffer\n(Pre-Roll Cushion + Drift Trim)"]
        Chunk["128-sample PCM blocks\n(~2.67 ms at 48 kHz)"]
        DSP["Incremental DSP\n(Upmix + Trims per block)"]
        Accum["Direct codec-frame staging\n(AC-3 1536 / DTS 512 samples)"]
        Enc{"Output Mode"}
        AC3["FFmpeg AC-3 Encoder\n1536 frames -> 6144B IEC 61937"]
        DTS["FFmpeg DCA (DTS) Encoder\n512 frames -> 2048B IEC 61937"]
        PCM["WASAPI Shared Render\nStereo 2.0 PCM"]
        Pass["WasapiPassthrough\n(EXCLUSIVE Event Master Clock)"]

        Cap --> Ring --> Chunk --> DSP --> Accum --> Enc
        Enc -->|Dolby Digital 5.1| AC3 --> Pass
        Enc -->|DTS 5.1| DTS --> Pass
        Enc -->|PCM 2.0| PCM
    end

    subgraph GUI["Dolby Control Center (control_center.exe)"]
        Shm["Shared Memory Telemetry\n(6-Ch Peak & RMS Levels)"]
        Ini["Live Hot-Reload IPC\n(virtual-ac3-encoder.conf + ac3-control.ini)"]
    end

    Hw["Optical S/PDIF (Toslink)\n-> AV Receiver / Soundbar"]

    Cable --> Cap
    DSP -.->|60 FPS Lock-Free Atomics| Shm
    Ini -.->|Seamless Live Re-Init| Engine
    Pass --> Hw
    PCM --> Hw
```

### 2. Dynamic Stereo & Surround Upmix Decision Flow

```mermaid
flowchart TD
    In["Incoming PCM Block\n(128 samples; 2.0 or 5.1 PCM)"] --> Mode{"Selected Upmix Mode"}

    Mode -->|Off| SwrPass["libswresample Rematrix\n(Direct Pass-Through)"]
    Mode -->|Dual Stereo Copy| Copy["Direct Copy: FL->SL, FR->SR\n(Center/LFE silent; native 5.1 untouched)"]
    Mode -->|Always 5.1| Filt51["FFmpeg surround Filter Graph\n(Full 5.1: FL, FR, C, LFE, SL, SR)"]
    Mode -->|Always Front Stage 3.1| Filt31["FFmpeg surround Filter Graph\n(3.1 Stage: FL, FR, C, LFE | SL/SR Muted)"]

    Mode -->|Dynamic 5.1 / 3.1 / 3.0 / 2.1| Detect{"Discrete Energy on\nC / LFE / SL / SR > -66 dB?\n(3s Multichannel Hold)"}

    Detect -->|Yes: True Discrete 5.1\ne.g. Netflix / 5.1 Game| SwrPass
    Detect -->|No: Stereo-in-5.1\ne.g. YouTube / Music / VoIP| Extract["Extract FL + FR Pair"]

    Extract --> StageChoice{"Speaker Layout Target"}
    StageChoice -->|Dynamic 5.1| Filt51
    StageChoice -->|Front Stage 3.1| Filt31
    StageChoice -->|Front Stage 3.0| Filt30["3.0 Stage: FL, FR, C\n(LFE & SL/SR Muted - HDMI ARC / Soundbar)"]
    StageChoice -->|Stereo + Sub 2.1| Filt21["2.1 Stage: FL, FR + LFE Bass\n(Center Folded to L/R, C & SL/SR Muted)"]

    SwrPass --> Gains["Per-Speaker Gain Trims (-60..0 dB)\n& 6-Channel Peak/RMS Metering"]
    Copy --> Gains
    Filt51 --> Gains
    Filt31 --> Gains
    Filt30 --> Gains
    Filt21 --> Gains
```

---

## Quick Start (Download & Install)

### 1. Install VB-CABLE (Virtual Audio Device)
1. Download and install **[VB-CABLE](https://vb-audio.com/Cable/)**.
2. Open **Windows Sound Settings (`mmsys.cpl`)**:
   - Set **CABLE Input (VB-Audio Virtual Cable)** as your **Default Playback Device**.
   - *(Optional)* Select **CABLE Input** -> **Configure** -> **5.1 Surround** (or leave as Stereo and let **Dynamic Surround 5.1** / **Front Stage 3.1** upmix automatically).
3. Dolby Control Center uses **WASAPI Loopback** by default to capture the CABLE Input playback endpoint. Keep Loopback enabled when using the included render-only WaveRT Virtual 5.1 driver, which has no recording endpoint. For a real recording endpoint such as VB-CABLE **CABLE Output**, you can disable Loopback in the GUI; direct capture then requests a **5 ms IAudioClient3 shared-mode period** (10 ms in Safe/Relaxed presets) when supported. Windows does not allow IAudioClient3 custom-period initialization with the LOOPBACK flag.
4. Connect your PC's **Optical (S/PDIF / Toslink)** port to your AV receiver or soundbar.

### 2. Install Dolby Control Center
Download the latest release from **[Releases](https://github.com/PatalJunior/virtual-ac3-encoder/releases/latest)**:
- **`virtual-ac3-encoder-setup-x.y.z.exe`** *(Recommended)* — Per-user installer (no admin required). Installs **Dolby Control Center**, sets up automatic system-tray startup at Windows logon, and adds Start Menu shortcuts.
- **`virtual-ac3-encoder-x.y.z-win64.zip`** — Portable zip; extract anywhere and run `control_center.exe` (or `engine.exe` from CLI).

---

## Dolby Control Center Guide

### Output Modes
- **DTS 5.1 Surround (`1536 kbps` / `768 kbps`)**: Highest-bitrate optical surround (`512`-sample frames for ultra-low latency).
- **Dolby Digital 5.1 (`640 kbps` / `448 kbps`)**: Universal AC-3 optical bitstream supported by virtually all AV receivers and soundbars.
- **PCM Stereo 2.0**: Uncompressed stereo pass-through over WASAPI shared mode.

### Stereo Upmix & Speaker Layout Modes
- **Dynamic Surround 5.1 (Default)**: Automatically detects whether the audio playing is stereo (music, YouTube, stereo games, Discord) or discrete 5.1 (movies, surround games). Stereo sources are upmixed across all 5.1 speakers; true 5.1 sources (including Center-only dialogue) pass through untouched with a 3-second hold window.
- **Front Stage 3.1 (Home Theater Recommended)**: Dynamically steers vocals to the **Center** speaker and deep bass to the **Subwoofer (LFE)** while keeping rear surrounds completely silent.
- **Front Stage 3.0 (HDMI ARC / Soundbar)**: Dynamically steers vocals to the **Center** speaker alongside **Front Left & Right** while keeping both Subwoofer (LFE) and rear surrounds silent — ideal for 3.0 soundbars and LCR setups without a dedicated subwoofer.
- **Stereo + Sub 2.1 (Bookshelf + Subwoofer)**: Extracts low-frequency bass (`< 140 Hz`) to the **Subwoofer (LFE)** while folding centered vocals back into **Front Left & Right** and keeping Center and rear surrounds silent.
- **Always Surround 5.1 / Always Front Stage 3.1**: Forces upmixing unconditionally on all inputs.
- **Dual Stereo Copy**: For stereo sources, copies Front Left directly to Surround Left and Front Right directly to Surround Right. Center and LFE are silent. The mode uses no matrix/FFT upmix filter and leaves detected native 5.1 content unchanged.
- **Time-domain Matrix 5.1**: Applies a low-latency time-domain matrix to stereo content. When the input endpoint is configured as 5.1, front-only content can also be detected as stereo and matrix-upmixed.
- **Time-domain Matrix + Native 5.1 Untouched**: Applies the same matrix only when the configured input endpoint is exactly 2-channel. Any multichannel endpoint (5.1/7.1) bypasses the matrix, even when only FL/FR currently carry audio. Select this when preserving a 5.1-configured source layout is more important than upmixing stereo-padded content.
- **Off**: Pure 1:1 channel mapping with no upmix filter.

### Speaker Level Trims & Master Release
- **6-Speaker Trims (`FL`, `FR`, `C`, `LFE`, `SL`, `SR`)**: Adjust any speaker from `-60 dB` to `0 dB` or toggle **Mute** in real time while audio is playing.
- **Master Power Button**: Click `[ ACTIVE: ENCODING TO S/PDIF ]` at the top of the window (or via the system tray icon) to immediately stop encoding and release exclusive ownership of the optical port.

---

## CLI Reference (`engine.exe`)

While `control_center.exe` manages `engine.exe` automatically, you can also run `engine.exe` standalone:

```powershell
# List all playback and recording endpoints
engine.exe --list

# Probe which playback endpoints support AC-3 passthrough
engine.exe --probe

# Capture "CABLE Input" via WASAPI loopback and output AC-3 5.1 to Realtek Digital Output
engine.exe --loopback --in "CABLE Input" --out "Realtek Digital Output"

# Capture "CABLE Output" recording endpoint directly
engine.exe --in "CABLE Output" --out-spdif --bitrate 640000 --upmix surround
```

| Flag | Description |
| :--- | :--- |
| `--list` | List all active Windows render and capture endpoints. |
| `--probe` | Query `IAudioClient::IsFormatSupported` for IEC 61937 AC-3 support on all render endpoints. |
| `--mon` | Run a 5-second capture-only throughput diagnostic. |
| `--loopback` | Capture `--in` as a *render* endpoint via WASAPI loopback; uses the Windows audio-engine period. |
| `--direct-capture` | Capture `--in` from a *recording* endpoint (e.g. `CABLE Output`); requests a 5 ms IAudioClient3 period when supported (10 ms in Safe/Relaxed). Recommended for lowest capture latency. |
| `--in <name>` / `--in-id <id>` | Capture device friendly-name substring or exact endpoint ID. |
| `--out <name>` / `--out-id <id>` | Optical output device friendly-name substring or exact endpoint ID. |
| `--out-spdif` | Automatically pick the first render endpoint with an `SPDIF` form factor. |
| `--bitrate <bps>` | AC-3 bitrate (`640000` or `448000`). |
| `--safe <frames>` | Target ring-buffer drift floor in sample frames. |
| `--upmix <mode>` | `surround` (`5.1`), `time_domain` (low-latency matrix), `dual_stereo`, `front` (`3.1`), `3.0`, `2.1`, `always`, `always_front`, or `off`. |
| `--config <path>` | Path to config file (defaults to `virtual-ac3-encoder.conf` next to `engine.exe`). |
| `--hidden` / `--log <path>` | Run with hidden console window / append output to log file. |

---

## Building from Source

### 1. User-Mode Engine & Dolby Control Center (`engine/`)
Requires **Visual Studio 2022** (Desktop development with C++ workload), **CMake 3.20+**, and **PowerShell**:

```powershell
# 1. Fetch prebuilt FFmpeg 8 shared dev libraries & Dear ImGui sources
.\scripts\fetch-ffmpeg.ps1
.\scripts\fetch-imgui.ps1

# 2. Configure & build Release x64 targets (ac3core, engine, control_center, encode_wav, unit_tests)
cmake -S engine -B engine\build -G "Visual Studio 17 2022" -A x64
cmake --build engine\build --config Release

# 3. Run the doctest unit test suite
ctest --test-dir engine\build -C Release --output-on-failure
```

### 2. Optional Kernel Virtual 5.1 Driver (`driver/`)
Derived from Microsoft's WaveRT `SimpleAudioSample`, configured as a single 5.1 render-only endpoint (`KSAUDIO_SPEAKER_5POINT1`, 48 kHz 16-bit). Requires **WDK 10.0.26100** and **Secure Boot OFF** (for test-signed loading):

```powershell
# Build with 64-bit MSBuild
& "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\amd64\MSBuild.exe" `
  driver\SimpleAudioSample.sln /p:Configuration=Release /p:Platform=x64 /p:SpectreMitigation=false

# Install (Elevated PowerShell, Secure Boot OFF):
.\scripts\install-driver.ps1 -EnableTestSigning   # Reboot after enabling test signing
.\scripts\install-driver.ps1
```

---

## License & Attributions

Licensed under the **MIT License** — see [`LICENSE`](LICENSE).
- Real-time master-clock ring-buffer architecture modeled on [SoundPusher](https://codeberg.org/q-p/SoundPusher) (MIT, Daniel Vollmer).
- Kernel virtual audio driver derived from Microsoft's [Windows Driver Samples `SimpleAudioSample`](https://github.com/microsoft/Windows-driver-samples) (MIT).
- Dynamically links **FFmpeg** (LGPL) and uses **Dear ImGui** (MIT) for the DirectX 11 Control Center. Full attributions in [`NOTICE.md`](NOTICE.md).
