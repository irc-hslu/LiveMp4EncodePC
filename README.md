# VpcCLiveStreamer

Live volumetric video pipeline: capture RGB-D point clouds from an **Orbbec** depth camera, stream a lightweight UDP preview to a browser, encode with **uvgVPCCenc** (MPEG V-PCC), and packetize the result as **V3C RTP** via **uvgV3CRTP**.

```
Orbbec camera
     │
     ▼
vpcclive_streamer ── UDP 8891 ──► Web relay (WS 8081) ──► Browser preview (Three.js)
     │
     └── UDP 8890 ──► Web relay ──► Browser V-PCC decoder
              │
              └── optional .v3c recording + SDP files
```

## Features

- **Orbbec capture** using device default stream profiles (similar to the official Python examples), with RGB-D alignment and point-cloud filtering
- **Decoupled preview and encode** — live point-cloud preview does not block V-PCC encoding
- **Configurable subsampling** — separate limits for capture, preview, and encode point counts
- **Per-frame quantization** — geometry origin/scale recomputed each frame for stable long-running streams
- **Web viewer** — browser-based live preview and V-PCC playback via a small Node.js UDP→WebSocket relay
- **Recording** — optional V3C sample stream (`.v3c`) and SDP output while streaming
- **Windows hardened** — SEH-safe Orbbec SDK wrappers, main-thread encode on Windows, crash logging

## Prerequisites

| Component | Version / notes |
|-----------|-----------------|
| **OS** | Windows 10/11 (primary target); Linux may build with minor path adjustments |
| **CMake** | 3.25 or newer |
| **Compiler** | MSVC 2022 (Windows) or GCC/Clang with C++20 |
| **Node.js** | 18+ (for the web relay and static file server) |
| **Orbbec SDK** | Placed at `orbbecSDK/` (see below) |
| **uvgVPCCenc** | Source tree at `uvgVPCCenc/` (bundled or cloned) |

### Orbbec SDK

1. Download the [Orbbec SDK](https://orbbec.com/developers/orbbec-sdk/) for your platform.
2. Extract/copy it so this layout exists:

```
orbbecSDK/
  include/libobsensor/ObSensor.hpp
  lib/OrbbecSDK.lib   (or obsensor.lib on some builds)
```

If the SDK is not found, the app builds with a **synthetic point-cloud fallback** (`--synthetic`).

### Dependencies layout

```
LiveMp4EncodePC/
├── uvgVPCCenc/          # V-PCC encoder (includes uvgV3CRTP as a sub-dependency)
├── orbbecSDK/           # Orbbec libobsensor (not committed; see .gitignore)
├── src/winfix/          # Application source
└── web/                 # Browser viewer + relay server
```

## Usage

Follow these four steps in order (same as [`cmd`](cmd)). Steps 2–4 need **separate terminals**. Replace `D:\LiveStream\LiveMp4EncodePC` if your project root differs.

**One-time setup** (before step 1):

```powershell
cd D:\LiveStream\LiveMp4EncodePC
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cd web\server
npm install
```

Close **Orbbec Viewer** before step 4.

### 1. Build project

```powershell
cmake --build D:\LiveStream\LiveMp4EncodePC\build --config Release --target vpcclive_streamer
```

### 2. Run web relay server (new terminal)

```powershell
cd D:\LiveStream\LiveMp4EncodePC\web\server
npm run relay
```

### 3. Run HTTP server (new terminal)

```powershell
cd D:\LiveStream\LiveMp4EncodePC\web\server
npm run serve
```

Open [http://localhost:8080](http://localhost:8080) and connect to `ws://localhost:8081`.

### 4. Run C++ capture → V-PCC encoding app (new terminal)

```powershell
cd D:\LiveStream\LiveMp4EncodePC\build\Release
vpcclive_streamer.exe --address 127.0.0.1 --port 8890 --preview-port 8891 --fps 30 --encode-fps 3 --encode-points 20000 --geo-bits 9 --preview-points 20000 --threads 4
```

| Port / service | Value |
|----------------|-------|
| V-PCC RTP (UDP) | `8890` (`--port`) |
| Preview (UDP) | `8891` (`--preview-port`) |
| WebSocket relay | `8081` (`npm run relay`) |
| Viewer (HTTP) | `8080` (`npm run serve`) |

Press **Q** or **Ctrl+C** to stop the streamer (twice to force quit).

## Build options

| CMake option | Default | Description |
|--------|---------|-------------|
| `VPCCLIVE_ENABLE_V3CRTP` | `ON` | Link V3C RTP streaming |
| `VPCCLIVE_ENABLE_ORBBEC` | `ON` | Link Orbbec SDK when `ORBBEC_SDK_DIR` is set |
| `ORBBEC_SDK_DIR` | `./orbbecSDK` | Path to Orbbec SDK root |
| `UVGVPCCENC_ROOT` | `./uvgVPCCenc` | Path to uvgVPCCenc source |

Additional targets: `v3c_smoke_test`, `kvazaar_smoke_test`, `v3c_replay`.

## All command-line flags

```
vpcclive_streamer [--address HOST] [--port PORT] [--preview-port PORT]
                  [--preview-points N] [--fps FPS] [--encode-fps FPS]
                  [--encode-points N] [--capture-points N] [--geo-bits N]
                  [--threads N] [--sdp DIR] [--record-dir DIR] [--record-file NAME]
                  [--color-width W] [--color-height H] [--synthetic]
                  [--profile-interval SEC] [--no-profile]
```

| Option | Default | Description |
|--------|---------|-------------|
| `--address HOST` | `127.0.0.1` | Destination for V-PCC RTP and preview UDP |
| `--port PORT` | `8890` | V-PCC RTP UDP port |
| `--preview-port PORT` | `8891` | Point-cloud preview UDP port |
| `--preview-points N` | `200000` | Max points per preview frame |
| `--fps FPS` | device default | Color/depth capture frame rate |
| `--encode-fps FPS` | `10` | Target V-PCC encode frame rate (`cmd` uses `3`) |
| `--encode-points N` | `80000` | Max points sent to the encoder (`cmd` uses `20000`) |
| `--capture-points N` | auto | Max points at capture; default = max(preview, encode) |
| `--geo-bits N` | `10` | Geometry bit depth (`cmd` uses `9` for speed) |
| `--threads N` | auto (max 4 on Windows) | uvgVPCCenc worker threads |
| `--color-width W` | device default | Force color width |
| `--color-height H` | device default | Force color height |
| `--sdp DIR` | — | Write SDP files |
| `--record-dir DIR` | — | Record `.v3c` stream + SDP while streaming |
| `--record-file NAME` | `v3c_live.v3c` | Recording filename |
| `--synthetic` | off | Synthetic point cloud (no camera) |
| `--profile-interval SEC` | `5` | `[profile]` log interval |
| `--no-profile` | off | Disable profiler output |

See [`cmd`](cmd) for the recommended default invocation.

## Project structure

```
src/winfix/
  main.cpp              Main loop, CLI, profiling
  OrbbecCapture.hpp     Orbbec pipeline + point-cloud extraction
  EncoderStreamer.hpp   uvgVPCCenc + uvgV3CRTP bridge
  PointCloudPreview.hpp UDP preview (PCV1 format)
  PipelineProfiler.hpp  Capture / preview / encode / RTP stats
  orbbec_safe.cpp       SEH-safe Orbbec SDK wrappers (Windows)
  v3c_replay.cpp        Replay tool
  v3c_smoke_test.cpp    RTP smoke test

web/
  index.html            Live viewer
  js/                   Three.js viewer, V-PCC decoder worker
  server/               Node.js UDP relay + npm scripts

uvgVPCCenc/             V-PCC encoder submodule / vendor tree
cameraconfig_4cam.json  Example multi-camera config (reference; not used by vpcclive_streamer directly)
cmd                     Local build/run cheat sheet
```

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `No Orbbec camera found` | USB, drivers, or Orbbec Viewer holding the device | Replug camera, close Orbbec Viewer, check Device Manager |
| Access violation on start | SDK conflict or bad USB state | Close other Orbbec apps; retry; use `--synthetic` to isolate |
| Low encode fps | High point count or `--geo-bits 10` | Lower `--encode-points`, use `--geo-bits 9`, reduce `--encode-fps` |
| Preview works, no V-PCC in browser | Relay not running or wrong ports | Start `npm run relay`; match `--port` / `--preview-port` |
| Patch generation errors after long run | Scene drift with stale quantization | Fixed in current code (per-frame origin); rebuild |
| Build fails on uvgVPCCenc | Submodule out of date or Kvazaar patch | Ensure `uvgVPCCenc/` is present and CMake configure succeeds |

Runtime logs include `[orbbec]`, `[profile]`, and uvgVPCCenc `[uvgVPCCenc]` lines. Orbbec SDK logs may appear under `build/Release/Log/`.

## Related projects

- [uvgVPCCenc](https://github.com/ultravideo/uvgVPCCenc) — V-PCC encoder
- [uvgV3CRTP](https://github.com/ultravideo/uvgV3CRTP) — V3C RTP packetization
- [Orbbec SDK](https://orbbec.com/developers/orbbec-sdk/) — Depth camera SDK

## License

This application integrates third-party libraries (uvgVPCCenc, uvgV3CRTP, uvgRTP, Kvazaar, Orbbec SDK, etc.), each under its own license. Refer to the respective vendor trees for license terms.
