#include <algorithm>
#include <atomic>

#include <chrono>

#include <condition_variable>

#include <csignal>

#include <cstdlib>

#include <exception>

#include <deque>

#include <iostream>

#include <mutex>

#include <string>

#include <thread>

#ifdef _WIN32

#include <conio.h>

#include <winsock2.h>

#include <windows.h>

#include <cstring>

#include <cstdio>

static void writeFatalLog(const char* message) {
  std::cerr << message << std::flush;
  const HANDLE stderrHandle = GetStdHandle(STD_ERROR_HANDLE);
  if (stderrHandle != nullptr && stderrHandle != INVALID_HANDLE_VALUE) {
    DWORD written = 0;
    WriteFile(stderrHandle, message, static_cast<DWORD>(std::strlen(message)), &written, nullptr);
    FlushFileBuffers(stderrHandle);
  }
}

LONG WINAPI unhandledExceptionFilter(EXCEPTION_POINTERS *info) {
  char buffer[256];
  if (info && info->ExceptionRecord) {
    std::snprintf(
        buffer,
        sizeof(buffer),
        "Unhandled Windows exception: 0x%08lx (thread %lu)\n",
        info->ExceptionRecord->ExceptionCode,
        GetCurrentThreadId());
  } else {
    std::snprintf(
        buffer,
        sizeof(buffer),
        "Unhandled Windows exception (thread %lu)\n",
        GetCurrentThreadId());
  }
  writeFatalLog(buffer);
  return EXCEPTION_EXECUTE_HANDLER;
}

#endif

#include "EncoderStreamer.hpp"

#include "OrbbecCapture.hpp"

#include "PipelineProfiler.hpp"

#include "PointCloudPreview.hpp"

namespace {

std::atomic<bool> g_running{true};
std::atomic<bool> g_shutdownAnnounced{false};
std::atomic<int> g_shutdownPressCount{0};

void requestShutdown(const char *reason) {
  const int pressCount = g_shutdownPressCount.fetch_add(1) + 1;
  g_running = false;

  if (pressCount >= 2) {
    std::cerr << "\nForce quit.\n" << std::flush;
    std::quick_exit(pressCount == 2 ? EXIT_SUCCESS : EXIT_FAILURE);
  }

  if (!g_shutdownAnnounced.exchange(true)) {
    std::cerr << "\n"
              << reason
              << " Shutting down (press Ctrl+C or Q again to force quit)...\n"
              << std::flush;
  }
}

void handleSignal(int) { requestShutdown("Interrupt signal received."); }

[[noreturn]] void terminateHandler() {
  std::cerr << "Fatal: std::terminate() called (unhandled exception in a worker thread).\n"
            << std::flush;
#ifdef _WIN32
  writeFatalLog("Fatal: std::terminate() called (unhandled exception in a worker thread).\n");
#endif
  std::abort();
}

#ifdef _WIN32
BOOL WINAPI consoleCtrlHandler(DWORD ctrlType) {
  switch (ctrlType) {
  case CTRL_C_EVENT:
  case CTRL_BREAK_EVENT:
    requestShutdown("Ctrl+C pressed.");
    return TRUE;
  case CTRL_CLOSE_EVENT:
    requestShutdown("Console window closing.");
    return TRUE;
  default:
    return FALSE;
  }
}

void keyboardListenerLoop(std::stop_token stopToken) {
  const HANDLE inputHandle = GetStdHandle(STD_INPUT_HANDLE);
  DWORD originalMode = 0;
  const bool consoleInput = inputHandle != nullptr &&
                            inputHandle != INVALID_HANDLE_VALUE &&
                            GetConsoleMode(inputHandle, &originalMode);

  if (consoleInput) {
    SetConsoleMode(inputHandle,
                   originalMode & ~(ENABLE_LINE_INPUT | ENABLE_ECHO_INPUT));
  }

  while (!stopToken.stop_requested() && g_running.load()) {
    bool quit = false;

    if (consoleInput) {
      INPUT_RECORD record{};
      DWORD recordsRead = 0;
      while (g_running.load() && !stopToken.stop_requested() &&
             ReadConsoleInput(inputHandle, &record, 1, &recordsRead) &&
             recordsRead > 0) {
        if (record.EventType != KEY_EVENT || !record.Event.KeyEvent.bKeyDown) {
          continue;
        }

        const WCHAR ch = record.Event.KeyEvent.uChar.UnicodeChar;
        if (ch == L'q' || ch == L'Q' ||
            record.Event.KeyEvent.wVirtualKeyCode == VK_ESCAPE) {
          quit = true;
          break;
        }
      }
    } else if (_kbhit()) {
      const int ch = _getch();
      if (ch == 'q' || ch == 'Q' || ch == 27) {
        quit = true;
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
      continue;
    }

    if (quit) {
      requestShutdown("Q pressed.");
      break;
    }

    if (!consoleInput) {
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
  }

  if (consoleInput) {
    SetConsoleMode(inputHandle, originalMode);
  }
}
#endif

void printUsage(const char *executable) {

  std::cerr

      << "Usage: " << executable

      << " [--address HOST] [--port PORT] [--preview-port PORT] "
         "[--preview-points N]"
      << " [--color-width W] [--color-height H]"
      << " [--fps FPS] [--encode-fps FPS] [--encode-points N] [--capture-points N] [--geo-bits N]"

      << " [--sdp DIR] [--record-dir DIR] [--record-file NAME] [--threads N] "
         "[--synthetic]"
      << " [--profile-interval SEC] [--no-profile]\n"

      << "\n"

      << "  Live preview (UDP 8891) is decoupled from V-PCC encode (UDP "
         "8890).\n"

      << "  Orbbec capture uses device default stream profiles (like "
         "05_point_cloud.py) at full resolution.\n"
      << "  Point cloud: AlignFilter + PointCloudFilter on synchronized "
         "framesets.\n"
      << "  Preview subsamples via --preview-points; V-PCC subsamples via "
         "--encode-points.\n"
      << "  Capture subsamples to max(preview, encode) unless --capture-points is set.\n"

      << "  V-PCC defaults: voxel 10, fast preset, encode rate = --encode-fps "
         "(10).\n"

      << "  --record-dir saves a V3C sample stream (.v3c) and SDP files while "
         "streaming.\n"

      << "  Press Q in this console for graceful shutdown (Ctrl+C also "
         "works).\n"
      << "  Press Q or Ctrl+C twice to force immediate exit.\n"

      << "  Use --geo-bits 9 for faster encode (~8 fps); --encode-points "
         "limits V-PCC input only (capture stays full resolution).\n"
      << "  Profiling: [profile] lines every --profile-interval seconds "
         "(capture/preview/encode/rtp).\n";
}

} // namespace

PointCloudFrame subsamplePointCloud(PointCloudFrame frame, size_t maxPoints) {
  if (maxPoints == 0 || frame.geometry.size() <= maxPoints) {
    return frame;
  }

  const size_t totalPoints = frame.geometry.size();
  const size_t stride = (totalPoints + maxPoints - 1) / maxPoints;
  PointCloudFrame out;
  out.frameNumber = frame.frameNumber;
  out.origin[0] = frame.origin[0];
  out.origin[1] = frame.origin[1];
  out.origin[2] = frame.origin[2];
  out.quantScale = frame.quantScale;
  out.geometry.reserve(maxPoints);
  out.attributes.reserve(maxPoints);

  for (size_t i = 0; i < totalPoints; i += stride) {
    out.geometry.push_back(frame.geometry[i]);
    out.attributes.push_back(frame.attributes[i]);
  }
  return out;
}

int main(int argc, char *argv[]) {

  std::set_terminate(terminateHandler);

  std::signal(SIGINT, handleSignal);

  std::signal(SIGTERM, handleSignal);

#ifdef _WIN32

  SetUnhandledExceptionFilter(unhandledExceptionFilter);

  if (!SetConsoleCtrlHandler(consoleCtrlHandler, TRUE)) {

    std::cerr << "Warning: could not install console Ctrl handler\n";
  }

#endif

  EncoderStreamerConfig streamConfig;

  OrbbecCaptureConfig captureConfig;

  PointCloudPreviewConfig previewConfig;

  bool forceSynthetic = false;

  int encodeFps = 10;
  size_t encodeMaxPoints = 80000;
  size_t captureMaxPoints = 0;
  bool captureMaxPointsExplicit = false;
  bool profileEnabled = true;
  int profileIntervalSec = 5;

#ifdef _WIN32

  WSADATA wsaData{};

  if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {

    std::cerr << "WSAStartup failed\n";

    return EXIT_FAILURE;
  }

#endif

  for (int i = 1; i < argc; ++i) {

    const std::string arg = argv[i];

    auto nextValue = [&](std::string &out) {
      if (i + 1 >= argc) {

        printUsage(argv[0]);

        std::exit(EXIT_FAILURE);
      }

      out = argv[++i];
    };

    if (arg == "--address") {

      nextValue(streamConfig.dstAddress);

      previewConfig.dstAddress = streamConfig.dstAddress;

    } else if (arg == "--port") {

      streamConfig.dstPort = static_cast<uint16_t>(std::stoi(argv[++i]));

    } else if (arg == "--preview-port") {

      previewConfig.dstPort = static_cast<uint16_t>(std::stoi(argv[++i]));

    } else if (arg == "--preview-points") {

      previewConfig.maxPointsPerFrame =
          static_cast<size_t>(std::stoul(argv[++i]));

    } else if (arg == "--color-width") {

      captureConfig.colorWidth = std::stoi(argv[++i]);
      captureConfig.useDeviceMaxResolution = false;
      captureConfig.useDefaultStreamProfiles = false;

    } else if (arg == "--color-height") {

      captureConfig.colorHeight = std::stoi(argv[++i]);
      captureConfig.useDeviceMaxResolution = false;
      captureConfig.useDefaultStreamProfiles = false;

    } else if (arg == "--fps") {

      captureConfig.colorFps = std::stoi(argv[++i]);

      captureConfig.depthFps = captureConfig.colorFps;

    } else if (arg == "--encode-fps") {

      encodeFps = std::stoi(argv[++i]);

    } else if (arg == "--encode-points") {

      encodeMaxPoints = static_cast<size_t>(std::stoul(argv[++i]));

    } else if (arg == "--capture-points") {

      captureMaxPoints = static_cast<size_t>(std::stoul(argv[++i]));
      captureMaxPointsExplicit = true;

    } else if (arg == "--geo-bits") {

      const auto bits = static_cast<size_t>(std::stoul(argv[++i]));

      captureConfig.geoBitDepthInput = bits;

      streamConfig.geoBitDepthInput = bits;

    } else if (arg == "--sdp") {

      nextValue(streamConfig.sdpOutputDir);

    } else if (arg == "--record-dir") {

      nextValue(streamConfig.recordDir);

    } else if (arg == "--record-file") {

      nextValue(streamConfig.recordFileName);

    } else if (arg == "--threads") {

      streamConfig.threads = std::stoi(argv[++i]);

    } else if (arg == "--synthetic") {

      forceSynthetic = true;

    } else if (arg == "--profile-interval") {

      profileIntervalSec = std::stoi(argv[++i]);

    } else if (arg == "--no-profile") {

      profileEnabled = false;

    } else if (arg == "--help" || arg == "-h") {

      printUsage(argv[0]);

      return EXIT_SUCCESS;

    } else {

      std::cerr << "Unknown argument: " << arg << "\n";

      printUsage(argv[0]);

      return EXIT_FAILURE;
    }
  }

  streamConfig.geoBitDepthInput = captureConfig.geoBitDepthInput;

  if (!captureMaxPointsExplicit) {
    const size_t streamingCap =
        (std::max)(encodeMaxPoints, previewConfig.maxPointsPerFrame);
    captureConfig.maxPointsPerFrame = streamingCap;
  } else {
    captureConfig.maxPointsPerFrame = captureMaxPoints;
  }

  PipelineProfiler profiler;
  profiler.setEnabled(profileEnabled);
  profiler.setIntervalSec(profileIntervalSec);
  captureConfig.profiler = profileEnabled ? &profiler : nullptr;
  previewConfig.profiler = profileEnabled ? &profiler : nullptr;
  streamConfig.profiler = profileEnabled ? &profiler : nullptr;

  const int previewIntervalMs =

      (captureConfig.colorFps > 0)
          ? (std::max)(1, 1000 / captureConfig.colorFps)
          : 33;

  const int encodeIntervalMs =

      (encodeFps > 0) ? (std::max)(1, 1000 / encodeFps) : 0;

  OrbbecCapture *capturePtr = nullptr;

  EncoderStreamer *streamerPtr = nullptr;

  std::atomic<bool> pipelineStopped{false};

  auto stopPipeline = [&]() {
    if (pipelineStopped.exchange(true)) {

      return;
    }

    std::cout << "Stopping pipeline...\n" << std::flush;

    if (capturePtr) {

      capturePtr->stop();
    }

    if (streamerPtr) {

      streamerPtr->flushAndStop();
    }

    std::cout << "Shutdown complete.\n" << std::flush;
  };

  try {

    std::cout << "vpcclive_streamer (live preview + V-PCC, build 2026-06-13d)\n"
              << std::flush;

    EncoderStreamer streamer(streamConfig);

    streamerPtr = &streamer;

    PointCloudPreviewSender previewSender(previewConfig);

    std::cout << "Initializing V-PCC encoder (voxel="
              << streamConfig.geoBitDepthInput

              << ", preset=fast, rate=32-42-4, doubleLayer=false)...\n"

              << std::flush;

    streamer.initialize();

    streamer.startRtpSender();

    if (forceSynthetic) {

      captureConfig.enableSyntheticFallback = true;
    }

    auto lastPreviewTime = std::chrono::steady_clock::time_point{};

    captureConfig.onFrameCaptured = [&](const PointCloudFrame &frame) {
      const auto now = std::chrono::steady_clock::now();

      if (now - lastPreviewTime <
          std::chrono::milliseconds(previewIntervalMs)) {

        return;
      }

      lastPreviewTime = now;

      previewSender.send(frame);
    };

    OrbbecCapture capture(captureConfig);

    capturePtr = &capture;

#if defined(ORBBEC_SDK_FOUND) && !forceSynthetic
    capture.prepareDevice();
#endif

#ifdef _WIN32

    std::jthread keyboardThread(keyboardListenerLoop);

#endif

    std::cout << "Starting capture thread"

              << (forceSynthetic ? " (synthetic)" : "") << "...\n"

              << std::flush;

    capture.start(forceSynthetic);

    std::cout << "Capture thread started.\n" << std::flush;

    std::cout << "Streaming V3C/RTP to " << streamConfig.dstAddress << ":"
              << streamConfig.dstPort << "\n"

              << std::flush;

    std::cout << "Preview UDP -> " << previewConfig.dstAddress << ":"
              << previewConfig.dstPort << " ("

              << previewConfig.maxPointsPerFrame << " pts, "
              << captureConfig.colorFps << " fps target)\n"

              << std::flush;

    std::cout << "Capture: up to " << (captureConfig.maxPointsPerFrame > 0
                                            ? std::to_string(captureConfig.maxPointsPerFrame)
                                            : std::string("all valid"))
              << " pts/frame\n"
              << std::flush;

    std::cout << "V-PCC encode: up to " << encodeMaxPoints << " pts/frame"

              << (encodeFps > 0
                      ? ", " + std::to_string(encodeFps) + " fps limit"
                      : ", no fps limit")

              << "\n"

              << std::flush;

    if (!streamConfig.recordDir.empty()) {

      std::cout << "Recording V3C stream to " << streamConfig.recordDir << "/"

                << streamConfig.recordFileName
                << " (+ SDP in same dir unless --sdp is set)\n"

                << std::flush;
    }

    std::atomic<bool> encodeFailed{false};

    std::string encodeError;

#ifndef _WIN32
    std::mutex encodeMutex;

    std::condition_variable encodeCv;

    std::deque<PointCloudFrame> encodeQueue;

    std::jthread encodeThread([&](std::stop_token stopToken) {
      try {

        while (!stopToken.stop_requested() && g_running.load()) {

          PointCloudFrame frame;

          {

            std::unique_lock lock(encodeMutex);

            encodeCv.wait(lock, [&] {
              return !encodeQueue.empty() || stopToken.stop_requested() ||
                     !g_running.load();
            });

            if (stopToken.stop_requested() || !g_running.load()) {

              break;
            }

            if (encodeQueue.empty()) {

              continue;
            }

            frame = std::move(encodeQueue.front());

            encodeQueue.pop_front();
          }

          std::cout << "Encoding frame " << frame.frameNumber << " ("
                    << frame.geometry.size() << " points)\n"

                    << std::flush;

          streamer.encodeFrame(std::move(frame));

          std::cout << "Frame submitted to encoder\n" << std::flush;
        }

      } catch (const std::exception &error) {

        encodeError = error.what();

        encodeFailed = true;

        g_running = false;
      }
    });
#endif

    std::cout << "Entering capture loop (preview real-time, V-PCC encode "
#ifdef _WIN32
              << "on main thread"
#else
              << "async"
#endif
              << ")\n"

              << "Press Q to quit gracefully.\n"

              << std::flush;

    auto lastEncodeSubmitTime = std::chrono::steady_clock::time_point{};

    auto lastHeartbeatTime = std::chrono::steady_clock::now();

    uint64_t encodeSubmitCount = 0;

    uint64_t encodeSkipCount = 0;

    while (g_running.load()) {

      profiler.tick();

      if (capture.captureFailed()) {

        const auto error = capture.captureError();

        throw std::runtime_error(error ? *error : "Orbbec capture failed");
      }

      if (streamer.rtpFailed()) {

        throw std::runtime_error(
            streamer.rtpError().empty() ? "V3C/RTP sender thread failed" : streamer.rtpError());
      }

      if (encodeFailed) {

        throw std::runtime_error(encodeError.empty() ? "Encoder thread failed"
                                                     : encodeError);
      }

      const auto now = std::chrono::steady_clock::now();

      if (now - lastHeartbeatTime >= std::chrono::seconds(15)) {

        std::cerr << "[heartbeat] running | encode submitted=" << encodeSubmitCount
                  << " skipped=" << encodeSkipCount << " | capture "
                  << (capture.isRunning() ? "ok" : "stopped") << "\n"
                  << std::flush;

        lastHeartbeatTime = now;
      }

      auto cloudFrame = capture.waitPopFrame(std::chrono::milliseconds(50));

      if (!cloudFrame) {

        if (!capture.isRunning() && !capture.captureFailed()) {

          std::cerr << "Capture thread stopped.\n";

          break;
        }

        continue;
      }

      if (encodeIntervalMs > 0) {

        if (now - lastEncodeSubmitTime <
            std::chrono::milliseconds(encodeIntervalMs)) {

          continue;
        }

        lastEncodeSubmitTime = now;
      }

#ifdef _WIN32
      if (!g_running.load()) {
        break;
      }

      std::cout << "Encoding frame " << cloudFrame->frameNumber << " ("
                << cloudFrame->geometry.size() << " capture pts -> "
                << (encodeMaxPoints > 0
                        ? std::to_string((std::min)(cloudFrame->geometry.size(), encodeMaxPoints))
                        : std::string("all"))
                << " encode pts)\n"
                << std::flush;

      bool submitted = false;

      try {
        submitted = streamer.tryEncodeFrame(
            subsamplePointCloud(std::move(*cloudFrame),
                                encodeMaxPoints > 0 ? encodeMaxPoints : SIZE_MAX));
      } catch (const std::exception &error) {
        throw std::runtime_error(std::string("V-PCC encode failed: ") + error.what());
      }

      if (!submitted) {
        ++encodeSkipCount;
        continue;
      }

      ++encodeSubmitCount;

      std::cout << "Frame submitted to encoder\n" << std::flush;
#else
      {
        std::lock_guard lock(encodeMutex);
        encodeQueue.clear();
        encodeQueue.push_back(std::move(*cloudFrame));
      }
      encodeCv.notify_one();
#endif
    }

    std::cerr << "[main] Loop exited (g_running=" << std::boolalpha << g_running.load()
              << ")\n"
              << std::flush;

#ifndef _WIN32
    encodeThread.request_stop();

    encodeCv.notify_all();

    encodeThread.join();
#endif

    stopPipeline();

    profiler.reportFinal();

  } catch (const std::exception &error) {

    std::cerr << "Fatal error: " << error.what() << "\n";

    stopPipeline();

    return EXIT_FAILURE;
  }

#ifdef _WIN32

  WSACleanup();

#endif

  return EXIT_SUCCESS;
}
