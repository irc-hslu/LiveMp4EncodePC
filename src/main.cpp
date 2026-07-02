#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "EncoderStreamer.hpp"
#include "OrbbecCapture.hpp"

namespace {

std::atomic<bool> g_running{true};

void handleSignal(int) { g_running = false; }

void printUsage(const char* executable) {
    std::cerr
        << "Usage: " << executable
        << " [--address HOST] [--port PORT] [--fps FPS] [--geo-bits N] [--sdp DIR] [--threads N]\n"
        << "\n"
        << "  Real-time Orbbec -> uvgVPCCenc -> V3C/RTP live streamer.\n"
        << "  Without Orbbec SDK, a synthetic point cloud source is used.\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    EncoderStreamerConfig streamConfig;
    OrbbecCaptureConfig captureConfig;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto nextValue = [&](std::string& out) {
            if (i + 1 >= argc) {
                printUsage(argv[0]);
                std::exit(EXIT_FAILURE);
            }
            out = argv[++i];
        };

        if (arg == "--address") {
            nextValue(streamConfig.dstAddress);
        } else if (arg == "--port") {
            streamConfig.dstPort = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--fps") {
            captureConfig.colorFps = std::stoi(argv[++i]);
            captureConfig.depthFps = captureConfig.colorFps;
        } else if (arg == "--geo-bits") {
            const auto bits = static_cast<size_t>(std::stoul(argv[++i]));
            captureConfig.geoBitDepthInput = bits;
            streamConfig.geoBitDepthInput = bits;
        } else if (arg == "--sdp") {
            nextValue(streamConfig.sdpOutputDir);
        } else if (arg == "--threads") {
            streamConfig.threads = std::stoi(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return EXIT_SUCCESS;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            printUsage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    try {
        OrbbecCapture capture(captureConfig);
        EncoderStreamer streamer(streamConfig);

        std::cout << "Initializing V-PCC encoder (preset=fast, rate=32-42-4, doubleLayer=false)...\n";
        streamer.initialize();

        std::cout << "Starting capture thread...\n";
        capture.start();

        std::cout << "Streaming V3C/RTP to " << streamConfig.dstAddress << ":" << streamConfig.dstPort << "\n";
        if (!streamConfig.sdpOutputDir.empty()) {
            std::cout << "Writing SDP descriptors to " << streamConfig.sdpOutputDir << "\n";
        }

        while (g_running) {
            std::cout << "Waiting for frame...\n";
            auto cloudFrame = capture.waitPopFrame(std::chrono::milliseconds(250));
            if (!cloudFrame) {
                std::cout << "No frame received\n";

                continue;
            }

            streamer.encodeFrame(std::move(*cloudFrame));
            std::cout << "Frame encoded\n";
        }

        std::cout << "Stopping pipeline...\n";
        capture.stop();
        streamer.flushAndStop();
    } catch (const std::exception& error) {
        std::cerr << "Fatal error: " << error.what() << "\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
