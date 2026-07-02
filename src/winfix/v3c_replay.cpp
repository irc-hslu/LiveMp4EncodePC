#include <array>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <uvgv3crtp/v3c_api.h>
#include <uvgv3crtp/version.h>

namespace {

void printUsage(const char* executable) {
    std::cerr
        << "Usage: " << executable
        << " --file PATH.v3c [--address HOST] [--port PORT] [--fps FPS] [--loop]\n"
        << "\n"
        << "  Replays a V3C sample stream file (from vpcclive_streamer --record-dir) over RTP.\n"
        << "  Point the web relay at UDP PORT and open web/playback.html or index.html.\n";
}

std::vector<char> readBinaryFile(const std::string& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + path);
    }

    const auto length = file.tellg();
    if (length <= 0) {
        throw std::runtime_error("File is empty: " + path);
    }

    std::vector<char> buffer(static_cast<size_t>(length));
    file.seekg(0, std::ios::beg);
    if (!file.read(buffer.data(), length)) {
        throw std::runtime_error("Failed to read file: " + path);
    }
    return buffer;
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string filePath;
    std::string address = "127.0.0.1";
    uint16_t port = 8890;
    int fps = 4;
    bool loop = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto nextValue = [&](std::string& out) {
            if (i + 1 >= argc) {
                printUsage(argv[0]);
                std::exit(EXIT_FAILURE);
            }
            out = argv[++i];
        };

        if (arg == "--file") {
            nextValue(filePath);
        } else if (arg == "--address") {
            nextValue(address);
        } else if (arg == "--port") {
            port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--fps") {
            fps = std::stoi(argv[++i]);
        } else if (arg == "--loop") {
            loop = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return EXIT_SUCCESS;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            printUsage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (filePath.empty()) {
        printUsage(argv[0]);
        return EXIT_FAILURE;
    }

    if (fps <= 0) {
        fps = 1;
    }

    try {
        std::cout << "v3c_replay (uvgV3CRTP " << uvgV3CRTP::get_version() << ")\n";
        std::cout << "Loading " << filePath << "...\n" << std::flush;

        const auto buffer = readBinaryFile(filePath);
        std::cout << "Loaded " << buffer.size() << " bytes\n" << std::flush;

        uint16_t ports[uvgV3CRTP::NUM_V3C_UNIT_TYPES] = {};
        std::fill(std::begin(ports), std::end(ports), port);

        auto runOnce = [&]() -> size_t {
            uvgV3CRTP::V3C_State<uvgV3CRTP::V3C_Sender> state(
                buffer.data(),
                buffer.size(),
                uvgV3CRTP::INIT_FLAGS::VPS | uvgV3CRTP::INIT_FLAGS::AD | uvgV3CRTP::INIT_FLAGS::OVD |
                    uvgV3CRTP::INIT_FLAGS::GVD | uvgV3CRTP::INIT_FLAGS::AVD,
                address.c_str(),
                ports);

            if (state.get_error_flag() != uvgV3CRTP::ERROR_TYPE::OK) {
                throw std::runtime_error(std::string("V3C parse/send init failed: ") + state.get_error_msg());
            }

            std::cout << "Streaming to " << address << ":" << port << " @ " << fps << " GOF/s\n" << std::flush;
            state.print_bitstream_info();

            size_t gofCount = 0;
            const auto frameDelay = std::chrono::milliseconds(1000 / fps);

            while (state.get_error_flag() == uvgV3CRTP::ERROR_TYPE::OK) {
                if (uvgV3CRTP::send_gof(&state) != uvgV3CRTP::ERROR_TYPE::OK) {
                    break;
                }
                ++gofCount;
                std::cout << "  GOF " << gofCount << " sent\n" << std::flush;
                state.next_gof();
                std::this_thread::sleep_for(frameDelay);
            }

            if (state.get_error_flag() != uvgV3CRTP::ERROR_TYPE::EOS) {
                std::cerr << "Stopped: " << state.get_error_msg() << "\n";
            }

            return gofCount;
        };

        size_t totalGofs = 0;
        do {
            totalGofs += runOnce();
            if (loop) {
                std::cout << "Looping playback...\n" << std::flush;
            }
        } while (loop);

        std::cout << "Replay finished (" << totalGofs << " GOF(s) sent).\n";
    } catch (const std::exception& error) {
        std::cerr << "Fatal error: " << error.what() << "\n";
        return EXIT_FAILURE;
    }

#ifdef _WIN32
    // uvgrtp teardown can AV on MSVC during V3C_State destruction.
    std::quick_exit(EXIT_SUCCESS);
#else
    return EXIT_SUCCESS;
#endif
}
