#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <uvgrtp/util.hh>
#include <uvgv3crtp/v3c_api.h>
#include <uvgv3crtp/version.h>

#include "OrbbecCapture.hpp"
#include "PipelineProfiler.hpp"
#include "V3cStreamRecorder.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

struct EncoderStreamerConfig {
    std::string dstAddress = "127.0.0.1";
    uint16_t dstPort = 8890;
    // Voxel 9 + fast preset targets ~30+ fps on i7-class CPUs (uvgVPCCenc 1.1 benchmarks).
    size_t geoBitDepthInput = 10;
    /// Minimum valid sizeGOF in uvgVPCCenc 1.2 is 8 (16 is the fast-preset default).
    size_t sizeGof = 8;
    /// uvgVPCCenc worker threads (patch/map/2D encode). Use --threads; 0 = auto-detect.
    int threads = 0;
    std::string sdpOutputDir;
    /// When set, append each encoded V3C chunk to this directory (V3C sample stream + SDP).
    std::string recordDir;
    std::string recordFileName = "v3c_live.v3c";
    std::string livePreset =
        "presetName=fast,rate=32-42-4,doubleLayer=false,geoBitDepthVoxelized=8,"
        "geoBitDepthRefineSegmentation=7,mode=AI";
    PipelineProfiler* profiler = nullptr;
};

/// Bridges uvgVPCCenc output chunks to uvgV3CRTP V3C RTP packetization (SSRC = vuh_unit_type + 1).
class EncoderStreamer {
public:
    explicit EncoderStreamer(EncoderStreamerConfig config = {}) : config_(std::move(config)) {}

    ~EncoderStreamer() { shutdown(); }

    EncoderStreamer(const EncoderStreamer&) = delete;
    EncoderStreamer& operator=(const EncoderStreamer&) = delete;

    void initialize() {
        if (initialized_) {
            return;
        }

        applyParameters(config_.livePreset);
        // Only set parameters not already covered by livePreset.
        uvgvpcc_enc::API::setParameter("geoBitDepthInput", std::to_string(config_.geoBitDepthInput));
        int encoderThreads = config_.threads;
        if (encoderThreads <= 0) {
            const unsigned hw = std::thread::hardware_concurrency();
            encoderThreads = hw > 0 ? static_cast<int>((std::min)(hw, 8U)) : 4;
        }
#ifdef _WIN32
        // High internal thread counts contend with Orbbec USB capture on the same CPU.
        encoderThreads = (std::min)(encoderThreads, 4);
#endif
        uvgvpcc_enc::API::setParameter("nbThreadPCPart", std::to_string(encoderThreads));
        uvgvpcc_enc::API::setParameter("occupancyEncodingNbThread", std::to_string(encoderThreads));
        uvgvpcc_enc::API::setParameter("geometryEncodingNbThread", std::to_string(encoderThreads));
        uvgvpcc_enc::API::setParameter("attributeEncodingNbThread", std::to_string(encoderThreads));
        uvgvpcc_enc::API::setParameter("logLevel", "INFO");
        uvgvpcc_enc::API::setParameter("errorsAreFatal", "False");
        uvgvpcc_enc::API::setParameter("sizeGOF", std::to_string(config_.sizeGof));
        uvgvpcc_enc::API::setParameter("sizeGOP2DEncoding", std::to_string(config_.sizeGof));
#ifdef _WIN32
        // Keep a small in-flight window so the main loop never blocks on encodeFrame().
        uvgvpcc_enc::API::setParameter("maxConcurrentFrames", "8");
#else
        // 0 lets uvgVPCCenc set max(4 * sizeGOF, 2 * threads).
#endif

        uvgvpcc_enc::API::initializeEncoder();
        initialized_ = true;
    }

    void startRtpSender() {
        if (rtpThread_.joinable()) {
            return;
        }

        if (!config_.recordDir.empty()) {
            const std::filesystem::path streamPath =
                std::filesystem::path(config_.recordDir) / config_.recordFileName;
            v3cRecorder_.open(streamPath);
            if (config_.sdpOutputDir.empty()) {
                sdpWriteDir_ = config_.recordDir;
            }
        }

        if (!v3cSender_) {
            uint16_t ports[uvgV3CRTP::NUM_V3C_UNIT_TYPES] = {};
            std::fill(std::begin(ports), std::end(ports), config_.dstPort);

            std::cout << "Creating V3C/RTP sender to " << config_.dstAddress << ":" << config_.dstPort << "\n"
                      << std::flush;
            v3cSender_ = std::make_unique<uvgV3CRTP::V3C_State<uvgV3CRTP::V3C_Sender>>(
                uvgV3CRTP::INIT_FLAGS::VPS | uvgV3CRTP::INIT_FLAGS::AD | uvgV3CRTP::INIT_FLAGS::OVD |
                    uvgV3CRTP::INIT_FLAGS::GVD | uvgV3CRTP::INIT_FLAGS::AVD,
                config_.dstAddress.c_str(),
                ports);
            if (v3cSender_->get_error_flag() != uvgV3CRTP::ERROR_TYPE::OK) {
                v3cSender_.reset();
                throw std::runtime_error("V3C sender connection failed");
            }
            std::cout << "V3C/RTP sender ready\n" << std::flush;
        }

        rtpThread_ = std::jthread([this](std::stop_token stopToken) {
            try {
                rtpSenderLoop(stopToken);
            } catch (const std::exception& error) {
                {
                    std::lock_guard lock(rtpErrorMutex_);
                    rtpError_ = error.what();
                }
                rtpFailed_.store(true);
                std::cerr << "RTP sender thread error: " << error.what() << "\n" << std::flush;
            }
        });
    }

    void encodeFrame(PointCloudFrame cloudFrame) {
        if (!tryEncodeFrame(std::move(cloudFrame))) {
            throw std::runtime_error("EncoderStreamer::encodeFrame blocked (encoder pipeline full)");
        }
    }

    /// Returns false when the encoder pipeline is full; the frame is not submitted.
    bool tryEncodeFrame(PointCloudFrame cloudFrame) {
        if (!initialized_) {
            throw std::runtime_error("EncoderStreamer::tryEncodeFrame called before initialize()");
        }

        auto frame = std::make_shared<uvgformat::uvgFrame>();
        frame->frameNumber = cloudFrame.frameNumber;
        frame->sourcePath = "live_orbbec_stream";

        uvgformat::GeometryRgb payload;
        payload.geometry = std::move(cloudFrame.geometry);
        payload.attribute = std::move(cloudFrame.attributes);
        frame->payload = std::move(payload);

        const size_t pointCount = std::get<uvgformat::GeometryRgb>(frame->payload).geometry.size();
        if (config_.profiler) {
            config_.profiler->noteEncodeSubmit(pointCount);
        }

        return uvgvpcc_enc::API::tryEncodeFrame(frame, &outputStream_);
    }

    bool rtpFailed() const { return rtpFailed_.load(); }

    std::string rtpError() const {
        std::lock_guard lock(rtpErrorMutex_);
        return rtpError_;
    }

    void flushAndStop() {
        if (!initialized_ || shutdownComplete_) {
            return;
        }

        // Wake RTP thread; stopEncoder runs once in shutdown() after the thread joins.
        {
            std::lock_guard lock(outputStream_.io_mutex);
            outputStream_.v3c_unit_batches.emplace();
        }
        outputStream_.available_chunks.release();

        shutdown();
    }

private:
    static void applyParameters(const std::string& parameterString) {
        static const std::regex parameterRegex(R"(([^=,]+)=([^,]+))");
        std::sregex_iterator begin(parameterString.begin(), parameterString.end(), parameterRegex);
        std::sregex_iterator end;
        for (auto it = begin; it != end; ++it) {
            uvgvpcc_enc::API::setParameter((*it)[1].str(), (*it)[2].str());
        }
    }

    void shutdown() {
        if (shutdownComplete_) {
            return;
        }
        shutdownComplete_ = true;

        if (rtpThread_.joinable()) {
            rtpThread_.request_stop();
            // Release rtp thread if it is blocked waiting for encoder batches.
            {
                std::lock_guard lock(outputStream_.io_mutex);
                if (outputStream_.v3c_unit_batches.empty()) {
                    outputStream_.v3c_unit_batches.emplace();
                }
            }
            outputStream_.available_chunks.release();
            rtpThread_.join();
        }
#ifdef _WIN32
        // uvgrtp/V3C teardown can crash on MSVC during process exit; leak the sender.
        if (v3cSender_) {
            v3cSender_.release();
        }
#else
        v3cSender_.reset();
#endif
        if (initialized_) {
            uvgvpcc_enc::API::stopEncoder();
            initialized_ = false;
        }
        v3cRecorder_.close();
    }

    std::string sdpOutputDirectory() const {
        return !config_.sdpOutputDir.empty() ? config_.sdpOutputDir : sdpWriteDir_;
    }

    void rtpSenderLoop(std::stop_token stopToken) {
        static constexpr uint8_t kForcedV3cSizePrecision = 5;

        auto& state = *v3cSender_;
        uint16_t ports[uvgV3CRTP::NUM_V3C_UNIT_TYPES] = {};
        std::fill(std::begin(ports), std::end(ports), config_.dstPort);

        bool reinit = true;
        bool writeSdp = !sdpOutputDirectory().empty();

        while (!stopToken.stop_requested()) {
            if (reinit) {
                if (state.init_sample_stream(kForcedV3cSizePrecision) != uvgV3CRTP::ERROR_TYPE::OK) {
                    throw std::runtime_error(std::string("V3C sender init failed: ") + state.get_error_msg());
                }
                reinit = false;
            }

            outputStream_.available_chunks.acquire();
            if (stopToken.stop_requested()) {
                break;
            }

            uint64_t gofBytes = 0;
            {
                std::lock_guard lock(outputStream_.io_mutex);
                const uvgvpcc_enc::API::v3c_unit_batch& batch = outputStream_.v3c_unit_batches.front();
                if (batch.v3c_units.empty()) {
                    break;
                }

                if (v3cRecorder_.isOpen()) {
                    v3cRecorder_.appendBatch(batch);
                }

                for (const auto& unit : batch.v3c_units) {
                    state.append_to_sample_stream(unit.data.get(), unit.len);
                    gofBytes += unit.len;
                }
                outputStream_.v3c_unit_batches.pop();
            }

            if (writeSdp) {
                writeSdpFiles(state, ports);
                writeSdp = false;
            }

            while (state.get_error_flag() == uvgV3CRTP::ERROR_TYPE::OK) {
                if (state.cur_gof_is_full()) {
                    if (uvgV3CRTP::send_gof(&state) != uvgV3CRTP::ERROR_TYPE::OK) {
                        throw std::runtime_error(std::string("V3C send_gof failed: ") + state.get_error_msg());
                    }
                    static std::atomic<bool> loggedFirstGof{false};
                    if (!loggedFirstGof.exchange(true)) {
                        std::cout << "First GOF sent via RTP to " << config_.dstAddress << ":" << config_.dstPort << "\n"
                                  << std::flush;
                    }
                    state.next_gof();
                    if (config_.profiler && gofBytes > 0) {
                        config_.profiler->noteRtpGof(gofBytes);
                        gofBytes = 0;
                    } else if (config_.profiler) {
                        config_.profiler->noteRtpGof(0);
                    }
                } else {
                    break;
                }
            }

            if (state.num_gofs() >= uvgV3CRTP::RECEIVE_BUFFER_SIZE) {
                state.clear_sample_stream();
                reinit = true;
                writeSdp = !sdpOutputDirectory().empty();
            }

            if (state.get_error_flag() == uvgV3CRTP::ERROR_TYPE::EOS) {
                state.reset_error_flag();
            }
        }
    }

    void writeSdpFiles(uvgV3CRTP::V3C_State<uvgV3CRTP::V3C_Sender>& state, const uint16_t ports[uvgV3CRTP::NUM_V3C_UNIT_TYPES]) {
        const std::string& sdpDir = sdpOutputDirectory();
        if (sdpDir.empty()) {
            return;
        }

        if (!std::filesystem::exists(sdpDir)) {
            std::filesystem::create_directories(sdpDir);
        }

        auto vps = std::unique_ptr<char, decltype(&free)>(
            state.get_cur_gof_unit_info_string(
                uvgV3CRTP::V3C_VPS,
                uvgV3CRTP::INFO_FMT::NONE,
                uvgV3CRTP::INFO_FMT::NONE,
                uvgV3CRTP::INFO_FMT::NONE,
                uvgV3CRTP::INFO_FMT::BASE64),
            &free);

        static const std::array<std::tuple<uvgV3CRTP::V3C_UNIT_TYPE, std::string, std::string, RTP_FORMAT, std::string>, 4>
            v3cToSdp{{{uvgV3CRTP::V3C_AD, "AD", "application", RTP_FORMAT_ATLAS, "v3c"},
                      {uvgV3CRTP::V3C_OVD, "OVD", "video", RTP_FORMAT_H265, "H265"},
                      {uvgV3CRTP::V3C_GVD, "GVD", "video", RTP_FORMAT_H265, "H265"},
                      {uvgV3CRTP::V3C_AVD, "AVD", "video", RTP_FORMAT_H265, "H265"}}};

        for (const auto& [type, typeStr, mediaName, format, codec] : v3cToSdp) {
            if (!state.cur_gof_has_unit(type)) {
                continue;
            }

            size_t headerLen = 0;
            auto unitHeader = std::unique_ptr<char, decltype(&free)>(
                state.get_cur_gof_unit_info_string(
                    type,
                    uvgV3CRTP::INFO_FMT::NONE,
                    uvgV3CRTP::INFO_FMT::BASE64,
                    uvgV3CRTP::INFO_FMT::NONE,
                    uvgV3CRTP::INFO_FMT::NONE,
                    &headerLen),
                &free);

            const std::string sdpFile = sdpDir + "/V3C_" + typeStr + ".sdp";
            const std::string sdp =
                "m=" + mediaName + " " + std::to_string(ports[type]) + " RTP/AVP " + std::to_string(format) + "\n" +
                "a=rtpmap:" + std::to_string(format) + " " + codec + "/" + std::to_string(uvgV3CRTP::RTP_CLOCK_RATE) + "\n" +
                "a=v3cfmtp:sprop-v3c-unit-header=" + std::string(unitHeader.get(), headerLen - 2) +
                ";\n  sprop-v3c-parameter-set=" + vps.get();

            std::ofstream out(sdpFile);
            if (!out.is_open()) {
                throw std::runtime_error("Failed to write SDP file: " + sdpFile);
            }
            out << sdp;
            std::cout << "[record] Wrote SDP " << sdpFile << "\n" << std::flush;
        }
    }

    EncoderStreamerConfig config_;
    uvgvpcc_enc::API::v3c_unit_stream outputStream_;
    std::unique_ptr<uvgV3CRTP::V3C_State<uvgV3CRTP::V3C_Sender>> v3cSender_;
    std::jthread rtpThread_;
    V3cStreamRecorder v3cRecorder_;
    std::string sdpWriteDir_;
    bool initialized_ = false;
    bool shutdownComplete_ = false;
    std::atomic<bool> rtpFailed_{false};
    mutable std::mutex rtpErrorMutex_;
    std::string rtpError_;
};
