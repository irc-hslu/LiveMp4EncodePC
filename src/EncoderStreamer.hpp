#pragma once

#include <algorithm>
#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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
#include "uvgvpcc/uvgvpcc.hpp"

struct EncoderStreamerConfig {
    std::string dstAddress = "127.0.0.1";
    uint16_t dstPort = 8890;
    size_t geoBitDepthInput = 10;
    size_t sizeGof = 8;
    int threads = 4;
    std::string sdpOutputDir;
    std::string livePreset =
        "presetName=fast,rate=32-42-4,doubleLayer=false,geoBitDepthVoxelized=8,mode=AI";
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
        uvgvpcc_enc::API::setParameter("geoBitDepthInput", std::to_string(config_.geoBitDepthInput));
        uvgvpcc_enc::API::setParameter("sizeGOF", std::to_string(config_.sizeGof));
        uvgvpcc_enc::API::setParameter("nbThreadPCPart", std::to_string(config_.threads));
        uvgvpcc_enc::API::setParameter("occupancyEncodingNbThread", std::to_string(config_.threads));
        uvgvpcc_enc::API::setParameter("geometryEncodingNbThread", std::to_string(config_.threads));
        uvgvpcc_enc::API::setParameter("attributeEncodingNbThread", std::to_string(config_.threads));
        uvgvpcc_enc::API::setParameter("logLevel", "INFO");

        uvgvpcc_enc::API::initializeEncoder();
        initialized_ = true;

        rtpThread_ = std::jthread([this](std::stop_token stopToken) { rtpSenderLoop(stopToken); });
    }

    void encodeFrame(PointCloudFrame cloudFrame) {
        if (!initialized_) {
            throw std::runtime_error("EncoderStreamer::encodeFrame called before initialize()");
        }

        auto frame = std::make_shared<uvgvpcc_enc::Frame>(
            encodedFrames_++,
            cloudFrame.frameNumber,
            "live_orbbec_stream");

        frame->pointsGeometry = std::move(cloudFrame.geometry);
        frame->pointsAttribute = std::move(cloudFrame.attributes);
        frame->pointCount = frame->pointsGeometry.size();

        uvgvpcc_enc::API::encodeFrame(frame, &outputStream_);
    }

    void flushAndStop() {
        if (!initialized_) {
            return;
        }

        uvgvpcc_enc::API::emptyFrameQueue();

        {
            std::lock_guard lock(outputStream_.io_mutex);
            outputStream_.v3c_chunks.emplace();
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
        if (rtpThread_.joinable()) {
            rtpThread_.request_stop();
            rtpThread_.join();
        }
        if (initialized_) {
            uvgvpcc_enc::API::stopEncoder();
            initialized_ = false;
        }
    }

    void rtpSenderLoop(std::stop_token stopToken) {
        static constexpr uint8_t kForcedV3cSizePrecision = 5;

        uint16_t ports[uvgV3CRTP::NUM_V3C_UNIT_TYPES] = {};
        std::fill(std::begin(ports), std::end(ports), config_.dstPort);

        uvgV3CRTP::V3C_State<uvgV3CRTP::V3C_Sender> state(
            uvgV3CRTP::INIT_FLAGS::VPS | uvgV3CRTP::INIT_FLAGS::AD | uvgV3CRTP::INIT_FLAGS::OVD |
                uvgV3CRTP::INIT_FLAGS::GVD | uvgV3CRTP::INIT_FLAGS::AVD,
            config_.dstAddress.c_str(),
            ports);

        bool reinit = true;
        bool writeSdp = !config_.sdpOutputDir.empty();

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

            std::lock_guard lock(outputStream_.io_mutex);
            const uvgvpcc_enc::API::v3c_chunk& chunk = outputStream_.v3c_chunks.front();
            if (chunk.data == nullptr && chunk.len == 0) {
                break;
            }

            std::ptrdiff_t offset = 0;
            for (const uint64_t unitSize : chunk.v3c_unit_sizes) {
                state.append_to_sample_stream(std::next(chunk.data.get(), offset), static_cast<size_t>(unitSize));
                offset += static_cast<std::ptrdiff_t>(unitSize);
            }
            outputStream_.v3c_chunks.pop();

            if (writeSdp) {
                writeSdpFiles(state, ports);
                writeSdp = false;
            }

            while (state.get_error_flag() == uvgV3CRTP::ERROR_TYPE::OK) {
                if (state.cur_gof_is_full()) {
                    if (uvgV3CRTP::send_gof(&state) != uvgV3CRTP::ERROR_TYPE::OK) {
                        throw std::runtime_error(std::string("V3C send_gof failed: ") + state.get_error_msg());
                    }
                    state.next_gof();
                } else {
                    break;
                }
            }

            if (state.num_gofs() >= uvgV3CRTP::RECEIVE_BUFFER_SIZE) {
                state.clear_sample_stream();
                reinit = true;
                writeSdp = !config_.sdpOutputDir.empty();
            }

            if (state.get_error_flag() == uvgV3CRTP::ERROR_TYPE::EOS) {
                state.reset_error_flag();
            }
        }
    }

    void writeSdpFiles(uvgV3CRTP::V3C_State<uvgV3CRTP::V3C_Sender>& state, const uint16_t ports[uvgV3CRTP::NUM_V3C_UNIT_TYPES]) {
        if (!std::filesystem::exists(config_.sdpOutputDir)) {
            std::filesystem::create_directories(config_.sdpOutputDir);
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

            const std::string sdpFile = config_.sdpOutputDir + "/V3C_" + typeStr + ".sdp";
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
        }
    }

    EncoderStreamerConfig config_;
    uvgvpcc_enc::API::v3c_unit_stream outputStream_;
    std::jthread rtpThread_;
    bool initialized_ = false;
    size_t encodedFrames_ = 0;
};
