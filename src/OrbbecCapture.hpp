#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "uvgutils/utils.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

using GeometryPoint = uvgutils::VectorN<uvgvpcc_enc::typeGeometryInput, 3>;
using AttributePoint = uvgutils::VectorN<uint8_t, 3>;

#if defined(ORBBEC_SDK_FOUND)
#include <algorithm>
#include <libobsensor/ObSensor.hpp>
#endif

/// Synchronized depth + color capture and voxel-quantized point cloud export for uvgVPCCenc.
struct PointCloudFrame {
    uint64_t frameNumber = 0;
    std::vector<GeometryPoint> geometry;
    std::vector<AttributePoint> attributes;
};

struct OrbbecCaptureConfig {
    int colorWidth = 1920;
    int colorHeight = 1080;
    int colorFps = 30;
    int depthWidth = 640;
    int depthHeight = 576;
    int depthFps = 30;
    size_t geoBitDepthInput = 10;
    size_t maxQueueDepth = 2;
    bool enableSyntheticFallback = true;
};

class OrbbecCapture {
public:
    explicit OrbbecCapture(OrbbecCaptureConfig config = {})
        : config_(std::move(config)) {}

    ~OrbbecCapture() { stop(); }

    OrbbecCapture(const OrbbecCapture&) = delete;
    OrbbecCapture& operator=(const OrbbecCapture&) = delete;

    void start() {
        if (running_.exchange(true)) {
            return;
        }

#if defined(ORBBEC_SDK_FOUND)
        captureThread_ = std::jthread([this](std::stop_token stopToken) { captureLoopOrbbec(stopToken); });
#else
        if (!config_.enableSyntheticFallback) {
            running_ = false;
            throw std::runtime_error(
                "Orbbec SDK not found at configure time. Set ORBBEC_SDK_DIR or enable synthetic fallback.");
        }
        captureThread_ = std::jthread([this](std::stop_token stopToken) { captureLoopSynthetic(stopToken); });
#endif
    }

    void stop() {
        if (!running_.exchange(false)) {
            return;
        }
        if (captureThread_.joinable()) {
            captureThread_.request_stop();
            captureThread_.join();
        }
        {
            std::lock_guard lock(queueMutex_);
            queue_.clear();
        }
        queueCv_.notify_all();
    }

    std::optional<PointCloudFrame> tryPopFrame() {
        std::lock_guard lock(queueMutex_);
        if (queue_.empty()) {
            return std::nullopt;
        }
        PointCloudFrame frame = std::move(queue_.front());
        queue_.pop_front();
        return frame;
    }

    std::optional<PointCloudFrame> waitPopFrame(std::chrono::milliseconds timeout) {
        std::unique_lock lock(queueMutex_);
        if (!queueCv_.wait_for(lock, timeout, [this] { return !queue_.empty() || !running_; })) {
            return std::nullopt;
        }
        if (queue_.empty()) {
            return std::nullopt;
        }
        PointCloudFrame frame = std::move(queue_.front());
        queue_.pop_front();
        return frame;
    }

    bool isRunning() const { return running_.load(); }

private:
    static bool coordinateFitsVoxelGrid(const GeometryPoint& point, size_t geoBitDepthInput) {
        const auto maxValue = static_cast<uvgvpcc_enc::typeGeometryInput>(
            (static_cast<uint32_t>(1) << geoBitDepthInput) - 1U);
        return point[0] <= maxValue && point[1] <= maxValue && point[2] <= maxValue;
    }

    void pushFrame(PointCloudFrame frame) {
        {
            std::lock_guard lock(queueMutex_);
            while (queue_.size() >= config_.maxQueueDepth) {
                queue_.pop_front();
            }
            queue_.push_back(std::move(frame));
        }
        queueCv_.notify_one();
    }

#if defined(ORBBEC_SDK_FOUND)
    void captureLoopOrbbec(std::stop_token stopToken) {
        ob::Pipeline pipeline;
        auto config = std::make_shared<ob::Config>();

        std::shared_ptr<ob::StreamProfile> colorProfile;
        const auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
        if (colorProfiles && colorProfiles->count() > 0) {
            colorProfile = colorProfiles->getVideoStreamProfile(
                config_.colorWidth, config_.colorHeight, OB_FORMAT_RGB, config_.colorFps);
            config->enableStream(colorProfile);
        }

        OBAlignMode alignMode = ALIGN_DISABLE;
        std::shared_ptr<ob::StreamProfileList> depthProfileList;
        if (colorProfile) {
            depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
            if (depthProfileList && depthProfileList->count() > 0) {
                alignMode = ALIGN_D2C_HW_MODE;
            } else {
                depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
                if (depthProfileList && depthProfileList->count() > 0) {
                    alignMode = ALIGN_D2C_SW_MODE;
                }
            }
            try {
                pipeline.enableFrameSync();
            } catch (const ob::Error&) {
            }
        } else {
            depthProfileList = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
        }

        if (!depthProfileList || depthProfileList->count() == 0) {
            throw std::runtime_error("OrbbecCapture: no depth stream profile available");
        }

        std::shared_ptr<ob::StreamProfile> depthProfile;
        try {
            if (colorProfile) {
                depthProfile = depthProfileList->getVideoStreamProfile(
                    OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, colorProfile->fps());
            }
        } catch (...) {
            depthProfile = nullptr;
        }
        if (!depthProfile) {
            depthProfile = depthProfileList->getProfile(OB_PROFILE_DEFAULT);
        }
        config->enableStream(depthProfile);
        config->setAlignMode(alignMode);
        cout << "alignMode: " << alignMode << endl;
        pipeline.start(config);

        auto pointCloudFilter = std::make_shared<ob::PointCloudFilter>();
        pointCloudFilter->setCreatePointFormat(OB_FORMAT_RGB_POINT);

        uint64_t frameNumber = 0;
        float minX = 0.F;
        float minY = 0.F;
        float minZ = 0.F;
        bool originInitialized = false;

        while (!stopToken.stop_requested() && running_) {
            auto frameSet = pipeline.waitForFrames(100);
            if (!frameSet || !frameSet->depthFrame()) {
                continue;
            }
            if (colorProfile && !frameSet->colorFrame()) {
                continue;
            }

            std::shared_ptr<ob::Frame> cloudFrame;
            try {
                cloudFrame = pointCloudFilter->process(frameSet);
            } catch (const ob::Error& error) {
                throw std::runtime_error(std::string("OrbbecCapture point cloud failed: ") + error.getMessage());
            }
            if (!cloudFrame) {
                continue;
            }

            const auto* points = reinterpret_cast<const OBColorPoint*>(cloudFrame->data());
            const uint32_t pointCount = cloudFrame->dataSize() / static_cast<uint32_t>(sizeof(OBColorPoint));
            if (pointCount == 0) {
                continue;
            }

            if (!originInitialized) {
                minX = points[0].x;
                minY = points[0].y;
                minZ = points[0].z;
                for (uint32_t i = 1; i < pointCount; ++i) {
                    minX = (std::min)(minX, points[i].x);
                    minY = (std::min)(minY, points[i].y);
                    minZ = (std::min)(minZ, points[i].z);
                }
                originInitialized = true;
            }

            const float scale = static_cast<float>(1 << config_.geoBitDepthInput);
            PointCloudFrame frame;
            frame.frameNumber = frameNumber++;
            frame.geometry.reserve(pointCount);
            frame.attributes.reserve(pointCount);

            for (uint32_t i = 0; i < pointCount; ++i) {
                if (points[i].z <= 0.F) {
                    continue;
                }

                const auto qx = static_cast<uvgvpcc_enc::typeGeometryInput>((points[i].x - minX) * scale);
                const auto qy = static_cast<uvgvpcc_enc::typeGeometryInput>((points[i].y - minY) * scale);
                const auto qz = static_cast<uvgvpcc_enc::typeGeometryInput>((points[i].z - minZ) * scale);
                const GeometryPoint geometry{qx, qy, qz};
                if (!coordinateFitsVoxelGrid(geometry, config_.geoBitDepthInput)) {
                    continue;
                }

                frame.geometry.push_back(geometry);
                frame.attributes.push_back(
                    AttributePoint{points[i].r, points[i].g, points[i].b});
            }

            if (frame.geometry.empty()) {
                continue;
            }
            pushFrame(std::move(frame));
        }

        pipeline.stop();
    }
#endif

    void captureLoopSynthetic(std::stop_token stopToken) {
        uint64_t frameNumber = 0;
        const float scale = static_cast<float>(1 << config_.geoBitDepthInput);

        while (!stopToken.stop_requested() && running_) {
            PointCloudFrame frame;
            frame.frameNumber = frameNumber++;
            frame.geometry.reserve(4096);
            frame.attributes.reserve(4096);

            for (int y = 0; y < 64; ++y) {
                for (int x = 0; x < 64; ++x) {
                    const float xf = static_cast<float>(x) / 64.F;
                    const float yf = static_cast<float>(y) / 64.F;
                    const float zf = 0.35F + 0.15F * std::sin((xf + yf + frameNumber * 0.05F) * 6.2831853F);

                    const auto qx = static_cast<uvgvpcc_enc::typeGeometryInput>(xf * scale * 0.5F);
                    const auto qy = static_cast<uvgvpcc_enc::typeGeometryInput>(yf * scale * 0.5F);
                    const auto qz = static_cast<uvgvpcc_enc::typeGeometryInput>(zf * scale * 0.5F);
                    const GeometryPoint geometry{qx, qy, qz};
                    if (!coordinateFitsVoxelGrid(geometry, config_.geoBitDepthInput)) {
                        continue;
                    }

                    frame.geometry.push_back(geometry);
                    frame.attributes.push_back(AttributePoint{
                        static_cast<uint8_t>(xf * 255.F),
                        static_cast<uint8_t>(yf * 255.F),
                        static_cast<uint8_t>(128 + 127 * std::sin(xf * 6.2831853F)),
                    });
                }
            }

            pushFrame(std::move(frame));
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / (std::max)(1, config_.colorFps)));
        }
    }

    OrbbecCaptureConfig config_;
    std::atomic<bool> running_{false};
    std::jthread captureThread_;
    std::mutex queueMutex_;
    std::condition_variable queueCv_;
    std::deque<PointCloudFrame> queue_;
};
