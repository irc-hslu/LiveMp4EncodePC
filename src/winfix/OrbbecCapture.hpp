#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <mutex>
#include <functional>
#include <optional>

#include "uvgutils/sync.hpp"
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "PipelineProfiler.hpp"

#include "uvgutils/utils.hpp"
#include "uvgvpcc/uvgvpcc.hpp"

using GeometryPoint = uvgutils::VectorN<uvgvpcc_enc::typeGeometryInput, 3>;
using AttributePoint = uvgutils::VectorN<uint8_t, 3>;

#if defined(ORBBEC_SDK_FOUND)
#include <algorithm>
#include <libobsensor/ObSensor.hpp>
#include "orbbec_safe.hpp"
#ifdef ERROR
#undef ERROR
#endif
#endif

/// Orbbec RGBD capture (open3d.cpp pipeline + Align/PointCloudFilter) for uvgVPCCenc.
struct PointCloudFrame {
    uint64_t frameNumber = 0;
    float origin[3]{0.F, 0.F, 0.F};
    float quantScale = 1.F;
    std::vector<GeometryPoint> geometry;
    std::vector<AttributePoint> attributes;
};

struct OrbbecCaptureConfig {
    /// Use device default stream profiles (matches 05_point_cloud.py / pointcloud_filter_o3d.py).
    bool useDefaultStreamProfiles = true;
    /// When true, color/depth use OB_WIDTH_ANY (open3d.cpp — device max resolution).
    bool useDeviceMaxResolution = true;
    int colorWidth = 1920;
    int colorHeight = 1080;
    int colorFps = 15;
    int depthWidth = 1024;
    int depthHeight = 1024;
    int depthFps = 15;
    size_t geoBitDepthInput = 10;
    size_t maxQueueDepth = 1;
    /// Max points kept at capture; 0 = keep all valid points (no stride subsampling).
    size_t maxPointsPerFrame = 0;
    /// Optional depth clip in mm; 0 disables each bound (invalid/zero depth is always skipped).
    float minDepthMm = 0.F;
    float maxDepthMm = 0.F;
    bool enableSyntheticFallback = true;
    /// Called on the capture thread when a new point cloud frame is ready (for live preview).
    std::function<void(const PointCloudFrame&)> onFrameCaptured;
    PipelineProfiler* profiler = nullptr;
};

class OrbbecCapture {
public:
    explicit OrbbecCapture(OrbbecCaptureConfig config = {})
        : config_(std::move(config)) {}

    ~OrbbecCapture() { stop(); }

    OrbbecCapture(const OrbbecCapture&) = delete;
    OrbbecCapture& operator=(const OrbbecCapture&) = delete;

#if defined(ORBBEC_SDK_FOUND)
    /// Probe for a camera on the main thread. Does not keep the device open (capture thread owns the pipeline).
    void prepareDevice() {
        std::cerr << "[orbbec] Querying devices...\n" << std::flush;
        ob::Context context;
        const auto deviceList = context.queryDeviceList();
        if (!deviceList || deviceList->getCount() == 0) {
            throw std::runtime_error(
                "No Orbbec camera found. Check USB, drivers, and close Orbbec Viewer if it is using the device.");
        }

        std::cerr << "[orbbec] Found " << deviceList->getCount() << " device(s)\n" << std::flush;
        if (auto device = deviceList->getDevice(0)) {
            if (auto info = device->getDeviceInfo()) {
                std::cerr << "[orbbec] Using device: " << info->name() << "\n" << std::flush;
            }
        }
    }
#endif

    void start(bool forceSynthetic = false) {
        if (running_.exchange(true)) {
            return;
        }

#if defined(ORBBEC_SDK_FOUND)
        if (forceSynthetic) {
            captureThread_ = std::jthread([this](std::stop_token stopToken) { captureLoopSynthetic(stopToken); });
        } else {
            captureThread_ = std::jthread([this](std::stop_token stopToken) { captureLoopOrbbec(stopToken); });
        }
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
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (running_.load()) {
            if (auto frame = tryPopFrame()) {
                return frame;
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                return std::nullopt;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return std::nullopt;
    }

    bool isRunning() const { return running_.load(); }

    bool captureFailed() const { return captureFailed_.load(); }

    std::optional<std::string> captureError() const {
        std::lock_guard lock(captureErrorMutex_);
        if (captureError_.empty()) {
            return std::nullopt;
        }
        return captureError_;
    }

private:
    void reportCaptureError(std::string message) {
        {
            std::lock_guard lock(captureErrorMutex_);
            captureError_ = std::move(message);
        }
        captureFailed_ = true;
        running_ = false;
        std::cerr << "[orbbec] ERROR: " << captureError_ << "\n" << std::flush;
    }
    static uvgvpcc_enc::typeGeometryInput quantizeCoord(float meters, float origin, float scale, size_t geoBitDepthInput) {
        const float qf = (meters - origin) * scale;
        const float maxValue = static_cast<float>((static_cast<uint32_t>(1) << geoBitDepthInput) - 1U);
        if (qf <= 0.F) {
            return 0;
        }
        if (qf >= maxValue) {
            return static_cast<uvgvpcc_enc::typeGeometryInput>(maxValue);
        }
        return static_cast<uvgvpcc_enc::typeGeometryInput>(qf);
    }

    static bool isValidPoint(float x, float y, float z, float minDepthMm, float maxDepthMm) {
        if (x == 0.F && y == 0.F && z == 0.F) {
            return false;
        }
        if (z <= 0.F) {
            return false;
        }
        if (minDepthMm > 0.F && z < minDepthMm) {
            return false;
        }
        if (maxDepthMm > 0.F && z > maxDepthMm) {
            return false;
        }
        return true;
    }

    static uint8_t colorChannelToByte(float channel) {
        const float clamped = (std::max)(0.F, (std::min)(channel, 255.F));
        return static_cast<uint8_t>(clamped);
    }

    static bool computeQuantizationFromCloud(
        const OBColorPoint* points,
        uint32_t pointCount,
        uint32_t stride,
        float& minX,
        float& minY,
        float& minZ,
        float& quantScale,
        size_t geoBitDepthInput,
        float minDepthMm,
        float maxDepthMm) {
        bool found = false;
        float maxX = 0.F;
        float maxY = 0.F;
        float maxZ = 0.F;
        for (uint32_t i = 0; i < pointCount; i += stride) {
            if (!isValidPoint(points[i].x, points[i].y, points[i].z, minDepthMm, maxDepthMm)) {
                continue;
            }
            if (!found) {
                minX = maxX = points[i].x;
                minY = maxY = points[i].y;
                minZ = maxZ = points[i].z;
                found = true;
                continue;
            }
            minX = (std::min)(minX, points[i].x);
            minY = (std::min)(minY, points[i].y);
            minZ = (std::min)(minZ, points[i].z);
            maxX = (std::max)(maxX, points[i].x);
            maxY = (std::max)(maxY, points[i].y);
            maxZ = (std::max)(maxZ, points[i].z);
        }
        if (!found) {
            return false;
        }
        const float extentX = maxX - minX;
        const float extentY = maxY - minY;
        const float extentZ = maxZ - minZ;
        const float maxExtent = (std::max)(extentX, (std::max)(extentY, extentZ));
        const float maxVoxel = static_cast<float>((static_cast<uint32_t>(1) << geoBitDepthInput) - 1U);
        quantScale = maxExtent > 1e-6F ? maxVoxel / maxExtent : 1.F;
        return true;
    }

    static bool computeQuantizationFromCloud(
        const OBPoint* points,
        uint32_t pointCount,
        uint32_t stride,
        float& minX,
        float& minY,
        float& minZ,
        float& quantScale,
        size_t geoBitDepthInput,
        float minDepthMm,
        float maxDepthMm) {
        bool found = false;
        float maxX = 0.F;
        float maxY = 0.F;
        float maxZ = 0.F;
        for (uint32_t i = 0; i < pointCount; i += stride) {
            if (!isValidPoint(points[i].x, points[i].y, points[i].z, minDepthMm, maxDepthMm)) {
                continue;
            }
            if (!found) {
                minX = maxX = points[i].x;
                minY = maxY = points[i].y;
                minZ = maxZ = points[i].z;
                found = true;
                continue;
            }
            minX = (std::min)(minX, points[i].x);
            minY = (std::min)(minY, points[i].y);
            minZ = (std::min)(minZ, points[i].z);
            maxX = (std::max)(maxX, points[i].x);
            maxY = (std::max)(maxY, points[i].y);
            maxZ = (std::max)(maxZ, points[i].z);
        }
        if (!found) {
            return false;
        }
        const float extentX = maxX - minX;
        const float extentY = maxY - minY;
        const float extentZ = maxZ - minZ;
        const float maxExtent = (std::max)(extentX, (std::max)(extentY, extentZ));
        const float maxVoxel = static_cast<float>((static_cast<uint32_t>(1) << geoBitDepthInput) - 1U);
        quantScale = maxExtent > 1e-6F ? maxVoxel / maxExtent : 1.F;
        return true;
    }

    void pushFrame(PointCloudFrame frame) {
        if (config_.profiler) {
            config_.profiler->noteCapture(frame.geometry.size());
        }
        if (config_.onFrameCaptured) {
            config_.onFrameCaptured(frame);
        }
        {
            std::lock_guard lock(queueMutex_);
            while (queue_.size() >= config_.maxQueueDepth) {
                queue_.pop_front();
            }
            queue_.push_back(std::move(frame));
        }
    }
 
    enum class OrbbecCaptureMode { Rgbd, DepthOnly };

    static void safePipelineStop(ob::Pipeline& pipeline) {
        try {
            pipeline.stop();
        } catch (...) {
        }
    }

    static const char* colorFormatName(OBFormat format) {
        switch (format) {
        case OB_FORMAT_RGB:
            return "RGB";
        case OB_FORMAT_MJPG:
            return "MJPG";
        default:
            return "unknown";
        }
    }

    static void logFrameResolution(const char* label, const std::shared_ptr<ob::Frame>& frame) {
        if (!frame) {
            return;
        }
        try {
            const auto video = frame->as<ob::VideoFrame>();
            std::cerr << "[orbbec] " << label << " frame: " << video->width() << "x" << video->height() << "\n"
                      << std::flush;
        } catch (...) {
        }
    }

    static void configurePointCloudFilter(
        ob::PointCloudFilter& filter, ob::Pipeline& pipeline, OBFormat format) {
        filter.setCreatePointFormat(format);
        try {
            filter.setCameraParam(pipeline.getCameraParam());
        } catch (...) {
        }
    }

    static void applyDepthValueScale(
        const std::shared_ptr<ob::PointCloudFilter>& pointCloudFilter,
        const std::shared_ptr<ob::DepthFrame>& depthFrame) {
        if (!pointCloudFilter || !depthFrame) {
            return;
        }
        try {
            pointCloudFilter->setPositionDataScaled(depthFrame->getValueScale());
        } catch (...) {
        }
    }

    void captureLoopOrbbec(std::stop_token stopToken) {
        try {
            captureLoopOrbbecImpl(stopToken);
        } catch (const ob::Error& error) {
            reportCaptureError(std::string("Orbbec SDK error: ") + error.getMessage());
        } catch (const std::exception& error) {
            reportCaptureError(error.what());
        } catch (...) {
            reportCaptureError(
                "Orbbec capture failed (SDK access violation or unexpected exception). "
                "Close Orbbec Viewer and retry.");
        }

        if (!stopToken.stop_requested() && !captureFailed_.load() && running_.load()) {
            reportCaptureError("Orbbec capture thread stopped unexpectedly");
        }
    }

    static void logVideoProfile(const char* label, const std::shared_ptr<ob::VideoStreamProfile>& profile) {
        if (!profile) {
            return;
        }
        std::cerr << "[orbbec] " << label << " profile: " << profile->width() << "x" << profile->height() << " @"
                  << profile->fps() << " fps\n"
                  << std::flush;
    }

    /// Matches 05_point_cloud.py: default depth + color stream profiles from the device list.
    bool tryStartProfileListRgbdPipeline(
        std::shared_ptr<ob::Pipeline>& pipeline,
        OBFormat colorFormat,
        std::shared_ptr<ob::Align>& alignFilter,
        std::shared_ptr<ob::PointCloudFilter>& pointCloudFilter) {
        pipeline = std::make_shared<ob::Pipeline>();
        auto streamConfig = std::make_shared<ob::Config>();

        try {
            const auto depthList = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
            if (!depthList || depthList->getCount() == 0) {
                return false;
            }
            const int depthFps = config_.depthFps > 0 ? config_.depthFps : OB_FPS_ANY;
            auto depthProfile = depthList->getVideoStreamProfile(
                OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_Y16, depthFps);
            streamConfig->enableStream(depthProfile);
            logVideoProfile("Depth", depthProfile);

            const auto colorList = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
            if (!colorList || colorList->getCount() == 0) {
                return false;
            }
            const int colorFps = config_.colorFps > 0 ? config_.colorFps : OB_FPS_ANY;
            auto colorProfile = colorList->getVideoStreamProfile(
                OB_WIDTH_ANY, OB_HEIGHT_ANY, colorFormat, colorFps);
            streamConfig->enableStream(colorProfile);
            logVideoProfile("Color", colorProfile);
        } catch (...) {
            pipeline.reset();
            return false;
        }

        streamConfig->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

        if (!orbbecSafePipelineStart(*pipeline, streamConfig, true)) {
            pipeline.reset();
            return false;
        }

        try {
            alignFilter = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            pointCloudFilter = std::make_shared<ob::PointCloudFilter>();
            configurePointCloudFilter(*pointCloudFilter, *pipeline, OB_FORMAT_RGB_POINT);
        } catch (...) {
            safePipelineStop(*pipeline);
            pipeline.reset();
            alignFilter.reset();
            pointCloudFilter.reset();
            return false;
        }

        std::cerr << "[orbbec] Profile-list RGBD: color=" << colorFormatName(colorFormat) << ", depth=Y16, frame sync\n"
                  << std::flush;
        return true;
    }

    /// Pipeline config matches open3d.cpp (RGB/Y16, ANY resolution, frame sync) + Align/PointCloudFilter.
    bool tryStartOpen3dRgbdPipeline(
        std::shared_ptr<ob::Pipeline>& pipeline,
        OBFormat colorFormat,
        std::shared_ptr<ob::Align>& alignFilter,
        std::shared_ptr<ob::PointCloudFilter>& pointCloudFilter) {
        pipeline = std::make_shared<ob::Pipeline>();
        auto streamConfig = std::make_shared<ob::Config>();

        if (config_.useDeviceMaxResolution) {
            streamConfig->enableVideoStream(
                OB_STREAM_COLOR, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, colorFormat);
            streamConfig->enableVideoStream(
                OB_STREAM_DEPTH, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_Y16);
        } else {
            streamConfig->enableVideoStream(
                OB_STREAM_COLOR, config_.colorWidth, config_.colorHeight, config_.colorFps, colorFormat);
            streamConfig->enableVideoStream(
                OB_STREAM_DEPTH, OB_WIDTH_ANY, OB_HEIGHT_ANY, config_.depthFps, OB_FORMAT_Y16);
        }
        streamConfig->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

        if (!orbbecSafePipelineStart(*pipeline, streamConfig, true)) {
            pipeline.reset();
            return false;
        }

        try {
            alignFilter = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            pointCloudFilter = std::make_shared<ob::PointCloudFilter>();
            configurePointCloudFilter(*pointCloudFilter, *pipeline, OB_FORMAT_RGB_POINT);
        } catch (...) {
            safePipelineStop(*pipeline);
            pipeline.reset();
            alignFilter.reset();
            pointCloudFilter.reset();
            return false;
        }

        std::cerr << "[orbbec] Open3D-style RGBD: color=" << colorFormatName(colorFormat) << ", depth=Y16, frame sync"
                  << (config_.useDeviceMaxResolution ? " (device max resolution)" : " (explicit color size)") << "\n"
                  << std::flush;
        return true;
    }

    bool tryStartDepthOnlyPipeline(
        std::shared_ptr<ob::Pipeline>& pipeline,
        std::shared_ptr<ob::PointCloudFilter>& pointCloudFilter) {
        pipeline = std::make_shared<ob::Pipeline>();
        auto streamConfig = std::make_shared<ob::Config>();
        streamConfig->enableVideoStream(
            OB_STREAM_DEPTH, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_Y16);

        if (!orbbecSafePipelineStart(*pipeline, streamConfig)) {
            pipeline.reset();
            return false;
        }

        pointCloudFilter = std::make_shared<ob::PointCloudFilter>();
        configurePointCloudFilter(*pointCloudFilter, *pipeline, OB_FORMAT_POINT);
        return true;
    }

    void logOrbbecDevice(const std::shared_ptr<ob::Pipeline>& pipeline) {
        if (const auto device = pipeline->getDevice()) {
            if (const auto info = device->getDeviceInfo()) {
                std::cerr << "[orbbec] Device: " << info->name() << "\n" << std::flush;
            }
        }
    }

    void captureLoopOrbbecImpl(std::stop_token stopToken) {
        std::cerr << "[orbbec] Opening pipeline...\n" << std::flush;

        OrbbecCaptureMode mode = OrbbecCaptureMode::Rgbd;
        std::shared_ptr<ob::Pipeline> pipeline;
        std::shared_ptr<ob::Align> alignFilter;
        std::shared_ptr<ob::PointCloudFilter> pointCloudFilter;

        // Femto Bolt often needs MJPG; RGB works on some devices (open3d.cpp uses RGB).
        const OBFormat colorFormats[] = {OB_FORMAT_MJPG, OB_FORMAT_RGB};
        bool started = false;
        for (const OBFormat colorFormat : colorFormats) {
            if (config_.useDefaultStreamProfiles) {
                std::cerr << "[orbbec] Trying profile-list RGBD (color=" << colorFormatName(colorFormat) << ")...\n"
                          << std::flush;
                if (tryStartProfileListRgbdPipeline(pipeline, colorFormat, alignFilter, pointCloudFilter)) {
                    mode = OrbbecCaptureMode::Rgbd;
                    started = true;
                    break;
                }
            }

            if (!started && config_.useDeviceMaxResolution) {
                std::cerr << "[orbbec] Trying Open3D-style RGBD (color=" << colorFormatName(colorFormat) << ")...\n"
                          << std::flush;
                if (tryStartOpen3dRgbdPipeline(pipeline, colorFormat, alignFilter, pointCloudFilter)) {
                    mode = OrbbecCaptureMode::Rgbd;
                    started = true;
                    break;
                }
            }

            if (!started && !config_.useDeviceMaxResolution) {
                std::cerr << "[orbbec] Trying configured RGBD (color=" << colorFormatName(colorFormat) << ", "
                          << config_.colorWidth << "x" << config_.colorHeight << ")...\n"
                          << std::flush;
                if (tryStartOpen3dRgbdPipeline(pipeline, colorFormat, alignFilter, pointCloudFilter)) {
                    mode = OrbbecCaptureMode::Rgbd;
                    started = true;
                    break;
                }
            }

            std::cerr << "[orbbec] RGBD " << colorFormatName(colorFormat) << " failed, trying next...\n" << std::flush;
        }

        if (!started) {
            std::cerr << "[orbbec] RGBD unavailable, falling back to depth-only point cloud...\n" << std::flush;
            if (!tryStartDepthOnlyPipeline(pipeline, pointCloudFilter)) {
                throw std::runtime_error(
                    "Orbbec pipeline.start() failed for RGBD and depth-only modes (SDK access violation or device busy)");
            }
            mode = OrbbecCaptureMode::DepthOnly;
            std::cerr << "[orbbec] Depth-only pipeline active (OB_FORMAT_POINT, gray attributes)\n" << std::flush;
        }

        logOrbbecDevice(pipeline);

        bool loggedResolution = false;
        for (int i = 0; i < 15 && !stopToken.stop_requested(); ++i) {
            const auto warmupSet = orbbecSafeWaitForFrameset(*pipeline, 200);
            if (!loggedResolution && warmupSet && warmupSet->colorFrame() && warmupSet->depthFrame()) {
                logFrameResolution("Color", warmupSet->colorFrame());
                logFrameResolution("Depth", warmupSet->depthFrame());
                loggedResolution = true;
            }
        }

        try {
            const OBFormat cloudFormat =
                (mode == OrbbecCaptureMode::DepthOnly) ? OB_FORMAT_POINT : OB_FORMAT_RGB_POINT;
            configurePointCloudFilter(*pointCloudFilter, *pipeline, cloudFormat);
        } catch (...) {
        }

        uint64_t frameNumber = 0;
        auto lastWaitLog = std::chrono::steady_clock::now();

        while (!stopToken.stop_requested() && running_) {
            auto frameSet = orbbecSafeWaitForFrameset(*pipeline, 1000);

            if (mode == OrbbecCaptureMode::Rgbd) {
                if (!frameSet || !frameSet->depthFrame() || !frameSet->colorFrame()) {
                    const auto now = std::chrono::steady_clock::now();
                    if (now - lastWaitLog >= std::chrono::seconds(5)) {
                        std::cerr << "[orbbec] Waiting for synchronized color+depth...\n" << std::flush;
                        lastWaitLog = now;
                    }
                    continue;
                }
            } else if (!frameSet || !frameSet->depthFrame()) {
                const auto now = std::chrono::steady_clock::now();
                if (now - lastWaitLog >= std::chrono::seconds(5)) {
                    std::cerr << "[orbbec] Waiting for depth frames...\n" << std::flush;
                    lastWaitLog = now;
                }
                continue;
            }
            lastWaitLog = std::chrono::steady_clock::now();

            std::shared_ptr<ob::Frame> cloudFrame;
            const auto depthFrame = frameSet->depthFrame();
            if (!depthFrame) {
                continue;
            }
            applyDepthValueScale(pointCloudFilter, depthFrame);

            if (mode == OrbbecCaptureMode::DepthOnly) {
                cloudFrame = orbbecSafeFilterProcess(*pointCloudFilter, depthFrame);
            } else {
                if (!alignFilter) {
                    continue;
                }
                auto alignedFrame = orbbecSafeFilterProcess(*alignFilter, frameSet);
                if (!alignedFrame) {
                    continue;
                }
                cloudFrame = orbbecSafeFilterProcess(*pointCloudFilter, alignedFrame);
            }

            if (!cloudFrame) {
                continue;
            }

            PointCloudFrame frame;
            frame.frameNumber = frameNumber++;

            if (mode == OrbbecCaptureMode::Rgbd) {
                if (cloudFrame->format() != OB_FORMAT_RGB_POINT ||
                    cloudFrame->dataSize() < sizeof(OBColorPoint)) {
                    std::cerr << "[orbbec] Cloud frame format or data size is invalid\n" << std::flush;
                    continue;
                }

                const auto* points = reinterpret_cast<const OBColorPoint*>(cloudFrame->data());
                const uint32_t pointCount = cloudFrame->dataSize() / static_cast<uint32_t>(sizeof(OBColorPoint));
                if (pointCount == 0) {
                    std::cerr << "[orbbec] Point count is 0\n" << std::flush;
                    continue;
                }

                const uint32_t stride =
                    (config_.maxPointsPerFrame > 0 && pointCount > config_.maxPointsPerFrame)
                        ? static_cast<uint32_t>((pointCount + config_.maxPointsPerFrame - 1) /
                                                config_.maxPointsPerFrame)
                        : 1U;

                float minX = 0.F;
                float minY = 0.F;
                float minZ = 0.F;
                float quantScale = 1.F;
                if (!computeQuantizationFromCloud(points,
                                                  pointCount,
                                                  1U,
                                                  minX,
                                                  minY,
                                                  minZ,
                                                  quantScale,
                                                  config_.geoBitDepthInput,
                                                  config_.minDepthMm,
                                                  config_.maxDepthMm)) {
                    continue;
                }
                if (frame.frameNumber == 0 || frame.frameNumber % 300 == 0) {
                    std::cerr << "[orbbec] RGBD quant origin (" << minX << "," << minY << "," << minZ << ") scale "
                              << quantScale << "\n"
                              << std::flush;
                }

                const uint32_t reserveCount = config_.maxPointsPerFrame > 0
                                                  ? (std::min)(pointCount / stride,
                                                               static_cast<uint32_t>(config_.maxPointsPerFrame))
                                                  : pointCount;

                frame.origin[0] = minX;
                frame.origin[1] = minY;
                frame.origin[2] = minZ;
                frame.quantScale = quantScale;
                frame.geometry.reserve(reserveCount);
                frame.attributes.reserve(reserveCount);

                for (uint32_t i = 0; i < pointCount; i += stride) {
                    if (!isValidPoint(
                            points[i].x, points[i].y, points[i].z, config_.minDepthMm, config_.maxDepthMm)) {
                        continue;
                    }

                    const auto qx = quantizeCoord(points[i].x, minX, quantScale, config_.geoBitDepthInput);
                    const auto qy = quantizeCoord(points[i].y, minY, quantScale, config_.geoBitDepthInput);
                    const auto qz = quantizeCoord(points[i].z, minZ, quantScale, config_.geoBitDepthInput);
                    const GeometryPoint geometry{qx, qy, qz};

                    frame.geometry.push_back(geometry);
                    frame.attributes.push_back(AttributePoint{
                        colorChannelToByte(points[i].r),
                        colorChannelToByte(points[i].g),
                        colorChannelToByte(points[i].b)});
                }

                if (frame.frameNumber == 0 && !frame.geometry.empty()) {
                    std::cerr << "[orbbec] First RGBD cloud: raw=" << pointCount << " kept=" << frame.geometry.size()
                              << " stride=" << stride << "\n"
                              << std::flush;
                }
            } else {
                if (cloudFrame->format() != OB_FORMAT_POINT || cloudFrame->dataSize() < sizeof(OBPoint)) {
                    continue;
                }

                const auto* points = reinterpret_cast<const OBPoint*>(cloudFrame->data());
                const uint32_t pointCount = cloudFrame->dataSize() / static_cast<uint32_t>(sizeof(OBPoint));
                if (pointCount == 0) {
                    continue;
                }

                const uint32_t stride =
                    (config_.maxPointsPerFrame > 0 && pointCount > config_.maxPointsPerFrame)
                        ? static_cast<uint32_t>((pointCount + config_.maxPointsPerFrame - 1) /
                                                config_.maxPointsPerFrame)
                        : 1U;

                float minX = 0.F;
                float minY = 0.F;
                float minZ = 0.F;
                float quantScale = 1.F;
                if (!computeQuantizationFromCloud(points,
                                                  pointCount,
                                                  1U,
                                                  minX,
                                                  minY,
                                                  minZ,
                                                  quantScale,
                                                  config_.geoBitDepthInput,
                                                  config_.minDepthMm,
                                                  config_.maxDepthMm)) {
                    continue;
                }
                if (frame.frameNumber == 0 || frame.frameNumber % 300 == 0) {
                    std::cerr << "[orbbec] Depth quant origin (" << minX << "," << minY << "," << minZ << ") scale "
                              << quantScale << "\n"
                              << std::flush;
                }

                const uint32_t reserveCount = config_.maxPointsPerFrame > 0
                                                  ? (std::min)(pointCount / stride,
                                                               static_cast<uint32_t>(config_.maxPointsPerFrame))
                                                  : pointCount;

                frame.origin[0] = minX;
                frame.origin[1] = minY;
                frame.origin[2] = minZ;
                frame.quantScale = quantScale;
                frame.geometry.reserve(reserveCount);
                frame.attributes.reserve(reserveCount);

                for (uint32_t i = 0; i < pointCount; i += stride) {
                    if (!isValidPoint(
                            points[i].x, points[i].y, points[i].z, config_.minDepthMm, config_.maxDepthMm)) {
                        continue;
                    }

                    const auto qx = quantizeCoord(points[i].x, minX, quantScale, config_.geoBitDepthInput);
                    const auto qy = quantizeCoord(points[i].y, minY, quantScale, config_.geoBitDepthInput);
                    const auto qz = quantizeCoord(points[i].z, minZ, quantScale, config_.geoBitDepthInput);
                    const GeometryPoint geometry{qx, qy, qz};

                    const float depthSpan =
                        (config_.maxDepthMm > config_.minDepthMm && config_.maxDepthMm > 0.F)
                            ? (config_.maxDepthMm - config_.minDepthMm)
                            : 3000.F;
                    const float depthBase = config_.minDepthMm > 0.F ? config_.minDepthMm : 0.F;
                    const uint8_t gray = static_cast<uint8_t>(
                        (std::max)(0.F, (std::min)(255.F, (points[i].z - depthBase) / depthSpan * 255.F)));
                    frame.geometry.push_back(geometry);
                    frame.attributes.push_back(AttributePoint{gray, gray, gray});
                }

                if (frame.frameNumber == 0 && !frame.geometry.empty()) {
                    std::cerr << "[orbbec] First depth cloud: raw=" << pointCount << " kept=" << frame.geometry.size()
                              << " stride=" << stride << "\n"
                              << std::flush;
                }
            }

            if (frame.geometry.empty()) {
                continue;
            }
            pushFrame(std::move(frame));
        }

        safePipelineStop(*pipeline);
    } 

    void captureLoopSynthetic(std::stop_token stopToken) {
        uint64_t frameNumber = 0;
        const float scale = static_cast<float>(1 << config_.geoBitDepthInput);

        while (!stopToken.stop_requested() && running_) {
            PointCloudFrame frame;
            frame.frameNumber = frameNumber++;
            frame.quantScale = 1.F;
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
    std::atomic<bool> captureFailed_{false};
    mutable uvgutils::sync_mutex captureErrorMutex_;
    std::string captureError_;
    std::jthread captureThread_;
    uvgutils::sync_mutex queueMutex_;
    std::deque<PointCloudFrame> queue_;
};
