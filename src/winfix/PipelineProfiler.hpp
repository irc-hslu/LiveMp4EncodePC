#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

/// Rolling FPS / bandwidth counters for each stage of vpcclive_streamer.
class PipelineProfiler {
public:
    void setEnabled(bool enabled) { enabled_ = enabled; }

    void setIntervalSec(int seconds) {
        intervalSec_ = (seconds > 0) ? seconds : 5;
    }

    bool enabled() const { return enabled_; }

    void noteCapture(uint64_t points) {
        if (!enabled_) {
            return;
        }
        captureEvents_.fetch_add(1, std::memory_order_relaxed);
        capturePoints_.fetch_add(points, std::memory_order_relaxed);
    }

    void notePreview(uint64_t bytes, uint64_t points, uint64_t udpPackets) {
        if (!enabled_) {
            return;
        }
        previewEvents_.fetch_add(1, std::memory_order_relaxed);
        previewBytes_.fetch_add(bytes, std::memory_order_relaxed);
        previewPoints_.fetch_add(points, std::memory_order_relaxed);
        previewPackets_.fetch_add(udpPackets, std::memory_order_relaxed);
    }

    void noteEncodeSubmit(uint64_t points) {
        if (!enabled_) {
            return;
        }
        encodeEvents_.fetch_add(1, std::memory_order_relaxed);
        encodePoints_.fetch_add(points, std::memory_order_relaxed);
    }

    void noteRtpGof(uint64_t v3cBytes) {
        if (!enabled_) {
            return;
        }
        rtpGofEvents_.fetch_add(1, std::memory_order_relaxed);
        rtpBytes_.fetch_add(v3cBytes, std::memory_order_relaxed);
    }

    void tick() {
        if (!enabled_) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (lastReport_ == std::chrono::steady_clock::time_point{}) {
            lastReport_ = now;
            return;
        }
        const auto elapsed = now - lastReport_;
        if (elapsed < std::chrono::seconds(intervalSec_)) {
            return;
        }
        report(now, false);
        lastReport_ = now;
    }

    void reportFinal() {
        if (!enabled_) {
            return;
        }
        report(std::chrono::steady_clock::now(), true);
    }

private:
    static double secondsSince(const std::chrono::steady_clock::time_point& now,
                               const std::chrono::steady_clock::time_point& last) {
        return std::chrono::duration<double>(now - last).count();
    }

    static std::string formatMbps(uint64_t bytes, double intervalSec) {
        if (intervalSec <= 0.0) {
            return "0.00 Mbps";
        }
        const double mbps = (static_cast<double>(bytes) * 8.0) / (intervalSec * 1'000'000.0);
        std::ostringstream out;
        out << std::fixed << std::setprecision(2) << mbps << " Mbps";
        return out.str();
    }

    static std::string formatFps(uint64_t events, double intervalSec) {
        if (intervalSec <= 0.0) {
            return "0.0 fps";
        }
        std::ostringstream out;
        out << std::fixed << std::setprecision(1) << (static_cast<double>(events) / intervalSec) << " fps";
        return out.str();
    }

    static std::string formatPointsPerFrame(uint64_t points, uint64_t events) {
        if (events == 0) {
            return "— pts";
        }
        std::ostringstream out;
        out << static_cast<uint64_t>(static_cast<double>(points) / static_cast<double>(events) + 0.5) << " pts/frame";
        return out.str();
    }

    void report(std::chrono::steady_clock::time_point now, bool finalReport) {
        const double intervalSec = secondsSince(now, lastReport_);
        if (intervalSec <= 0.0) {
            return;
        }

        const uint64_t captureEvents = captureEvents_.exchange(0, std::memory_order_relaxed);
        const uint64_t capturePoints = capturePoints_.exchange(0, std::memory_order_relaxed);
        const uint64_t previewEvents = previewEvents_.exchange(0, std::memory_order_relaxed);
        const uint64_t previewBytes = previewBytes_.exchange(0, std::memory_order_relaxed);
        const uint64_t previewPoints = previewPoints_.exchange(0, std::memory_order_relaxed);
        const uint64_t previewPackets = previewPackets_.exchange(0, std::memory_order_relaxed);
        const uint64_t encodeEvents = encodeEvents_.exchange(0, std::memory_order_relaxed);
        const uint64_t encodePoints = encodePoints_.exchange(0, std::memory_order_relaxed);
        const uint64_t rtpGofEvents = rtpGofEvents_.exchange(0, std::memory_order_relaxed);
        const uint64_t rtpBytes = rtpBytes_.exchange(0, std::memory_order_relaxed);

        std::cerr << (finalReport ? "[profile] Final summary (" : "[profile] ")
                  << std::fixed << std::setprecision(1) << intervalSec << "s"
                  << (finalReport ? ")" : "") << "\n"
                  << "  capture  " << formatFps(captureEvents, intervalSec) << " | "
                  << formatPointsPerFrame(capturePoints, captureEvents) << "\n"
                  << "  preview  " << formatFps(previewEvents, intervalSec) << " | "
                  << formatMbps(previewBytes, intervalSec) << " | "
                  << previewPackets << " UDP pkts | "
                  << formatPointsPerFrame(previewPoints, previewEvents) << "\n"
                  << "  encode   " << formatFps(encodeEvents, intervalSec) << " | "
                  << formatPointsPerFrame(encodePoints, encodeEvents) << "\n"
                  << "  rtp/v3c  " << formatFps(rtpGofEvents, intervalSec) << " GOF/s | "
                  << formatMbps(rtpBytes, intervalSec) << "\n"
                  << std::flush;
    }

    bool enabled_{true};
    int intervalSec_{5};
    std::chrono::steady_clock::time_point lastReport_{};

    std::atomic<uint64_t> captureEvents_{0};
    std::atomic<uint64_t> capturePoints_{0};
    std::atomic<uint64_t> previewEvents_{0};
    std::atomic<uint64_t> previewBytes_{0};
    std::atomic<uint64_t> previewPoints_{0};
    std::atomic<uint64_t> previewPackets_{0};
    std::atomic<uint64_t> encodeEvents_{0};
    std::atomic<uint64_t> encodePoints_{0};
    std::atomic<uint64_t> rtpGofEvents_{0};
    std::atomic<uint64_t> rtpBytes_{0};
};
