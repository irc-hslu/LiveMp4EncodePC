#pragma once

#include "OrbbecCapture.hpp"
#include "PipelineProfiler.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

/// UDP preview stream for the browser Three.js viewer (magic PCV1, chunked for MTU).
struct PointCloudPreviewConfig {
    std::string dstAddress = "127.0.0.1";
    uint16_t dstPort = 8891;
    /// Max points per preview frame (browser UDP path subsamples above this).
    size_t maxPointsPerFrame = 200000;
    /// Max UDP payload per datagram (must stay below ~65 KiB OS limit; 1400 fits standard MTU).
    size_t maxUdpPayloadBytes = 1400;
    PipelineProfiler* profiler = nullptr;
};

class PointCloudPreviewSender {
public:
    explicit PointCloudPreviewSender(PointCloudPreviewConfig config = {})
        : config_(std::move(config)) {
#ifdef _WIN32
        socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket_ == INVALID_SOCKET) {
            throw std::runtime_error("PointCloudPreviewSender: socket() failed");
        }
#else
        socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_ < 0) {
            throw std::runtime_error("PointCloudPreviewSender: socket() failed");
        }
#endif
    }

    ~PointCloudPreviewSender() {
        if (socketValid()) {
#ifdef _WIN32
            closesocket(socket_);
#else
            close(socket_);
#endif
        }
    }

    PointCloudPreviewSender(const PointCloudPreviewSender&) = delete;
    PointCloudPreviewSender& operator=(const PointCloudPreviewSender&) = delete;

    void send(const PointCloudFrame& frame) {
        if (!socketValid() || frame.geometry.empty()) {
            return;
        }

        const size_t totalPoints = frame.geometry.size();
        const size_t subsampleStride =
            (totalPoints > config_.maxPointsPerFrame)
                ? (totalPoints + config_.maxPointsPerFrame - 1) / config_.maxPointsPerFrame
                : 1U;
        const size_t subsampledCount = totalPoints / subsampleStride;

        static constexpr size_t kHeaderBytes = 32;
        static constexpr size_t kBytesPerPoint = 9;
        const size_t maxPointsPerChunk =
            (config_.maxUdpPayloadBytes > kHeaderBytes)
                ? (config_.maxUdpPayloadBytes - kHeaderBytes) / kBytesPerPoint
                : 1U;
        const size_t chunkCount =
            (subsampledCount + maxPointsPerChunk - 1) / maxPointsPerChunk;

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(config_.dstPort);
        if (inet_pton(AF_INET, config_.dstAddress.c_str(), &addr.sin_addr) != 1) {
            std::cerr << "[preview] Invalid preview address: " << config_.dstAddress << "\n";
            return;
        }

        size_t globalIndex = 0;
        uint64_t bytesSent = 0;
        uint64_t packetsSent = 0;
        for (size_t chunk = 0; chunk < chunkCount; ++chunk) {
            const size_t pointsInChunk =
                (std::min)(maxPointsPerChunk, subsampledCount - chunk * maxPointsPerChunk);
            const size_t payloadBytes = kHeaderBytes + pointsInChunk * kBytesPerPoint;
            packet_.resize(payloadBytes);

            writeU32(0, 0x31564350U); // 'PCV1'
            writeU32(4, static_cast<uint32_t>(frame.frameNumber));
            writeU16(8, static_cast<uint16_t>(chunk));
            writeU16(10, static_cast<uint16_t>(chunkCount));
            writeF32(12, frame.origin[0]);
            writeF32(16, frame.origin[1]);
            writeF32(20, frame.origin[2]);
            writeF32(24, frame.quantScale);
            writeU32(28, static_cast<uint32_t>(pointsInChunk));

            size_t offset = kHeaderBytes;
            for (size_t i = 0; i < pointsInChunk; ++i) {
                const size_t src = (globalIndex + i) * subsampleStride;
                const auto& g = frame.geometry[src];
                const auto& a = frame.attributes[src];
                const uint16_t qx = static_cast<uint16_t>(g[0]);
                const uint16_t qy = static_cast<uint16_t>(g[1]);
                const uint16_t qz = static_cast<uint16_t>(g[2]);
                packet_[offset++] = static_cast<uint8_t>(qx & 0xffU);
                packet_[offset++] = static_cast<uint8_t>(qx >> 8);
                packet_[offset++] = static_cast<uint8_t>(qy & 0xffU);
                packet_[offset++] = static_cast<uint8_t>(qy >> 8);
                packet_[offset++] = static_cast<uint8_t>(qz & 0xffU);
                packet_[offset++] = static_cast<uint8_t>(qz >> 8);
                packet_[offset++] = a[0];
                packet_[offset++] = a[1];
                packet_[offset++] = a[2];
            }
            globalIndex += pointsInChunk;

#ifdef _WIN32
            const int sent = sendto(socket_, reinterpret_cast<const char*>(packet_.data()),
                                    static_cast<int>(packet_.size()), 0, reinterpret_cast<sockaddr*>(&addr),
                                    sizeof(addr));
            if (sent == static_cast<int>(packet_.size())) {
                bytesSent += static_cast<uint64_t>(sent);
                packetsSent += 1;
            } else if (frame.frameNumber <= 2) {
                std::cerr << "[preview] sendto failed (chunk " << chunk << "/" << chunkCount
                          << ", " << packet_.size() << " B): WSA error " << WSAGetLastError() << "\n";
            }
#else
            const ssize_t sent =
                sendto(socket_, packet_.data(), packet_.size(), 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
            if (sent == static_cast<ssize_t>(packet_.size())) {
                bytesSent += static_cast<uint64_t>(sent);
                packetsSent += 1;
            } else if (frame.frameNumber <= 2) {
                std::cerr << "[preview] sendto failed (chunk " << chunk << ")\n";
            }
#endif
        }

        if (config_.profiler) {
            config_.profiler->notePreview(bytesSent, subsampledCount, packetsSent);
        }

        if (frame.frameNumber <= 2) {
            std::cout << "[preview] Frame " << frame.frameNumber << ": " << subsampledCount << " points in "
                      << chunkCount << " UDP chunks (" << packet_.size() << " B each)\n"
                      << std::flush;
        }
    }

private:
    void writeU16(size_t offset, uint16_t value) {
        packet_[offset + 0] = static_cast<uint8_t>(value);
        packet_[offset + 1] = static_cast<uint8_t>(value >> 8);
    }

    void writeU32(size_t offset, uint32_t value) {
        packet_[offset + 0] = static_cast<uint8_t>(value);
        packet_[offset + 1] = static_cast<uint8_t>(value >> 8);
        packet_[offset + 2] = static_cast<uint8_t>(value >> 16);
        packet_[offset + 3] = static_cast<uint8_t>(value >> 24);
    }

    void writeF32(size_t offset, float value) {
        uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(float));
        writeU32(offset, bits);
    }

    bool socketValid() const {
#ifdef _WIN32
        return socket_ != INVALID_SOCKET;
#else
        return socket_ >= 0;
#endif
    }

    PointCloudPreviewConfig config_;
    std::vector<uint8_t> packet_;
#ifdef _WIN32
    SOCKET socket_ = INVALID_SOCKET;
#else
    int socket_ = -1;
#endif
};
