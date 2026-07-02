#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "uvgvpcc/uvgvpcc.hpp"

/// Appends uvgVPCCenc V3C unit batches to a V3C sample stream file (uvgVPCCenc file_writer format).
class V3cStreamRecorder {
public:
    static constexpr size_t kV3cSizePrecision = 5;

    void open(const std::filesystem::path& outputPath) {
        if (file_.is_open()) {
            return;
        }

        const auto parent = outputPath.parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent);
        }

        file_.open(outputPath, std::ios::binary | std::ios::out);
        if (!file_.is_open()) {
            throw std::runtime_error("V3cStreamRecorder: could not open " + outputPath.string());
        }

        const char header = static_cast<char>((kV3cSizePrecision - 1U) << 5U);
        file_.write(&header, 1);
        outputPath_ = outputPath;
        std::cout << "[record] Writing V3C sample stream to " << outputPath_.string() << "\n" << std::flush;
    }

    void appendBatch(const uvgvpcc_enc::API::v3c_unit_batch& batch) {
        if (!file_.is_open() || batch.v3c_units.empty()) {
            return;
        }

        for (const auto& unit : batch.v3c_units) {
            std::array<char, kV3cSizePrecision> sizeField{};
            writeSizeField(unit.len, sizeField.data());
            file_.write(sizeField.data(), static_cast<std::streamsize>(kV3cSizePrecision));
            file_.write(unit.data.get(), static_cast<std::streamsize>(unit.len));
            bytesWritten_ += kV3cSizePrecision + unit.len;
        }

        file_.flush();
        ++batchesWritten_;
    }

    void close() {
        if (!file_.is_open()) {
            return;
        }
        file_.close();
        std::cout << "[record] Closed V3C stream (" << batchesWritten_ << " batches, " << bytesWritten_ << " payload bytes): "
                  << outputPath_.string() << "\n"
                  << std::flush;
    }

    bool isOpen() const { return file_.is_open(); }

    const std::filesystem::path& path() const { return outputPath_; }

private:
    static void writeSizeField(uint64_t value, char* dst) {
        constexpr uint64_t mask = 0xFFU;
        for (size_t i = 0; i < kV3cSizePrecision; ++i) {
            dst[kV3cSizePrecision - 1U - i] = static_cast<char>((value >> (8U * i)) & mask);
        }
    }

    std::ofstream file_;
    std::filesystem::path outputPath_;
    size_t batchesWritten_ = 0;
    uint64_t bytesWritten_ = 0;
};
