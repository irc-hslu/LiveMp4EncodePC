#pragma once

#include <memory>

#include <libobsensor/ObSensor.hpp>

/// Starts Orbbec pipeline; on Windows (/EHa) catches SDK access violations during start.
bool orbbecSafePipelineStart(
    ob::Pipeline& pipeline,
    const std::shared_ptr<ob::Config>& config,
    bool enableFrameSync = false);

/// waitForFrameset wrapper — returns nullptr on SDK error or access violation.
std::shared_ptr<ob::FrameSet> orbbecSafeWaitForFrameset(ob::Pipeline& pipeline, uint32_t timeoutMs);

/// Filter::process wrapper — returns nullptr on SDK error or access violation.
std::shared_ptr<ob::Frame> orbbecSafeFilterProcess(
    ob::Filter& filter, const std::shared_ptr<ob::Frame>& input);
