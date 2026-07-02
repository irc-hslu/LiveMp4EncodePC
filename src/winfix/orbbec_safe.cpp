#include "orbbec_safe.hpp"

#include <iostream>

bool orbbecSafePipelineStart(ob::Pipeline& pipeline, const std::shared_ptr<ob::Config>& config, bool enableFrameSync) {
    try {
        if (enableFrameSync) {
            pipeline.enableFrameSync();
        }
        pipeline.start(config);
        return true;
    } catch (const ob::Error& error) {
        std::cerr << "[orbbec] pipeline.start SDK error: " << error.getMessage() << "\n" << std::flush;
    } catch (const std::exception& error) {
        std::cerr << "[orbbec] pipeline.start error: " << error.what() << "\n" << std::flush;
    } catch (...) {
        std::cerr << "[orbbec] pipeline.start failed (unknown / access violation)\n" << std::flush;
    }
    return false;
}

std::shared_ptr<ob::FrameSet> orbbecSafeWaitForFrameset(ob::Pipeline& pipeline, uint32_t timeoutMs) {
    try {
        return pipeline.waitForFrameset(timeoutMs);
    } catch (const ob::Error& error) {
        std::cerr << "[orbbec] waitForFrameset SDK error: " << error.getMessage() << "\n" << std::flush;
    } catch (const std::exception& error) {
        std::cerr << "[orbbec] waitForFrameset error: " << error.what() << "\n" << std::flush;
    } catch (...) {
        std::cerr << "[orbbec] waitForFrameset failed (unknown / access violation)\n" << std::flush;
    }
    return nullptr;
}

std::shared_ptr<ob::Frame> orbbecSafeFilterProcess(ob::Filter& filter, const std::shared_ptr<ob::Frame>& input) {
    if (!input) {
        return nullptr;
    }
    try {
        return filter.process(input);
    } catch (const ob::Error& error) {
        std::cerr << "[orbbec] filter.process SDK error: " << error.getMessage() << "\n" << std::flush;
    } catch (const std::exception& error) {
        std::cerr << "[orbbec] filter.process error: " << error.what() << "\n" << std::flush;
    } catch (...) {
        std::cerr << "[orbbec] filter.process failed (unknown / access violation)\n" << std::flush;
    }
    return nullptr;
}
