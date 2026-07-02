#include <kvazaar.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

#ifdef _WIN32
#include <windows.h>

LONG WINAPI smokeExceptionFilter(EXCEPTION_POINTERS* info) {
    std::cerr << "Kvazaar smoke test: unhandled exception 0x" << std::hex
              << (info && info->ExceptionRecord ? info->ExceptionRecord->ExceptionCode : 0) << std::dec << "\n";
    return EXCEPTION_EXECUTE_HANDLER;
}
#endif

int main() {
#ifdef _WIN32
    SetUnhandledExceptionFilter(smokeExceptionFilter);
#endif

    const kvz_api* api = kvz_api_get(8);
    kvz_config* config = api->config_alloc();
    api->config_init(config);
    api->config_parse(config, "width", "256");
    api->config_parse(config, "height", "256");
    api->config_parse(config, "lossless", "1");
    api->config_parse(config, "input-format", "P420");
    api->config_parse(config, "period", "1");
    api->config_parse(config, "gop", "0");
    api->config_parse(config, "preset", "ultrafast");
    api->config_parse(config, "threads", "0");
    api->config_parse(config, "owf", "1");
    api->config_parse(config, "wpp", "0");
    api->config_parse(config, "enable-logging", "0");

    kvz_encoder* enc = api->encoder_open(config);
    if (!enc) {
        std::cerr << "encoder_open failed\n";
        return 1;
    }

    kvz_picture* pic = api->picture_alloc(256, 256);
    const size_t lumaSize = 256U * 256U;
    std::vector<uint8_t> yuv(lumaSize + (lumaSize >> 1U), 128);
    std::memcpy(pic->y, yuv.data(), lumaSize);
    std::memcpy(pic->u, yuv.data() + lumaSize, lumaSize >> 2U);
    std::memcpy(pic->v, yuv.data() + lumaSize + (lumaSize >> 2U), lumaSize >> 2U);

    kvz_data_chunk* chunks = nullptr;
    uint32_t len = 0;
    if (!api->encoder_encode(enc, pic, &chunks, &len, nullptr, nullptr, nullptr)) {
        std::cerr << "encoder_encode failed\n";
        return 2;
    }
    if (!api->encoder_encode(enc, nullptr, &chunks, &len, nullptr, nullptr, nullptr) || len == 0) {
        std::cerr << "encoder flush failed\n";
        return 3;
    }

    if (chunks) {
        api->chunk_free(chunks);
    }
    api->picture_free(pic);
    api->encoder_close(enc);
    api->config_destroy(config);

    std::cout << "Kvazaar smoke test OK (len=" << len << ")\n";
    return 0;
}
