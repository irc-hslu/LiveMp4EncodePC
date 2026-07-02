cmake_minimum_required(VERSION 3.25)

project(VpcCLiveStreamer
    VERSION 1.0.0
    DESCRIPTION "Orbbec capture -> uvgVPCCenc -> V3C RTP live streaming pipeline"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(UVGVPCCENC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/uvgVPCCenc" CACHE PATH "Path to uvgVPCCenc source tree")
set(UVGV3CRTP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/uvgV3CRTP" CACHE PATH "Path to uvgV3CRTP source tree")
set(ORBBEC_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/orbbecSDK" CACHE PATH "Path to Orbbec SDK (libobsensor)")

option(VPCCLIVE_ENABLE_V3CRTP "Enable V3C RTP streaming via uvgV3CRTP" ON)
option(VPCCLIVE_ENABLE_ORBBEC "Link Orbbec SDK when ORBBEC_SDK_DIR is set" ON)

if(NOT EXISTS "${UVGVPCCENC_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR "uvgVPCCenc not found at ${UVGVPCCENC_ROOT}")
endif()

set(ENABLE_V3CRTP ${VPCCLIVE_ENABLE_V3CRTP} CACHE BOOL "" FORCE)
if(MSVC)
    # Shared uvgutils.dll does not produce a usable import .lib for vpcclive_streamer on MSVC.
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "Static-link uvgVPCCenc dependencies on Windows" FORCE)
endif()
add_subdirectory("${UVGVPCCENC_ROOT}" "${CMAKE_BINARY_DIR}/uvgVPCCenc")

if(VPCCLIVE_ENABLE_V3CRTP AND NOT TARGET uvgv3crtp)
    message(FATAL_ERROR "uvgv3crtp target missing. Build uvgVPCCenc with ENABLE_V3CRTP=ON.")
endif()

add_executable(vpcclive_streamer
    src/winfix/main.cpp
    src/winfix/orbbec_safe.cpp
)

target_include_directories(vpcclive_streamer PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/winfix"
    "${UVGVPCCENC_ROOT}/src/libuvgvpccenc/include"
    "${UVGVPCCENC_ROOT}/src/libuvgformat/include"
    "${UVGVPCCENC_ROOT}/src/libuvgutils/include"
)

target_link_libraries(vpcclive_streamer PRIVATE
    uvgvpccenc
    uvgutils
    mapEncodingLibrary
    kvazaar_stdio
)

if(VPCCLIVE_ENABLE_V3CRTP)
    target_compile_definitions(vpcclive_streamer PRIVATE ENABLE_V3CRTP=1 NOMINMAX WIN32_LEAN_AND_MEAN KVZ_STATIC_LIB)
    target_link_libraries(vpcclive_streamer PRIVATE uvgv3crtp)
    target_include_directories(vpcclive_streamer PRIVATE "${UVGV3CRTP_ROOT}/include")
elseif(MSVC)
    target_compile_definitions(vpcclive_streamer PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

if(VPCCLIVE_ENABLE_ORBBEC AND ORBBEC_SDK_DIR)
    find_path(ORBBEC_INCLUDE_DIR
        NAMES libobsensor/ObSensor.hpp
        PATHS "${ORBBEC_SDK_DIR}/include" "${ORBBEC_SDK_DIR}/include/libobsensor"
        NO_DEFAULT_PATH)

    find_library(ORBBEC_LIBRARY
        NAMES OrbbecSDK obsensor
        PATHS "${ORBBEC_SDK_DIR}/lib" "${ORBBEC_SDK_DIR}/lib/x64"
        NO_DEFAULT_PATH)

    if(ORBBEC_INCLUDE_DIR AND ORBBEC_LIBRARY)
        message(STATUS "Orbbec SDK found: ${ORBBEC_LIBRARY}")
        target_compile_definitions(vpcclive_streamer PRIVATE ORBBEC_SDK_FOUND=1)
        target_include_directories(vpcclive_streamer PRIVATE "${ORBBEC_INCLUDE_DIR}")
        target_link_libraries(vpcclive_streamer PRIVATE "${ORBBEC_LIBRARY}")
    else()
        message(WARNING "ORBBEC_SDK_DIR set but libobsensor was not found. Building synthetic capture fallback.")
    endif()
else()
    message(STATUS "Orbbec SDK disabled or not configured. Building synthetic capture fallback.")
endif()

if(MSVC)
    # /EHa: catch(...) in capture code can handle Orbbec SDK access violations on Windows.
    target_compile_options(vpcclive_streamer PRIVATE /W4 /Zc:__cplusplus /EHa /FI"${CMAKE_CURRENT_SOURCE_DIR}/src/WinPreamble.h")
else()
    target_compile_options(vpcclive_streamer PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(VPCCLIVE_ENABLE_V3CRTP)
    add_executable(v3c_smoke_test src/winfix/v3c_smoke_test.cpp)
    target_link_libraries(v3c_smoke_test PRIVATE uvgv3crtp)
    target_include_directories(v3c_smoke_test PRIVATE "${UVGV3CRTP_ROOT}/include")
    if(MSVC)
        target_compile_definitions(v3c_smoke_test PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    endif()

    add_executable(kvazaar_smoke_test src/winfix/kvazaar_smoke_test.cpp)
    target_link_libraries(kvazaar_smoke_test PRIVATE mapEncodingLibrary kvazaar_stdio)
    target_include_directories(kvazaar_smoke_test PRIVATE
        "${UVGVPCCENC_ROOT}/src/libuvgvpccenc/mapEncoding"
        "${CMAKE_BINARY_DIR}/uvgVPCCenc/src/libuvgvpccenc/mapEncoding/include")
    if(MSVC)
        target_compile_definitions(kvazaar_smoke_test PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN KVZ_STATIC_LIB)
        target_compile_options(kvazaar_smoke_test PRIVATE /FI"${CMAKE_CURRENT_SOURCE_DIR}/src/WinPreamble.h")
    endif()

    add_executable(v3c_replay src/winfix/v3c_replay.cpp)
    target_link_libraries(v3c_replay PRIVATE uvgv3crtp)
    target_include_directories(v3c_replay PRIVATE "${UVGV3CRTP_ROOT}/include")
    if(MSVC)
        target_compile_definitions(v3c_replay PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
        target_compile_options(v3c_replay PRIVATE /FI"${CMAKE_CURRENT_SOURCE_DIR}/src/WinPreamble.h")
    endif()
endif()

install(TARGETS vpcclive_streamer RUNTIME DESTINATION bin)
if(VPCCLIVE_ENABLE_V3CRTP)
    install(TARGETS v3c_replay RUNTIME DESTINATION bin)
endif()
