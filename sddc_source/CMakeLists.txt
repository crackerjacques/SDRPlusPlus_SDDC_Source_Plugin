cmake_minimum_required(VERSION 3.13)
project(sddc_source)

file(GLOB SRC "src/*.cpp")

# SDR++のコアヘッダーの場所を明示的に設定
set(SDRPP_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/core/src
    ${CMAKE_SOURCE_DIR}
)

# ファームウェアヘッダーの生成関数
function(bin2h INPUT_FILE OUTPUT_FILE)
    if(EXISTS ${INPUT_FILE})
        file(READ ${INPUT_FILE} CONTENT HEX)
        string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," OUTPUT ${CONTENT})
        file(WRITE ${OUTPUT_FILE}
            "#ifdef __cplusplus\n"
            "extern \"C\" {\n"
            "#endif\n"
            "static const unsigned char FIRMWARE[] = {${OUTPUT}0x00};\n"
            "static const unsigned int firmware_size = sizeof(FIRMWARE);\n"
            "#ifdef __cplusplus\n"
            "}\n"
            "#endif\n"
        )
    endif()
endfunction()

include(${SDRPP_MODULE_CMAKE})

if(NOT WIN32)
    find_package(PkgConfig)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
endif()

# ファームウェアヘッダーの生成
set(FIRMWARE_IMAGE ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/SDDC_FX3.img)
set(FIRMWARE_HEADER ${CMAKE_CURRENT_BINARY_DIR}/firmware.h)
if(EXISTS ${FIRMWARE_IMAGE})
    bin2h(${FIRMWARE_IMAGE} ${FIRMWARE_HEADER})
else()
    message(FATAL_ERROR "Firmware image not found at: ${FIRMWARE_IMAGE}")
endif()

# Core library components
set(CORE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/RadioHandler.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/PScope_uti.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/config.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/fft_mt_r2iq.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/fft_mt_r2iq_avx.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/fft_mt_r2iq_avx2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/fft_mt_r2iq_avx512.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/fft_mt_r2iq_def.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/fft_mt_r2iq_neon.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/fir.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/pffft/pf_mixer.cpp
)

# Add radio hardware files
set(RADIO_HARDWARE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/radio/BBRF103Radio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/radio/HF103Radio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/radio/RX888R2Radio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/radio/RX888R3Radio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/radio/RX888Radio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/radio/RX999Radio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/radio/RXLucy.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/radio/RadioHardware.cpp
)

if(WIN32)
    list(APPEND CORE_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/arch/win32/FX3handler.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/arch/win32/CyAPI/CyAPI.cpp
    )
else()
    list(APPEND CORE_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/arch/linux/FX3handler.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/arch/linux/ezusb.c
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/arch/linux/logging.c
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/arch/linux/streaming.c
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/arch/linux/usb_device.c
    )
endif()

# Add all sources to sddc_source target
target_sources(sddc_source PRIVATE
    ${SRC}
    ${CORE_SOURCES}
    ${RADIO_HARDWARE_FILES}
)

# Include directories
target_include_directories(sddc_source PRIVATE
    ${SDRPP_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/core/src
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/pffft
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/dsp
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ExtIO_sddc/Core/arch/linux
    ${CMAKE_CURRENT_BINARY_DIR}  # firmware.h用
)

if(NOT WIN32)
    target_include_directories(sddc_source PRIVATE ${LIBUSB_INCLUDE_DIRS})
    target_link_libraries(sddc_source PRIVATE ${LIBUSB_LIBRARIES})
endif()

target_link_libraries(sddc_source PRIVATE sdrpp_core)

if(MSVC)
    target_compile_options(sddc_source PRIVATE /O2 /Ob2 /std:c++17 /EHsc)
else()
    target_compile_options(sddc_source PRIVATE -O3)
endif()