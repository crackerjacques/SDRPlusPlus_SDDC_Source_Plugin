cmake_minimum_required(VERSION 3.13)
project(ExtIO_sddc)

# Core library components
add_library(SDDC_CORE STATIC
    Core/RadioHandler.cpp
    Core/PScope_uti.cpp
    Core/config.cpp
    Core/fft_mt_r2iq.cpp
    Core/fft_mt_r2iq_avx.cpp
    Core/fft_mt_r2iq_avx2.cpp
    Core/fft_mt_r2iq_avx512.cpp
    Core/fft_mt_r2iq_def.cpp
    Core/fft_mt_r2iq_neon.cpp
    Core/fir.cpp
    Core/pffft/pf_mixer.cpp
)

# Platform specific sources
if(WIN32)
    target_sources(SDDC_CORE PRIVATE
        Core/arch/win32/FX3handler.cpp
        Core/arch/win32/CyAPI/CyAPI.cpp
    )
else()
    target_sources(SDDC_CORE PRIVATE
        Core/arch/linux/FX3handler.cpp
        Core/arch/linux/ezusb.c
        Core/arch/linux/logging.c
        Core/arch/linux/streaming.c
        Core/arch/linux/usb_device.c
    )
endif()

# Include directories
target_include_directories(SDDC_CORE PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/pffft
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/dsp
)

# Platform specific settings
if(WIN32)
    target_include_directories(SDDC_CORE PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/arch/win32
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/arch/win32/CyAPI
    )
else()
    target_include_directories(SDDC_CORE PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/arch/linux
        ${LIBUSB_INCLUDE_DIRS}
    )
    target_link_libraries(SDDC_CORE PUBLIC ${LIBUSB_LIBRARIES})
endif()

# Compiler specific settings
if(MSVC)
    target_compile_options(SDDC_CORE PRIVATE /O2 /Ob2 /std:c++17 /EHsc)
else()
    target_compile_options(SDDC_CORE PRIVATE -O3)
endif()
