cmake_minimum_required(VERSION 3.13)
project(libsddc VERSION 1.0.0 LANGUAGES CXX C)

set(SDDC_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

set(CPPUNIT_HEADER "${SDDC_ROOT_DIR}/libsddc_shared/unittest/LIBCPPUNIT-prefix/src/LIBCPPUNIT/CppUnitTestFramework.hpp")

function(add_include_to_file FILE_PATH)
    if(EXISTS ${FILE_PATH})
        file(READ ${FILE_PATH} CONTENT)
        string(FIND "${CONTENT}" "#include <cstdint>" HAS_INCLUDE)
        if(${HAS_INCLUDE} EQUAL -1)
            string(REGEX REPLACE "(#include <optional>)" "\\1\n#include <cstdint>" MODIFIED_CONTENT "${CONTENT}")
            file(WRITE ${FILE_PATH} "${MODIFIED_CONTENT}")
        endif()
    endif()
endfunction()


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


if(EXISTS ${CPPUNIT_HEADER})
    add_include_to_file(${CPPUNIT_HEADER})
endif()

set(FIRMWARE_IMAGE ${SDDC_ROOT_DIR}/SDDC_FX3.img)
set(FIRMWARE_HEADER ${CMAKE_CURRENT_BINARY_DIR}/firmware.h)
if(EXISTS ${FIRMWARE_IMAGE})
    bin2h(${FIRMWARE_IMAGE} ${FIRMWARE_HEADER})
endif()

set(LIBSDDC_SOURCES
    ${SDDC_ROOT_DIR}/libsddc/libsddc.cpp
    ${SDDC_ROOT_DIR}/libsddc/wavewrite.c
)

set(CORE_SOURCES
    ${SDDC_ROOT_DIR}/Core/RadioHandler.cpp
    ${SDDC_ROOT_DIR}/Core/PScope_uti.cpp
    ${SDDC_ROOT_DIR}/Core/config.cpp
    ${SDDC_ROOT_DIR}/Core/fft_mt_r2iq.cpp
    ${SDDC_ROOT_DIR}/Core/fft_mt_r2iq_avx.cpp
    ${SDDC_ROOT_DIR}/Core/fft_mt_r2iq_avx2.cpp
    ${SDDC_ROOT_DIR}/Core/fft_mt_r2iq_avx512.cpp
    ${SDDC_ROOT_DIR}/Core/fft_mt_r2iq_def.cpp
    ${SDDC_ROOT_DIR}/Core/fft_mt_r2iq_neon.cpp
    ${SDDC_ROOT_DIR}/Core/fir.cpp
    ${SDDC_ROOT_DIR}/Core/pffft/pf_mixer.cpp
    ${SDDC_ROOT_DIR}/Core/radio/BBRF103Radio.cpp
    ${SDDC_ROOT_DIR}/Core/radio/HF103Radio.cpp
    ${SDDC_ROOT_DIR}/Core/radio/RX888R2Radio.cpp
    ${SDDC_ROOT_DIR}/Core/radio/RX888R3Radio.cpp
    ${SDDC_ROOT_DIR}/Core/radio/RX888Radio.cpp
    ${SDDC_ROOT_DIR}/Core/radio/RX999Radio.cpp
    ${SDDC_ROOT_DIR}/Core/radio/RXLucy.cpp
    ${SDDC_ROOT_DIR}/Core/radio/RadioHardware.cpp
)

if(WIN32)
    list(APPEND CORE_SOURCES
        ${SDDC_ROOT_DIR}/Core/arch/win32/FX3handler.cpp
        ${SDDC_ROOT_DIR}/Core/arch/win32/CyAPI/CyAPI.cpp
    )
else()
    list(APPEND CORE_SOURCES
        ${SDDC_ROOT_DIR}/Core/arch/linux/FX3handler.cpp
        ${SDDC_ROOT_DIR}/Core/arch/linux/ezusb.c
        ${SDDC_ROOT_DIR}/Core/arch/linux/logging.c
        ${SDDC_ROOT_DIR}/Core/arch/linux/streaming.c
        ${SDDC_ROOT_DIR}/Core/arch/linux/usb_device.c
    )

    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
endif()

add_library(sddc SHARED 
    ${LIBSDDC_SOURCES}
    ${CORE_SOURCES}
)

set_target_properties(sddc PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER "${SDDC_ROOT_DIR}/libsddc/libsddc.h"
)

target_include_directories(sddc
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${SDDC_ROOT_DIR}/libsddc>
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
        ${SDDC_ROOT_DIR}/Core
        ${SDDC_ROOT_DIR}/Core/pffft
        ${SDDC_ROOT_DIR}/Core/dsp
        ${LIBUSB_INCLUDE_DIRS}
)

if(NOT WIN32)
    target_link_libraries(sddc PRIVATE ${LIBUSB_LIBRARIES})
endif()

if(MSVC)
    target_compile_options(sddc PRIVATE /O2 /Ob2 /std:c++17 /EHsc)
else()
    target_compile_options(sddc PRIVATE -O3 -fPIC)
endif()

include(GNUInstallDirs)
install(TARGETS sddc
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/libsddc.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/libsddc.pc
    @ONLY
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libsddc.pc
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)
