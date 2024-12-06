cmake_minimum_required(VERSION 3.13)
project(sdaudio_sink)

file(GLOB SRC "src/*.cpp")

include(${SDRPP_MODULE_CMAKE})

set(SPECBLEACH_ROOT "${CMAKE_SOURCE_DIR}/deps/specbleach")

if (MSVC)
    target_link_directories(sdaudio_sink PRIVATE 
        "C:/Program Files (x86)/RtAudio/lib"
        "${SPECBLEACH_ROOT}/lib"
    )

    target_include_directories(sdaudio_sink PRIVATE 
        "C:/Program Files (x86)/RtAudio/include/rtaudio"
        "${SPECBLEACH_ROOT}/include"
    )

    target_link_libraries(sdaudio_sink PRIVATE 
        rtaudio
        libspecbleach
    )
else()
    find_package(PkgConfig)
    pkg_check_modules(RTAUDIO REQUIRED rtaudio)

    find_library(SPECBLEACH_LIBRARY
        NAMES specbleach libspecbleach
        PATHS "${SPECBLEACH_ROOT}/lib" "/usr/local/lib"
    )

    if(NOT SPECBLEACH_LIBRARY)
        message(FATAL_ERROR "SpecBleach library not found!")
    endif()

    find_path(SPECBLEACH_INCLUDE_DIR
        NAMES specbleach_denoiser.h
        PATHS "${SPECBLEACH_ROOT}/include" "/usr/local/include"
    )

    if(NOT SPECBLEACH_INCLUDE_DIR)
        message(FATAL_ERROR "SpecBleach headers not found!")
    endif()

    target_include_directories(sdaudio_sink PRIVATE 
        ${RTAUDIO_INCLUDE_DIRS}
        ${SPECBLEACH_INCLUDE_DIR}
    )

    target_link_directories(sdaudio_sink PRIVATE 
        ${RTAUDIO_LIBRARY_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/deps/specbleach/lib
    )

    target_link_libraries(sdaudio_sink PRIVATE 
        ${RTAUDIO_LIBRARIES}
        ${SPECBLEACH_LIBRARY}
    )

    message(STATUS "Found SpecBleach: ${SPECBLEACH_LIBRARY}")
    message(STATUS "SpecBleach include: ${SPECBLEACH_INCLUDE_DIR}")
endif()