cmake_minimum_required(VERSION 3.13)
project(fxaudio_sink)

set(JUCE_DIR "/usr/local/lib/cmake/JUCE*" CACHE PATH "Path to JUCE installation")
set(VST2_SDK_DIR "$ENV{HOME}/VST2_SDK" CACHE PATH "Path to VST2 SDK")

file(GLOB SRC "src/*.cpp")
include(${SDRPP_MODULE_CMAKE})

file(GLOB JUCE_CMAKE_PATHS ${JUCE_DIR})
list(GET JUCE_CMAKE_PATHS 0 FIRST_JUCE_PATH)
if(FIRST_JUCE_PATH)
    set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${FIRST_JUCE_PATH})
endif()

find_package(JUCE QUIET)

if (MSVC)
    # Win
    target_link_directories(fxaudio_sink PRIVATE "C:/Program Files (x86)/RtAudio/lib")
    target_include_directories(fxaudio_sink PRIVATE
        "C:/Program Files (x86)/RtAudio/include/rtaudio"
    )
    target_link_libraries(fxaudio_sink PRIVATE rtaudio)
else()
    # Linux/macOS
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(RTAUDIO REQUIRED rtaudio)
    target_include_directories(fxaudio_sink PRIVATE ${RTAUDIO_INCLUDE_DIRS})
    target_link_directories(fxaudio_sink PRIVATE ${RTAUDIO_LIBRARY_DIRS})
    target_link_libraries(fxaudio_sink PRIVATE ${RTAUDIO_LIBRARIES})
endif()

if(JUCE_FOUND)
    add_definitions(-DUSE_JUCE=1)
    
    target_include_directories(fxaudio_sink PRIVATE
        /usr/local/include/JUCE-7.0.12/modules
        ${VST2_SDK_DIR}
        ${VST2_SDK_DIR}/public.sdk/source/vst2.x
    )

    target_link_libraries(fxaudio_sink PRIVATE
        juce::juce_audio_basics
        juce::juce_audio_processors
        juce::juce_audio_formats
        juce::juce_audio_devices
        juce::juce_core
        juce::juce_data_structures
        juce::juce_events
        juce::juce_gui_basics
        juce::juce_gui_extra
    )

    target_compile_definitions(fxaudio_sink PRIVATE
        JUCE_GLOBAL_MODULE_SETTINGS_INCLUDED=1
        JUCE_VST3_CAN_REPLACE_VST2=0
        JUCE_USE_CURL=0
        JUCE_WEB_BROWSER=0
        JUCE_PLUGINHOST_VST3=1
        JUCE_PLUGINHOST_VST=1
        JUCE_PLUGINHOST_LV2=1
        VST2_SDK_DIR="${VST2_SDK_DIR}"
        JUCE_VST2_PATH="${VST2_SDK_DIR}/public.sdk/source/vst2.x"
    )

    message(STATUS "JUCE found - VST/LV2 support will be enabled")
    message(STATUS "Using VST2 SDK from: ${VST2_SDK_DIR}")
    message(STATUS "VST2 headers path: ${VST2_SDK_DIR}/pluginterfaces/vst2.x")
else()
    message(WARNING "JUCE not found - VST/LV2 support will be disabled")
endif()

if(USE_LV2 AND JUCE_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LILV REQUIRED lilv-0)
    pkg_check_modules(SUIL REQUIRED suil-0)
    
    target_link_libraries(fxaudio_sink PRIVATE
        ${LILV_LIBRARIES}
        ${SUIL_LIBRARIES}
    )
    
    target_include_directories(fxaudio_sink PRIVATE
        ${LILV_INCLUDE_DIRS}
        ${SUIL_INCLUDE_DIRS}
    )
endif()

target_include_directories(fxaudio_sink PRIVATE
    ${SDRPP_INCLUDE_DIRS}
)

target_link_libraries(fxaudio_sink PRIVATE
    sdrpp_core
)

if(MSVC)
    target_compile_options(fxaudio_sink PRIVATE /O2 /Ob2 /std:c++17 /EHsc)
else()
    target_compile_options(fxaudio_sink PRIVATE -O3)
endif()

install(TARGETS fxaudio_sink DESTINATION lib/sdrpp/plugins)