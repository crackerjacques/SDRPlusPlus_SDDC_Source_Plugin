cmake_minimum_required(VERSION 3.13)
project(nraudio_sink)
file(GLOB SRC "src/*.cpp")

include(${SDRPP_MODULE_CMAKE})

if (MSVC)
    # Windows specific settings
    target_link_directories(nraudio_sink PRIVATE "C:/Program Files (x86)/RtAudio/lib")
    target_include_directories(nraudio_sink PRIVATE "C:/Program Files (x86)/RtAudio/include/rtaudio")
    target_link_libraries(nraudio_sink PRIVATE rtaudio)
else (MSVC)
    # Linux/Unix settings
    find_package(PkgConfig)
    
    # RtAudio
    pkg_check_modules(RTAUDIO REQUIRED rtaudio)
    target_include_directories(nraudio_sink PRIVATE ${RTAUDIO_INCLUDE_DIRS})
    target_link_directories(nraudio_sink PRIVATE ${RTAUDIO_LIBRARY_DIRS})
    target_link_libraries(nraudio_sink PRIVATE ${RTAUDIO_LIBRARIES})

    # RNNoise
    pkg_check_modules(RNNOISE REQUIRED rnnoise)
    target_include_directories(nraudio_sink PRIVATE ${RNNOISE_INCLUDE_DIRS})
    target_link_directories(nraudio_sink PRIVATE ${RNNOISE_LIBRARY_DIRS})
    target_link_libraries(nraudio_sink PRIVATE ${RNNOISE_LIBRARIES})
    
    # SDR++ Core
    target_include_directories(nraudio_sink PRIVATE ${SDRPP_INCLUDE_DIRS})
    target_link_libraries(nraudio_sink PRIVATE sdrpp_core)
endif ()

target_compile_options(nraudio_sink PRIVATE -O3 -std=c++17)
set_target_properties(nraudio_sink PROPERTIES PREFIX "")
install(TARGETS nraudio_sink DESTINATION "lib/sdrpp/plugins")

# Add models directory to installation
#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models/ 
#        DESTINATION share/sdrpp/rnnoise_models
#        FILES_MATCHING PATTERN "*.bin")