cmake_minimum_required(VERSION 3.13)
project(evp_detector)

file(GLOB_RECURSE SRC "src/*.cpp")

add_library(evp_detector SHARED ${SRC})
target_link_libraries(evp_detector PRIVATE sdrpp_core)
set_target_properties(evp_detector PROPERTIES PREFIX "")

# Core include paths
target_include_directories(evp_detector PRIVATE "src/")
target_include_directories(evp_detector PRIVATE "${SDRPP_CORE_ROOT}")
target_include_directories(evp_detector PRIVATE "${CMAKE_SOURCE_DIR}")
target_include_directories(evp_detector PRIVATE "${CMAKE_SOURCE_DIR}/core/src")

if (MSVC)
    target_compile_options(evp_detector PRIVATE ${SDRPP_MODULE_COMPILER_FLAGS})
else()
    target_compile_options(evp_detector PRIVATE ${SDRPP_MODULE_COMPILER_FLAGS})
endif()

# Install directives
if (MSVC)
    install(TARGETS evp_detector DESTINATION "lib/sdrpp/plugins")
else()
    install(TARGETS evp_detector DESTINATION "lib/sdrpp/plugins")
endif()