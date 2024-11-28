cmake_minimum_required(VERSION 3.13)
project(sddc2_source)

file(GLOB SRC "src/*.cpp")

include(${SDRPP_MODULE_CMAKE})

if (MSVC)
    # Windowsの場合、インストールされたlibsddcのパスを指定
    target_link_directories(sddc2_source PRIVATE "C:/Program Files/PothosSDR/bin/")
    target_link_libraries(sddc2_source PRIVATE sddc)
elseif (ANDROID)
    # Androidの場合の設定
    target_include_directories(sddc2_source PUBLIC
        /sdr-kit/${ANDROID_ABI}/include/libsddc
    )

    target_link_libraries(sddc2_source PUBLIC
        /sdr-kit/${ANDROID_ABI}/lib/libusb1.0.so
        /sdr-kit/${ANDROID_ABI}/lib/libsddc.so
    )
else (MSVC)
    # Linux/macOSの場合、pkg-configを使用
    find_package(PkgConfig)
    pkg_check_modules(LIBSDDC REQUIRED libsddc)

    target_include_directories(sddc2_source PRIVATE ${LIBSDDC_INCLUDE_DIRS})
    target_link_directories(sddc2_source PRIVATE ${LIBSDDC_LIBRARY_DIRS})
    target_link_libraries(sddc2_source PRIVATE ${LIBSDDC_LIBRARIES})
endif ()