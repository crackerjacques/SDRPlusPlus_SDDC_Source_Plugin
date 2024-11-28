# Audio Sink with VST LV2 Support for SDRpp

# Dep

JUCE Framefork
```
https://github.com/juce-framework/JUCE
```

# HOWTO
copy this dir to your SDRPP sink_modules dir .
```
cp -r SDRPlusPlus_SDDC_Source_Plugin/fxaudio_sink  SDRPlusPlus/sink_modules

```

and edit CMakeLists.txt on SDRPP root dir
add those lines.

```
option(OPT_BUILD_FXAUDIO_SINK "Build fxaudio Sink Module (Dependencies: JUCE)" OFF)
```
and
```
if (OPT_BUILD_FXAUDIO_SINK)
    add_subdirectory("sink_modules/fxaudio_sink")
endif (OPT_BUILD_FXAUDIO_SINK)
```

then do cmake or ccmake in your build dir.

```
cd build
make .. -DOPT_BUILD_FXAUDIO_SINK=ON -DJUCE_DIR=/usr/local/lib/cmake/JUCE* -DVST2_SDK_DIR=$HOME/VST2_SDK
```
or

```
cd build
ccmake ..
```

```
make -j8
sudo make install
```
