# RTAudio Sink with AI-BASED Noise Reduction for SDR++

Maybe useful dig weak signal and Communication with a guy who lives in a noisy environment.

# Prepare
```
git clone https://github.com/xiph/rnnoise.git
cd rnnoise
./configure
make
sudo make install
sudo ldconfig
```

# Build

Put those line to CMakeLists.txt SDRPP root dir.
```
option(OPT_BUILD_NRAUDIO_SINK "Build Audio Sink with Noise Reduction Module (Dependencies: rnnnoise)" OFF)

```
and

```
if (OPT_BUILD_NRAUDIO_SINK)
    add_subdirectory("sink_modules/nraudio_sink")
endif (OPT_BUILD_NRAUDIO_SINK)
```

Copy this dir to your SDRPP sink modules.
```
cp -r nraudio_sink SDRPlusPlus/sink_modules
```

move to SDRPP build dir
```
cd SDRPlusPlus/build
cmake .. -DOPT_BUILD_NRAUDIO_SINK=ON
make 
sudo make install
```
