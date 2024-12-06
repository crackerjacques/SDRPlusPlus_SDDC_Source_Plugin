# Spectral Denoise Audio Sink

De-noise Audio with old-school method.


It works in the following order.
Spectral Denoise -> Filtre -> Gate

## Prepare
```
  git clone https://github.com/lucianodato/noise-repellent.git
  cd noise-repellent
  meson build --buildtype=release --prefix=/usr/local --libdir=/usr/local/lib
  meson compile -C build -v
  sudo meson install -C build
```
**Copy this dir to your SDRPP**
```
cp -r sdaudio_sink SDRPlusPlus/sink_modules
```
**Add those line to your CMakeLists in SDRPP root directory**

```
option(OPT_BUILD_SDAUDIO_SINK "Build Audio Sink with Noise Reduction Module (Dependencies: SD)" ON)
```

**and**

```
if (OPT_BUILD_SDAUDIO_SINK)
    add_subdirectory("sink_modules/sdaudio_sink")
endif (OPT_BUILD_SDAUDIO_SINK)
```

**Then move to build dir.**

```
cd SDRPlusPlus/build
cmake .. -DOPT_BUILD_SDAUDIO_SINK=ON
make 
sudo make install
```
