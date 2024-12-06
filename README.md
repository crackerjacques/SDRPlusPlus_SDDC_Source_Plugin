# SDR++ SDDC Source Plugin
RX888 MK2 for SDR++ Plugin(WIP)

and my study a.k.a garbage plugins for SDR++
Whenever I feel like it, I maintain it or add new item and features.

# WIP.
Because I don't have an RX 888 in my house.
Therefore, I do not know if it will work.
And I don't have enough pocket money to order at the moment.

# Current Plug-ins

**SDDC Plugin** - Includes ExtIO and works as a stand-alone.

**Use SDDC Plugin2** - Refer to libsddc.so.

**FX AUDIO Sink Plugin** -Up to 4 VST plug-in chain can be inserted into RTAudio's audio sink.

**NRAudio Sink Plugin** - Improves signal-to-noise ratio using AI noise reduction RNNnoise. The current default model is ok for speech, but not suitable for listening to music.

**SDAudio Sink Plugin**  - Old-school Noise reduction. Spectral denoise, filter, gate.

# HOWTO

Preparing
```
git clone https://github.com/AlexandreRouma/SDRPlusPlus.git
git clone https://github.com/crackerjacques/SDRPlusPlus_SDDC_Source_Plugin.git
git clone https://github.com/ik1xpv/ExtIO_sddc.git
cp -r SDRPlusPlus_SDDC_Source_Plugin/sddc_source SDRPlusPlus/source_modules
cp -r ExtIO_sddc SDRPlusPlus/source_modules/sddc_source/3rdparty

nano SDRPlusPlus/CMakeLists.txt

```
put those line to CMakeLists.txt in SDRpp root dir

```
option(OPT_BUILD_SDDC_SOURCE "Build SDDC Source Module (Dependencies: libsddc)" OFF)
```
and

```
if (OPT_BUILD_SDDC_SOURCE)
    add_subdirectory("source_modules/sddc_source")
endif (OPT_BUILD_SDDC_SOURCE)
```

Then

```
cd SDRPlusPlus
mkdir build ; cd build

cmake .. -DOPT_BUILD_SDDC_SOURCE=ON
make
sudo make install
or

ccmake ..
make
sudo make install

```

I was able to confirm that it was compiled and displayed in the menu.
I'll say it twice because it's important.
I don't know if it will work yet.
