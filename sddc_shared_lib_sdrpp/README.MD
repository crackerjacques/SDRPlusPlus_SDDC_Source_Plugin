# SDRPP SDDC Source Plugin with shared library

First build libsddc.

```
git clone https://github.com/ik1xpv/ExtIO_sddc.git

cp -r sddc_shared_lib_sdrpp/libsddc_shared ExtIO_sddc

cd ExtIO_sddc/libsddc_shared
mkdir build
cd build
make 
sudo make install
sudo ldconfig
```
Then copy sddc2_source to sdrpp/source_modules

```
cp -r  sddc_shared_lib_sdrpp/sddc2_source SDRPlusPLus/source_modules
```

Add the following two Lines to CMakeLists.txt in the root of SDRPP

```
option(OPT_BUILD_SDDC2_SOURCE "Build SDDC Source Module (Dependencies: libsddc)" OFF)
```
and
```
if (OPT_BUILD_SDDC2_SOURCE)
    add_subdirectory("source_modules/sddc2_source")
endif (OPT_BUILD_SDDC2_SOURCE)
```

Then move to build dir,
```
cd build
cmake .. -DOPT_BUILD_SDDC2_SOURCE=ON
make 
sudo make install

```

