# EVP Detector 

I was watching a Discovery Channel ghost hunt show the other day.
The gadget the performers were using to record ghost voices appeared to be a frequency sweep, or random access to the shortwave band from what I could see, so I made one anyway.

The running results are very similar to what they have.

Since I live in Japan, most of the broadcasts on the shortwave band are music and language channels from neighboring countries and North Korean propaganda broadcasts, so I regret that they do not sound as ghostly as North American broadcasts.

# TIPS

10-25 ms for sweeps
100-250 ms for random

...seems like a good interval for bluffing ghost hunter program.

# HOWTO

```
cp -r SDRPlusPlus_SDDC_Source_Plugin/evp_detector SDRPlusPlus/source_modules

```

and put those line to SDRPlusPlus/CMakeLists.txt

```
option(OPT_BUILD_EVP_DETECTOR "Build EVP Detector Module" ON)
```
and

```
if (OPT_BUILD_EVP_DETECTOR)
add_subdirectory("source_modules/evp_detector")
endif (OPT_BUILD_EVP_DETECTOR)
```

then

```
mkdir build ; cd build
cmake .. -DOPT_BUILD_EVP_DETECTOR=ON
make 
sudo make install
```
