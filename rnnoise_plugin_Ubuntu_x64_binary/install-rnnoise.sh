#!/bin/bash

# deps via apt
sudo apt-get update
sudo apt-get install -y \
    librtaudio-dev \
    libvolk-dev \
    libjack-dev \
    libpulse-dev \
    libasound2-dev \
    libglfw3-dev \
    libfftw3-dev \
    libzstd-dev \
    liborc-0.4-dev \
    libdb5.3-dev \
    libdbus-1-dev \
    libx11-dev \
    libsndfile1-dev \
    libflac-dev \
    libvorbis-dev \
    libopus-dev \
    libogg-dev \
    libmpg123-dev \
    libmp3lame-dev

# rnnoise
if [ ! -f "/usr/local/lib/librnnoise.so.0" ]; then
    echo "Installing rnnoise from source..."
    git clone https://github.com/xiph/rnnoise.git
    cd rnnoise
    ./autogen.sh
    ./configure
    make -j$(nproc)
    sudo make install
    sudo ldconfig
    cd ..
    rm -rf rnnoise
fi

echo "Dependencies installation completed."
