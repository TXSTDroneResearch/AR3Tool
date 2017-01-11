#!/usr/bin/env bash

CUR_DIR=$PWD
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
LIB_ROOT="$SCRIPT_DIR/../libs"

sudo apt-get -y install autoconf automake build-essential libass-dev libfreetype6-dev \
  libsdl1.2-dev libtheora-dev libtool libva-dev libvdpau-dev libvorbis-dev libxcb1-dev libxcb-shm0-dev \
  libxcb-xfixes0-dev pkg-config texinfo yasm zlib1g-dev
	
# Devtools
sudo apt-get -y install git

cd $LIB_ROOT
if [ ! -d ffmpeg ]; then
  git clone git://source.ffmpeg.org/ffmpeg.git
  cd ffmpeg
  git apply "$SCRIPT_DIR/ffmpeg.diff"
else
  cd ffmpeg
  git pull
fi

mkdir -p build
mkdir -p bin
# ./configure --prefix="$LIB_ROOT/ffmpeg/build" --pkg-config-flags="--static" --extra-cflags="-fPIC -m64" --enable-pic --extra-ldexeflags=-pie --enable-shared
./configure --prefix="/usr/local" --bindir="/usr/local/bin" \
  --extra-cflags="-I/usr/local/include" \
  --extra-ldflags="-L/usr/local/lib" \
  --pkg-config-flags="--static"
make -j
sudo make install

cd $CUR_DIR