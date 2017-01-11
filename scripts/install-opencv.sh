# KEEP UBUNTU OR DEBIAN UP TO DATE

# sudo apt-get -y update
# sudo apt-get -y upgrade
# sudo apt-get -y dist-upgrade
# sudo apt-get -y autoremove

CUR_DIR=$PWD
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
LIB_ROOT="$SCRIPT_DIR/../libs"

# INSTALL THE DEPENDENCIES
# A billion dependencies
sudo apt-get install -y build-essential cmake qt5-default libvtk6-dev zlib1g-dev libjpeg-dev \
  libwebp-dev libpng-dev libtiff5-dev libjasper-dev libopenexr-dev libgdal-dev libdc1394-22-dev \
  libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev \
  libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev \
  libtbb-dev libeigen3-dev python-dev python-tk python-numpy python3-dev python3-tk python3-numpy \ 
  ant default-jdk doxygen git

# INSTALL THE LIBRARY (YOU CAN CHANGE '3.1.0' FOR THE LAST STABLE VERSION)

sudo apt-get install -y git

cd $LIB_ROOT
if [ ! -d opencv ]; then
  git clone https://github.com/opencv/opencv.git
  cd opencv
else
  cd opencv
  git pull
fi

mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_SHARED_LIBS=ON -DWITH_QT=ON -DWITH_OPENGL=ON \
  -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON -DWITH_XINE=ON \
  -DWITH_FFMPEG=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF \
  ..
make -j
sudo make install

# WSL compatability fix :)
# disable executable stack flag (why would you ever enable this?)
sudo execstack -c /usr/local/lib/*opencv*.so*
sudo ldconfig

cd $CUR_DIR

# EXECUTE SOME OPENCV EXAMPLES AND COMPILE A DEMONSTRATION

# To complete this step, please visit 'http://milq.github.io/install-opencv-ubuntu-debian'.