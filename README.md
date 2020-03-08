# EE134
Repository for EE134 Code

## Build Status
[![wwerst](https://circleci.com/gh/wwerst/EE134.svg?style=svg)](https://circleci.com/gh/wwerst/EE134)

## Air Hockey robot (hockbot)

Code for air hockey robot is in hockbot catkin package 

## Installation

### OpenCV with GPU

First, uninstall previous versions of OpenCV.
Then, in a new folder (e.g. ~/src_installs):
```
git clone <opencv_repo> -b 3.2.0
git clone <opencv_contrib_repo> -b 3.2.0
cd <opencv_repo>
mkdir build
cd build
```
Then, if you are running CUDA version 9.0 and up, modify 
the OpenCV build config according to https://stackoverflow.com/questions/46584000/cmake-error-variables-are-set-to-notfound
Then, execute build:
```
cmake -D CMAKE_BUILD_TYPE=RELEASE \-D CMAKE_INSTALL_PREFIX=/usr/local \-D INSTALL_C_EXAMPLES=ON \-D INSTALL_PYTHON_EXAMPLES=ON \-D WITH_TBB=ON \-D WITH_V4L=ON \-D WITH_QT=ON \-D WITH_OPENGL=ON \-D WITH_CUDA=ON \-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \-D BUILD_EXAMPLES=ON -D ENABLE_PRECOMPILED_HEADERS=OFF ..

make -j6
sudo checkinstall make install
```