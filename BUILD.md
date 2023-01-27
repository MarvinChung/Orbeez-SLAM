# How to build this project

## Steps:

- [Requirements](#requirements)
- [Setup the environment](#setup-the-environment)
  - [Docker](#docker)
  - [Local](#local)
- [Clone the repository](#clone-the-repository)
  - [HTTPS](#https)
  - [SSH](#ssh)
- [Uncompress vocabulary](#uncompress-vocabulary)
- [Compilation](#compilation)
- [Test the Program](#test-the-program)
  - [Download the TUM fr3/office Dataset](#download-the-tum-fr3office-dataset)
  - [Execute Program (Monocular Version)](#execute-program-monocular-version)

## Setup the environment

### Docker

If you wish to run our program with docker, you could simply use the image `marvinchung/orbeez-slam:latest` with the prerequisite installed. For detail, please refer to [Docker.md](./Docker.md).

### Local

Using ubuntu 20.04 is recommended and the following tutorial should be operated on ubuntu 20.04.
You may need these packages before building, you can go to the next section and check whether `cmake` is fine.

Higher version of `cmake` is required, install by

```
sudo apt install libssl-dev
git clone https://github.com/Kitware/CMake
cd CMake
./configure
make -j $(nproc)
sudo make install
```

Install `Eigen 3` by

```
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar zxvf eigen-3.4.0.tar.gz
cd eigen-3.4.0/
mkdir build && cd build
cmake ..
make -j $(nproc)
sudo make install
```

Install the latest version of `OpenCV` by

- Method 1: From online

```
ver=4.6.0

sudo apt-get install libgtk2.0-dev libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libpng-dev libjpeg-dev libopenexr-dev libtiff-dev libwebp-dev pkg-config
wget -O opencv.zip https://github.com/opencv/opencv/archive/$ver.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/$ver.zip
unzip opencv.zip
unzip opencv_contrib.zip
mkdir -p build && cd build
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-$ver/modules ../opencv-$ver
make -j $(nproc)
sudo make install
```

- Method 2: From submodule in this repo (We provide the git submodule to opencv-4.5.5)

```
cd ThirdParty/opencv-4.5.5
cmake . -B build
cmake --build build --parallel $(nproc --all)
```

Headless machine may encounter problems when building opencv. Try:

```
cmake -D CMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local -DWITH_TBB=ON -DWITH_V4L=ON -DWITH_QT=ON -DWITH_OPENGL=ON -DBUILD_TIFF=ON . -B build
cmake --build build --parallel $(nproc --all)
```

You can decide whether to install opencv to your machine in your machine. We will link to it even if you do not install it.

```
sudo make install
```

Glew is required for GUI.

```
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libglew-dev
```

## Clone the repository

### HTTPS

Consider creating [personal token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token#creating-a-token) if you wished to clone by HTTPS (useful in docker). Remember to paste the token to somewhere as it will appear only once.

```
git clone --recursive https://github.com/MarvinChung/NerfSlam.git
git submodule update --init --recursive
```

### SSH

```
git clone --recursive git@github.com:MarvinChung/NerfSlam.git
git submodule update --init --recursive
```

## Uncompress vocabulary

```
cd Vocabulary
tar zxf ORBvoc.txt.tar.gz
```

## Compilation

Check the cuda architecture of the device and setup the environment variables accordingly. For 30X0, use 86; for 20X0, use 75.

The GPU arch number is list as below

| RTX 30X0 | A100 | RTX 20X0 | TITAN V / V100 | GTX 10X0 / TITAN Xp | GTX 9X0 | K80 |
| -------- | ---- | -------- | -------------- | ------------------- | ------- | --- |
| 86       | 80   | 75       | 70             | 61                  | 52      | 37  |

```
export PATH="/usr/local/cuda-11.6/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda-11.6/lib64:$LD_LIBRARY_PATH"
export TCNN_CUDA_ARCHITECTURES=<GPU arch number>
```

Build with GUI

```
cmake -B build -DCMAKE_BUILD_TYPE=Release .
cmake --build build --parallel $(nproc --all)
```

Build without GUI

```
cmake -B build -DCMAKE_BUILD_TYPE=Release -DORBEEZ_BUILD_WITH_GUI=OFF .
cmake --build build --parallel $(nproc --all)
```

## Test the program

### Download the TUM fr3/office Dataset

```
wget https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz --no-check-certificate
tar -xvzf rgbd_dataset_freiburg3_long_office_household.tgz
```

### Execute Program (Monocular Version)

```
./build/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM3.yaml ./rgbd_dataset_freiburg3_long_office_household/
```
