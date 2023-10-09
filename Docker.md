# Docker Tutorial

## Requirements

1. Either `sudo` privilege or in `docker` group if you wished to run in docker.

   ```
   sudo usermod -aG docker $(whoami)
   newgrp docker # or logout and login
   ```

2. `docker` with `nvidia` runtime installed.

   - For installation on docker with nvidia runtime, please check [this link](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).

3. Linux OS, or even WSL2 on Windows.

## Build

1. Clone the repo.

   ```
   git clone https://github.com/MarvinChung/Orbeez-SLAM.git
   git submodule update --init --recursive
   ```

2. Grab the image by either pulling from docker hub or building from Dockerfile.

   - Pull from docker hub (For cuda version 11.7, architecture RTX2080Ti)

     ```
     docker pull mingupingu/orbeez-slam:latest
     ```

   - Build from Dockerfile
     You might need to change some config in the Dockerfile. The default Dockerfile is for cuda 11.8 and RTX 2080Ti.
     ```
     FROM nvidia/cuda:11.7.0-devel-ubuntu22.04
     WORKDIR /root

     ENV PATH="/usr/local/cuda-11.7/bin:$PATH"
     ENV LD_LIBRARY_PATH="/usr/local/cuda-11.7/lib64:$LD_LIBRARY_PATH"
     ENV TCNN_CUDA_ARCHITECTURES=86
     ```
     Change the `11.7` to your compatiable version (https://hub.docker.com/r/nvidia/cuda/tags) and `TCNN_CUDA_ARCHITECTURES=XX` based on 

     | H100 | 40X0 | 30X0 | A100 | 20X0 | TITAN V / V100 | 10X0 / TITAN Xp | 9X0 | K80 |
     |:----:|:----:|:----:|:----:|:----:|:--------------:|:---------------:|:---:|:---:|
     |   90 |   89 |   86 |   80 |   75 |             70 |              61 |  52 |  37 |

     Then build the Dockerfile.
     ```
     cd Orbeez-SLAM
     docker build -t orbeez-slam:latest .
     ```

## Run

- For WSL2 with GUI support, run the container with

  ```
  docker run --name orbeezslam --gpus=all -it -v $(pwd):/program -v /data:/data -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -e DISPLAY -e WAYLAND_DISPLAY -e XDG_RUNTIME_DIR -e PULSE_SERVER mingupingu/orbeez-slam /bin/bash
  ```

  - Build

    ```
    mkdir build
    cmake . -B build
    cmake --build build --parallel $(nproc --all)
    ```

- If you do not require GUI support, run the container with

  ```
  docker run --name orbeezslam --gpus=all -it -v $(pwd):/program -v /data:/data mingupingu/orbeez-slam /bin/bash
  ```

  - Build

    ```
    mkdir build
    cmake . -B build -DORBEEZ_BUILD_WITH_GUI=OFF
    cmake --build build --parallel $(nproc --all)
    ```
