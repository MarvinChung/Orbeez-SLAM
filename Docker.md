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

   - Pull from docker hub

     ```
     docker pull marvinchung/orbeez-slam:latest
     ```

   - Build from Dockerfile

     ```
     cd Orbeez-SLAM
     docker build -t orbeez-slam:latest .
     ```

## Run

- For WSL2 with GUI support, run the container with

  ```
  docker run --name orbeezslam --gpus=all -it -v $(pwd):/program -v /data:/data -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -e DISPLAY -e WAYLAND_DISPLAY -e XDG_RUNTIME_DIR -e PULSE_SERVER marvinchung/orbeez-slam /bin/bash
  ```

  - Build

    ```
    mkdir build
    cmake . -B build
    cmake --build build --parallel $(nproc --all)
    ```

- If you do not require GUI support, run the container with

  ```
  docker run --name orbeezslam --gpus=all -it -v $(pwd):/program -v /data:/data marvinchung/orbeez-slam /bin/bash
  ```

  - Build

    ```
    mkdir build
    cmake . -B build -DORBEEZ_BUILD_WITH_GUI=OFF
    cmake --build build --parallel $(nproc --all)
    ```
