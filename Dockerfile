FROM nvidia/cuda:11.7.0-devel-ubuntu22.04
WORKDIR /root

ENV PATH="/usr/local/cuda-11.7/bin:$PATH"
ENV LD_LIBRARY_PATH="/usr/local/cuda-11.7/lib64:$LD_LIBRARY_PATH"
ENV TCNN_CUDA_ARCHITECTURES=86

ENV N_THREAD=16

RUN apt update && apt install git unzip wget gcc cmake build-essential mesa-common-dev \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libglew-dev \
    libxinerama-dev libxcursor-dev libxi-dev \
    python3-pip --assume-yes

RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
RUN tar xvf eigen-3.4.0.tar.gz
WORKDIR /root/eigen-3.4.0
RUN cmake . -B build
RUN cmake --build build --target install --parallel $N_THREAD

WORKDIR /root
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.6.0.zip
RUN wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.6.0.zip
RUN unzip opencv.zip
RUN unzip opencv_contrib.zip
RUN mkdir -p build
WORKDIR /root/build
RUN cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.6.0/modules ../opencv-4.6.0
RUN make -j $(nproc)
RUN make install

WORKDIR /root
RUN rm -rf *

RUN pip3 install numpy scipy matplotlib

ENTRYPOINT [ "/bin/bash", "-l", "-c" ]


