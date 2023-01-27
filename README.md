# Orbeez-SLAM

The paper is available in https://arxiv.org/abs/2209.13274
- [Video](https://www.youtube.com/watch?v=XVrVLun0ckk&ab_channel=%E9%8D%BE%E8%B5%B7%E9%B3%B4)

More details can be found in my master thesis (Main content is written in English).
- [offical link (require registration to download)](https://etds.ncl.edu.tw/cgi-bin/gs32/gsweb.cgi/ccd=ldn10D/record?r1=1&h1=0)
- [pdf](https://drive.google.com/file/d/1WYjB8JAu0lATqvtImMTieT8PcewzS5CM/view)

Orbeez-SLAM 🔫 is implemented with two open source projects

- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) (GPLv3 license)
- [instant-ngp](https://github.com/NVlabs/instant-ngp) (Nvidia Source Code License-NC)

## License
This repo is GPLv3 Licensed (inherit ORB-SLAM2). It reimplements parts of ORB-SLAM2. Our changes to instant-NGP (Nvidia License) are released in [our fork of instant-ngp](https://github.com/MarvinChung/instant-ngp-kf/tree/8344e4e63af70b0a792ff83bfdeb4c67b477681e) and added here as a thirdparty dependency using git submodules. Several evaluation files in `scripts` are BSD License.

## How to build
Please refer to [BUILD.md](./BUILD.md)

## RUN on TUM
Please refer to [TUM.md](./TUM.md)

## RUN on Replica
Please refer to [Replica.md](./Replica.md)

## RUN on ScanNet
Please refer to [ScanNet.md](./ScanNet.md)

## RUN on a custom dataset
Please refer to [Custom.md](./Custom.md)

## Evaluation
Please refer to [Eval.md](./Eval.md)

## Acknowledgments
This work has been possible thanks to the open-source code from [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) and [instant-ngp](https://github.com/NVlabs/instant-ngp), as well as the open-source datasets [Replica](https://github.com/facebookresearch/Replica-Dataset) and [TUM-dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset/download).