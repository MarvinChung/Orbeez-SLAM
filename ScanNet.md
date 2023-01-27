# How to Run ScanNet
## Download Dataset
1. Request to download the dataset as mentioned in https://github.com/ScanNet/ScanNet.
- If the request is accepted, they will send you a script called `download-scannet.py`.
2. Place the file `download-scannet.py` under `scripts/`.
3. Run `preprocess-scannet.sh` and you should be ready to run ScanNet on Orbeez SLAM.
   *Note: Please run the script under `scripts` due to the dicrectory structure.*

The directory structure would be as follow
```
└─📁 scans/
  ├──📁 scene0000_00/
  │ ...
  ├─📁 scene0059_00/
  │ ├─📁 depth/
  │ ├─📄 trajectory.txt
  │ ├─📄 scene0059_00.sens
  │ ├─📁 pose/
  │ └─📁 color/
  ├─📁 scene0207_00/
  │ ...
  ├─📁 scene0169_00/
  │ ...
  ├─📁 scene0181_00/
  │ ...
  ├─📁 scene0106_00/
  │ ...
```

## Run Guideline
### RGB-D
The main program is in Examples/RGB-D/rgbd_scannet.cu
Usage:
```
rgbd_scannet path_to_vocabulary path_to_settings path_to_sequence
```
- Run scene0000_00
```
./build/rgbd_scannet ./Vocabulary/ORBvoc.txt ./configs/RGB-D/ScanNet/scene0000_00.yaml /data/scene0000_00/
```
- Run scene0059_00
```
./build/rgbd_scannet ./Vocabulary/ORBvoc.txt ./configs/RGB-D/ScanNet/scene0059_00.yaml /data/scene0059_00/
```
- Run scene0106_00
```
./build/rgbd_scannet ./Vocabulary/ORBvoc.txt ./configs/RGB-D/ScanNet/scene0106_00.yaml /data/scene0106_00/
```
- Run scene0169_00
```
./build/rgbd_scannet ./Vocabulary/ORBvoc.txt ./configs/RGB-D/ScanNet/scene0169_00.yaml /data/scene0169_00/
```
- Run scene0181_00
```
./build/rgbd_scannet ./Vocabulary/ORBvoc.txt ./configs/RGB-D/ScanNet/scene0181_00.yaml /data/scene0181_00/
```
- Run scene0207_00
```
./build/rgbd_scannet ./Vocabulary/ORBvoc.txt ./configs/RGB-D/ScanNet/scene0207_00.yaml /data/scene0207_00/
```

### Monocular
The main program is in Examples/Monocular/mono_scannet.cu
Usage:
```
mono_scannet path_to_vocabulary path_to_settings path_to_sequence
```
Note that monocular does not perform well and we did not find a config ('path_to_settings') to make it work. Therefore, we do not provide config files for `mono_scannet`  under configs/Monocular. If you want to run the monocular setting, you can try to put the RGB-D config path in the 'path_to_settings' to see the outcome.
- i.e.: Change the `build/rgbd_scannet` above to `build/mono_scannet`.

---

## Acknowledgement
`SensorData.py` and `reader.py` are modified from https://github.com/ScanNet/ScanNet, where we convert them from Python 2 to Python 3.
