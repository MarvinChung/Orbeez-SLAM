# RUN on TUM
## Download Dataset
We run on fr1/desk fr2/xyz fr3/office dataset.
Download fr1/desk
```
wget https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz --no-check-certificate
tar -xvzf rgbd_dataset_freiburg1_desk.tgz
```
Download fr2/xyz
```
wget https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_xyz.tgz --no-check-certificate
tar -xvzf rgbd_dataset_freiburg2_xyz.tgz
```
Download fr3/office
```
wget https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz --no-check-certificate
tar -xvzf rgbd_dataset_freiburg3_long_office_household.tgz
```

## Run Guideline
### RGB-D
The main program is in Examples/RGB-D/rgbd_tum.cu
Usage:
```
rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association
```
- Run fr1/desk
```
./build/rgbd_tum Vocabulary/ORBvoc.txt configs/RGB-D/TUM/freiburg1_desk.yaml rgbd_dataset_freiburg1_desk configs/RGB-D/TUM/associations/fr1_desk.txt
```
- Run fr2/xyz
```
./build/rgbd_tum Vocabulary/ORBvoc.txt configs/RGB-D/TUM/freiburg2_xyz.yaml rgbd_dataset_freiburg2_xyz configs/RGB-D/TUM/associations/fr2_xyz.txt
```
- Run fr3/office
```
./build/rgbd_tum Vocabulary/ORBvoc.txt configs/RGB-D/TUM/freiburg3_office.yaml rgbd_dataset_freiburg3_long_office_household configs/RGB-D/TUM/associations/fr3_office.txt
```

### Monocular
The main program is in Examples/Monocular/mono_tum.cu
Usage:
```
mono_tum path_to_vocabulary path_to_settings path_to_sequence
```
- Run fr1/desk
```
./build/mono_tum Vocabulary/ORBvoc.txt configs/Monocular/TUM/freiburg1_desk.yaml rgbd_dataset_freiburg1_desk
```
- Run fr2/xyz
```
./build/mono_tum Vocabulary/ORBvoc.txt configs/Monocular/TUM/freiburg2_xyz.yaml rgbd_dataset_freiburg2_xyz
```
- Run fr3/office
```
./build/mono_tum Vocabulary/ORBvoc.txt configs/Monocular/TUM/freiburg3_office.yaml rgbd_dataset_freiburg3_long_office_household
```


