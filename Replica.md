# RUN on Replica
## Download Dataset
We run on office0 office1 office2 office3 office4 room0 room1 room2 from the Replica dataset made by [NICE-SLAM](https://github.com/cvg/nice-slam/blob/master/scripts/download_replica.sh).
Download Replica from NICE-SLAM
```
wget https://cvg-data.inf.ethz.ch/nice-slam/data/Replica.zip --no-check-certificate
unzip Replica.zip
```

Modify the dataset to our directory structure  
The original structure (Take office0 as example):
```
Replica
- office0
	- results (contain rgb and depth images)
```
Our structure (We move the rgb and depth images from results into independent folders): 
```
Replica
- office0
	- frame (contain rgb images only)
	- depth (contain depth images only)
```

## Run Guideline
### RGB-D
The main program is in Examples/RGB-D/rgbd_tum.cu
Usage:
```
rgbd_replica path_to_vocabulary path_to_settings path_to_sequence
```
- Run office0
```
./build/rgbd_replica Vocabulary/ORBvoc.txt configs/RGB-D/Replica/office0.yaml Replica/office0/
```
- Run office1
```
./build/rgbd_replica Vocabulary/ORBvoc.txt configs/RGB-D/Replica/office1.yaml Replica/office1/
```
- Run office2 
```
./build/rgbd_replica Vocabulary/ORBvoc.txt configs/RGB-D/Replica/office2.yaml Replica/office2/
```
- Run office3
```
./build/rgbd_replica Vocabulary/ORBvoc.txt configs/RGB-D/Replica/office3.yaml Replica/office3/
```
- Run office4
```
./build/rgbd_replica Vocabulary/ORBvoc.txt configs/RGB-D/Replica/office4.yaml Replica/office4/
```
- Run room0
```
./build/rgbd_replica Vocabulary/ORBvoc.txt configs/RGB-D/Replica/room0.yaml Replica/room0/
```
- Run room1
```
./build/rgbd_replica Vocabulary/ORBvoc.txt configs/RGB-D/Replica/room1.yaml Replica/room1/
```
- Run room2
```
./build/rgbd_replica Vocabulary/ORBvoc.txt configs/RGB-D/Replica/room2.yaml Replica/room2/
```

### Monocular
The main program is in Examples/Monocular/mono_replica.cu
Usage:
```
mono_replica path_to_vocabulary path_to_settings path_to_sequence
```
- Run office0
```
./build/mono_replica Vocabulary/ORBvoc.txt configs/Monocular/Replica/office0.yaml Replica/office0/
```
- Run office1
```
./build/mono_replica Vocabulary/ORBvoc.txt configs/Monocular/Replica/office1.yaml Replica/office1/
```
- Run office2 (Tune this)
```
./build/mono_replica Vocabulary/ORBvoc.txt configs/Monocular/Replica/office2.yaml Replica/office2/
```
- Run office3
```
./build/mono_replica Vocabulary/ORBvoc.txt configs/Monocular/Replica/office3.yaml Replica/office3/
```
- Run room0
```
./build/mono_replica Vocabulary/ORBvoc.txt configs/Monocular/Replica/room0.yaml Replica/room0/
```
- Run room1
```
./build/mono_replica Vocabulary/ORBvoc.txt configs/Monocular/Replica/room1.yaml Replica/room1/
```
- Run room2
```
./build/mono_replica Vocabulary/ORBvoc.txt configs/Monocular/Replica/room2.yaml Replica/room2/
```


