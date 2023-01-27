#README

## Monocular SLAM  
For monocular SLAM, The scale of the image is depend on the initialization of the first two image. That is, we don't know the correct scale to the real unit such as meter. Therefore, the absolute trajectory error is meanless for monocular. However, the plot image can be a reference to see whether the monocular SLAM learn the trajectory. The estimate trajectory should be similar to the ground truth trajectory with a unknown scale.

## RGB-D SLAM
The absolute trajectory error unit is meter. The estimate trajectory should be almost same as the ground truth trajectory.