# RUN on a custom dataset
## Write a main program
- The main programs are put under Examples. You need to write an `<mono/rgbd>_<custom-dataset>.cu` under the corresponding folder. 
```
int main()
{
	// Load images from your dataset
	LoadImages(...)

	/* 
	Create SLAM system. It initializes all system threads and gets ready to process frames.
	Usage: System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const bool bTrainExtrinsicWithPhotometric = false);
	The third argument should be ORBEEZ::System::RGBD or ORBEEZ::System::MONO depends on your camera
    */
    // If camera is RGBD
    ORBEEZ::System SLAM(argv[1], argv[2], ORBEEZ::System::RGBD, true, false);
    // If camera is MONO
    ORBEEZ::System SLAM(argv[1], argv[2], ORBEEZ::System::MONO, true, false);

    // main loop
    for(...)
    {
    	// tframe is the timestamp. If your dataset does not have it, you can pass [0,1,2,...] (The index of the sequence into it.
    	imRGB, imD, tframe = read_img(...);

    	// Pass the image to the SLAM system
    	// If camera is RGBD
    	SLAM.TrackRGBD(imRGB, imD, tframe);
    	// If camera is MONO
    	SLAM.TrackMonocular(imRGB, tframe);
    }

    // Evaluatoin Code
    You can refer other main programs
}
```
## Add to CMakeLists.txt
- Scroll down to the end of CMakeLists.txt. You will see something like this
```
# Build examples

# Monocular
add_executable(mono_tum Examples/Monocular/mono_tum.cu)
target_link_libraries(mono_tum ORBEEZ ngp tiny-cuda-nn)
target_compile_options(mono_tum PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(mono_tum PUBLIC ${ORBEEZ_DEFINITIONS})

add_executable(mono_replica Examples/Monocular/mono_replica.cu)
target_link_libraries(mono_replica ORBEEZ ngp tiny-cuda-nn)
target_compile_options(mono_replica PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(mono_replica PUBLIC ${ORBEEZ_DEFINITIONS})

add_executable(mono_scannet Examples/Monocular/mono_scannet.cu)
target_link_libraries(mono_scannet ORBEEZ ngp tiny-cuda-nn)
target_compile_options(mono_scannet PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(mono_scannet PUBLIC ${ORBEEZ_DEFINITIONS})

# RGBD
add_executable(rgbd_tum Examples/RGB-D/rgbd_tum.cu)
target_link_libraries(rgbd_tum ORBEEZ ngp tiny-cuda-nn)
target_compile_options(rgbd_tum PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(rgbd_tum PUBLIC ${ORBEEZ_DEFINITIONS})

add_executable(rgbd_replica Examples/RGB-D/rgbd_replica.cu)
target_link_libraries(rgbd_replica ORBEEZ ngp tiny-cuda-nn)
target_compile_options(rgbd_replica PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(rgbd_replica PUBLIC ${ORBEEZ_DEFINITIONS})

add_executable(rgbd_scannet Examples/RGB-D/rgbd_scannet.cu)
target_link_libraries(rgbd_scannet ORBEEZ ngp tiny-cuda-nn)
target_compile_options(rgbd_scannet PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(rgbd_scannet PUBLIC ${ORBEEZ_DEFINITIONS})
```
- Add your own one.
```
# Monocular
...

add_executable(<mono>_<custom-dataset> Examples/Monocular/<mono>_<custom-dataset>.cu)
target_link_libraries(<mono>_<custom-dataset> ORBEEZ ngp tiny-cuda-nn)
target_compile_options(<mono>_<custom-dataset> PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(<mono>_<custom-dataset> PUBLIC ${ORBEEZ_DEFINITIONS})

# RGBD
...

add_executable(<rgbd>_<custom-dataset> Examples/RGB-D/<rgbd>_<custom-dataset>.cu)
target_link_libraries(<rgbd>_<custom-dataset> ORBEEZ ngp tiny-cuda-nn)
target_compile_options(<rgbd>_<custom-dataset> PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>)
target_compile_definitions(<rgbd>_<custom-dataset> PUBLIC ${ORBEEZ_DEFINITIONS})

```

## Compile the code
Please refer to Compilation in [BUILD.md](./BUILD.md)
If the compilation process is successfull. You can see your program under the `build` folder.
```
<mono>_<custom-dataset>
<rgbd>_<custom-dataset>
```

## Write a config file for your program
The config files are under configs. Put your config file under your folder. You can refer to other config file.
```
cd configs/<Monocular/RGB-D>
mkdir <custom-dataset>
```
- RGB-D config format
```
%YAML 1.2
---
#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------

# Camera calibration and distortion parameters (OpenCV) 
Camera:
  fx: 
  fy: 
  cx: 
  cy: 
  k1: 
  k2: 
  p1: 
  p2: 
  width: 
  height: 
# Camera frames per second 
  fps: 
# IR projector baseline times fx (aprox.)
  bf: 
# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
  RGB: 

#--------------------------------------------------------------------------------------------
# Depth Parameters.
#--------------------------------------------------------------------------------------------

# Close/Far threshold. Baseline times.
ThDepth: 

# Depthmap values factor (RealDepthValue * DepthMapFactor = DepthImageValue, this depends on dataset)
DepthMapFactor: 

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor:
  nFeatures:
# ORB Extractor: Scale factor between levels in the scale pyramid   
  scaleFactor: 
# ORB Extractor: Number of levels in the scale pyramid  
  nLevels: 
# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast     
  iniThFAST: 
  minThFAST: 

#--------------------------------------------------------------------------------------------
# NeRF Parameters
#--------------------------------------------------------------------------------------------
NeRF:
  aabb_scale: 
  scale: 
  offset: 
  network_config_path: 
```
- Monocular config format
```
%YAML 1.2
---
#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------

# Camera calibration and distortion parameters (OpenCV) 
Camera:
  fx: 
  fy: 
  cx: 
  cy: 
  k1: 
  k2: 
  p1: 
  p2: 
  width: 
  height: 
# Camera frames per second 
  fps: 
# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
  RGB: 

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor:
  nFeatures:
# ORB Extractor: Scale factor between levels in the scale pyramid   
  scaleFactor: 
# ORB Extractor: Number of levels in the scale pyramid  
  nLevels: 
# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast     
  iniThFAST: 
  minThFAST: 

#--------------------------------------------------------------------------------------------
# NeRF Parameters
#--------------------------------------------------------------------------------------------
NeRF:
  aabb_scale: 
  scale: 
  offset: 
  network_config_path: 
```