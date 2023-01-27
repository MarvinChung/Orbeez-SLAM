# Evaluation
### Files generated after Running
There are several files will be generated after running the program
- Temporary 
	1. Orbeez-SLAM.json: A json file converted from the configs' yaml and pass to the instant-ngp
- Evaluation
	1. evaluation/`<MONO/RGBD>_<dataset name>_<folder name>`.msgpack: The saved NeRF model
	2. evaluation/`<MONO/RGBD>_<dataset name>_<folder name>`.KeyFrameTrajectory.txt:
		The format is as follows
		```
		timestamp tx ty tz qx qy qz qw
		```
	3. evaluation/`<MONO/RGBD>_<dataset name>_<folder name>`.KeyFrameTrajectory.json:
		It contains "path" and "trajectory"
		- path: This contains the SLAM path in the NeRF (instant-ngp) coordinate which is scaled and translated from the ORB-SLAM2 coordinate. You can use it to render images/videos from the evaluation/`<MONO/RGBD>_<dataset name>_<folder name>`.msgpack. 
		- trajectory: This contains the SLAM trajectory in ORB-SLAM2 coordiante.
	4. evaluation/`<MONO/RGBD>_<dataset name>_<folder name>`_KeyFrameTrajectory_rpj.png: The image shows the difference between ground truth trajectory and the trajecotry optimized with reprojection error.
	5. evaluation/`<MONO/RGBD>_<dataset name>_<folder name>`_KeyFrameTrajectory_rpj+pht.png: The image shows the difference between ground truth trajectory and the trajecotry optimized with reprojection error + photometric error. (The photometric error should be manually turn on)

### Trajectory Evaluation
- Run scripts/evalute_ate.py 
```
python3 scripts/evaluate_ate.py <Ground Truth Trajectory Path> evaluation/`<MONO/RGBD>_<dataset name>_<folder name>`.KeyFrameTrajectory.txt
```
- We evaluate the trajectory automatically in our provided code. Therefore, you can see the following output in the end of the program. (The default setting does not optimize extrinsic with photometric error)
```
ATE w/ reprojection error:
len(matches)= 232
compared_pose_pairs 232 pairs
absolute_translational_error.rmse 0.011689 m
absolute_translational_error.mean 0.010528 m
absolute_translational_error.median 0.009911 m
absolute_translational_error.std 0.005079 m
absolute_translational_error.min 0.001331 m
absolute_translational_error.max 0.034854 m

ATE w/ reprojection error (+ photometric error if optimize extrinsic == true):
len(matches)= 232
compared_pose_pairs 232 pairs
absolute_translational_error.rmse 0.011689 m
absolute_translational_error.mean 0.010528 m
absolute_translational_error.median 0.009911 m
absolute_translational_error.std 0.005079 m
absolute_translational_error.min 0.001331 m
absolute_translational_error.max 0.034854 m
Show Ground Truth Traj in GUI
Press ctrl + c to exit the program 
```

### Optimize extrinsic with photometric error
- From main program (under Examples folder)
1. Change the parameter in System function
```
// Usage: System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const bool bTrainExtrinsicWithPhotometric = false);
// Change the last argument to true
ORBEEZ::System SLAM(argv[1],argv[2],ORBEEZ::System::RGBD, true, false->true);
```
2. Compile the code
```
cd build
make
```
- From GUI
1. Check the `Train extrinsics` in the toolbox.
![Alt text](extrinsic.png?raw=true "extrinsic")

### GUI Tips
- Open the `debug visualization` in the toolbox.
![Alt text](tips.png?raw=true "tips")
1. You can check the box for visualization
2. If you can not find the correct viewpoint. You can click `First` that moves the viewpoint to the first image.