/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"   // IWYU pragma: associated

// #include <pangolin/pangolin.h>
#include <unistd.h>
#include <stdlib.h>
#include <thread>
#include <iomanip>
#include <algorithm>
#include <iostream>
#include <list>

#include "Converter.h"
#include "Frame.h"
#include "FrameDrawer.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "NerfMapping.h"

#include "LoopClosing.h"
#include "Map.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "opencv2/core/eigen.hpp"

#include <neural-graphics-primitives/camera_path.h> 

namespace ORBEEZ
{
class MapPoint;
class NerfMapping;

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, const bool bTrainExtrinsicWithPhotometric):
        mSensor(sensor), 
        mpViewer(static_cast<Viewer*>(NULL)),  
        mbReset(false),
        mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "Orbeez-SLAM Copyright (C) 2022-2023 Chi-Ming Chung, National Taiwan University." << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "The ORB-SLAM2 part in Orbeez-SLAM comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "The ORB-SLAM2 part in Orbeez-SLAM is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    // Create the Map
    std::string strSlamTransform = "Orbeez-SLAM.json";
    mpMap = new Map(strSettingsFile, strSlamTransform, bTrainExtrinsicWithPhotometric);

    // Create Drawers. These are used by the Viewer but are not only used for GUI. 
    // Therefore, don't comment it if no gui (bUseViewer == false)
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Nerf Mapping thread and launch
    mpNerfMapper = new NerfMapping(mpMap, mSensor==MONOCULAR);
    mptNerfMapping = new thread(&ORBEEZ::NerfMapping::Run, mpNerfMapper);

    // //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORBEEZ::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        //如果指定了，程序的运行过程中需要运行可视化部分
        //新建viewer
        mpViewer = new Viewer(this,             //又是这个
                              mpFrameDrawer,    //帧绘制器
                              mpMapDrawer,      //地图绘制器
                              mpTracker,        //追踪器
                              strSettingsFile); //配置文件的访问路径
        //新建viewer线程
        mptViewer = new thread(&Viewer::Run, mpViewer);
        //给运动追踪器设置其查看器
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetNerfMapper(mpNerfMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpNerfMapper->SetTracker(mpTracker);
    mpNerfMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetNerfMapper(mpNerfMapper);

}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpNerfMapper->RequestStop();

            // Wait until Nerf Mapper has effectively stopped
            while(!mpNerfMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpNerfMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpNerfMapper->RequestStop();

            // Wait until Nerf Mapping has effectively stopped
            while(!mpNerfMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpNerfMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpNerfMapper->RequestStop();

            // Wait until Nerf Mapping has effectively stopped
            while(!mpNerfMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpNerfMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Spin()
{
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }
    
    mpMap->StopTraining();
}

void System::Shutdown()
{
    mpNerfMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpNerfMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not Nerfized (tracking failure) are not saved.

    list<ORBEEZ::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

// Use for SLAMCameraKeyFrame
void to_json(nlohmann::json& j, const SLAMCameraKeyFrame& p) {
    j = nlohmann::json{{"R", p.R}, {"T", p.T}, {"slice", p.slice}, {"scale", p.scale}, {"fov", p.fov}, {"dof", p.dof}};
}

// Use for Trajectory
void to_json(nlohmann::json& j, const Trajectory& p) {
    j = nlohmann::json{{"timestamp", p.timestamp}, {"quat", p.quat}, {"t", p.t}};
}

void System::SaveKeyFrameTrajectoryNGP(const string &filename)
{
    cout << endl << "Saving NGP Nerf trajectory to " << filename << std::endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // use for render
    std::vector<SLAMCameraKeyFrame> path;
    // use for comparison with gt
    std::vector<Trajectory> trajectory;

    // Need to get transforms in Nerf

    nlohmann::json scene_config = mpMap->GetSceneConfig();
    float scale  = scene_config["scale"];
    std::vector<float> temp = scene_config["offset"].get<std::vector<float>>();
    Eigen::Vector3f offset(temp.data());

    for(size_t i=0; i<vpKFs.size(); i++)
    {   
        SLAMCameraKeyFrame current;
        KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;

        /*
        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);

        // quat: x y z w
        Eigen::Vector4f quat(q.data());
        Eigen::Vector3f trans;
        cv::cv2eigen(pKF->GetCameraCenter(), trans);

        Eigen::Vector3f scaled_trans = trans * scale + offset;

        float slice_plane_z, scale, fov, dof;
        mpMap->GetCameraInfo(&slice_plane_z, &scale, &fov, &dof);
        path.push_back(SLAMCameraKeyFrame(quat, scaled_trans, slice_plane_z, scale, fov, dof));
        trajectory.push_back(Trajectory(pKF->mTimeStamp, quat, trans));
        */

        Eigen::Matrix<float, 3, 4> xform = pKF->GetPoseWithPhotometric();
        Eigen::Matrix3f Rwc = xform.block(0,0,3,3);
        Eigen::Vector3f twc = xform.col(3);
        Eigen::Quaternionf q(Rwc);
        Eigen::Vector4f v(4);
        v[0] = q.x();
        v[1] = q.y();
        v[2] = q.z();
        v[3] = q.w();
        
        Eigen::Vector3f scaled_trans = twc * scale + offset;
        float slice_plane_z, scale, fov, dof;
        mpMap->GetCameraInfo(&slice_plane_z, &scale, &fov, &dof);
        path.push_back(SLAMCameraKeyFrame(v, scaled_trans, slice_plane_z, scale, fov, dof));
        trajectory.push_back(Trajectory(pKF->mTimeStamp, v, twc));
    }

    float time = 1.f;
    nlohmann::json j = {
        {"time", time},
        {"trajectory", trajectory},
        {"path", path}
    };

    f << std::setw(4) << j << std::endl;
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveSnapShot(const string &filename)
{
    cout << endl << "Saving NeRF Snap Shot to " << filename << endl;
    mpMap->SaveSnapShot(filename);

    cout << endl << "snapshot saved!" << endl;
}

void System::SaveMesh(const string &filename, uint32_t marching_cubes_res)
{
    cout << endl << "Saving NGP Mesh to " << filename << endl;
    // Cull empty region for better mesh
    // mpMap->CullEmptyRegion(); Not improving
    mpMap->SaveMesh(filename, marching_cubes_res);

    cout << endl << "mesh saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not Nerfized (tracking failure) are not saved.

    list<ORBEEZ::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORBEEZ::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

void System::AddGroundTruthTraj(const std::string& gtTrajPath)
{
    cout << "Show Ground Truth Traj in GUI" << endl;
    mpMap->AddGroundTruthTraj(gtTrajPath);
}

} //namespace ORBEEZ
