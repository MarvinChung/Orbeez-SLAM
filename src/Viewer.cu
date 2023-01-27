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

#include "Viewer.h"   // IWYU pragma: associated

// #include <GL/glew.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <mutex>

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "System.h"
#include <Eigen/Dense>

namespace ORBEEZ
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    cv::FileNode camera_node = fSettings["Camera"];
    float fps = KeyCheck(camera_node, "fps");
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    //从配置文件中获取图像的长宽参数
    mImageWidth  = KeyCheck(camera_node, "width");;
    mImageHeight = KeyCheck(camera_node, "height");;
    if(mImageWidth<1 || mImageHeight<1)
    {   
        //默认值
        mImageWidth = 640;
        mImageHeight = 480;
    }
}

cv::FileNode Viewer::KeyCheck(cv::FileNode& parent_node, const string& key) 
{
    cv::FileNode node = parent_node[key];
    if (node.empty()) 
    {
        throw runtime_error{string{"The key: "} + key + string{" is not in the yaml. "}};
    }
    else{
        return node;
    }
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

#ifdef ORBEEZ_GUI
    // Frame window
    cv::namedWindow("Orbeez-SLAM: Current Frame");

    while(1) {

        cv::Mat im = mpFrameDrawer->DrawFrame();
        cv::imshow("Orbeez-SLAM: Current Frame", im);
        cv::waitKey(mT);

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }
        if(CheckFinish())
            break;
    }

    cv::destroyWindow("Orbeez-SLAM: Current Frame");
#endif
    
    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
