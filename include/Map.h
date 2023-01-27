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

#ifndef MAP_H
#define MAP_H

#include <set>
#include <mutex>
#include <vector>

#include "MapPoint.h"
#include "KeyFrame.h"

#include <neural-graphics-primitives/testbed.h> 

namespace ORBEEZ
{

class MapPoint;
class KeyFrame;

class Map
{
public:
    Map(const string &strSettingPath, const string &strSlamTransform, const bool bTrainCameraWithPhotometric);

    cv::FileNode KeyCheck(cv::FileNode& parent_node, const string& key);
    void init_window();
    void StopTraining();
    bool frame();
    void update_transformsGPU();
    bool NerfCameraIsUpdated();
    Eigen::Matrix<float, 3, 4> KeyFrameWorldPoseToNGPFormat(const Eigen::Matrix<float, 3, 4>& slam_matrix) const;
    Eigen::Matrix<float, 3, 4> KeyFrameNGPFormatToWorldPose(const Eigen::Matrix<float, 3, 4>& ngp_matrix) const;
    Eigen::Matrix<float, 3, 4> PoseWithPhotometric(int index) const;
    // void UpdatePosteriorWithNNOptimizer();
    void AddKeyFrame(KeyFrame* pKF);
    void GetCameraInfo(float *slice_plane_z, float *scale, float *fov, float *dof);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    nlohmann::json GetSceneConfig();
    void CullEmptyRegion();
    void SaveSnapShot(const string &filename);
    void SaveMesh(const string &filename, uint32_t marching_cubes_res);  
    void AddGroundTruthTraj(const std::string& gtTrajPath);

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    std::vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

protected:
    // cv::FileNode fSettingsLoadCheck(cv::FileStorage& fSettings, const string& key);
    // cv::FileNode KeyCheck(cv::FileNode& parent_node, const string& key);

    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;
    std::map<int, KeyFrame*> mId2KeyFrame;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;

    // Instant ngp pointer
    std::shared_ptr<ngp::Testbed> mpTestbed;
    nlohmann::json m_scene_config;

    bool mbDataIsReady;
};

} //namespace ORBEEZ

#endif // MAP_H
