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

#include "Map.h"   // IWYU pragma: associated

#include "KeyFrame.h"
#include "MapPoint.h"

#include "opencv2/core/eigen.hpp"
#include <unistd.h>

#include <neural-graphics-primitives/common.h> 

#include<mutex>

namespace Eigen
{
    // reference from https://github.com/nlohmann/json/issues/3267
    template<typename Scalar, int Rows, int Cols>
    void to_json(nlohmann::json& j, const Matrix<Scalar, Rows, Cols>& matrix) {
        for (int row = 0; row < matrix.rows(); ++row) {
            nlohmann::json column = nlohmann::json::array();
            for (int col = 0; col < matrix.cols(); ++col) {
                column.push_back(matrix(row, col));
            }
            j.push_back(column);
        }
    }

    template<typename Scalar, int Rows, int Cols>
    void from_json(const nlohmann::json& j, Matrix<Scalar, Rows, Cols>& matrix) {        
        for (std::size_t row = 0; row < j.size(); ++row) {
            const auto& jrow = j.at(row);
            for (std::size_t col = 0; col < jrow.size(); ++col) {
                const auto& value = jrow.at(col);
                value.get_to(matrix(row, col));
            }
        }
    }
}

namespace ORBEEZ
{

Map::Map(const string &strSettingPath, const string &strSlamTransform, const bool bTrainCameraWithPhotometric):mnMaxKFid(0), mnBigChangeIdx(0), mbDataIsReady(false)
{
    mpTestbed = std::make_shared<ngp::Testbed>(ngp::ETestbedMode::NerfSlam);
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    // Must exist
    cv::FileNode camera_node = fSettings["Camera"];
    float fl_x                        = KeyCheck(camera_node, "fx");
    float fl_y                        = KeyCheck(camera_node, "fy");
    float k1                          = KeyCheck(camera_node, "k1");
    float k2                          = KeyCheck(camera_node, "k2");
    float p1                          = KeyCheck(camera_node, "p1");
    float p2                          = KeyCheck(camera_node, "p2");
    float cx                          = KeyCheck(camera_node, "cx");
    float cy                          = KeyCheck(camera_node, "cy");
    int width                         = KeyCheck(camera_node, "width");
    int height                        = KeyCheck(camera_node, "height");
    cv::FileNode NeRF_node   = fSettings["NeRF"];
    int aabb_scale                    = KeyCheck(NeRF_node, "aabb_scale");
    float scale                       = KeyCheck(NeRF_node, "scale");
    cv::FileNode offset               = KeyCheck(NeRF_node, "offset");
    std::string network_config_path   = KeyCheck(NeRF_node, "network_config_path");

    std::vector<float> offset_vector;
    for(cv::FileNodeIterator it = offset.begin(); it != offset.end(); it++)
    {
        offset_vector.push_back((float)*it);
    }
    // nlohmann::json offset_array = nlohmann::json::parse(offset_vector.begin(), offset_vector.end());
    
    // Only if RGB-D
    cv::FileNode depth_node           = fSettings["DepthMapFactor"];

    if (!depth_node.empty())
    {
        float DepthMapFactor = fSettings["DepthMapFactor"];
        // Although is name as integer, instant-ngp requires float.
        m_scene_config["integer_depth_scale"] = 1.0f/DepthMapFactor;
    }

    m_scene_config["fl_x"]                  = fl_x;
    m_scene_config["fl_y"]                  = fl_y;
    m_scene_config["k1"]                    = k1;
    m_scene_config["k2"]                    = k2;
    m_scene_config["p1"]                    = p1;
    m_scene_config["p2"]                    = p2;
    m_scene_config["cx"]                    = cx;
    m_scene_config["cy"]                    = cy;
    m_scene_config["w"]                     = width;
    m_scene_config["h"]                     = height;
    m_scene_config["aabb_scale"]            = aabb_scale;
    m_scene_config["scale"]                 = scale;
    m_scene_config["offset"]                = offset_vector;

    std::string json_string =  m_scene_config.dump(4);
    std::cout << json_string << std::endl;

    std::ofstream ofs(strSlamTransform, std::ofstream::trunc);
    ofs << json_string << std::endl;
    ofs.close();

    mpTestbed->load_training_data(strSlamTransform);
    mpTestbed->reload_network_from_file(network_config_path);
    mpTestbed->m_train = true;
    mpTestbed->m_nerf.training.optimize_extrinsics = bTrainCameraWithPhotometric;
}

cv::FileNode Map::KeyCheck(cv::FileNode& parent_node, const string& key) 
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

void Map::init_window()
{
    unique_lock<mutex> lock(mMutexMap);
#ifdef ORBEEZ_GUI
    mpTestbed->init_window(1920, 1080);
#endif
}

void Map::StopTraining()
{
    mpTestbed->m_train = false;
}

bool Map::frame()
{
    // The lock ensure modifying dataset (add keyframe) and training won't do simultaneously
    unique_lock<mutex> lock(mMutexMap);

    if (mbDataIsReady){

        // Draw sparse point cloud
        const vector<MapPoint*> &vpMPs    = vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
        const vector<MapPoint*> &vpRefMPs = mvpReferenceMapPoints;
        set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        std::vector<Eigen::Vector3f> map_points;
        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
            if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
                continue;
            cv::Mat pos = vpMPs[i]->GetWorldPos();
            Eigen::Vector3f map_point;
            cv::cv2eigen(pos, map_point);
            map_points.push_back(map_point);
        }

        std::vector<Eigen::Vector3f> ref_map_points;
        for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
        {
            if((*sit)->isBad())
                continue;
            cv::Mat pos = (*sit)->GetWorldPos();
            Eigen::Vector3f ref_map_point;
            cv::cv2eigen(pos, ref_map_point);
            ref_map_points.push_back(ref_map_point);
        }

        mpTestbed->add_sparse_point_cloud(map_points, ref_map_points);

        // train and render
        bool value = mpTestbed->frame();

        if(mpTestbed->m_train)
            tlog::info() << "iteration=" << mpTestbed->m_training_step << " loss=" << mpTestbed->m_loss_scalar.val();

        return value;
    }
    else
        return false;
}

Eigen::Matrix<float, 3, 4> Map::KeyFrameWorldPoseToNGPFormat(const Eigen::Matrix<float, 3, 4>& slam_matrix) const
{
    return mpTestbed->m_nerf.training.dataset.slam_matrix_to_ngp(slam_matrix);
}

Eigen::Matrix<float, 3, 4> Map::KeyFrameNGPFormatToWorldPose(const Eigen::Matrix<float, 3, 4>& ngp_matrix) const
{
    return mpTestbed->m_nerf.training.dataset.ngp_matrix_to_slam(ngp_matrix);
}

Eigen::Matrix<float, 3, 4> Map::PoseWithPhotometric(int index) const
{
    return KeyFrameNGPFormatToWorldPose(mpTestbed->m_nerf.training.transforms[index].start);
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    mId2KeyFrame[pKF->mnId] = pKF;

    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;

    // if (mspKeyFrames.size() < 5)
    // if (pKF->mnId % 5 == 0) {
    Eigen::MatrixXf transform_matrix;
    mbDataIsReady = true;
    cv::cv2eigen(pKF->GetPoseInverse(), transform_matrix);

    nlohmann::json frame_config;
    frame_config["Id"]                        = pKF->mnId;  
    frame_config["fl_x"]                      = m_scene_config["fl_x"];
    frame_config["fl_y"]                      = m_scene_config["fl_y"];
    frame_config["transform_matrix"]          = transform_matrix;
    frame_config["w"]                         = m_scene_config["w"];
    frame_config["h"]                         = m_scene_config["h"];

    cv::Mat color_image                       = pKF->GetColorImage();
    cv::Mat depth_image                       = pKF->GetDepthImage();

    // std::cout << "[Map] frame_config:" << frame_config.dump(4) << std::endl;

    if (frame_config["w"] != color_image.cols || frame_config["h"] != color_image.rows)
    {
        throw runtime_error("Image size does not match the yaml. Maybe use the wrong yaml");
    }

    // instant ngp requires image to have 4 channels
    cv::Mat rgba;
    if (color_image.channels() == 3) {
        // https://stackoverflow.com/questions/32290096/python-opencv-add-alpha-channel-to-rgb-image
        // First create the image with alpha channel
        cv::cvtColor(color_image, rgba, cv::COLOR_BGR2RGBA);

        // Split the image for access to alpha channel
        cv::Mat channels[4];
        cv::split(rgba, channels);

        // Assign the mask to the last channel of the image
        channels[3] = 255 * cv::Mat::ones(channels[3].rows, channels[3].cols, CV_8UC1);

        // Finally concat channels for rgba image
        cv::merge(channels, 4, rgba);
    }
    else if (color_image.channels() == 4){
        // instant-ngp may release the image memory. Therefore, clone the image.
        rgba = color_image.clone();
    }
    else{
        throw std::runtime_error("incorrect image format");
    }

    assert(rgba.isContinuous());

    // prepare an additional memory for instant-ngp. It will be free in instant-ngp
    uint8_t *img = (uint8_t*)malloc(sizeof(uint8_t) * rgba.rows * rgba.cols * rgba.channels());
    memcpy(img, rgba.data, sizeof(uint8_t) * rgba.rows * rgba.cols * rgba.channels());

    uint16_t *depth = nullptr;
    
    // if(!depth_image.empty()){

    //     if (frame_config["w"] != depth_image.cols || frame_config["h"] != depth_image.rows)
    //     {
    //         throw runtime_error("depth image must be same as color_image. Please resize them in the main program");
    //     }

    //     depth = (uint16_t*)malloc(sizeof(uint16_t) * depth_image.rows * depth_image.cols * depth_image.channels());
    //     memcpy(depth, depth_image.data, sizeof(uint8_t) * depth_image.rows * depth_image.cols * depth_image.channels());
    // }

    std::tuple<ngp::TrainingXForm*,int> t = mpTestbed->add_training_image(frame_config, img, depth);
    ngp::TrainingXForm *pXform = std::get<0>(t);
    int index = std::get<1>(t);

    pKF->SetNerfXformPointer(pXform, index);
}

void Map::update_transformsGPU()
{
    unique_lock<mutex> lock(mMutexMap);
    mpTestbed->update_camera(mpTestbed->m_training_stream);
}

bool Map::NerfCameraIsUpdated()
{
    unique_lock<mutex> lock(mMutexMap);
    return (mpTestbed->m_nerf.training.optimize_extrinsics) && (mpTestbed->m_nerf.training.n_steps_since_cam_update == 0);
}

void Map::GetCameraInfo(float *slice_plane_z, float *scale, float *fov, float *dof)
{
    *slice_plane_z = mpTestbed->m_slice_plane_z;
    *scale = mpTestbed->scale();
    *fov = mpTestbed->fov();
    *dof = mpTestbed->m_dof;
}

nlohmann::json Map::GetSceneConfig()
{
    return m_scene_config;
}

void Map::CullEmptyRegion()
{
    unique_lock<mutex> lock(mMutexMap);
    std::cout << "Cull empty region (camera can't see) in density grid" << std::endl;
    // Mesh use inference stream
    mpTestbed->cull_empty_region(false, mpTestbed->m_inference_stream);
}

void Map::SaveSnapShot(const string &filename)
{
    unique_lock<mutex> lock(mMutexMap);
    mpTestbed->save_snapshot(filename, false);
}

void Map::SaveMesh(const string &filename, uint32_t marching_cubes_res)
{
    unique_lock<mutex> lock(mMutexMap);
    // std::cout << "SaveMesh" << std::endl;
    Eigen::Vector3i res3d(3);
    res3d << marching_cubes_res, marching_cubes_res, marching_cubes_res;
    mpTestbed->compute_and_save_marching_cubes_mesh(filename.c_str(), res3d);
}

void Map::AddGroundTruthTraj(const std::string& gtTrajPath)
{
    unique_lock<mutex> lock(mMutexMap);
    mpTestbed->AddGroundTruthTraj(gtTrajPath);
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mId2KeyFrame.erase(pKF->mnId);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    mpTestbed->clear_training_data();
    mpTestbed->load_nerfslam();

    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mId2KeyFrame.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

} //namespace ORBEEZ
