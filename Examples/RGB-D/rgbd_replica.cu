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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <sys/wait.h>
#include<unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <dirent.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgcodecs/legacy/constants_c.h>

#include<System.h>

using namespace std;

void LoadImages(const string &path, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD);
string GetDatasetName(const string &strSequencePath);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    string path = string(argv[3]);
    LoadImages(path, vstrImageFilenamesRGB, vstrImageFilenamesD);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORBEEZ::System SLAM(argv[1],argv[2],ORBEEZ::System::RGBD, true, false);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = ni;

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

    }

    // Stop orb-viewer and tracking. 
    // The user can watch the Nerf screen
    SLAM.Spin();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    string dataset_name = GetDatasetName(string(argv[3])); 
    auto trajString = "evaluation/RGBD_Replica_"+dataset_name+"_KeyFrameTrajectory";
    auto snapString = "evaluation/RGBD_Replica_"+dataset_name+".msgpack";
    auto gtJsonTrajString = "evaluation/RGBD_Replica_"+dataset_name+"_gtTraj.json";

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("evaluation/RGBD_Replica_"+dataset_name+"_CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM(trajString+".txt");  // rpj only
    SLAM.SaveKeyFrameTrajectoryNGP(trajString+".json"); // rpj (+ pht if train extrinsics) 
    SLAM.SaveSnapShot(snapString);

    int pid = fork();
    if (pid < 0)
    {
        cout << "fork failed" << endl;
    }
    else if( pid == 0 )
    {
        // For headless version, we do not need to spin the program.
        // But instead, terminate training process and execute evaluation script.
        auto gtString = string(argv[3]) + "/traj.txt";
        auto trajPathString = trajString + ".txt";
        auto plotString = trajString + "_rpj.png";
        char *gtPath = (char *)(gtString.c_str());
        char *trajPath = (char *)(trajPathString.c_str());
        char *plotPath = (char *)(plotString.c_str());
        char *gtJsonTrajPath = (char *)(gtJsonTrajString.c_str());

        std::cout << "ATE w/ reprojection error:" << std::endl;
        char *execArgs[] = {"python3", "scripts/evaluate_ate.py", gtPath, trajPath, "--verbose", "--plot", plotPath, "--save_gt_json", gtJsonTrajPath, NULL};
        execvp("python3", execArgs);
    }
    wait(NULL);

    std::cout << std::endl;

    pid = fork();
    if (pid < 0)
    {
        cout << "fork failed" << endl;
    }
    else if( pid == 0 )
    {
        // For headless version, we do not need to spin the program.
        // But instead, terminate training process and execute evaluation script.
        auto gtString = string(argv[3]) + "/traj.txt";
        auto trajPathString = trajString + ".json";
        auto plotString = trajString + "_rpj+pht.png";
        char *gtPath = (char *)(gtString.c_str());
        char *trajPath = (char *)(trajPathString.c_str());
        char *plotPath = (char *)(plotString.c_str());

        std::cout << "ATE w/ reprojection error (+ photometric error if optimize extrinsic == true):" << std::endl;
        char *execArgs[] = {"python3", "scripts/evaluate_ate.py", gtPath, trajPath, "--verbose", "--plot", plotPath, NULL};
        execvp("python3", execArgs);
    }
    wait(NULL);

#ifdef ORBEEZ_GUI
    cout << "Press ctrl + c to exit the program " << endl;

    // Don't stop program, to see the Nerf training result
    volatile int keep_spinning = 1;
    while (keep_spinning) ; // spin
#endif  

    return 0;
}

// void LoadImages(const string &path, vector<string> &vstrImageFilenamesRGB,
//                 vector<string> &vstrImageFilenamesD)
// {
//     DIR *mydir;
//     struct dirent *myfile;
//     struct stat mystat;
    
//     mydir = opendir(path.c_str());
//     if (!mydir) {
//         cerr << "Unable to open " << path << endl;
//         exit(1);
//     }

//     while((myfile = readdir(mydir)) != NULL)
//     {
//         stat(myfile->d_name, &mystat); 
//         std::string filename = std::string(myfile->d_name);

//         if (filename.substr (0,5) == "frame"){
//             vstrImageFilenamesRGB.push_back(filename);
//         }
//         else if(filename.substr (0,5) == "depth"){
//             vstrImageFilenamesD.push_back(filename);
//         }
//     }
//     closedir(mydir);
//     sort(vstrImageFilenamesRGB.begin(), vstrImageFilenamesRGB.end());
//     sort(vstrImageFilenamesD.begin(), vstrImageFilenamesD.end());
// }

void ls(const string &path, vector<string> &vstrImageFilenames) {
    DIR *mydir;
    struct dirent *myfile;
    struct stat mystat;

    mydir = opendir(path.c_str());

    if (!mydir) {
        cerr << "Unable to open " << path << endl;
        exit(1);
    }
    while((myfile = readdir(mydir)) != NULL)
    {
        stat(myfile->d_name, &mystat); 
        std::string filename = std::string(myfile->d_name);
        if (filename != "." && filename != "..")
            vstrImageFilenames.push_back(path+filename);
    }
    closedir(mydir);
    sort(vstrImageFilenames.begin(), vstrImageFilenames.end());
}

void LoadImages(const string &path, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD)
{
    std::string frame_path = path + "/frame/";
    std::string depth_path = path + "/depth/";

    ls(frame_path, vstrImageFilenamesRGB);
    ls(depth_path, vstrImageFilenamesD); 
}

string GetDatasetName(const string &strSequencePath) 
{
    string s(strSequencePath);
    std::string delimiter = "/";

    size_t pos = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        s.erase(0, pos + delimiter.length());
    }

    if (s.length() == 0)
        return token;
    else
        return s;
}