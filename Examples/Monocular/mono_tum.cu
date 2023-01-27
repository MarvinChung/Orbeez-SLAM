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

#include<opencv2/core/core.hpp>
#include<opencv2/imgcodecs/legacy/constants_c.h>

#include<System.h>

// #define COMPILEDWITHC11 // Hack through this
using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);
string GetDatasetName(const string &strSequencePath);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORBEEZ::System SLAM(argv[1],argv[2],ORBEEZ::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
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
    auto trajString = "evaluation/MONO_TUM_"+dataset_name+"_KeyFrameTrajectory";
    auto snapString = "evaluation/MONO_TUM_"+dataset_name+".msgpack";
    auto gtJsonTrajString = "evaluation/MONO_TUM_"+dataset_name+"_gtTraj.json";

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(trajString+".txt");  // rpj only
    SLAM.SaveKeyFrameTrajectoryNGP(trajString+".json"); // rpj (+ pht if train extrinsics) 
    SLAM.SaveSnapShot(snapString);

    int pid = fork();
    if (pid < 0)
    {
        cout << "fork failed" << endl;
    }
    else if (pid == 0)
    {
        // For headless version, we do not need to spin the program.
        // But instead, terminate training process and execute evaluation script.
        auto gtString = string(argv[3]) + "/groundtruth.txt";
        auto trajPathString = trajString + ".txt";
        auto plotString = trajString + ".png";
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
        auto gtString = string(argv[3]) + "/groundtruth.txt";
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

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
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