#ifndef NERFMAPPING_H
#define NERFMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "ORBmatcher.h"

#include <mutex>
#include <Eigen/Dense>
#include <cassert>

namespace ORBEEZ{

class Tracking;
class LoopClosing;

class NerfMapping
{
public:
    NerfMapping(Map* pMap, const float bMonocular);
    void SetLoopCloser(LoopClosing* pLoopCloser);
    void SetTracker(Tracking* pTracker);
    void Run();
    void InsertKeyFrame(KeyFrame* pKF);
    void RequestStop();
    void RequestReset();

    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested(); 
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);
    void InterruptBA();
    void RequestFinish();
    bool isFinished();
    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:
    bool mbMonocular;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;
    LoopClosing* mpLoopCloser;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    bool mbAcceptKeyFrames;

    Tracking* mpTracker;
    std::list<KeyFrame*> mlNewKeyFrames; 
    KeyFrame* mpCurrentKeyFrame;
    std::list<MapPoint*> mlpRecentAddedMapPoints;
    std::mutex mMutexNewKFs;

    std::mutex mMutexStop;
    std::mutex mMutexAccept;

    bool CheckNewKeyFrames();
    // void ProcessNewKeyFrame();

    void ProcessNewKeyFrame();

    void CreateNewMapPoints();

    void MapPointCulling();

    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

};

}

#endif