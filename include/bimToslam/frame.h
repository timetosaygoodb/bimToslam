#pragma once

#ifndef BIMTOSLAM_FRAME_H
#define BIMTOSLAM_FRAME_H

#include "bimToslam/common_include.h"
#include "bimToslam/ORBextractor.h"
#include "bimToslam/ISSextractor.h"
#include "bimToslam/camera.h"

namespace bimToslam
{
class MapPoint;
class Feature;
class Camera;

class Frame {

public:
    Frame();

    Frame(const Frame &frame);

    // Constructor for stereo cameras.

    /**
     * @brief 为双目相机准备的构造函数
     * 
     * @param[in] imLeft            左目图像
     * @param[in] imRight           右目图像
     *  
     */
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight);
    
    SE3 Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex);
        return Tcw;
    }

    void SetPose(const SE3 &Tcw_) {
        std::unique_lock<std::mutex> lck(pose_mutex);
        Tcw = Tcw_;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    cv::Mat imLeft_, imRight_;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> features_right_;

    std::vector<std::pair<cv::KeyPoint, cv::KeyPoint>> mvpORBmatches;

    std::vector<std::pair<Vec3, Vec3>> mvpISSmatches;

    std::vector<cv::DMatch> ORBmatches_;
    // point in camera
    std::vector<Vec3> camerapoint; 

    std::vector<int> pointID;

    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    ISSextractor* mpISSextractorply, *mpISSextractorcamera;


    // Number of KeyPoints.
    int N; 

    SE3 Tcw;

    std::mutex pose_mutex;
};

} // namespace bimToslam

#endif
