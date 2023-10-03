#pragma once
#ifndef BIMTOSLAM_FRONTEND_H
#define BIMTOSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>
#include "bimToslam/common_include.h"
#include "bimToslam/frame.h"
#include "bimToslam/camera.h"

namespace bimToslam {


enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

/**
 * 前端
 * 估计当前帧Pose，在满足关键帧条件时向地图加入关键帧并触发优化
 */
class Frontend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;
    
    std::string plyfile;

    Frontend();

    /// 外部接口，添加一个帧并计算其定位结果
    bool AddFrame(Frame::Ptr frame);


    FrontendStatus GetStatus() const { return status_; }

    void SetCameras(Camera::Ptr left) {
        camera_left_ = left;
    }

    void print_pose(const double R[3][3], const double t[3]);

   private:
    /**
     * Track in normal mode
     * @return true if success
     */
    bool Track();

    /**
     * Try init the frontend with stereo images saved in current_frame_
     * @return true if success
     */
    bool StereoInit();

    int ExtractandMatchORB();

    void DepthformDisparity();

    void ExtractandMatchISS();

    /**
     * Build the initial map with single image
     * @return true if succeed
     */
    bool BuildInitMap();

    /**
     * Triangulate the 2D points in current frame
     * @return num of triangulated points
     */
    int TriangulateNewPoints();


    // data
    FrontendStatus status_ = FrontendStatus::INITING;

    Frame::Ptr current_frame_ = nullptr;  // 当前帧
    Frame::Ptr last_frame_ = nullptr;     // 上一帧
    Camera::Ptr camera_left_ = nullptr;   // 左侧相机
    Camera::Ptr camera_right_ = nullptr;  // 右侧相机


    
    SE3 relative_motion_;  // 当前帧与上一帧的相对运动，用于估计当前帧pose初值

    int tracking_inliers_ = 0;  // inliers, used for testing new keyframes

    // params
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

};

}  // namespace bimToslam

#endif  // MYSLAM_FRONTEND_H
