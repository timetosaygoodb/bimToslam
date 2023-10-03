#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "bimToslam/frontend.h"
#include "bimToslam/feature.h"
#include "bimToslam/dataset.h"
#include "bimToslam/ICPsolver.h"

#include <iostream>

using namespace std;
using namespace cv;
using namespace pcl;

namespace bimToslam
{

Frontend::Frontend() {};

bool Frontend::AddFrame(Frame::Ptr frame) {
    current_frame_ = frame;

    switch (status_) {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            // Track();
            break;
        case FrontendStatus::LOST:
            // Reset();
            break;  
    }

    last_frame_ = current_frame_;
    return true;
}

bool Frontend::StereoInit() {

    // 提取左右图像特征点进行匹配
    int num_features_ = ExtractandMatchORB();
    if (num_features_ < num_features_init_) {
        return false;
    }
    // 计算特征点深度，获取相机坐标下的点
    DepthformDisparity();
    // 对BIM点云与相机坐标下的点进行特征提取并匹配
    ExtractandMatchISS();
    // 计算在全局坐标下的位姿
    ICPsolver PoseSolver(current_frame_, camera_left_);
    double R[3][3], t[3];
    PoseSolver.EstimateCurrentPose(10, R, t);

    print_pose(R, t);

    cout << "success" << endl;

    return true;
}

int Frontend::ExtractandMatchORB() {

    Mat descriptors, descriptorsright;
    vector<KeyPoint> keypoints, keypointsright;

    (current_frame_->mpORBextractorLeft)->operator()(current_frame_->imLeft_, keypoints, descriptors);
    (current_frame_->mpORBextractorRight)->operator()(current_frame_->imRight_, keypointsright, descriptorsright);
    
    current_frame_->mDescriptors = descriptors;
    current_frame_->mDescriptorsRight = descriptorsright;

    // 匹配
    cv::Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    vector<DMatch> matches;
    matcher->match(descriptors, descriptorsright, matches);

    // 筛选
    auto min_max = std::minmax_element(matches.begin(), matches.end(),
        [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    
    double min_dist = min_max.first->distance;

    vector<DMatch> good_matches;
    for (int i = 0; i < descriptors.rows; i++){
        if (matches[i].distance <= max(2 * min_dist, 30.0)){
            good_matches.push_back(matches[i]);
        }
    }

    current_frame_->ORBmatches_ = good_matches;

    current_frame_->mvpORBmatches.resize(good_matches.size());

    current_frame_->features_left_.resize(good_matches.size());
    current_frame_->features_right_.resize(good_matches.size());

    for (size_t i = 0; i < good_matches.size(); i++){
        current_frame_->mvpORBmatches[i].first = keypoints[good_matches[i].queryIdx];
        current_frame_->mvpORBmatches[i].second = keypointsright[good_matches[i].trainIdx];

        Feature::Ptr feature_left(new Feature(current_frame_, current_frame_->mvpORBmatches[i].first));
        current_frame_->features_left_[i] = feature_left;

        Feature::Ptr feature_right(new Feature(current_frame_, current_frame_->mvpORBmatches[i].second));
        current_frame_->features_right_[i] = feature_right;
    }

    return good_matches.size();

}

void Frontend::DepthformDisparity() {

    auto orbmatches = current_frame_->mvpORBmatches;
    current_frame_->camerapoint.resize(orbmatches.size());

    for (size_t i = 0; i < orbmatches.size(); i++){
        Point2d left_point = orbmatches[i].first.pt;
        Point2d right_point = orbmatches[i].second.pt;

        Vec2 point;
        point << left_point.x, left_point.y;

        float disparity = left_point.x - right_point.x;
        double depth;

        if (disparity > 0) {
            depth = (camera_left_->fb_) / disparity;
            current_frame_->camerapoint[i] = camera_left_->pixel2camera(point, depth);
        }

        else
            continue;
    }
}

void Frontend::ExtractandMatchISS() {

    PointCloud<PointXYZ>::Ptr camera_cloud(new PointCloud<PointXYZ>), BIM_cloud(new PointCloud<PointXYZ>);

    for (const auto& feat : current_frame_->camerapoint) {
        PointXYZ pcl_point;
        pcl_point.x = feat(0);
        pcl_point.y = feat(1);
        pcl_point.z = feat(2);
        camera_cloud->points.push_back(pcl_point);
    }

    PointCloud<FPFHSignature33>::Ptr camera_descriptors(new PointCloud<FPFHSignature33>);
    (current_frame_->mpISSextractorcamera)->operator()(camera_cloud, 20, 20, 0.02, 0.975, 0.975, 0.02, camera_descriptors);
    
    vector<Vec3> plypoint = readPLYAsync(plyfile, 50); 

    for (const auto& feat : plypoint) {
        PointXYZ pcl_point;
        pcl_point.x = feat(0);
        pcl_point.y = feat(1);
        pcl_point.z = feat(2);
        BIM_cloud->points.push_back(pcl_point);
    }

    PointCloud<FPFHSignature33>::Ptr bim_descriptors(new PointCloud<FPFHSignature33>);
    (current_frame_->mpISSextractorply)->operator()(BIM_cloud, 20, 0.03, 0.02, 0.975, 0.975, 0.02, bim_descriptors);

    registration::CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> corr_est;
    CorrespondencesPtr correspondences(new Correspondences);
    corr_est.setInputSource(camera_descriptors);
    corr_est.setInputTarget(bim_descriptors);
    corr_est.determineReciprocalCorrespondences(*correspondences);

    current_frame_->N = correspondences->size();

    current_frame_->mvpISSmatches.resize(correspondences->size());
    current_frame_->pointID.resize(correspondences->size());

    for (size_t i = 0; i < current_frame_->mvpISSmatches.size(); i++) {
        current_frame_->mvpISSmatches[i].first = current_frame_->camerapoint[correspondences->at(i).index_query];
        current_frame_->mvpISSmatches[i].second = plypoint[correspondences->at(i).index_match];

        current_frame_->pointID.push_back(correspondences->at(i).index_query);
        
    }
}

void Frontend::print_pose(const double R[3][3], const double t[3])
{
  cout << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << t[0] << endl;
  cout << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << t[1] << endl;
  cout << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << t[2] << endl;
}



} // namespace bimToslam
