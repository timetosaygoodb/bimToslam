#include "bimToslam/ORBextractor.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;


namespace bimToslam
{

void ORBextractor::operator()(InputArray &_image, vector<KeyPoint>& _keypoints, OutputArray _descriptors)
{
    // 检查图像有效性
    if (_image.empty())
    {
        return;
    }

    Mat image = _image.getMat();
    assert(image.type() == CV_8UC3);
    
    // 初始化
    cv::Ptr<FeatureDetector> detector = ORB::create();
    cv::Ptr<DescriptorExtractor> descriptor = ORB::create();

    // 检测角点
    detector->detect(image, _keypoints);
    // 计算描述子
    descriptor->compute(image, _keypoints, _descriptors);

    /** test
    *    Mat outimg;
    *    drawKeypoints(image, keypoints, outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    *    imshow("test", outimg);
    *    waitKey(0);
    */ 
    
    
}
} // namespace bimToslam
