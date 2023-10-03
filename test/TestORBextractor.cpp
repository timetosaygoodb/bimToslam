#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "bimToslam/ORBextractor.h"

using namespace cv;
using namespace bimToslam;

TEST(ORBextractorTest, FeatureExtraction) {
    
    Mat testImage = imread("L1_C106_IFC_left.jpg", -1);
    ORBextractor orbExtractor;
    Mat descriptors;
    std::vector<KeyPoint> keypoints;

    orbExtractor(testImage, keypoints, descriptors);

    ASSERT_FALSE(descriptors.empty()); // 检查描述子是否非空
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
