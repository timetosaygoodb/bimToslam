#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "bimToslam/frontend.h"

using namespace bimToslam;
using namespace cv;

TEST(BIMtoSLAM, GetPose) {

    Mat Left_img = imread("L1_C106_IFC_left.jpg");
    Mat right_img = imread("L1_C106_IFC_right.jpg");
    Frame::Ptr frame(new Frame(Left_img, right_img));

    Camera::Ptr camera(new Camera(596.87, 447.65, 320.0, 240.0, 179.06));


    Frontend testFrontend;
    testFrontend.SetCameras(camera);
    testFrontend.plyfile = "./dataset/GLT_L1_C106 - Cloud.ply";
    testFrontend.AddFrame(frame);

}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}