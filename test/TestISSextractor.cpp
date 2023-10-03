#include <gtest/gtest.h>
#include "bimToslam/frontend.h"
#include "bimToslam/dataset.h"
#include "bimToslam/ISSextractor.h"

using namespace bimToslam;
using namespace pcl;

TEST(ISSextractorTest, Featurematch) {

    std::string plyfile = "./dataset/GLT_L1_C106 - Cloud.ply";
    PointCloud<PointXYZ>::Ptr BIM_cloud(new PointCloud<PointXYZ>);

    BIM_cloud = TestreadPLYAsync(plyfile, 50);

    
    PointCloud<FPFHSignature33>::Ptr bim_descriptors(new PointCloud<FPFHSignature33>);

    ISSextractor issextractor_;
    issextractor_(BIM_cloud, 20, 0.03, 0.02, 0.975, 0.975, 0.02, bim_descriptors);

    ASSERT_FALSE(BIM_cloud->empty());
    ASSERT_FALSE(bim_descriptors->empty());

}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}