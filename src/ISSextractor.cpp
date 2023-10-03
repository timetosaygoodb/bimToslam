#include "bimToslam/ISSextractor.h"

using namespace pcl;

namespace bimToslam {
    void ISSextractor::operator()(PointCloud<PointXYZ>::Ptr cloud, int num_ksearch, double SalientRadius, double NonMaxRadius, double Threshold21, 
                                  double Threshold32, double RadiusSearch,  PointCloud<FPFHSignature33>::Ptr descriptors)

    {
    if (cloud == nullptr)
        return;                    
    // 法线估计
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
    NormalEstimation<PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(num_ksearch);  // 设置搜索邻域的点数
    ne.compute(*normals);
    
    // ISS检测器
    PointCloud<PointXYZ>::Ptr keypoints(new PointCloud<PointXYZ>);
    ISSKeypoint3D<PointXYZ, PointXYZ> iss;
    iss.setInputCloud(cloud);
    iss.setSearchMethod(tree);
    iss.setSalientRadius(SalientRadius);   // 设置关键点搜索参数
    iss.setNonMaxRadius(NonMaxRadius);
    iss.setThreshold21(Threshold21);
    iss.setThreshold32(Threshold32);
    iss.compute(*keypoints);

    //int n1 = keypoints->size();
    //int n3 = cloud->size();
    //int n2 = normals->size();

    // 计算FPFH描述符
    FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);

    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(RadiusSearch);  // 设置搜索半径
    fpfh.compute(*descriptors);

    }
}