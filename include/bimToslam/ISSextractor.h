#pragma once
#ifndef BIMTOSLAM_ISSEXTRACTOR_H
#define BIMTOSLAM_ISSEXTRACTOR_H

#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <vector>

using namespace pcl;

namespace bimToslam {

class ISSextractor {
    public:
        ISSextractor() {};

        void operator()(PointCloud<PointXYZ>::Ptr cloud, int num_ksearch, double SalientRadius, 
                double NonMaxRadius, double Threshold21, double Threshold32, double RadiusSearch, PointCloud<FPFHSignature33>::Ptr descriptors);

    public:

        PointCloud<FPFHSignature33>::Ptr descriptors_;
};

}


#endif