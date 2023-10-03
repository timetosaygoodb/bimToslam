#pragma once
#ifndef BIMTOSLAM_ICPSOLVER_H
#define BIMTOSLAM_ICPSOLVER_H

#include <opencv2/core/core.hpp>
#include "bimToslam/frame.h"
#include "bimToslam/camera.h"

namespace bimToslam {

class Frame;
class Camera;

class ICPsolver {
public:
    ICPsolver(const Frame::Ptr &frame, const Camera::Ptr &camera);

    void EstimateCurrentPose(int iterate, double R[3][3], double t[3]);

    ~ICPsolver();
private:
    
    void estimate_R_and_t(double R[3][3], double t[3]);

    double reprojection_error(const double R[3][3], const double t[3]);

    double dot(const double * v1, const double * v2);

    double uc, vc, fu, fv;

    double * pws, * pcs, * us;

    std::vector<std::pair<Vec3, Vec3>> pointmatches;

    int number_of_correspondences = 4;

    int iteration;

};
}

#endif