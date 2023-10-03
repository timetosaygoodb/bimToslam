#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <algorithm>
#include "bimToslam/ICPsolver.h"

using namespace std;

namespace bimToslam {

ICPsolver::ICPsolver(const Frame::Ptr &frame, const Camera::Ptr &camera):
    pws(new double[frame->mvpISSmatches.size() * 3]{}),
    pcs(new double[frame->mvpISSmatches.size() * 3]{}), 
    us(new double[frame->mvpISSmatches.size() * 2]{})

{
    fu = camera->fx_;
    fv = camera->fy_;
    uc = camera->cx_;
    vc = camera->cy_;

    for (size_t i = 0; i < frame->mvpISSmatches.size(); i++) {
        int pointid = frame->pointID[i];
        us[i] = frame->mvpORBmatches[pointid].first.pt.x;
        us[i + 1] = frame->mvpORBmatches[pointid].first.pt.y;

        pointmatches.resize(frame->mvpISSmatches.size());
        pointmatches[i].first = frame->mvpISSmatches[i].first;
        pointmatches[i].second = frame->mvpISSmatches[i].second;
    }

}

ICPsolver::~ICPsolver()
{
    delete [] pws;
    delete [] pcs;
    delete [] us;
}

void ICPsolver::estimate_R_and_t(double R[3][3], double t[3]) {
    double pc0[3],
           pw0[3];

    pc0[0] = pc0[1] = pc0[2] = 0.0;
    pw0[0] = pw0[1] = pw0[2] = 0.0;

    for(int i = 0; i < number_of_correspondences; i++) {
        const double * pc = pcs + 3 * i;
        const double * pw = pws + 3 * i;

        for(int j = 0; j < 3; j++) {
            pc0[j] += pc[j];
            pw0[j] += pw[j];
        }
   }

    for(int j = 0; j < 3; j++) {
        pc0[j] /= number_of_correspondences;
        pw0[j] /= number_of_correspondences;
   }

    // 准备构造矩阵A,B以及B^T*A的SVD分解的值
    double abt[3 * 3], abt_d[3], abt_u[3 * 3], abt_v[3 * 3];
    CvMat ABt   = cvMat(3, 3, CV_64F, abt);       // H=B^T*A
    CvMat ABt_D = cvMat(3, 1, CV_64F, abt_d);     // 奇异值分解得到的特征值
    CvMat ABt_U = cvMat(3, 3, CV_64F, abt_u);     // 奇异值分解得到的左特征矩阵
    CvMat ABt_V = cvMat(3, 3, CV_64F, abt_v);     // 奇异值分解得到的右特征矩阵

    // Step 2 构造矩阵H=B^T*A,不过这里是隐含的构造
    cvSetZero(&ABt);
    // 遍历每一个3D点
    for(int i = 0; i < number_of_correspondences; i++) {
    // 定位
        double * pc = pcs + 3 * i;
        double * pw = pws + 3 * i;

    // 计算H=B^T*A,其中的两个矩阵构造和相乘的操作被融合在一起了
        for(int j = 0; j < 3; j++) {
            abt[3 * j    ] += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
            abt[3 * j + 1] += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
            abt[3 * j + 2] += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
        }
   }

    // Step 3 对得到的H矩阵进行奇异值分解
    cvSVD(&ABt, &ABt_D, &ABt_U, &ABt_V, CV_SVD_MODIFY_A);

    // Step 4 R=U*V^T, 并且进行合法性检查
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            R[i][j] = dot(abt_u + 3 * i, abt_v + 3 * j);
  
   // 注意在得到了R以后,需要保证 det(R)=1>0
    const double det =
     R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
     R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];
   // 如果小于0那么就要加负号
    if (det < 0) {
        R[2][0] = -R[2][0];
        R[2][1] = -R[2][1];
        R[2][2] = -R[2][2];
   }

  // Step 5 根据R计算t
    t[0] = pc0[0] - dot(R[0], pw0);
    t[1] = pc0[1] - dot(R[1], pw0);
    t[2] = pc0[2] - dot(R[2], pw0);
}     
    
double ICPsolver::reprojection_error(const double R[3][3], const double t[3]) {
    // 统计误差的平方
    double sum2 = 0.0;

    // 遍历每个3D点
    for(int i = 0; i < number_of_correspondences; i++) {
        // 指针定位
        double * pw = pws + 3 * i;
        // 计算这个3D点在相机坐标系下的坐标,逆深度表示
        double Xc = dot(R[0], pw) + t[0];
        double Yc = dot(R[1], pw) + t[1];
        double inv_Zc = 1.0 / (dot(R[2], pw) + t[2]);
        // 计算投影点
        double ue = uc + fu * Xc * inv_Zc;
        double ve = vc + fv * Yc * inv_Zc;
        // 计算投影点与匹配2D点的欧氏距离的平方
        double u = us[2 * i], v = us[2 * i + 1];
        // 得到其欧式距离并累加
        sum2 += sqrt( (u - ue) * (u - ue) + (v - ve) * (v - ve) );
    }
    // 返回平均误差
    return sum2 / number_of_correspondences;    
}

double ICPsolver::dot(const double * v1, const double * v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

void ICPsolver::EstimateCurrentPose(int iterate, double R[3][3], double t[3]) {
    
    iteration = iterate;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> distribution(0, pointmatches.size() - 1);

    vector<pair<Vec3, Vec3>> random_point;
    double min_error = 100000.0;

    for (int n = 0; n < iteration; n++) {
        random_point.clear();

        for (int i = 0; i < number_of_correspondences; i++) {
            int index = distribution(gen);
            random_point.push_back(pointmatches[index]);
        }

        for (size_t j = 0; j < random_point.size(); j++) {
            pcs[j * 3] = random_point[j].first(0, 0);
            pcs[j * 3 + 1] = random_point[j].first(1, 0);
            pcs[j * 3 + 2] = random_point[j].first(2, 0);

            pws[j * 3] = random_point[j].second(0, 0);
            pws[j * 3 + 1] = random_point[j].second(1, 0);
            pws[j * 3 + 2] = random_point[j].second(2, 0);
        }

        double R_[3][3], t_[3];
        estimate_R_and_t(R_, t_);

        double error_ = reprojection_error(R_, t_);
        if (error_ < min_error) {
            min_error = error_;

            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    R[j][k] = R_[j][k];
                }
                t[j] = t_[j];
            }
        }

    }
}

}