#pragma once

#ifndef BIMTOSLAM_ORBEXTRACTOR_H
#define BIMTOSLAM_ORBEXTRACTOR_H

#include "bimToslam/common_include.h"

namespace bimToslam
{
class Feature;

class ORBextractor {
public:
    ORBextractor() {};

    void operator()(cv::InputArray &image, std::vector<cv::KeyPoint>& _keypoints, cv::OutputArray descriptors);
};
} // namespace bimToslam

#endif
