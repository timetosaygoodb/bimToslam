#include "bimToslam/frame.h"


using namespace std;
using namespace cv;

namespace bimToslam {

Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight) 
: imLeft_(imLeft), imRight_(imRight) {}

}