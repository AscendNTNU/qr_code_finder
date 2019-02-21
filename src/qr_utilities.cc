#include "qr_utilities.h"

cv::Size_<int> operator/(const cv::Size_<int> &leftSize, const cv::Size_<int> &rightSize)
{
    return cv::Size_<int>(leftSize.width / rightSize.width, leftSize.height / rightSize.height);
}