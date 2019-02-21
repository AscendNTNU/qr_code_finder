#pragma once
#include <opencv2/opencv.hpp>

namespace settings
{
/*
    General
*/
const bool DRAW_ALL_POINTS = false;
const bool DRAW_POINTS = false;
const bool DRAW_MEAN_POINT = false;
const bool DRAW_LINES = false;
const bool DRAW_BOUNDS = false;

/*
    Image sectioning
*/

const cv::Size SECTION_GRID_SIZE_1{3,3};
const cv::Size SECTION_GRID_SIZE_2{2,2};


/* 
    Point generation
*/

// Blur kernel size
const cv::Size BLUR_KERNEL_SIZE = cv::Size(3, 3);

// Harris block size
const int HARRIS_BLOCK_SIZE = 2;

// Harris kernel size
const int HARRIS_KERNEL_SIZE = 3;

// The brightness of a pixel on normalized Harris image needed to count as a corner point
const int CORNER_POINT_THRESHOLD = 180;

// The amount of corner points allowed before image is scrapped
const int MAX_ALLOWED_POINTS = 500;

// Multiplier for getting the maximum allowad distance from the mean point. 
const float MEAN_POINT_DISTANCE_MULTIPLIER = 1.4;

/* 
    QR validation
*/

// The amount lines are allowed to differ from horizontal or vertical before the candidate is discarded.
const float ANGLE_MARGIN = 0.01;

// The amount cropped images are allowed to differ from square before the candidate is discarded.
const float SQUARENESS_MARGIN = 0.1;

// Minimum amount of rows and columns required to keep the candidate
const int MIN_CROP_ROWS = 50;

} // namespace settings