#include "debug.h"
#include "candidate.h"
#include "settings.h"

using namespace cv;
using namespace std;

// Generates a vector of points from corners in image. Returns success status
bool Candidate::generateCornerPoints(Mat src, vector<Point> &points)
{
    Mat grayFrame, grayFrameBlurred, cornersFrame, cornersFrameNorm;
    cv::cvtColor(src, grayFrame, CV_BGR2GRAY);
    cv::blur(grayFrame, grayFrameBlurred, settings::BLUR_KERNEL_SIZE);
    cv::cornerHarris(grayFrameBlurred, cornersFrame, settings::HARRIS_BLOCK_SIZE, settings::HARRIS_KERNEL_SIZE, 0.04, BORDER_DEFAULT);
    cv::normalize(cornersFrame, cornersFrameNorm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());

    // Collect points
    vector<Point> cornerPoints;
    for (int j = 0; j < cornersFrameNorm.rows; j++)
    {
        for (int i = 0; i < cornersFrameNorm.cols; i++)
        {
            if ((int)cornersFrameNorm.at<float>(j, i) > settings::CORNER_POINT_THRESHOLD)
            {
                cornerPoints.emplace_back(Point(i, j));

                // Discard image if too many points are found. Most likely unusable
                // This is inside the loop to avoid a bug that finds thousands of points.
                if (cornerPoints.size() > settings::MAX_ALLOWED_POINTS)
                    return (false); // FAIL
            }
        }
    }
    points = cornerPoints;
    return (true); // SUCCESS
}

// Crop image to the rotated bounding box of the points
bool Candidate::cropImageToPointsRotated(cv::Mat src, cv::Mat &dst, vector<Point> points)
{
    try
    {
        cv::RotatedRect bBox = minAreaRect(points);

        if (false)
        {
            //DRAW FOR DEBUG
            Point2f vertices[4];
            bBox.points(vertices);
            for (int i = 0; i < 4; i++)
            {
                line(src, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 2);
            }
        }
        Mat M, rotated, cropped;
        // get angle and size from the bounding box
        float angle = bBox.angle;
        Size rect_size = bBox.size;
        if (bBox.angle < -45.)
        {
            angle += 90.0;
            int hold = rect_size.width;
            rect_size.width = rect_size.height;
            rect_size.height = hold;
            //swap(rect_size.width, rect_size.height);
        }
        // get the rotation matrix
        // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
        M = getRotationMatrix2D(bBox.center, angle, 1.0);
        // perform the affine transformation
        warpAffine(image, rotated, M, image.size(), INTER_CUBIC);
        // crop the resulting image
        getRectSubPix(rotated, rect_size, bBox.center, cropped);
        if(false)
        {
            src.copyTo(dst);
        }else{
            cropped.copyTo(dst);
        }
        return (true); // SUCCESS
    }
    catch (...)
    {
        src.copyTo(dst);
        return (false); // FAIL
    }
}


// QR candidate constructor
Candidate::Candidate(cv::Mat src)
{
    relevant = false;

    if (src.empty())
    {
       ROS_ERROR("Empty image");
        return; // Don't handle empty images
    }

    src.copyTo(fullFrame); // Store image in class
    src.copyTo(image);

    /*
        If any of these functions fail, the candidate is useless. 
        Return is called and relevant is still set to false.
    */
    if (!generateCornerPoints(fullFrame, allPoints))
    {
        if(settings::VERBOSE_DEBUG)
            ROS_ERROR("generateCornerPoints failed");

        return;
    }
    if (!cropImageToPointsRotated(fullFrame, image, allPoints))
    {
        if(settings::VERBOSE_DEBUG)
            ROS_ERROR("cropImageToPoints failed");

        return;
    }
    relevant = true;
}