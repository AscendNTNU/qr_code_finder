#include "debug.h"
#include "candidate.h"
#include "settings.h"

using namespace cv;
using namespace std;

// Generates a vector of points from corners in image. Returns success status
bool Candidate::generateCornerPoints(Mat src, vector<Point> &points)
{
    Mat grayFrame, grayFrameBlurred, cornersFrame, cornersFrameNorm;
    // Mat blank = Mat::zeros(src.size(), CV_32FC1);
    cv::cvtColor(src, grayFrame, CV_BGR2GRAY);
    cv::blur(grayFrame, grayFrameBlurred, settings::BLUR_KERNEL_SIZE);
    cv::cornerHarris(grayFrameBlurred, cornersFrame, settings::HARRIS_BLOCK_SIZE, settings::HARRIS_KERNEL_SIZE, 0.04, BORDER_DEFAULT);
    cv::normalize(cornersFrame, cornersFrameNorm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());

    // Collect points
    Point pSum = Point(0, 0);
    vector<Point> cornerPoints;
    for (int j = 0; j < cornersFrameNorm.rows; j++)
    {
        for (int i = 0; i < cornersFrameNorm.cols; i++)
        {
            if ((int)cornersFrameNorm.at<float>(j, i) > settings::CORNER_POINT_THRESHOLD)
            {
                cornerPoints.push_back(Point(i, j));

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

// Get the mean from a list of points
bool Candidate::generateMeanPoint(std::vector<cv::Point> points, cv::Point &meanPoint)
{
    // Don't divide by 0
    if (points.size() == 0)
    {
        cout << "NO CORNERS | ";
        return (false); //FAIL
    }

    Point sumPoint;
    for (Point p : points)
    {
        sumPoint += p;
    }
    meanPoint = Point(sumPoint.x / points.size(), sumPoint.y / points.size());
    return (true); // SUCCESS
}

// Crop image to the bounding box of the points
bool Candidate::cropImageToPoints(cv::Mat src, cv::Mat &dst, vector<Point> points)
{
    try
    {
        // Make contour from points and draw bounding box
        Mat outFrame;
        vector<Point> contour_poly;
        approxPolyDP(points, contour_poly, 3, true);
        Rect boundRect = boundingRect(contour_poly);

        /*
            SCALES THE BOUNDING BOX UP. NOT IN USE
            This may be used if the crop turns out to be too small, but feels really hacky
        */
        // double scaleFactor = 1.2;

        // int newHeight = static_cast<int>(static_cast<double>(boundRect.height) * scaleFactor);
        // int newWidth = static_cast<int>(static_cast<double>(boundRect.width) * scaleFactor);

        // int dHeight = newHeight - boundRect.height;
        // int dWidth = newWidth - boundRect.width;

        // boundRect.height = newHeight;
        // boundRect.width = newWidth;
        // boundRect -= Point(dWidth / 2, dHeight / 2);

        dst = src(boundRect);
        return (true); // SUCCESS
    }
    catch (Exception e)
    {
        src.copyTo(dst);
        return (false); // FAIL
    }
}

// Crop image to the rotated bounding box of the points
bool Candidate::cropImageToPointsRotated(cv::Mat src, cv::Mat &dst, vector<Point> points)
{
    try
    {
        cv::RotatedRect bBox = minAreaRect(points);

        if (settings::DRAW_BOUNDS)
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
        cropped.copyTo(dst);
        return (true); // SUCCESS
    }
    catch (Exception e)
    {
        src.copyTo(dst);
        return (false); // FAIL
    }
}

// Filter away outliers (TODO: This is a really bad way of doing this but sorta works, fix later)
bool Candidate::filterPoints(std::vector<cv::Point> unfilteredPoints, std::vector<cv::Point> &filteredPoints, cv::Point targetPoint)
{
    vector<Point> goodPoints;
    vector<float> pointDistances;
    float maxDist = 0;

    // Calculate maximum permitted distance from target point
    for (Point p : unfilteredPoints)
    {
        float dist = ((p.x - targetPoint.x) * (p.x - targetPoint.x) + (p.y - targetPoint.y) * (p.y - targetPoint.y));
        pointDistances.push_back(dist);
        maxDist += dist;
    }

    maxDist /= pointDistances.size();
    maxDist *= settings::MEAN_POINT_DISTANCE_MULTIPLIER;

    // Only add points closer than the max distance
    for (int i = 0; i < unfilteredPoints.size(); i++)
    {
        if (pointDistances[i] < maxDist)
        {
            goodPoints.push_back(unfilteredPoints[i]);
        }
    }

    filteredPoints = goodPoints;
    return (true); // SUCCESS
}

// QR candidate constructor
Candidate::Candidate(cv::Mat src)
{
    current = false;

    if (src.empty())
    {
        cout << "Empty image | ";
        return; // Don't handle empty images
    }

    src.copyTo(fullFrame); // Store image in class

    /*
        If any of these functions fail, the candidate is useless. 
        Return is called and current is still set to false.    
    */
    src.copyTo(image);
    if (!generateCornerPoints(fullFrame, allPoints))
    {
#ifdef DEBUG
        cout << "generateCornerPoints failed | ";
#endif //DEBUG
        return;
    }
    if (!generateMeanPoint(allPoints, globalMeanPoint))
    {
#ifdef DEBUG
        cout << "generateMeanPoint failed | ";
#endif //DEBUG
        return;
    }
    if (!filterPoints(allPoints, activePoints, globalMeanPoint))
    {
#ifdef DEBUG
        cout << "filterPoints failed | ";
#endif //DEBUG
        return;
    }
    // if (!cropImageToPoints(fullFrame, image, activePoints))
    if (!cropImageToPointsRotated(fullFrame, image, activePoints))
    {
#ifdef DEBUG
        cout << "cropImageToPoints failed | ";
#endif //DEBUG
        return;
    }
    current = true;
}