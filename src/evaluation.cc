#include "evaluation.h"
#include "settings.h"
#include "debug.h"
using namespace cv;
using namespace std;

bool checkSquareness(cv::Mat image)
{
    double squareScore = static_cast<double>(image.cols) / static_cast<double>(image.rows);
    if (abs(squareScore) >= 1 + settings::SQUARENESS_MARGIN || abs(squareScore) <= 1 - settings::SQUARENESS_MARGIN)
    { //
        return false;
    }
    return true;
}

bool checkImageDetail(cv::Mat image, std::vector<cv::Point> points)
{
    if (image.cols < settings::MIN_CROP_ROWS || image.rows < settings::MIN_CROP_ROWS)
    { // Too small image, not enough detail
        return false;
    }
    else if (points.size() < 8 || points.size() > 200)
    { // Too few corners
        // TODO: Switch this out with something that checks inside the cropped area
        return false;
    }
    return true;
}

bool checkImageLines(cv::Mat image)
{
    Mat sobelFrame, sobelFrameNorm, outFrameGray, outFrameBin;
    cvtColor(image, outFrameGray, CV_BGR2GRAY);

    // Detect horizontal lines
    // Sobel(outFrameGray, sobelFrame, CV_16S, 0, 1, 3, 1, 0, BORDER_DEFAULT);
    Canny(outFrameGray, sobelFrame, 80, 255, 3, false);
    convertScaleAbs(sobelFrame, sobelFrameNorm);

    // sobelFrameNorm.copyTo(outFrame);
    vector<Vec2f> lines;
    HoughLines(sobelFrameNorm, lines, 1, CV_PI / 180, 40);

    if (lines.empty())
    {
        if(settings::VERBOSE_DEBUG)
           ROS_DEBUG("NO LINES FOUND");

        return false;
    }


    if(settings::VERBOSE_DEBUG)
        ROS_DEBUG("NUMBER OF LINES: %i", lines.size());

    int counted = 0;
    int wrongAngleCount = 0;
    for (Vec2f l : lines)
    {
        if (l[0] < 20) continue; // Line is too short to use

        float theta = l[1];

        bool lineAngleIsWrong = (abs(0 - theta) > settings::ANGLE_MARGIN) &&
                                (abs(CV_PI - theta) > settings::ANGLE_MARGIN) &&
                                (abs(CV_PI / 2 - theta) > settings::ANGLE_MARGIN) &&
                                (abs(CV_PI * 3 / 2 - theta) > settings::ANGLE_MARGIN);

        counted++;
        if (lineAngleIsWrong)
        {
            wrongAngleCount++;
        }
    }

    if (settings::DRAW_LINES)
    {
        // Draw the lines
        for (size_t i = 0; i < lines.size(); i++)
        {
            float rho = lines[i][0], theta = lines[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a * rho, y0 = b * rho;
            pt1.x = cvRound(x0 + 1000 * (-b));
            pt1.y = cvRound(y0 + 1000 * (a));
            pt2.x = cvRound(x0 - 1000 * (-b));
            pt2.y = cvRound(y0 - 1000 * (a));
            line(image, pt1, pt2, Scalar(0, 0, 255), 1);
        }
    }

    if (wrongAngleCount >= counted / 2)
    {
        return false;
    }
    return true;
}