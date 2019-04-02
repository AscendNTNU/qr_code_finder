#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <ros/console.h>

// Candidate for being a QR fourth
class Candidate
{
private:
    // Is the candidate still relevant?
    // This is a bad way of doing this, but works
    bool relevant = false;

    bool generateCornerPoints(cv::Mat src, std::vector<cv::Point> &points);
    bool cropImageToPointsRotated(cv::Mat src, cv::Mat &dst, std::vector<cv::Point> points);


public:
    // Candidate();
    explicit Candidate(cv::Mat src);
    bool isRelevant() { return(relevant);};
    std::vector<cv::Point> allPoints; // All corner points
    cv::Mat fullFrame; // The full frame the candidate is taken from
    cv::Mat image;     // The cropped QR code

};