#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

// Candidate for being a QR fourth
class Candidate
{
private:
  // Measurement for the blurryness of an image
  float laplacianVariance;
  void evaluateLaplacianVariance();
  // Is the candidate still current?
  bool current;
  bool filterPoints(std::vector<cv::Point> unfilteredPoints, std::vector<cv::Point> &filteredPoints, cv::Point targetPoint);
  bool generateCornerPoints(cv::Mat src, std::vector<cv::Point> &points);
  bool generateMeanPoint(std::vector<cv::Point> points, cv::Point &meanPoint);
  bool cropImageToPoints(cv::Mat src, cv::Mat &dst, std::vector<cv::Point> points);
  bool cropImageToPointsRotated(cv::Mat src, cv::Mat &dst, std::vector<cv::Point> points);
  std::vector<cv::Point> allPoints; // All corner points


public:
  // Candidate();
  Candidate(cv::Mat src);
  bool isCurrent(){return (current);};
  float getLVariance() {return laplacianVariance;};
  std::vector<cv::Point> activePoints; //Filtered corner points
  cv::Point globalMeanPoint;
  cv::Mat fullFrame; // The full frame the candidate is taken from
  cv::Mat image;     // The cropped QR code

};