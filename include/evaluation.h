#pragma once
#include <opencv2/opencv.hpp>
#include <ros/console.h>


bool checkSquareness(cv::Mat image);
bool checkImageDetail(cv::Mat image, std::vector<cv::Point> points);
bool checkImageLines(cv::Mat image);