#ifndef QRFINDER_H_
#define QRFINDER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "candidate.h"

#include <cmath>

class QRFinder
{
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_1;
    image_transport::Subscriber image_sub_2;
    image_transport::Subscriber image_sub_3;
    image_transport::Subscriber image_sub_4;
    image_transport::Publisher image_pub_;
    std::vector<Candidate> splitImageIntoCandidates(cv::Mat &originalImage, cv::Size gridSize);
    void evaluateCandidates(std::vector<Candidate> candidates, cv_bridge::CvImagePtr cv_ptr);

  public:
    QRFinder(std::array<std::string, 4> itopic, std::string otopic);
    cv::Mat evaluateQR(Candidate qrCandidate);
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
};

#endif //QRFINDER_H_