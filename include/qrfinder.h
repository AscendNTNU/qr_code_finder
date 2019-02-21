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
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv::Mat bestImage;
    std::vector<Candidate> splitImageIntoCandidates(cv::Mat &originalImage, cv::Size gridSize);
    bool gridSizeToggleBool;

  public:
    QRFinder(std::string itopic, std::string otopic);
    cv::Mat evaluateQR(Candidate qrCandidate);
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    void updateCurrentPublishImage(cv::Mat croppedImage);
};

#endif //QRFINDER_H_