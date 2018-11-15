#ifndef QRFINDER_H_
#define QRFINDER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>

class QRFinder
{
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv::Mat bestImage;
    float bestWeight;

  public:
    QRFinder();
    cv::Mat findQR(cv::Mat src);
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    bool checkCleanBorder(cv::Mat binimg);
    void updateCurrentPublishImage(cv::Mat croppedImage, float score);
    float calculateQRWeight(cv::Mat croppedImage);
    vector<Point> findCandidate(cv::Mat binary_image);
};

#endif //QRFINDER_H_