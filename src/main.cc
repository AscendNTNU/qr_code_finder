#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cv_camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/qr_code_finder/output_video", 1);
  }

  // Finds a QR code in the image
  cv::Mat findQR(cv::Mat src)
{
    // Holds the processed image
    cv::Mat src_gray;

    // Convert image to gray and blur it
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);
    cv::blur(src_gray, src_gray, Size(3, 3));

    // Stores convolution results
    cv::Mat vert;
    cv::Mat horz;

    //Initialize kernels
    float vdata[] = {1,2,1,0,0,0,-1,-2,-1};
    Mat vkernel(3,3,CV_32F,vdata);

    float hdata[] = {1,0,-1,2,0,-2,1,0,-1};
    Mat hkernel(3,3,CV_32F,hdata);

    cv::filter2D(src_gray, vert, -1, vkernel);
    cv::filter2D(src_gray, horz, -1, hkernel);
    

    cv::Mat fin;
    cv::min(vert, horz, fin);

    cv::threshold(fin, fin, 100,255,THRESH_BINARY);
    float edata[] = {2,2,2,2,2
                    ,2,2,2,2,2
                    ,2,2,4,2,2
                    ,2,2,2,2,2
                    ,2,2,2,2,2};
    Mat ekernel(5,5,CV_32F,edata);

    cv::filter2D(fin, fin, -1, ekernel);
    cv::cvtColor(fin, fin, CV_GRAY2BGR);


    return(fin);
}

  /*
   Callback when receiving image
   Republishes image with most likely QR marked
  */
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Mark image with most likely QR code
    cv_ptr->image = findQR(cv_ptr->image);

    // Output modified image
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "qr_code_finder");
  ImageConverter ic;
  ros::spin();
  return 0;
}