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
    // Subscribe to input video feed and advertise output video feed
    image_sub_ = it_.subscribe("/cv_camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/qr_code_finder/output_video", 1);
  }

  // Find center from array of points
  Point getCenter(InputArray Points)
  {
    Point centerCoord;
    Moments mom = moments(Points);
    centerCoord.x = int(mom.m10 / mom.m00);
    centerCoord.y = int(mom.m01 / mom.m00);
    return centerCoord;
  }

  // Finds a QR code in the image
  cv::Mat findQR(cv::Mat src)
{
    // Holds the processed image
    cv::Mat src_gray;

    // Convert image to gray and blur it
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);
    cv::blur(src_gray, src_gray, Size(3, 3));

    cv::Mat edges;
    cv::Canny(src_gray, edges, 100,255);
    cv::Mat fin = edges;
    
    vector<Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI/180,50,50,10);
    
    cv::cvtColor(fin, fin, CV_GRAY2BGR);
    for( size_t i = 0; i < lines.size(); i++ )
    {
      Vec4i l = lines[i];
      line( fin, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }
    // rectangle(fin, qrCenter, qrCenter, Scalar(0,0,255),8);


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