#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
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

    // cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
  }

  // Finds a QR code in the image
  cv::Mat findQR(cv::Mat src)
{
    // Holds the processed image
    cv::Mat src_gray;

    // Convert image to gray and blur it
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);
    cv::blur(src_gray, src_gray, Size(3, 3));

    cv::Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Detect edges using canny
    cv::Canny(src_gray, canny_output, 255, 255 * 2, 3);
    // Find contours
    cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_KCOS, Point(0, 0));

    // Stores weights of contours
    vector<int> weights;

    // Determine the weighting and store it
    for (int i = 0; i < contours.size(); i++)
    {
        cv::approxPolyDP(contours[i], contours[i], 4, true);
        int shapeweight = abs((int)(4 - contours[i].size()))+1;
        shapeweight *= 2; //Weigh shape more
        int areaweight = (int)cv::contourArea(contours[i]);
        weights.push_back(areaweight / shapeweight);
    }


    int bestContour = 0;
    int bestWeight = 0;
    //Find the highest weighted contour
    for (int i = 0; i <= weights.size(); i++)
    {
        if (weights[i] > bestWeight){
            bestContour = i;
            bestWeight = weights[i];
        }
    }

    //Draw the contour on the source image
    Scalar color = Scalar(0, 0, 255);
    try
    {
      cv::drawContours(src, contours, bestContour, color, 2, 8, hierarchy, 0, Point());
    }
    catch (cv::Exception e)
    {

    }
    // Return marked image
    return(src);
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

    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);

    // Output modified video stream
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