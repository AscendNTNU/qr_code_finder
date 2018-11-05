#include "qrfinder.h"

using namespace cv;
using namespace std;

QRFinder::QRFinder() : it_(nh_)
{
    // Subscribe to input video feed and advertise output video feed
    image_sub_ = it_.subscribe("/cv_camera/image_raw", 1,
                               &QRFinder::imageCb, this);
    image_pub_ = it_.advertise("/qr_code_finder/output_video", 1);
}

/*
    Callback for when an image is received
    republishes with most likely QR code marked
*/
void QRFinder::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Mark image with most likely QR code
    cv_ptr->image = findQR(cv_ptr->image);

    // Output modified image
    image_pub_.publish(cv_ptr->toImageMsg());
}

/*
    Return image with most likely QR code marked
*/
cv::Mat QRFinder::findQR(cv::Mat src)
{
    // Holds the processed image
    cv::Mat src_gray;

    // Convert image to gray and blur it
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);
    cv::blur(src_gray, src_gray, Size(3, 3));

    // Detect lines in the image
    cv::Mat edges;
    cv::Canny(src_gray, edges, 100, 255);
    vector<Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);

    // Draw lines and detect contours
    cv::Mat fin = Mat::zeros(edges.size(), edges.type());
    cv::Mat lns = Mat::zeros(edges.size(), edges.type());
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        line(lns, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 5, CV_AA);
    }


    cv::Canny(lns,lns, 100, 255);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cv::findContours(lns,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_TC89_KCOS, Point(0,0));
    ROS_INFO("contours: %i", contours.size());
    cv::cvtColor(fin, fin, CV_GRAY2BGR);
    for(int i=0; i< contours.size(); i++)
    {
        cv::approxPolyDP(contours[i], contours[i], 100, true);
        if(contours[i].size() == 4)
        {
            cv::drawContours(fin,contours,i,cv::Scalar(255,255,255),2,8,hierarchy,0,Point(0,0));
        }
    }

    return (fin);
}
