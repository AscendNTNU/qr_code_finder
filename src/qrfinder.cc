#include "qrfinder.h"
#include "candidate.h"
#include "evaluation.h"
#include "debug.h"
#include "settings.h"

using namespace cv;
using namespace std;

QRFinder::QRFinder(std::string itopic, std::string otopic) : it_(nh_)
{
    // Subscribe to input video feed and advertise output video feed
    image_sub_ = it_.subscribe(itopic, 1,
                               &QRFinder::imageCb, this);
    image_pub_ = it_.advertise(otopic, 1);

    // Initialize image variables for conserving output
    cv::cvtColor(Mat::zeros(cv::Size(1, 1), CV_32F), this->bestImage, CV_GRAY2BGR);
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
    cv_ptr->image = evaluateQR(cv_ptr->image);

    // Output modified image
    image_pub_.publish(cv_ptr->toImageMsg());
}

/*
    Return image with most likely QR code fourth marked
*/
cv::Mat QRFinder::evaluateQR(cv::Mat src)
{
    Candidate qrCandidate(src);


    // Defaut to save images
    // Will be changed to false if the candidate is bad
    bool doSave = true;

    if (!qrCandidate.isCurrent())
    {
        
    }
    
    if (settings::DRAW_MEAN_POINT)
    {
        // Draw mean point
        cv::circle(qrCandidate.image, qrCandidate.globalMeanPoint, 2, Scalar(255, 0, 0), 2);
    }

    // Check for squareness
    if (!checkSquareness(qrCandidate.image))
    {
        doSave = false;
    }
    else if (!checkImageDetail(qrCandidate.image, qrCandidate.activePoints))
    {
        doSave = false;
    }
    else
    {
        try
        {
            if (!checkImageLines(qrCandidate.image))
            {
                doSave = false;
            }
        }
        catch (Exception e)
        {
            doSave = false;
        }
    }
    // Update publish image to new cropped image if good
    if (doSave)
    {
        updateCurrentPublishImage(qrCandidate.image);
    }

    // Return current highest weighted candidate
    return (this->bestImage);
}

/*
    To be called when a better QR code candidate is found
    Replaces the current published image
*/
void QRFinder::updateCurrentPublishImage(cv::Mat croppedImage)
{
        this->bestImage = croppedImage;
}