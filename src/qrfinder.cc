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

    for(const cv::Size &s : settings::SECTION_GRID_SIZES)
    {
        evaluateCandidates(splitImageIntoCandidates(cv_ptr->image, s), cv_ptr);
    }

}

void QRFinder::evaluateCandidates(std::vector<Candidate> candidates, cv_bridge::CvImagePtr cv_ptr)
{
    for (const Candidate &c : candidates)
    {
        try{
            // This will throw if no usable fourth is found
            Mat result = evaluateQR(c);
            cv_ptr->image = result;
            image_pub_.publish(cv_ptr->toImageMsg());
        }catch(...)
        {
        }
    }
}

// Splits image into smaller images in grid pattern
std::vector<Candidate> QRFinder::splitImageIntoCandidates(cv::Mat &originalImage, cv::Size gridSize)
{
    using namespace std;
    using namespace cv;
    vector<Candidate> tempVector;

    cv::Size_<int> boxSize(originalImage.size().width / gridSize.width, originalImage.size().height / gridSize.height);
    for (int x = 0; x < gridSize.width; x++)
    {
        for (int y = 0; y < gridSize.height; y++)
        {
            Rect box(Point(boxSize.width * x, boxSize.height * y), boxSize);
            Candidate boxCandidate(originalImage(box));
            tempVector.push_back(boxCandidate);
        }
    }
    return tempVector;
}

/*
 * Return image cropped to qr fourth. Throws if nothing is found
*/
cv::Mat QRFinder::evaluateQR(Candidate qrCandidate)
{

    // Default to save images
    // Will be changed to false if the candidate is bad
    bool doSave = true;

    if (!qrCandidate.isRelevant())
    {
        doSave = false;
    }
    if (!checkSquareness(qrCandidate.image))
    {
        doSave = false;
    }
    else if (!checkImageDetail(qrCandidate.image, qrCandidate.allPoints))
    {
        doSave = false;
    }
    else
    {
        try
        {
            // Are lines orthogonal to each other?
            if (!checkImageLines(qrCandidate.image))
            {
                doSave = false;
            }
        }
        catch (...)
        {
            doSave = false;
        }
    }
    if (doSave)
    {
        return qrCandidate.image;
    } else
    {
        throw -1; // Don't publish anything
        // This will be caught after call to publish()
    }
}

