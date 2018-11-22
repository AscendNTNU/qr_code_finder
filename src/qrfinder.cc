#include "qrfinder.h"

using namespace cv;
using namespace std;

QRFinder::QRFinder(std::string itopic, std::string otopic) : it_(nh_)
{
    // Subscribe to input video feed and advertise output video feed
    image_sub_ = it_.subscribe(itopic, 1,
                               &QRFinder::imageCb, this);
    image_pub_ = it_.advertise(otopic, 1);

    // Initialize image variables for conserving output
    cv::cvtColor(Mat::zeros(cv::Size(1,1),CV_32F),this->bestImage,CV_GRAY2BGR);
    this->bestWeight = 0;
}

/*
    Checks if none of the border pixels are white by
    applying a border mask and checking for nonzero pixels
    Used to discard image if QR code is cropped
*/
bool QRFinder::checkCleanBorder(cv::Mat binimg)
{
    cv::Mat mask, fin;
    int borderWidth = 5; // Width of mask border
    cv::Mat canvas = cv::Mat::zeros(binimg.size(),binimg.type());
    cv::copyMakeBorder(canvas,canvas,borderWidth,borderWidth,borderWidth,borderWidth,BORDER_CONSTANT,cv::Scalar(255,255,255));
    cv::resize(canvas,mask,binimg.size());
    binimg.copyTo(fin,mask);
    return(cv::countNonZero(fin) == 0);
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
    Return image with most likely QR code fourth marked
*/
cv::Mat QRFinder::findQR(cv::Mat src)
{
    // Holds the processed image
    cv::Mat src_gray, mat_out, edges;
    cv::Mat blank = cv::Mat::zeros(src.size(),src.type());
    
    // Convert image to gray and blur it
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);
    cv::blur(src_gray, src_gray, Size(20, 20));

    // Convert to binary image. Makes it easier to separate QR code from environment.
    cv::adaptiveThreshold(src_gray, src_gray,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY_INV,11,2);


    // Find the best contour in the frame
    vector<Point> candidate;
    findCandidate(src_gray, candidate);

    // Crop source image to candidate bounds
    Rect boundR = cv::boundingRect(cv::Mat(candidate));
    cv::Mat croppedImage = src(boundR);

    // Calculate QR weight
    float candidateWeight = calculateQRWeight(croppedImage);

    // Only update if weight is better than current published image
    if (this->bestWeight <= candidateWeight) 
    {
        updateCurrentPublishImage(croppedImage, candidateWeight);
    }

    // Return current highest weighted candidate
    return(this->bestImage);
}


/*
    Finds the largest square-like contour in the image
    This is the contour most likely to contain the QR code
*/
void QRFinder::findCandidate(cv::Mat binary_image, vector<Point> &candidate)
{
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<Vec4i> lines;

    // Draw lines and detect contours
    cv::HoughLinesP(binary_image, lines, 1, CV_PI / 180, 50, 50, 10);
    cv::Mat lns = Mat::zeros(binary_image.size(), binary_image.type());
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        line(lns, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255),1, CV_AA);
    }

    cv::findContours(lns,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_TC89_KCOS, Point(0,0));    

    // No need to check every contour, slows down the processing
    // Check only the biggest contour detected in each frame.
    vector<vector<Point> > screenCandidates;
    float biggestArea = 0;
    int biggestAreaIndex = 0;
    for(int i=0; i< contours.size(); i++)
    {
        cv::approxPolyDP(contours[i], contours[i], 80, true);
        if(std::abs(4-contours[i].size()) <= 1)
        {
            screenCandidates.push_back(contours[i]);
            if((float)cv::contourArea(contours[i]) > biggestArea)
            {
                biggestArea = (float)cv::contourArea(contours[i]);
                biggestAreaIndex = i;
            }
        }
    }
    if(biggestArea != 0)
    {
        candidate = contours[biggestAreaIndex];
    } else {
        // TODO: Figure out better way to deal with this than to return a random contour
        candidate = contours[0];
    }
}

/*
    To be called when a better QR code candidate is found
    Replaces the current published image
*/
void QRFinder::updateCurrentPublishImage(cv::Mat croppedImage, float score)
{
    // Convert cropped image to grayscale
    cv::Mat croppedBin;
    cv::cvtColor(croppedImage, croppedImage, CV_BGR2GRAY);

    // Only output images if the QR code is not cropped
    cv::blur(croppedImage,croppedBin,Size(2,2));
    cv::threshold(croppedBin, croppedBin,200,255,THRESH_BINARY);
    if (this->checkCleanBorder(croppedBin))
    {
    // Image needs to be in BGR format for cv_bridge
    cv::cvtColor(croppedImage, croppedImage, CV_GRAY2BGR);
    this->bestWeight = score;
    this->bestImage = croppedImage;
    // ROS_INFO("New best weight: %f", totScore);
    }
}

float QRFinder::calculateQRWeight(cv::Mat croppedImage)
{   
    // Weigh cropped image based on edgyness
    cv::Mat cEdges = cv::Mat::zeros(croppedImage.size(),croppedImage.type());
    cv::Canny(croppedImage, cEdges, 100,255);
    cv::Canny(cEdges, cEdges, 100, 255);        
    cv::Scalar eScore = cv::mean(cEdges);
    float edgeScore = (eScore[0] + eScore[1] + eScore[2] + eScore[3])/4;
    
    // Weigh based on squareness
    float squareScore = std::abs(croppedImage.size().height - croppedImage.size().width);

    // Combine weights
    float totScore = 0;
    if (squareScore != 0)
    {
        totScore = (edgeScore / squareScore) + (edgeScore / 7);
    }
    return(totScore);
}