#include "qrfinder.h"

using namespace cv;
using namespace std;

QRFinder::QRFinder() : it_(nh_)
{
    // Subscribe to input video feed and advertise output video feed
    image_sub_ = it_.subscribe("/bottomcamera/image_raw", 1,
                               &QRFinder::imageCb, this);
    image_pub_ = it_.advertise("/qr_code_finder/output_image", 1);

    // Initialize image variables for smoothing of output
    cv::cvtColor(Mat::zeros(cv::Size(1,1),CV_32F),this->lastImage,CV_GRAY2BGR);
    this->lastScore = 0;
}

/*
    Checks if none of the border pixels are white
    Used to discard image if QR code is cropped
*/
bool QRFinder::checkCleanBorder(cv::Mat binimg)
{
    // return(true);
    cv::Mat canvas;
    int borderWidth = 10;
    cv::Mat mask = cv::Mat::ones(binimg.size(),binimg.type());
    cv::Rect mrect = cv::Rect(borderWidth, borderWidth, mask.rows - borderWidth, mask.cols - borderWidth);
    cv::rectangle(mask,mrect,0,-1);
    binimg.copyTo(canvas,mask);
    return(cv::countNonZero(canvas) == 0);
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
    cv::Mat src_gray, mat_out, edges;
    cv::Mat blank = cv::Mat::zeros(src.size(),src.type());
    // Convert image to gray and blur it
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);
    cv::blur(src_gray, src_gray, Size(20, 20));

    cv::adaptiveThreshold(src_gray, src_gray,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY_INV,11,2);
    //cv::threshold(src_gray,mat_out,20,255,THRESH_BINARY); 

    // cv::blur(src_gray, src_gray, Size(20, 20));
    // cv::threshold(src_gray, src_gray,60,255,THRESH_BINARY);



    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    // cv::Canny(src_gray, edges, 100, 255);
    vector<Vec4i> lines;
    cv::HoughLinesP(src_gray, lines, 1, CV_PI / 180, 50, 50, 10);

    // Draw lines and detect contours
    cv::Mat lns = Mat::zeros(src_gray.size(), src_gray.type());
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        line(lns, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255),1, CV_AA);
    }
    // cv::Canny(lns,lns, 100, 255);
    

    cv::findContours(lns,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_TC89_KCOS, Point(0,0));
    cv::cvtColor(lns, mat_out, CV_GRAY2BGR);
    
    // for(int i = 0; i<contours.size();i++)
    // {
    //     cv::approxPolyDP(contours[i], contours[i], 80, true);
    //     if(std::abs(4-contours[i].size()) <= 1)
    //     {    
    //         cv::drawContours(mat_out,contours,i,cv::Scalar(0,255,0),5);
    //     }
    // }
    

    // return(mat_out);
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

    // Update current publish image if a new candidate is found
    if (biggestArea != 0)
    {
        // Crop source image to candidate bounds
        Rect boundR = cv::boundingRect(cv::Mat(contours[biggestAreaIndex]));
        cv::Mat croppedImage = src(boundR);
        
        // Weigh cropped image based on edgyness
        cv::Mat cEdges = cv::Mat::zeros(croppedImage.size(),croppedImage.type());
        cv::Canny(croppedImage, cEdges, 100,255);
        cv::Canny(cEdges, cEdges, 100, 255);        
        cv::Scalar eScore = cv::mean(cEdges);
        float edgeScore = (eScore[0] + eScore[1] + eScore[2] + eScore[3])/4;
        
        float squareScore = std::abs(croppedImage.size().height - croppedImage.size().width);

        // Combine weights
        float totScore = 0;
        if (squareScore != 0)
        {
            totScore = (edgeScore / squareScore) + (edgeScore / 7);
        }

        // Debug
        // ROS_INFO("edgeScore: %f",edgeScore);
        // ROS_INFO("squareScore: %f",squareScore);
        // ROS_INFO("totScore: %f",totScore);


        // Only update if score is better than current candidate
        if (this->lastScore <= totScore) 
        {
            try {
                float xFactor = boundR.width / 10;
                boundR -= cv::Point(xFactor, xFactor);
                boundR += cv::Size(2 * xFactor, 2* xFactor);
                croppedImage = src(boundR);
            } catch (Exception e){
                ROS_WARN("Could not expand image");
                return (this->lastImage);
            }

            cv::Mat croppedBin;
            cv::cvtColor(croppedImage, croppedImage, CV_BGR2GRAY);
            cv::blur(croppedImage,croppedBin,Size(2,2));
            cv::threshold(croppedBin, croppedBin,200,255,THRESH_BINARY);

            // Only output images if the QR code is not cropped
            if (this->checkCleanBorder(croppedBin))
            {
            cv::cvtColor(croppedImage, croppedImage, CV_GRAY2BGR);
            this->lastScore = totScore;
            this->lastImage = croppedImage;
            ROS_INFO("New best weight: %f", totScore);
            }
        }
    }

    // Return current candidate
    return(this->lastImage);
}
