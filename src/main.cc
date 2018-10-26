#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>

using namespace cv;
using namespace std;

// Initializing global variables
Mat src;
Mat src_gray;
int thresh = 255;

/** 
 * Main function
 * Displays the most QR-like contour on top of source image 
**/
int main(int argc, char **argv)
{
    // Load source image
    src = imread(argv[1], 1);

    // Convert image to gray and blur it
    cvtColor(src, src_gray, CV_BGR2GRAY);
    blur(src_gray, src_gray, Size(3, 3));

    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Detect edges using canny
    Canny(src_gray, canny_output, thresh, thresh * 2, 3);
    // Find contours
    findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_KCOS, Point(0, 0));

    // Stores weights of contours
    vector<int> weights;

    // Determine the weighting and store it
    for (int i = 0; i < contours.size(); i++)
    {
        approxPolyDP(contours[i], contours[i], 4, true);
        int shapeweight = abs((int)(4 - contours[i].size()))+1;
        shapeweight *= 2; //Weigh shape more
        int areaweight = (int)contourArea(contours[i]);
        weights.push_back(areaweight / shapeweight);
        cout << shapeweight << ", " << areaweight << " | "<< weights[i] << endl;
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
    drawContours(src, contours, bestContour, color, 2, 8, hierarchy, 0, Point());
    cout <<"Edges: " << contours[bestContour].size() << endl;
    cout <<"Weight: " << weights[bestContour] << endl;

    /// Show in a window
    cv::resize(src, src, cv::Size(), 0.3, 0.3);
    namedWindow("Contours", CV_WINDOW_AUTOSIZE);
    imshow("Contours", src);
    waitKey(0);
    return (0);
}