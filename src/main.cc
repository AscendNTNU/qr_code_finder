#include "qrfinder.h"

using namespace cv;
using namespace std;


/*
Callback when receiving image
Republishes image with most likely QR marked
*/
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "qr_code_finder");
  QRFinder ic;
  ros::spin();
  return 0;
}