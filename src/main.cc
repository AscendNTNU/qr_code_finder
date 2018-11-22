#include "qrfinder.h"

using namespace cv;
using namespace std;
  
int main(int argc, char **argv)
{
  string inpath = argv[1];
  string outpath = argv[2];

  ros::init(argc, argv, "qr_code_finder");
  QRFinder ic(inpath, outpath);
  ros::spin();
  return 0;
}