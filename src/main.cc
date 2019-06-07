#include "qrfinder.h"

using namespace cv;
using namespace std;
  
int main(int argc, char **argv)
{
  //string inpath = argv[1];
  string outpath = "qr_code_finder/output_image";

  std::array<std::string, 4> inpath {
    "drone_1/data/video1",
    "drone_2/data/video1",
    "drone_3/data/video1",
    "drone_4/data/video1",
  };

  ros::init(argc, argv, "qr_code_finder");
  QRFinder ic(inpath, outpath);
  ros::spin();
  return 0;
}