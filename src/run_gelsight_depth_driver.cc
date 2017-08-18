#include <unistd.h>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "argagg.hpp"
#include "gelsight_depth_driver.h"

using namespace std;

int main(int argc, char** argv) {
  argagg::parser argparser{
      {{"help", {"-h", "--help"}, "shows this help message", 0},
       {"normals",
        {"-n", "--normals"},
        "Generate normals? (default: false)",
        0},
       {"video",
        {"-v", "--video"},
        "Video device #? (default: -1 (automatic))",
        1}}};

  argagg::parser_results args;
  try {
    args = argparser.parse(argc, argv);
  } catch (const exception& e) {
    cerr << e.what() << endl;
    return -1;
  }

  if (args["help"]) {
    cerr << argparser;
    return 0;
  }

  if (args["normals"]) {
    cerr << "Can't run with normals yet..." << endl;
    return -1;
  }

  string video_source = "-1";
  if (args["video"]) video_source = args["video"].as<string>();

  GelsightDepthDriver driver(video_source);

  cv::namedWindow("RawImage", cv::WINDOW_AUTOSIZE);
  cv::startWindowThread();

  while (1) {
    driver.GetNewWebcamImage();
    auto last_image = driver.get_last_raw_image();
    if (last_image.rows > 0 && last_image.cols > 0) {
      cv::imshow("RawImage", driver.get_last_raw_image());
    }
    usleep(1000);
  }
  return 0;
}
