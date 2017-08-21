#include <unistd.h>
#include <cstdlib>
#include <iomanip>
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
        1},
       {"save_raws",
        {"-s", "--saveraws"},
        "Directory in which to save raw images (default: none)",
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

  string save_raws_dir = "";
  if (args["save_raws"]) {
    save_raws_dir = args["save_raws"].as<string>();
    ostringstream command;
    command << "mkdir -p ";
    command << save_raws_dir;
    int error = system(command.str().c_str());
    if (error == -1) {
      printf("Error creating directory %s\n", save_raws_dir.c_str());
      return -1;
    }
  }

  string video_source = "-1";
  if (args["video"]) video_source = args["video"].as<string>();

  GelsightDepthDriver driver(video_source);

  cv::namedWindow("Raw Image", cv::WINDOW_AUTOSIZE);
  cv::startWindowThread();

  int num_images = 0;
  while (1) {
    driver.GetNewWebcamImage();
    auto last_image = driver.get_last_raw_image();
    if (last_image.rows > 0 && last_image.cols > 0) {
      cv::imshow("Raw Image", driver.get_last_raw_image());

      // Save raw image, if requested.
      if (save_raws_dir.size() > 0) {
        ostringstream output_filename;
        output_filename << save_raws_dir;
        output_filename << "/";
        output_filename << setfill('0') << setw(7) << num_images;
        output_filename << ".jpg";
        imwrite(output_filename.str(), last_image);
      }

      num_images++;
    }
    usleep(1000);
  }
  return 0;
}
