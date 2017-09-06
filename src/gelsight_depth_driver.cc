#include "gelsight_depth_driver.h"
#include <chrono>

using namespace std;
using namespace cv;

/**
 * Tests a string for integer status, the C way (no exceptions involved).
 *
 * From:
 * http://stackoverflow.com/questions/2844817/how-do-i-check-if-a-c-string-is-an-int
 * Accessed: 07/13/2016
 */
static inline bool IsInteger(const std::string& s) {
  if (s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+')))
    return false;

  char* p;
  strtol(s.c_str(), &p, 10);

  return (*p == 0);
};

GelsightDepthDriver::GelsightDepthDriver(const string& video_source) {
  if (IsInteger(video_source)) {
    int video_source_num = (int)strtol(video_source.c_str(), NULL, 10);
    capture_.open(video_source_num);

    if(!capture_.isOpened()){
        cout << "Failed to connect to the camera." << endl;
        return;
    }

    // Clear the camera buffer and set the camera exposure.
    Mat temp_image;
    for (int i = 0; i < 10; i++) {
      capture_ >> temp_image;
    }

    // If video_source_num is -1, OpenCV will find the first camera
    // available. Otherwise, it'll grab /dev/video<n>.
    char buf[200];
    int ret = 1;

    struct timespec tim, tim2;  // use awkward c convention to
    tim.tv_sec = 0;             // set up half-second duration
    tim.tv_nsec = 250000000L;

    if (video_source_num < 0) {
      video_source_num = 0;
      while (ret != 0 && video_source_num < 10) {
        nanosleep(&tim, &tim2);
        sprintf(buf,
                "v4l2-ctl --device=/dev/video%d "
                "--set-ctrl=white_balance_temperature_auto=0,backlight_"
                "compensation=0,exposure_auto=1,exposure_absolute=30,exposure_"
                "auto_priority=0,gain=50",
                video_source_num);
        printf("Trying %s\n", buf);
        ret = system(buf);
        video_source_num++;
      }
      if (ret != 0) {
        printf("Failed to find a camera!\n");
        exit(1);
      }
    } else {
      nanosleep(&tim, &tim2);
      sprintf(buf,
              "v4l2-ctl --device=/dev/video%d "
              "--set-ctrl=white_balance_temperature_auto=0,backlight_"
              "compensation=0,exposure_auto=1,exposure_absolute=30,exposure_"
              "auto_priority=0,gain=50",
              video_source_num);
      printf("Trying %s\n", buf);
      ret = system(buf);

      if (ret != 0) {
        printf("Failed to find a camera on /dev/video%d\n", video_source_num);
        exit(1);
      }
    }
  } else {
    capture_.open(video_source);
  }

  if (config_["raw_width"])
    capture_.set(CV_CAP_PROP_FRAME_WIDTH, config_["raw_width"].as<int>());
  else
    capture_.set(CV_CAP_PROP_FRAME_WIDTH, 640);

  if (config_["raw_height"])
    capture_.set(CV_CAP_PROP_FRAME_HEIGHT, config_["raw_height"].as<int>());
  else
    capture_.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
}

void GelsightDepthDriver::GetNewWebcamImage() {
  // Time how long it takes to get a frame,
  // and repeat till it takes a non-instantaneous amount of
  // time.
  auto time_start = chrono::high_resolution_clock::now();
  int k = 0;
  // while (chrono::duration<double>(
  //           chrono::high_resolution_clock::now() - time_start).count() <
  //           0.001 &&
  //       k < 100) {
  capture_.grab();
  //  k++;
  //}
  // printf("Escaped with k=%d\n", k);
  capture_.retrieve(last_raw_image_);
}

void GelsightDepthDriver::UpdateNormalMap() {}
void GelsightDepthDriver::UpdateDepthEstimate() {}