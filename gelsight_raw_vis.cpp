
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <fstream>  // nice number I/O
// #include <unistd.h> // for sleep
#include <opencv2/opencv.hpp>
#include <lcm/lcm-cpp.hpp>
// #include <math.h>


// #include "Eigen/IterativeLinearSolvers" // for least squares solving
// #include "lib/libkdtree/kdtree++/kdtree.hpp" //
// #include "rgbToGradientOctNode.hpp"          // for rgb octree
// #include <sys/stat.h> // mkdir
// #include "lib/ezOptionParser/ezOptionParser.hpp"

// #include <lcmtypes/bot_core_image_t.h>
#include <lcmtypes/bot_core/image_t.hpp>

using namespace std;
using namespace cv;

void *lcmMonitor(void *plcm) {
  lcm_t *lcm = (lcm_t *) plcm;
  while (1)
    lcm_handle(lcm);
}

float decode_half_uchar(unsigned char u) {
    return (u <= 128) ? (((float)u)/128.0) : (( ((float)u) - 256.0 ) / 128.0);
}

class Handler {
    public:
        Mat latestImage;
        bool latestImageChanged;
        Handler() {
          latestImageChanged = false;
        }
        ~Handler() {}
        void handleMessage(const lcm::ReceiveBuffer* buffer, const std::string& channel,
                    const bot_core::image_t* imagemsg) {
/*            imagemsg.utime = getUnixTime() * 1000 * 1000;
            imagemsg.width = width;
            imagemsg.height = height;
            imagemsg.row_stride = height;
            imagemsg.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG;
            imagemsg.size = width*height*3;
            imagemsg.data = &(img_values[0]);
            imagemsg.nmetadata = 0; */

            int height = imagemsg->height;
            int width = imagemsg->width;

            cv::Mat3f RawImageEnc(height, width);
            RawImageEnc.convertTo(RawImageEnc, CV_32FC3);
            {
                // uint8_t * img_values = (uint8_t*) (imagemsg->data);
                // std::vector<unsigned char *> img_values = imagemsg->data;
                for (int i=0; i<height; i++) {
                    for (int j=0; j<width; j++) {
                        Vec3f color;
                        color[2] = decode_half_uchar(imagemsg->data[(i*width + j)*3 + 0]);
                        //((float)(imagemsg->data[(i*width + j)*3 + 0])) / 256.0; //B
                        color[1] = decode_half_uchar(imagemsg->data[(i*width + j)*3 + 1]);
                        //((float)(imagemsg->data[(i*width + j)*3 + 1])) / 256.0; //G
                        color[0] = decode_half_uchar(imagemsg->data[(i*width + j)*3 + 2]);
                        //((float)(imagemsg->data[(i*width + j)*3 + 2])) / 256.0; //R

                        RawImageEnc.at<Vec3f>(i,j) = color;                        
                    }
                }
            }
            
            latestImage = RawImageEnc;
            latestImageChanged = true;
        }
    
};

int main(int argc, char** argv) {

    if (argc != 2 && argc != 3) {
        std::cout << "Usage: gelsight_raw_vis LCM_CHANNEL_NAME [background_image_file.jpg]" << std::endl;
        return 1;
    }
    
    Mat BGImage;
    if (argc == 3) {
        BGImage = imread(argv[2]);
        BGImage.convertTo(BGImage, CV_32FC3);
        BGImage /= 255.0;
    }

    Handler imageHandler;

    //lcm_t * lcm = lcm_create("udpm://239.255.76.67:7667?ttl=0");
    lcm::LCM lcm;
    lcm.subscribe("GELSIGHT_RAW", &Handler::handleMessage, &imageHandler);
    
    // pthread_t lcmThread;
    // pthread_create(&lcmThread, NULL, lcmMonitor, lcm);

    namedWindow("GELSIGHT RGB IMAGE", cv::WINDOW_NORMAL);
    startWindowThread();
  

    while (0 == lcm.handle()) {
        if (imageHandler.latestImageChanged) {
            if (BGImage.empty()) {
                imshow("GELSIGHT RGB IMAGE", imageHandler.latestImage);
            } else {
                resize(BGImage, BGImage, Size(imageHandler.latestImage.cols, imageHandler.latestImage.rows));
                imshow("GELSIGHT RGB IMAGE", imageHandler.latestImage + BGImage);
            }
            imageHandler.latestImageChanged = false;
        }
    }

    return 0;
}
