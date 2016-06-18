/****
Copies images from a webcam and broadcasts them to LCM
leveraging OpenCV's nice webcam drivers.

MIT Hyperloop 2016 -- gizatt

I'm working from this guy's nice starting code to get going
https://wimsworld.wordpress.com/2013/07/19/webcam-on-beagleboardblack-using-opencv/
****/

#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <fstream>  // nice number I/O
#include <unistd.h> // for sleep
#include <opencv2/opencv.hpp>


#include "../eigen/Eigen/SparseCholesky" // for least squares solving
#include <opencv2/core/eigen.hpp>
#include <sys/stat.h> // mkdir

#include <lcmtypes/bot_core_image_t.h>

using namespace std;
using namespace cv;

#define DOT_THRESHOLD 0.05
#define DOT_DILATE_AMT 3 // removes small spurious points
#define DOT_ERODE_AMT 8 // expands dots a big to get rid of edges
#define FILTER_ROWS 15
#define FILTER_COLS 15
#define FILTER_SIZE (FILTER_ROWS * FILTER_COLS)
#define FILTER_IMROWS 96

void *lcmMonitor(void *plcm) {
  lcm_t *lcm = (lcm_t *) plcm;
  while (1)
    lcm_handle(lcm);
}

static double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

int main( int argc, char *argv[] )
{
    if (argc != 3){
        printf("Usage: gelsight_depth_driver <save images to ./output?> <visualize?>\n");
        return 0;
    }

    // load convolution kernel for converting Gelsight Image to raw image
    Mat conv_kernel[3][3]; // conv_kernel[c1][c2] is kernel for mapping channel c1 to channel c2
    std::cout << "Loading filters from filter_data/..." << std::endl;
    for (int c1=0; c1<3; c1++) {
      for (int c2=0; c2<3; c2++) {
        char filt_fn[26]; // |"filter_data/filter_XX.txt"|=13 chars, + '\0'
        sprintf(filt_fn, "filter_data/filter_%01d%01d.txt", c1, c2);
        std::fstream myFile(filt_fn, std::ios_base::in);

        float * buf = new float[FILTER_SIZE];
        unsigned int counter = 0;
        while (counter < FILTER_SIZE && myFile >> buf[counter]) {
          printf("%f\n", buf[counter]);
          counter++;
        }
        printf("counter: %d\n", counter);
        std::cout << filt_fn << std::endl;

        Mat tempMat(FILTER_ROWS, FILTER_COLS, CV_32FC1, buf);
        tempMat.copyTo(conv_kernel[2-c1][c2]);
      }
    }

    printf("%f, %f, %f\n",conv_kernel[2][2].at<float>(0,0),conv_kernel[2][2].at<float>(0,1),conv_kernel[2][2].at<float>(1,0));
    printf("%f, %f, %f\n",conv_kernel[0][0].at<float>(0,0),conv_kernel[0][0].at<float>(0,1),conv_kernel[0][0].at<float>(1,0));
    printf("%f, %f, %f\n",conv_kernel[1][1].at<float>(0,0),conv_kernel[1][1].at<float>(0,1),conv_kernel[1][1].at<float>(1,0));

    bool save_images = atoi(argv[1]);
    bool visualize = atoi(argv[2]);
    if (save_images)
        mkdir("output", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    lcm_t * lcm = lcm_create("udpm://239.255.76.67:7667?ttl=0");
    pthread_t lcmThread;
    pthread_create(&lcmThread, NULL, lcmMonitor, lcm);

    VideoCapture capture(-1);   // Using -1 tells OpenCV to grab whatever camera is available.
    if(!capture.isOpened()){
        std::cout << "Failed to connect to the camera." << std::endl;
        return(1);
    }
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    //capture.set(CV_CAP_PROP_EXPOSURE, -2);
    // use a command line tool to do this instead:
    char buf[100];
    int ret = 1;
    int vidi=0;
    // in case we're not on video0, loop through...
    while (ret != 0){
      sprintf(buf, "v4lctl -c /dev/video%d bright 0", vidi);
      printf("Trying %s\n", buf);
      ret = system(buf);
      vidi++;
    }

    // get calibration image immediately
    Mat BGImage;
    capture >> BGImage;
    BGImage.convertTo(BGImage, CV_32FC3);
    BGImage /= 255.0;

    Mat BGDotsMap;
    cvtColor(BGImage, BGDotsMap, CV_RGB2GRAY);
    cv::threshold(BGDotsMap, BGDotsMap, DOT_THRESHOLD, 1.0, CV_THRESH_BINARY);
    Mat dilateElement = getStructuringElement( MORPH_RECT,
                                       Size( 2*DOT_DILATE_AMT + 1, 2*DOT_DILATE_AMT+1 ),
                                       Point( DOT_DILATE_AMT, DOT_DILATE_AMT ) );
    Mat erodeElement = getStructuringElement( MORPH_RECT,
                                       Size( 2*DOT_ERODE_AMT + 1, 2*DOT_ERODE_AMT+1 ),
                                       Point( DOT_ERODE_AMT, DOT_ERODE_AMT ) );
    dilate(BGDotsMap, BGDotsMap, dilateElement);
    erode(BGDotsMap, BGDotsMap, erodeElement);

    Mat elementOne = getStructuringElement( MORPH_RECT,
                                       Size( 2 + 1, 2+1 ),
                                       Point( 1, 1 ) );

    if (visualize){
      namedWindow( "RawImage", cv::WINDOW_AUTOSIZE );
      namedWindow( "GradientImage", cv::WINDOW_AUTOSIZE );
      namedWindow( "ContactImage", cv::WINDOW_AUTOSIZE );
      namedWindow( "NormalsImage", cv::WINDOW_AUTOSIZE );
      startWindowThread();
    }

    double last_send_time = getUnixTime();
    int OutputImageNum = 0;
    while (1) {
        // wasteful but lower latency.
        Mat RawImage;
        capture >> RawImage;
        if (getUnixTime() - last_send_time > 0.0333){
            last_send_time = getUnixTime();
            if(!RawImage.empty())
            {
                if (save_images){
                    std::ostringstream OutputFilename;
                    OutputFilename << "output/img_";
                    OutputFilename << setfill('0') << setw(7) << OutputImageNum;
                    OutputFilename << ".jpg";
                    imwrite(OutputFilename.str(), RawImage);
                }
                RawImage.convertTo(RawImage, CV_32FC3);
                RawImage /= 255.0;

                Mat RawImageDotsMap;
                cvtColor(RawImage, RawImageDotsMap, CV_RGB2GRAY);
                cv::threshold(RawImageDotsMap, RawImageDotsMap, DOT_THRESHOLD, 1.0, CV_THRESH_BINARY);
                dilate(RawImageDotsMap, RawImageDotsMap, dilateElement);
                erode(RawImageDotsMap, RawImageDotsMap, erodeElement);

                // process into depth image
                Mat GradientImage(RawImage.rows, RawImage.cols, CV_32FC1);
                for(long int v=0; v<GradientImage.rows; v++) {
                  for(long int u=0; u<GradientImage.cols; u++ ) {
                    Vec3f color = RawImage.at<Vec3f>(v, u);
                    Vec3f bg_color = BGImage.at<Vec3f>(v, u);

                    // throw out black dots in either BG r 

                    if (RawImageDotsMap.at<float>(v, u) == 0.0 || // throw out the black dots
                        BGDotsMap.at<float>(v, u) == 0.0){
                      GradientImage.at<float>(v, u) = 0.0;
                    } else {
                      for (int k=0; k<3; k++) color[k] /= bg_color[k];
                      // sort in descending order with bubblesort
                      for (int k=1; k>=0; k--){
                          float lesser = fmin(color[k], color[k+1]);
                          float greater = fmax(color[k], color[k+1]);
                          color[k] = greater; color[k+1] = lesser;
                      }
                      GradientImage.at<float>(v, u) = color[0]*4.4 + color[1]*2.2 + color[2]*0.4;
                    }
                  }
                }
                
                // make subsampled version of RawImage
                unsigned int drows = FILTER_IMROWS;
                unsigned int dcols = (unsigned int)(RawImage.cols * FILTER_IMROWS / RawImage.rows);
                Size dsize(dcols, drows);
                Mat RawImageSmall(drows, dcols, CV_32FC3);
                resize(RawImage, RawImageSmall, dsize);
                GaussianBlur(RawImageSmall,RawImageSmall,Size(19,19),1.0);
                // process into subsampled normals image
                Mat NormalImageChannels[3];
                Mat RawImageSmallChannels[3];
                split(RawImageSmall, RawImageSmallChannels);
                for (int cc=0; cc<3; cc++) {
                  NormalImageChannels[cc] = Mat::zeros(RawImageSmall.rows, RawImageSmall.cols, CV_32FC1);
                }
                for (int c2=0; c2<2; c2++) { // skip channel 2, it's normal-Z, not important
                  for (int c1=0; c1<3; c1++) {
                    Mat tempMat(RawImageSmall.rows, RawImageSmall.cols, CV_32FC1);
                    filter2D(-300*RawImageSmallChannels[c1], tempMat, -1, conv_kernel[c1][c2], Point((int)(FILTER_ROWS/2) + 1, (int)(FILTER_COLS/2) + 1), 0.0, BORDER_DEFAULT);
                    NormalImageChannels[c2] += tempMat;
                  }
                }
                // ...at this point, NormalImageChannels[0] and NormalImageChannels[1] store row- and column-wise gradients.
                Mat NormalImage(RawImageSmall.rows, RawImageSmall.cols, CV_32FC3);
                merge(NormalImageChannels, 3, NormalImage); // NOT strictly needed; just for vis
                
                // make Eigen.cpp vectors out of gradients
//                Eigen:MatrixXf EigenGradrVec(RawImageSmall.rows);
//                cv2eigen(NormalImageChannels[0], EigenGradrVec); // need to make a 640*480 - by - 1 column vectort out of channels 0 & 1
                // TODO: build sparse matrices for gradients and for border
                // TODO: use Eigen least squares to derive heightmap
                
                // binary contact sensing across image
                Mat ContactImage(RawImage.rows, RawImage.cols, CV_32FC1);                
                for(long int v=0; v<GradientImage.rows; v++) {
                  for(long int u=0; u<GradientImage.cols; u++ ) {
                    Vec3f color = RawImage.at<Vec3f>(v, u);
                    Vec3f bg_color = BGImage.at<Vec3f>(v, u);

                    color -= bg_color;
                    float max_color = fmax(fmax(color[0], color[1]), color[2]);
                    ContactImage.at<float>(v, u) = (max_color > 0.3);
                  }
                }
                erode(ContactImage, ContactImage, elementOne);
                erode(ContactImage, ContactImage, elementOne);
                
                // do some compression
                vector<uchar> buf;
                Mat ContactImageEnc;
                ContactImage.copyTo(ContactImageEnc);
                ContactImageEnc *= 255.0;
                ContactImageEnc.convertTo(ContactImageEnc, CV_8UC1);
                bool success = imencode(".jpg", ContactImageEnc, buf);
                if (success){

                  // LCM encode and publish contact image
                  bot_core_image_t imagemsg;
                  imagemsg.utime = getUnixTime() * 1000 * 1000;
                  imagemsg.width = ContactImage.cols;
                  imagemsg.height = ContactImage.rows;
                  imagemsg.row_stride = 0;
                  imagemsg.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG;
                  imagemsg.size = sizeof(uchar) * buf.size();
                  imagemsg.data = &buf[0];
                  imagemsg.nmetadata = 0;
                  bot_core_image_t_publish(lcm, "GELSIGHT_CONTACT", &imagemsg);
                }

                // LCM encode and publish raw rgb array
                Mat RawImageEnc;
                RawImage.copyTo(RawImageEnc);
                {
                  int scale = 4;
                  int width = (RawImageEnc.cols/scale);
                  int height = (RawImageEnc.rows/scale);

                  uint8_t img_values[width*height*3];
                  for (int i=0; i<height; i++) {
                    for (int j=0; j<width; j++) {
                      Vec3f color = RawImageEnc.at<Vec3f>(i*scale,j*scale);
                      img_values[(i*width + j)*3 + 0] = 125.0*color[2]; //B
                      img_values[(i*width + j)*3 + 1] = 125.0*color[1]; //G
                      img_values[(i*width + j)*3 + 2] = 125.0*color[0]; //R
                    }
                  }
                  bot_core_image_t imagemsg;
                  imagemsg.utime = getUnixTime() * 1000 * 1000;
                  imagemsg.width = width;
                  imagemsg.height = height;
                  imagemsg.row_stride = height;
                  imagemsg.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG;
                  imagemsg.size = width*height*3;
                  imagemsg.data = &(img_values[0]);
                  imagemsg.nmetadata = 0;
                  bot_core_image_t_publish(lcm, "GELSIGHT_RAW", &imagemsg);
                }

                OutputImageNum++;
                if (visualize){
                  cv::imshow("RawImage", RawImage);
                  cv::imshow("GradientImage", GradientImage / 100.);
                  cv::imshow("ContactImage", ContactImage);
                  cv::imshow("NormalsImage", NormalImage);
                }
            }
        }
    }
    return 0;
}
