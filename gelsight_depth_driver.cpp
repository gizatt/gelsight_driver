/****
Copies images from a webcam and broadcasts them to LCM
leveraging OpenCV's nice webcam drivers.

MIT 2016 -- gizatt

Parameters:
  - gradient gain - scale factor on gradients in lookup table
  - edge gain - weight in Least Squares solve given to keeping the depth map near zero around the border

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
#include <math.h>


#include "Eigen/IterativeLinearSolvers" // for least squares solving
#include "lib/libkdtree/kdtree++/kdtree.hpp" //
#include "rgbToGradientOctNode.hpp"          // for rgb octree
#include <sys/stat.h> // mkdir
#include "lib/ezOptionParser/ezOptionParser.hpp"

#ifndef BUILD_STANDALONE
  #include "RemoteTreeViewerWrapper.hpp"
  #include <lcmtypes/bot_core_image_t.h>
#endif


using namespace std;
using namespace cv;
using namespace ez;

#define DOT_THRESHOLD 0.05
#define DOT_DILATE_AMT 3 // removes small spurious points
#define DOT_ERODE_AMT 8 // expands dots a big to get rid of edges

#define BINS_PER_COLOR (16)
// #define COLOR_TO_BIN(c) ((size_t)(c * BINS_PER_COLOR))
#define COLOR_TO_BIN(c) ((size_t)((1+((c)*255/256))/2 * BINS_PER_COLOR))
#define COLOR_TO_BIN_IMAGE(c) ((1+((c)*255/256))/2 * BINS_PER_COLOR)
#define COLOR_TO_INDEX(c0,c1,c2) ((c0)*BINS_PER_COLOR*BINS_PER_COLOR + (c1)*BINS_PER_COLOR + (c2))

#define FILTER_ROWS 15
#define FILTER_COLS 15
#define FILTER_SIZE (FILTER_ROWS * FILTER_COLS)
#define FILTER_IMROWS 48 // 96
#define FILTER_IMCOLS 64 // 128

#define BALL_RADIUS_GUESS 45  // radius of detected ball. TODO: Make this a CL argument
#define BALL_RADIUS_GUESS_MARGIN (BALL_RADIUS_GUESS + 20)  // margin of error on ball radius, for HoughCircle
#define BALL_RADIUS_TIGHT (.85 * BALL_RADIUS_GUESS) // the assumed actual radius of the sphere, as opposed to
                                                    // the radius of the image it generates.
#define SNAPSHOT_WIDTH (2*BALL_RADIUS_GUESS_MARGIN)
#define REF_PT_ROWS 3  // grid resolution of final lookup table (how many lookup locations there are) TODO: CL argument
#define REF_PT_COLS 4
#define REF_GET_IMROW(ptrow, imrows) ( ((1+(ptrow)) * (imrows)) / (1+REF_PT_ROWS) )
#define REF_GET_IMCOL(ptcol, imcols) ( ((1+(ptcol)) * (imcols)) / (1+REF_PT_COLS) )
#define REF_GET_PTROW(imrow, imrows) ( ( (imrow) * (REF_PT_ROWS+1) ) / (imrows) )
#define REF_GET_PTCOL(imcol, imcols) ( ( (imcol) * (REF_PT_COLS+1) ) / (imcols) )
#define SQDIST(x, y) ((x)*(x) + (y)*(y))
#define MACRO_MAX(x, y) ((x) > (y) ? (x) : (y))

#ifndef BUILD_STANDALONE
  void *lcmMonitor(void *plcm) {
    lcm_t *lcm = (lcm_t *) plcm;
    while (1)
      lcm_handle(lcm);
  }
#endif

static double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

void Usage(ezOptionParser& opt) {
  std::string usage;
  opt.getUsage(usage);
  std::cout << usage;
};

/**
 * Tests for integer status, the C way (no exceptions involved).
 *
 * From: http://stackoverflow.com/questions/2844817/how-do-i-check-if-a-c-string-is-an-int
 * Accessed: 07/13/2016
 */
static inline bool isInteger(const std::string & s)
{
   if(s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false ;

   char * p ;
   strtol(s.c_str(), &p, 10) ;

   return (*p == 0) ;
};

int main( int argc, const char *argv[] )
{
    bool save_images;
    bool visualize;
    bool live_camera;
    bool background_set;
    string background_string;
    string lookup_table_string;

    const float edge_gain = 1.0;
    const float grad_gain = 10.0/3.0; // 6.0/5.0; //1.0;

    // Input argument parsing stuff:

    ezOptionParser opt;

    opt.overview = "Accepts an imput source of Gelsight images and produces "
                   "heightmaps from the images, publishing them to LCM and "
                   "optionally writing them to disk (writes raw images to "
                   "\"output\" and depth images to \"outputdepth\").\n\n"

                  "  video_source - If an integer is provided, this will be "
                    "treated as a camera source (eg. for Linux: \"1\" -> \"/dev/video1\"); "
                    "otherwise, treated as a path to a contiguous set of "
                    "numbered images (eg. \"myimages/img_%07d.jpg\")."; // Help description.
    opt.syntax = "gelsight_depth_driver [OPTIONS] video_source [OPTIONS]";
    opt.example = "gelsight_depth_driver 1 -v 0 -o 1    # Grab frames from camera 1, no vis, "
                                                             "write output files\n\n";
    opt.footer = "Robot Locomotion Group, geronm and gizatt\n";

    opt.add(
      "", // Default.
      0, // Required?
      0, // Number of args expected.
      0, // Delimiter if expecting multiple args.
      "Display usage instructions.", // Help description.
      "-h",     // Flag token.
      "-help",  // Flag token.
      "--help", // Flag token.
      "--usage" // Flag token.
    );

    opt.add(
      "", // Default.
      0, // Required?
      1, // Number of args expected.
      0, // Delimiter if expecting multiple args.
      "Output folder, into which to write files (in default case, no files written).", // Help description.
      "-o",     // Flag token.
      "--output" // Flag token.
    );

    opt.add(
      "1", // Default.
      0, // Required?
      1, // Number of args expected.
      0, // Delimiter if expecting multiple args.
      "Visualize (\"0\" or \"1\").", // Help description.
      "-v",     // Flag token.
      "--visualization" // Flag token.
    );

    opt.add(
      "", // Default.
      0, // Required?
      1, // Number of args expected.
      0, // Delimiter if expecting multiple args.
      "Path to file containing lookup table (eg. \"/home/stuff/trained_lookup.dat\".", // Help description.
      "-l",     // Flag token.
      "--lookup-table" // Flag token.
    );

    opt.add(
      "", // Default.
      0, // Required?
      1, // Number of args expected.
      0, // Delimiter if expecting multiple args.
      "Path to an image to use as the background image (for subtraction)."
      " If omitted, the first frame will be used as the background.", // Help description.
      "-b",     // Flag token.
      "--background-image" // Flag token.
    );

    opt.add(
      "", // Default.
      0, // Required?
      1, // Number of args expected.
      0, // Delimiter if expecting multiple args.
      "Whether to visualize point cloud with RemoteTreeViewer.", // Help description.
      "-p",     // Flag token.
      "--point-pub" // Flag token.
    );

    opt.parse(argc, argv);

    if (opt.isSet("-h")) {
      Usage(opt);
      return 1;
    }

    int totalNumArgs = opt.firstArgs.size() + opt.lastArgs.size() + opt.unknownArgs.size();

    if (totalNumArgs != 2) {  // includes program name argument.
      std::cerr << "ERROR: Expected 1 argument.\n\n";
      Usage(opt);
      return 1;
    }

    // Check for bad options
    {
      vector<string> badOptions;
      int i;
      if(!opt.gotRequired(badOptions)) {
        for(i=0; i < badOptions.size(); ++i)
          std::cerr << "ERROR: Missing required option " << badOptions[i] << ".\n\n";
        Usage(opt);
        return 1;
      }

      if(!opt.gotExpected(badOptions)) {
        for(i=0; i < badOptions.size(); ++i)
          std::cerr << "ERROR: Got unexpected number of arguments for option " << badOptions[i] << ".\n\n";

        Usage(opt);
        return 1;
      }
    }

    // Make vector of non-option arguments (script can take them interleaved with options).
    vector<string> allArgs;
    for (int i=0; i<opt.firstArgs.size(); i++) {
      allArgs.push_back(*(opt.firstArgs[i]));
    }
    for (int i=0; i<opt.unknownArgs.size(); i++) {
      allArgs.push_back(*(opt.unknownArgs[i]));
    }
    for (int i=0; i<opt.lastArgs.size(); i++) {
      allArgs.push_back(*(opt.lastArgs[i]));
    }

    assert(allArgs.size() == totalNumArgs);

    // First non-option argument is source of video.
    string videoSource = allArgs[1];

    save_images = true;
    if (opt.isSet("-o")) {
      int boolAsNum = 1;
      opt.get("-o")->getInt(boolAsNum);
      save_images = (boolAsNum != 0);
    }

    background_set = false;
    background_string = "";
    if (opt.isSet("-b")) {
      background_set = true;
      opt.get("-b")->getString(background_string);
    }

    visualize = true;
    if (opt.isSet("-v")) {
      int boolAsNum = 1;
      opt.get("-v")->getInt(boolAsNum);
      visualize = (boolAsNum != 0);
    }

    lookup_table_string = "trained_lookup.dat";
    if (opt.isSet("-l")) {
      opt.get("-l")->getString(lookup_table_string);
    }

    live_camera = false;
    VideoCapture capture;
    if (isInteger(videoSource)) {
      char* p;
      capture.open((int)strtol(videoSource.c_str(), NULL, 10)); // Using -1 would tell OpenCV to grab whatever camera is available.
      live_camera = true;
    } else {
      capture.open(videoSource);
    }

    if(!capture.isOpened()){
        std::cout << "Failed to connect to the camera." << std::endl;
        return(1);
    }

    enum GradMode { kUseLookupOctree, kUseLookupTable, kUseConvFilter, kUseOtherMethod };

    GradMode gradMode = kUseLookupTable; //kUseConvFilter; //kUseLookupOctree; //kUseConvFilter; //
    if (save_images) {
        mkdir("output", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        mkdir("outputdepth", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    #ifndef BUILD_STANDALONE
      lcm_t * lcm = lcm_create("udpm://239.255.76.67:7667?ttl=0");
      pthread_t lcmThread;
      pthread_create(&lcmThread, NULL, lcmMonitor, lcm);

    //capture.set(CV_CAP_PROP_BUFFERSIZE, 2); // Small software buffer
      RemoteTreeViewerWrapper rm;
    #endif

    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);


    if (live_camera) {
      // if we're using a live camera, need to clear
      // buffer and set exposure

      Mat temp_image; // TODO: Move to after options change!
      for (int i=0; i<10; i++) {
        capture >> temp_image; // warm up capture
      }

      //capture.set(CV_CAP_PROP_EXPOSURE, -2);
      // use a command line tool to do this instead:
      char buf[300];
      int ret = 1;
      int vidi=0;
      // in case we're not on video0, loop through...

      struct timespec tim, tim2; // use awkward c convention to
      tim.tv_sec = 0;            // set up half-second duration
      tim.tv_nsec = 250000000L;

      while (ret != 0){
        nanosleep(&tim, &tim2);
        sprintf(buf, "v4l2-ctl --device=/dev/video%d --set-ctrl=white_balance_temperature_auto=0,backlight_compensation=0,exposure_auto=1,exposure_absolute=30,exposure_auto_priority=0,gain=50", vidi);
        printf("Trying %s\n", buf);
        ret = system(buf);
        vidi++;
      }
    }

    if (visualize){
      namedWindow( "BGImage", cv::WINDOW_NORMAL );
      namedWindow( "RawImage", cv::WINDOW_NORMAL );
      namedWindow( "GradientVisImage", cv::WINDOW_NORMAL );
      namedWindow( "DepthImage", cv::WINDOW_NORMAL );
      startWindowThread();
    }

    Mat conv_kernel[3][3]; // conv_kernel[c1][c2] is kernel for mapping channel c1 to channel c2
    typedef KDTree::KDTree<3,rgbToGradientOctNode> RGBGradOctree; // octree, in case needed
    RGBGradOctree lookupTrees[REF_PT_ROWS*REF_PT_COLS];
    float grad_r_lookup[BINS_PER_COLOR*BINS_PER_COLOR*BINS_PER_COLOR]; // lookup array, in case we are using that
    float grad_c_lookup[BINS_PER_COLOR*BINS_PER_COLOR*BINS_PER_COLOR];


    switch (gradMode) {
      case kUseConvFilter:
      {
        // load convolution kernel for converting Gelsight Image to raw image
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
            flip(tempMat, tempMat, -1);
            tempMat.copyTo(conv_kernel[2-c1][c2]);
          }
        }
        printf("%f, %f, %f\n",conv_kernel[2][2].at<float>(0,0),conv_kernel[2][2].at<float>(0,1),conv_kernel[2][2].at<float>(1,0));
        printf("%f, %f, %f\n",conv_kernel[0][0].at<float>(0,0),conv_kernel[0][0].at<float>(0,1),conv_kernel[0][0].at<float>(1,0));
        printf("%f, %f, %f\n",conv_kernel[1][1].at<float>(0,0),conv_kernel[1][1].at<float>(0,1),conv_kernel[1][1].at<float>(1,0));
      }
      break;
      case kUseLookupOctree:
      {
        // populate octrees for RGB-to-Gradient lookup-based conversion
        for (int refr=0; refr<REF_PT_ROWS; refr++) {
          for (int refc=0; refc<REF_PT_COLS; refc++) {
            // Load up the (r,c)-reference image
            Mat RefPtImage;
            {
              std::ostringstream ImageFilename;
              ImageFilename << "groundtruth/sphererefptimgs/img_";
              ImageFilename << "r" << setfill('0') << setw(4) << refr;
              ImageFilename << "c" << setfill('0') << setw(4) << refc;
              ImageFilename << ".jpg";

              RefPtImage = imread(ImageFilename.str());
            }
            RefPtImage.convertTo(RefPtImage, CV_32FC3);
            RefPtImage /= 255.0;

            RGBGradOctree * currentTree = &(lookupTrees[refr*REF_PT_COLS + refc]);

            int node_index = 0;
            for (int imr=0; imr<RefPtImage.rows; imr++) {
              for (int imc=0; imc<RefPtImage.cols; imc++) {

                double rdist = hypot(imr - (RefPtImage.rows/2), imc - (RefPtImage.cols/2));
                if (rdist < BALL_RADIUS_TIGHT) { // only take points that we believe are on the surface of the sphere
                  // compute gradients at current imr,imc place them in tree under RGB value
                  Vec3f bgr = RefPtImage.at<Vec3f>(imc,imr);

                  rgbToGradientOctNode node;
                  node.xyz[0] = bgr[0];
                  node.xyz[1] = bgr[1];
                  node.xyz[2] = bgr[2];
                  node.index = node_index;

                  int im_ori_r = (imr - (RefPtImage.rows/2));
                  int im_ori_c = (imc - (RefPtImage.cols/2));
                  node.rcgradient[1] = sqrt(MACRO_MAX(BALL_RADIUS_TIGHT*BALL_RADIUS_TIGHT - ((im_ori_r+1)*(im_ori_r+1)), 0));
                  node.rcgradient[1] -= sqrt(MACRO_MAX(BALL_RADIUS_TIGHT*BALL_RADIUS_TIGHT - (im_ori_r*im_ori_r),0));
                  node.rcgradient[0] = sqrt(MACRO_MAX(BALL_RADIUS_TIGHT*BALL_RADIUS_TIGHT - ((im_ori_c+1)*(im_ori_c+1)),0));
                  node.rcgradient[0] -= sqrt(MACRO_MAX(BALL_RADIUS_TIGHT*BALL_RADIUS_TIGHT - (im_ori_c*im_ori_c),0));

                  currentTree->insert(node);

                  node_index++;
                } else {
                  Vec3f white(1,1,1);
                  RefPtImage.at<Vec3f>(imc,imr) /= 2;
                  RefPtImage.at<Vec3f>(imc,imr) += .5 * white; //average self with white
                }
              }
            }

            #if DEBUG_SHOW_REF
            cv::namedWindow("DEBUGRefPtImage", cv::WINDOW_AUTOSIZE);
            cv::imshow("DEBUGRefPtImage", RefPtImage);
            {
              struct timespec tim, tim2;
              tim.tv_sec = 0;
              tim.tv_nsec = 050000000L;
              nanosleep(&tim,&tim2);
            }
            cv::imshow("DEBUGRefPtImage", RefPtImage);
            #endif
          }
        }
      }
      break;
      case kUseLookupTable:
      {
        // prepare lookup table
        // Make sure entries can be read properly
        ifstream table_read_file;
        table_read_file.open(lookup_table_string.c_str());
        if (table_read_file) {
          for (int i=0; i < BINS_PER_COLOR; i++) {
            for (int j=0; j < BINS_PER_COLOR; j++) {
              for (int k=0; k < BINS_PER_COLOR; k++) {
                float read_float;
                table_read_file >> read_float;
                grad_r_lookup[COLOR_TO_INDEX(i,j,k)] = read_float;
              }
            }
          }
          for (int i=0; i < BINS_PER_COLOR; i++) {
            for (int j=0; j < BINS_PER_COLOR; j++) {
              for (int k=0; k < BINS_PER_COLOR; k++) {
                float read_float;
                table_read_file >> read_float;
                grad_c_lookup[COLOR_TO_INDEX(i,j,k)] = read_float;
              }
            }
          }
        }
      }
      break;
    }

    // calibration image; read if provided, otherwise use first frame
    Mat BGImage;
    if (background_set) {
      assert(BGImage.empty());

      printf("Reading in background image...\n");

      BGImage = imread(background_string.c_str());
      BGImage.convertTo(BGImage, CV_32FC3);
      BGImage /= 255.0;

      assert(!BGImage.empty());
    }

    // make subsampled version of RawImage
    unsigned int drows = 2*FILTER_IMROWS;
    unsigned int dcols = 2*FILTER_IMCOLS;
    Size dsize(dcols, drows);
    Mat RawImageSmall(drows, dcols, CV_32FC3);
    Mat RawImageWithBGSmall(drows, dcols, CV_32FC3);

    typedef Eigen::Triplet<float> T;

    std::vector<T> A_coeffs;
    // generate constraints from row gradients
    int a_row = 0;
    int b_row = 0;
    for (int u=1; u<RawImageSmall.cols-1; u++){
      for (int v=1; v<RawImageSmall.rows; v++){
        A_coeffs.push_back(T(a_row, u*RawImageSmall.rows + (v - 1),  -1));
        A_coeffs.push_back(T(a_row, u*RawImageSmall.rows + (v), 1));
        a_row++;
      }
    }

    // and col gradients
    for (int u=1; u<RawImageSmall.cols; u++){
      for (int v=1; v<RawImageSmall.rows-1; v++){
        A_coeffs.push_back(T(a_row, (u-1)*RawImageSmall.rows + v,  -1));
        A_coeffs.push_back(T(a_row, (u)*RawImageSmall.rows + v, 1));
        a_row++;
      }
    }

    // borders -- left and right border
    for (int i=0; i<RawImageSmall.rows; i++){
      A_coeffs.push_back(T(a_row, i, edge_gain));
      a_row++;
      A_coeffs.push_back(T(a_row, (RawImageSmall.cols-1)*RawImageSmall.rows + i, edge_gain));
      a_row++;
    }
    // borders -- top and bottom border
    for (int i=0; i<RawImageSmall.cols; i++){
      A_coeffs.push_back(T(a_row, i*RawImageSmall.rows + 0, edge_gain));
      a_row++;
      A_coeffs.push_back(T(a_row, (i+1)*RawImageSmall.rows - 1, edge_gain));
      a_row++;
    }
    Eigen::SparseMatrix<float> A(a_row, RawImageSmall.rows * RawImageSmall.cols);
    A.setFromTriplets(A_coeffs.begin(), A_coeffs.end());
    Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<float> > solver;
    solver.setMaxIterations(100);
    printf("Going into qr solve...\n");
    double starttime = getUnixTime();
    solver.compute(A);
    Eigen::VectorXf x(a_row);
    x.setZero();

    printf("computed QR fact in %f seconds\n", getUnixTime() - starttime);

    double last_send_time = getUnixTime();
    int OutputImageNum = 0;
    int NumFramesToShow = -1;  // NumFrames > 0 allows camera to be released; < 0 is infinite
    while (OutputImageNum != NumFramesToShow) {
        // wasteful but lower latency.
        Mat RawImage;
        Mat RawImageWithBG;

        // in case of live camera, reduce latency via buffer depletion
        if (live_camera) {
          capture.grab();
        }
        if (getUnixTime() - last_send_time > 0.03){ //033){
            capture >> RawImageWithBG;
            // capture.retrieve(RawImageWithBG, 3);
            last_send_time = getUnixTime();
            if(!RawImageWithBG.empty() && BGImage.empty()) {
              // acquire the background on the first image. This necessarily
              // implies an all-zero heightmap, so we output this.
              printf("Grabbing first frame for background...\n");

              if (save_images){
                    std::ostringstream OutputFilename;
                    OutputFilename << "output/img_";
                    OutputFilename << setfill('0') << setw(7) << OutputImageNum;
                    OutputFilename << ".jpg";
                    imwrite(OutputFilename.str(), RawImageWithBG);
              }

              RawImageWithBG.convertTo(RawImageWithBG, CV_32FC3);
              RawImageWithBG /= 255.0;
              RawImageWithBG.copyTo(BGImage);

              if (save_images){
                  Mat DepthImageOut;
                  RawImageWithBG.copyTo(DepthImageOut);
                  DepthImageOut *= 0.0;
                  DepthImageOut.convertTo(DepthImageOut, CV_16UC1);
                  std::ostringstream OutputFilename;
                  OutputFilename << "outputdepth/img_";
                  OutputFilename << setfill('0') << setw(7) << OutputImageNum;
                  OutputFilename << ".png";
                  imwrite(OutputFilename.str(), DepthImageOut);
                }

                background_set = true;
                OutputImageNum++;
            } else if (!RawImageWithBG.empty() && !BGImage.empty()) {
                if (save_images){
                    std::ostringstream OutputFilename;
                    OutputFilename << "output/img_";
                    OutputFilename << setfill('0') << setw(7) << OutputImageNum;
                    OutputFilename << ".jpg";
                    imwrite(OutputFilename.str(), RawImageWithBG);
                }
                RawImageWithBG.convertTo(RawImageWithBG, CV_32FC3);
                RawImageWithBG /= 255.0;
                RawImageWithBG.copyTo(RawImage);

                RawImage -= BGImage;

                resize(RawImage, RawImageSmall, dsize);
                resize(RawImageWithBG, RawImageWithBGSmall, dsize);
                // GaussianBlur(RawImageSmall,RawImageSmall,Size(19,19),1.0);
                // process into subsampled normals image


                Mat RCGradientImageChannels[2];

                switch (gradMode) {

                  case kUseConvFilter:
                    {
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
                      RCGradientImageChannels[0] = NormalImageChannels[0];
                      RCGradientImageChannels[1] = NormalImageChannels[1];
                    }
                    break;
                  case kUseLookupOctree:
                    {
                      printf("Readying lookup process...\n");

                      RCGradientImageChannels[0].create(RawImageWithBGSmall.rows, RawImageWithBGSmall.cols, CV_32FC1);
                      RCGradientImageChannels[1].create(RawImageWithBGSmall.rows, RawImageWithBGSmall.cols, CV_32FC1);

                      for (int imr=0; imr < RawImageWithBGSmall.rows; imr++) {
                        for (int imc=0; imc < RawImageWithBGSmall.cols; imc++) {
                          Vec3f color = RawImageWithBGSmall.at<Vec3f>(imr,imc);

                          // find closest refpt for (imr, imc)
                          int ptrow = max(0,min(REF_PT_ROWS-1,REF_GET_PTROW(imr, RawImageWithBGSmall.rows)));
                          int ptcol = max(0,min(REF_PT_COLS-1,REF_GET_PTCOL(imc, RawImageWithBGSmall.cols)));

                          // lookup color in that refpt's octree
                          RGBGradOctree* currentTree = &(lookupTrees[ptrow * REF_PT_COLS + ptcol]);

                          rgbToGradientOctNode testNode;
                          testNode.xyz[0] = color[0];
                          testNode.xyz[1] = color[1];
                          testNode.xyz[2] = color[2];

                          //std::pair<RGBGradOctree::const_iterator,double> found = currentTree->find_nearest(testNode);
                          //rgbToGradientOctNode nearestNode = *found.first;

                          //if (found.second > .2) { // if the match is too far from any reference color take null hypothesis of flat slope
                          //  nearestNode.rcgradient[0] = 0.0001;
                          //  nearestNode.rcgradient[1] = 0.0001;
                          //}
                          //if (abs(nearestNode.rcgradient[0]) < .1 && abs(nearestNode.rcgradient[1]) < .1) {
                          //  nearestNode.rcgradient[0] = 0.0001;
                          //  nearestNode.rcgradient[1] = 0.0001;
                          //}

                          rgbToGradientOctNode nearestNode;

                          double limit = 0.5; //0.2*1.78;
                          vector<rgbToGradientOctNode> howClose;
                          currentTree->find_within_range(testNode,limit,back_insert_iterator<vector<rgbToGradientOctNode> >(howClose));
                          nearestNode.rcgradient[0] = 0.0001;
                          nearestNode.rcgradient[1] = 0.0001;

                          double weightTotal = 0;
                          for (int i=0; i<howClose.size(); ++i) {
                            double distance = nearestNode.distance(testNode);
                            double weight = pow(1.0/2.71,distance*distance);
                            nearestNode.rcgradient[0] += howClose[i].rcgradient[0] * weight;
                            nearestNode.rcgradient[1] += howClose[i].rcgradient[1] * weight;
                            weightTotal += weight;
                          }
                          if (weightTotal > 0) {
                            nearestNode.rcgradient[0] /= weightTotal;
                            nearestNode.rcgradient[1] /= weightTotal;                           
                          }

                          // assign gradient to RCGradientImageChannels
                          RCGradientImageChannels[0].at<float>(imr,imc) = nearestNode.rcgradient[0];
                          RCGradientImageChannels[1].at<float>(imr,imc) = nearestNode.rcgradient[1];
                        }
                      }
                    }
                    break;
                  case kUseLookupTable:
                    {
                      printf("Readying lookup process...\n");

                      RCGradientImageChannels[0].create(RawImage.rows, RawImage.cols, CV_32FC1);
                      RCGradientImageChannels[1].create(RawImage.rows, RawImage.cols, CV_32FC1);

                      for (int i=0; i<RawImage.rows; i++) {
                        for (int j=0; j<RawImage.cols; j++) {
                          Vec3f rawColor = RawImage.at<Vec3f>(i,j);

                          size_t c0 = COLOR_TO_BIN(rawColor[0]);
                          size_t c1 = COLOR_TO_BIN(rawColor[1]);
                          size_t c2 = COLOR_TO_BIN(rawColor[2]);

                          RCGradientImageChannels[0].at<float>(i,j) = grad_gain*grad_r_lookup[COLOR_TO_INDEX(c0,c1,c2)];
                        }
                      }

                      for (int i=0; i<RawImage.rows; i++) {
                        for (int j=0; j<RawImage.cols; j++) {
                          Vec3f rawColor = RawImage.at<Vec3f>(i,j);

                          size_t c0 = COLOR_TO_BIN(rawColor[0]);
                          size_t c1 = COLOR_TO_BIN(rawColor[1]);
                          size_t c2 = COLOR_TO_BIN(rawColor[2]);

                          RCGradientImageChannels[1].at<float>(i,j) = grad_gain*grad_c_lookup[COLOR_TO_INDEX(c0,c1,c2)];
                        }
                      }

                      cv::resize(RCGradientImageChannels[0], RCGradientImageChannels[0], cv::Size(RawImageSmall.cols, RawImageSmall.rows));
                      cv::resize(RCGradientImageChannels[1], RCGradientImageChannels[1], cv::Size(RawImageSmall.cols, RawImageSmall.rows));
                    }
                    break;
                  default:
                    std::cerr << "ERROR: Unexpected gradient finder mode encountered.\n";
                    return 1;
                }

                Mat GradientVisImage(RawImageSmall.rows, RawImageSmall.cols, CV_32FC3);
                Mat GradientVisImageChannels[3];
                GradientVisImageChannels[0] = .75*max(-RCGradientImageChannels[0],0) + .25*max(RCGradientImageChannels[1],0);
                GradientVisImageChannels[1] = .75*max(RCGradientImageChannels[0],0) + .25*max(RCGradientImageChannels[1],0);
                GradientVisImageChannels[2] = .75*max(-RCGradientImageChannels[1],0) + .25*max(RCGradientImageChannels[1],0);
                merge(GradientVisImageChannels, 3, GradientVisImage); // NOT strictly needed; just for vis

                // now use least squares to generate the height map:
                // we'll create a linear problem Ax = b
                // where each constaint (row in A, value in b) penalizes the
                // deviation of a pair of adjacent points in the image
                // from the expected gradient value

                Eigen::VectorXf b(a_row);
                b.setZero();
                b_row = 0;
                for (int u=1; u<RawImageSmall.cols-1; u++){
                  for (int v=1; v<RawImageSmall.rows; v++){
                    b(b_row) = RCGradientImageChannels[0].at<float>(v, u);
                    b_row++;
                  }
                }
                // and col gradients
                for (int u=1; u<RawImageSmall.cols; u++){
                  for (int v=1; v<RawImageSmall.rows-1; v++){
                    b(b_row) = RCGradientImageChannels[1].at<float>(v, u);
                    b_row++;
                  }
                }
                // and we set zero so the borders are taken care of
                printf("constructed, solving...\n");
                // aaaaand done
/*
                if (isnan(x(0)))
                  x = solver.solve(b);
                else
                  x = solver.solveWithGuess(b, x);
                */ x = solver.solve(b);

                printf("solved\n");

                Mat DepthImage(RawImageSmall.rows, RawImageSmall.cols, CV_32FC1);
                for (int u=0; u<RawImageSmall.cols; u++){
                  for (int v=0; v<RawImageSmall.rows; v++){
                    DepthImage.at<float>(v, u) = x(u*RawImageSmall.rows + v);
                  }
                }
                // DEBUG prints
                for (int u=0; u<10; u++){
                  for (int v=0; v<10; v++){
                    cout << x(u*RawImageSmall.rows + v) << ",";
                  }
                  cout << "\n";
                }

                if (save_images){
                  Mat DepthImageOut;
                  cv::resize(DepthImage, DepthImageOut, cv::Size(640, 480));
                  DepthImageOut *= 65535.0;
                  DepthImageOut.convertTo(DepthImageOut, CV_16UC1);
                  std::ostringstream OutputFilename;
                  OutputFilename << "outputdepth/img_";
                  OutputFilename << setfill('0') << setw(7) << OutputImageNum;
                  OutputFilename << ".png";
                  imwrite(OutputFilename.str(), DepthImageOut);
                }

                // do some compression
                {
                  vector<uchar> buf;
                  Mat DepthImageEnc;
                  DepthImage.copyTo(DepthImageEnc);
                  DepthImageEnc *= 255.0;
                  DepthImageEnc.convertTo(DepthImageEnc, CV_8UC1);
                  bool success = imencode(".jpg", DepthImageEnc, buf);
                  if (success){
                    #ifndef BUILD_STANDALONE
                      // LCM encode and publish contact image
                      bot_core_image_t imagemsg;
                      imagemsg.utime = getUnixTime() * 1000 * 1000;
                      imagemsg.width = DepthImage.cols;
                      imagemsg.height = DepthImage.rows;
                      imagemsg.row_stride = 0;
                      imagemsg.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG;
                      imagemsg.size = sizeof(uchar) * buf.size();
                      imagemsg.data = &buf[0];
                      imagemsg.nmetadata = 0;
                      bot_core_image_t_publish(lcm, "GELSIGHT_DEPTH", &imagemsg);
                    #endif
                  }
                }

                // do some compression
                {
                  vector<uchar> buf;
                  Mat RawImageWithBGEnc;
                  RawImageWithBGSmall.copyTo(RawImageWithBGEnc);
                  RawImageWithBGEnc *= 255.0;
                  RawImageWithBGEnc.convertTo(RawImageWithBGEnc, CV_8UC3);
                  bool success = imencode(".jpg", RawImageWithBGEnc, buf);
                  if (success){
                    #ifndef BUILD_STANDALONE
                      // LCM encode and publish contact image
                      bot_core_image_t imagemsg;
                      imagemsg.utime = getUnixTime() * 1000 * 1000;
                      imagemsg.width = RawImageWithBG.cols;
                      imagemsg.height = RawImageWithBG.rows;
                      imagemsg.row_stride = 0;
                      imagemsg.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG;
                      imagemsg.size = sizeof(uchar) * buf.size();
                      imagemsg.data = &buf[0];
                      imagemsg.nmetadata = 0;
                      bot_core_image_t_publish(lcm, "GELSIGHT_RAW", &imagemsg);
                    #endif
                  }
                }

                if (visualize){
                  Mat GradientVisImageBigger;
                  Mat DepthImageBigger;
                  cv::resize(GradientVisImage, GradientVisImageBigger, cv::Size(640, 480));
                  cv::resize(DepthImage, DepthImageBigger, cv::Size(640, 480));
                  cv::imshow("RawImage", RawImageWithBG);
                  cv::imshow("GradientVisImage", 5*GradientVisImageBigger);
                  cv::imshow("DepthImage", DepthImageBigger);
                  cv::imshow("BGImage", BGImage);
                }

                #ifndef BUILD_STANDALONE
                  if (opt.isSet("-p")){
                    // Visualize point cloud info.
                    int x = DepthImage.cols;
                    int y = DepthImage.rows;
                    vector<vector<double>> colors;
                    Eigen::Matrix3Xd pts(3, x*y);
                    for (int u = 0; u < x; u++){
                      for (int v = 0; v < y; v++){
                        pts(0, u*y+v) = ((double)(u - x/2)) / ((double) (x/2));
                        pts(1, u*y+v) = ((double)((y-v) - y/2)) / ((double) (y/2));
                        pts(2, u*y+v) = DepthImage.at<float>(v, u);
                        float saturated_depth = max(min(pts(2, u*y+v), 1.0), 0.0);
                        //colors.push_back({1.0-saturated_depth, saturated_depth, (saturated_depth)*(saturated_depth - 1.)*4.});
                        Vec3f col = RawImageWithBGSmall.at<Vec3f>(v, u);
                        colors.push_back({col[0], col[1], col[2]});
                      }
                      rm.publishPointCloud(scene_pts_out, {"gelsight_pc"}, {0.1, 1.0, 0.1});
                    }
                  }
                #endif
                OutputImageNum++;
            }
        }
    }

    capture.release();

    return 0;
}
