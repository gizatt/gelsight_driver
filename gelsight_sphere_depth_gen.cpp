/****
Takes several images of a ball bearing pressed into a gelsight and
produces ground-truth depth images based on all the spheres
detected.

MIT 2016 -- geronm, gizatt

Allows user to specify which aligned ball-image should be used for
reference. It is assumed that this ball is centered in the image.

Working from this guy's nice starting code to get going
https://wimsworld.wordpress.com/2013/07/19/webcam-on-beagleboardblack-using-opencv/
****/

#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <fstream>  // nice number I/O
#include <unistd.h> // for sleep
#include <opencv2/opencv.hpp>
#include <math.h> // for exponentiation

#include "../eigen/Eigen/IterativeLinearSolvers" // for least squares solving
#include <sys/stat.h> // mkdir
#include "lib/ezOptionParser/ezOptionParser.hpp"

using namespace std;
using namespace cv;
using namespace ez;

#define DOT_THRESHOLD 0.05
#define DOT_DILATE_AMT 3  // removes small spurious points
#define DOT_ERODE_AMT 8  // expands dots a big to get rid of edges
#define BALL_RADIUS_GUESS 45  // radius of detected ball. TODO: Make this a CL argument
#define BALL_RADIUS_GUESS_MARGIN (BALL_RADIUS_GUESS + 20)  // margin of error on ball radius, for HoughCircle
#define BALL_RADIUS_TIGHT (.85 * BALL_RADIUS_GUESS) // the assumed actual radius of the sphere, as opposed to the radius of the image it generates.
#define SNAPSHOT_WIDTH (2*BALL_RADIUS_GUESS_MARGIN)
#define REF_PT_ROWS 3  // grid resolution of final lookup table (how many lookup locations there are) TODO: CL argument
#define REF_PT_COLS 4
#define REF_GET_IMROW(ptrow, imrows) ( ((1+(ptrow)) * (imrows)) / (1+REF_PT_ROWS) )
#define REF_GET_IMCOL(ptcol, imcols) ( ((1+(ptcol)) * (imcols)) / (1+REF_PT_COLS) )
#define SQDIST(x, y) ((x)*(x) + (y)*(y))
#define CENTER_DIST_THRESHOLD (15)  // max amount that the center of a ball can be slid by the RGB-alignment
                                    //procedure, otherwise it is deemed a failed detection.

static double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

/**
 * A quick power-function which calculates something
 * like a mean-zero Gaussian PDF with a narrow sigma.
 * Specifically, it calculates 10000^(-(r*r)).
 */
static inline float erf(float r) {
    return pow(.0001,r*r);
}

void Usage(ezOptionParser& opt) {
  std::string usage;
  opt.getUsage(usage);
  std::cout << usage;
};

int main(int argc, const char *argv[])
{

  // Input argument parsing stuff:

  ezOptionParser opt;

  opt.overview = "Takes several images of a ball bearing pressed into a gelsight and"
                    "produces ground-truth depth images based on all the spheres"
                    "detected.";
  opt.syntax = "groundtruth_gen [OPTIONS] path_to_video [OPTIONS]";
  opt.example = "groundtruth_gen sphereference/img_%07d.jpg -v 0\n\n";
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
    "-1", // Default.
    0, // Required?
    1, // Number of args expected.
    0, // Delimiter if expecting multiple args.
    "Path to detected ball frame image, to use as reference; all others will be aligned to this image."
    " For best results, sphere should be centered in the image. If empty, the program will use the"
    " first detected sphere as reference. Whichever image is used will be written to"
    " ./groundtruth/sphere_standard.jpg. After running, look in ./groundtruth/sphereextracted/ to"
    " find candidates for standard sphere.", // Help description.
    "-r",     // Flag token.
    "--ref-frame" // Flag token.
  );

  opt.add(
    "1", // Default.
    0, // Required?
    1, // Number of args expected.
    0, // Delimiter if expecting multiple args.
    "Visualize.", // Help description.
    "-v",     // Flag token.
    "--visualization" // Flag token.
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

  string refFramePath = "";
  if (opt.isSet("-r")) {
    opt.get("-r")->getString(refFramePath);
  }

  bool visualize = true;
  if (opt.isSet("-v")) {
    int boolAsNum = 1;
    opt.get("-v")->getInt(boolAsNum);
    visualize = (boolAsNum != 0);
  }

  mkdir("groundtruth", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  mkdir("groundtruth/sphereextracted", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir("groundtruth/spherealigned", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir("groundtruth/sphererefptimgs", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir("groundtruth/depth", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);


  ofstream resultsFile;
  resultsFile.open("groundtruth/circle_index.csv");

  Mat SphereReference;
  if (refFramePath.length() != 0) {
    SphereReference = imread(refFramePath.c_str());
    SphereReference.convertTo(SphereReference, CV_32FC3);

    {
      std::ostringstream OutputAlignedFilename;
      OutputAlignedFilename << "groundtruth/sphere_standard.jpg";
      imwrite(OutputAlignedFilename.str(), SphereReference);
    }

    assert(!SphereReference.empty());
  }

  VideoCapture capture(videoSource);  // Using -1 tells OpenCV to grab whatever camera is available.
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
  int vidi = 0;
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

  // Prepare table in which to store final lookup images,
  // each of same size as image slices from alignment
  Mat referencePointImgs[REF_PT_ROWS][REF_PT_COLS];
  float referencePointWeights[REF_PT_ROWS][REF_PT_COLS];

  for (size_t r = 0; r < REF_PT_ROWS; r++) {
    for (size_t c = 0; c < REF_PT_COLS; c++) {
      Mat temp(SNAPSHOT_WIDTH, SNAPSHOT_WIDTH, CV_32FC3);
      referencePointImgs[r][c] = temp;

      referencePointWeights[r][c] = 0.0; // unnecessary, but good practice to write this explicitly, I feel.
    }
  }

  if (visualize){
    namedWindow( "RawImage", cv::WINDOW_AUTOSIZE );
    namedWindow( "SphereImage", cv::WINDOW_AUTOSIZE );
    startWindowThread();
  }

  double last_send_time = getUnixTime();
  int OutputImageNum = 0;
  vector<Point> gt_centers;
  vector<Point> gt_centers_corrected;
  Mat RawImageWithBG;
  do {
      // wasteful but lower latency.
      Mat RawImage;
      capture >> RawImageWithBG;
      last_send_time = getUnixTime();
      if(!RawImageWithBG.empty())
      {
          RawImageWithBG.copyTo(RawImage);
          RawImage.convertTo(RawImage, CV_32FC3);
          RawImage /= 255.0;

          RawImage -= BGImage;


          Mat RawImageDotsMap;
          cvtColor(RawImage, RawImageDotsMap, CV_RGB2GRAY);
          cv::threshold(RawImageDotsMap, RawImageDotsMap, DOT_THRESHOLD, 1.0, CV_THRESH_BINARY);
          dilate(RawImageDotsMap, RawImageDotsMap, dilateElement);
          erode(RawImageDotsMap, RawImageDotsMap, erodeElement);


          // Find circle in image
          Mat GrayImage, SphereImage;
          cvtColor(RawImageWithBG, GrayImage, CV_RGB2GRAY);
          GrayImage.convertTo(GrayImage, CV_8UC1);
          vector<Vec3f> circles;
          HoughCircles(GrayImage, circles, CV_HOUGH_GRADIENT,
              1, GrayImage.rows/4, 100, 30, BALL_RADIUS_GUESS-10, BALL_RADIUS_GUESS+10);

          RawImageWithBG.copyTo(SphereImage);
          cvtColor(SphereImage, SphereImage, CV_RGB2GRAY);
          cvtColor(SphereImage, SphereImage, CV_GRAY2RGB);

          for( size_t i = 0; i < gt_centers.size(); i++ )
          {
               // draw previous circle centers
               circle(SphereImage, gt_centers[i], 3, Scalar(0,255,0), -1, 8, 0 );
          }

          for( size_t i = 0; i < gt_centers_corrected.size(); i++ )
          {
               // draw previous circle centers
               circle(SphereImage, gt_centers_corrected[i], 3, Scalar(0,255,255), -1, 8, 0 );
          }

          for( size_t i = 0; i < circles.size(); i++ )
          {
              Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
              int radius = cvRound(circles[i][2]);
              // draw the circle center
              circle(SphereImage, center, 3, Scalar(0,255,0), -1, 8, 0 );
              // draw the circle outline
              circle(SphereImage, center, 50, Scalar(0,0,255), 3, 8, 0 );
          }

          Mat RawImageWithBGBlurred;
          RawImageWithBG.copyTo(RawImageWithBGBlurred);
          GaussianBlur(RawImageWithBGBlurred,RawImageWithBGBlurred,Size(51,51),1.0);

          if (circles.size() > 0) {
            // Write aligned sphere image
            Rect circleRect(cvRound(circles[0][0])-BALL_RADIUS_GUESS_MARGIN,cvRound(circles[0][1])-BALL_RADIUS_GUESS_MARGIN,SNAPSHOT_WIDTH,SNAPSHOT_WIDTH);

            if (!((circleRect.x < 0) || (circleRect.y < 0) || (circleRect.x + circleRect.width > RawImageWithBG.cols)
              || (circleRect.y + circleRect.height > RawImageWithBG.rows))) {

              Point aCenter (circles[0][0], circles[0][1]);

              Mat SphereExtracted;
              SphereExtracted = RawImageWithBGBlurred(circleRect);

              {
                std::ostringstream OutputAlignedFilename;
                OutputAlignedFilename << "groundtruth/sphereextracted/img_";
                OutputAlignedFilename << setfill('0') << setw(7) << OutputImageNum;
                OutputAlignedFilename << ".jpg";
                imwrite(OutputAlignedFilename.str(), SphereExtracted);
              }

              // Check to see whether a reference frame was supplied by the user; if
              // not, the first detected sphere frame will serve as reference.
              if (SphereReference.empty()) {
                SphereExtracted.copyTo(SphereReference);
                SphereReference.convertTo(SphereReference, CV_32FC3);

                {
                  std::ostringstream OutputAlignedFilename;
                  OutputAlignedFilename << "groundtruth/sphere_standard.jpg";
                  imwrite(OutputAlignedFilename.str(), SphereReference);
                }

                assert(!SphereReference.empty());
              }

              printf("SphereReference value: %f\n",SphereReference.at<Vec3f>(10,10)[0]);

              Mat CorrImg;
              filter2D(SphereExtracted/255, CorrImg, CV_32FC3, SphereReference/255, Point(cvRound(SphereReference.rows/2),cvRound(SphereReference.cols/2)), 0.0, BORDER_REPLICATE);
              CorrImg = CorrImg / 10;

              // Find max point in correlation image
              {
                float maxVal = 0;
                int r = 0;
                int c = 0;
                for (int i=0; i<CorrImg.rows; i++) {
                  for (int j=0; j<CorrImg.cols; j++) {
                    Vec3f color = CorrImg.at<Vec3f>(i,j);
                    float norm = (color[0]*color[0] + color[1]*color[1] + color[2]*color[2]);
//                          float norm = color[0]; // uncomment for mindist instead of squared norm
//                          norm = (norm > color[1]) ? color[1] : norm;
//                          norm = (norm > color[2]) ? color[2] : norm;
                    if (maxVal < norm) {
                      maxVal = norm;
                      r = i;
                      c = j;
                    }
                  }
                }
                Vec3f black(1.0, 0, 1.0);
                CorrImg.at<Vec3f>(r,c) = black;

                r -= cvRound(SphereReference.rows/2);
                c -= cvRound(SphereReference.cols/2);

                Mat SphereAligned;
                int top = (r > 0) ? 0 : -r;
                int bottom = (r < 0) ? 0 : r;
                int left = (c > 0) ? 0 : -c;
                int right = (c < 0) ? 0 : c;
                copyMakeBorder(SphereExtracted, SphereAligned, top, bottom, left, right, BORDER_REPLICATE);
                Rect alignedSphere (left+c,top+r,SphereReference.cols,SphereReference.rows);
                printf("%d,%d,%d,%d | %d,%d,%d,%d | %d,%d\n", \
                    top,bottom,left,right,alignedSphere.x,alignedSphere.y,alignedSphere.width,alignedSphere.height,
                    SphereAligned.rows, SphereAligned.cols);
                SphereAligned = SphereAligned(alignedSphere);

                // Final check: Did we detect a circle?
                if (r < CENTER_DIST_THRESHOLD && c < CENTER_DIST_THRESHOLD && r > -CENTER_DIST_THRESHOLD && c > -CENTER_DIST_THRESHOLD) {
                  // If we are here, then that means we have:
                  //  1. detected a circle in the image, and..
                  //  2. seem to have successfully aligned said point to the reference image.
                  // Thus, SphereAligned can become a reference image to contribute to our lookup tables!

                  Point aCenterCorrected(aCenter.x + c, aCenter.y + r);

                  gt_centers.push_back(aCenter);
                  gt_centers_corrected.push_back(aCenterCorrected);

                  // Write out aligned image
                  {
                    std::ostringstream OutputAlignedFilename;
                    OutputAlignedFilename << "groundtruth/spherealigned/img_";
                    OutputAlignedFilename << setfill('0') << setw(7) << OutputImageNum;
                    OutputAlignedFilename << ".jpg";
                    bool status = imwrite(OutputAlignedFilename.str(), SphereAligned);
                    if (!status) {
                      printf("Warning: image write failure for path %s\n", OutputAlignedFilename.str().c_str());
                    }
                  }
                  // Write csv entry for successfully-aligned sphere image to reference images table file
                  resultsFile << OutputImageNum << "," << aCenterCorrected.x << "," << aCenterCorrected.y << std::endl;

                  // Formatting
                  Mat SphereAlignedHighRes;
                  SphereAligned.convertTo(SphereAlignedHighRes, CV_32FC3);

                  // Make a contribution to each reference-pt image which is proportional to
                  //  this reference image's closeness to each reference-pt
                  for (size_t r = 0; r < REF_PT_ROWS; r++) {
                    for (size_t c = 0; c < REF_PT_COLS; c++) {
                      int imrow = REF_GET_IMROW (r, RawImageWithBG.rows);
                      int imcol = REF_GET_IMCOL (c, RawImageWithBG.cols);
                      int centerrow = (aCenter.y+top+r+(SphereReference.rows/2));
                      int centercol = (aCenter.x+left+c+(SphereReference.cols/2));

                      float ptDist = sqrt(SQDIST(imrow-centerrow, imcol-centercol));
                      float weight = erf(2*ptDist / RawImageWithBG.cols);

                      //~for every lookup point, make a contribution from each r-away reference point of N(r;0,sigma)
                      referencePointImgs[r][c] += weight * SphereAlignedHighRes;
                      referencePointWeights[r][c] += weight;
                    }
                  }
                  
                  // Output a depth image, under the assumption that the sphere is the only source of depth in the image.
                  Mat DepthImage(RawImageWithBG.rows, RawImageWithBG.cols, CV_32FC1, Scalar(0)); // Black image
                  for (int dx = -BALL_RADIUS_TIGHT; dx < BALL_RADIUS_TIGHT; dx++) {
                    for (int dy = -BALL_RADIUS_TIGHT; dy < BALL_RADIUS_TIGHT; dy++) {
                      int imrow = aCenterCorrected.y + dy;
                      int imcol = aCenterCorrected.x + dx;
                      if (imrow >= 0 && imrow < DepthImage.rows && imcol >= 0 && imcol < DepthImage.cols) {
                        float normdist = hypot(dx, dy) / BALL_RADIUS_TIGHT;
                        float height = sqrt(1 - (normdist*normdist));
                        DepthImage.at<float>(imrow, imcol) = height;
                      }
                    }
                  }
                  DepthImage *= 255.0;
                  DepthImage.convertTo(DepthImage, CV_8UC1);
                  {
                    std::ostringstream OutputAlignedFilename;
                    OutputAlignedFilename << "groundtruth/depth/img_";
                    OutputAlignedFilename << setfill('0') << setw(7) << OutputImageNum;
                    OutputAlignedFilename << ".jpg";
                    bool status = imwrite(OutputAlignedFilename.str(), DepthImage);
                    if (!status) {
                      printf("Warning: image write failure for path %s\n", OutputAlignedFilename.str().c_str());
                    }
                  }
                }
              }
            }
          }

          OutputImageNum++;
          if (visualize){
            cv::imshow("RawImage", RawImageWithBG);
            cv::imshow("SphereImage", SphereImage);
          }
      }
  } while (!RawImageWithBG.empty());

  resultsFile.close();

  //~save out the average images resulting from this operation
  for (size_t r = 0; r < REF_PT_ROWS; r++) {
    for (size_t c = 0; c < REF_PT_COLS; c++) {

      int imrow = REF_GET_IMROW (r, RawImageWithBG.rows);
      int imcol = REF_GET_IMCOL (c, RawImageWithBG.cols);

      // TODO Write csv entry for reference pt sphere image to reference pt images table
//        resultsFile << OutputImageNum << "," << aCenterCorrected.x << "," << aCenterCorrected.y << std::endl;

      // renormalize image
      referencePointImgs[r][c] /= referencePointWeights[r][c];

//        // draw the circle center
//        circle(SphereImage, center, 3, Scalar(0,255,0), -1, 8, 0 );
//        // draw the circle outline
//        circle(SphereImage, center, 50, Scalar(0,0,255), 3, 8, 0 );

      // Write out aligned image
      std::ostringstream OutputAlignedFilename;
      OutputAlignedFilename << "groundtruth/sphererefptimgs/img_";
      OutputAlignedFilename << "r" << setfill('0') << setw(4) << r;
      OutputAlignedFilename << "c" << setfill('0') << setw(4) << c;
      OutputAlignedFilename << ".jpg";

      printf("filename: ");
      cout << OutputAlignedFilename.str();
      printf(".....");

      bool status = imwrite(OutputAlignedFilename.str(), referencePointImgs[r][c]);
      if (status) {
        printf("success!\n");
      } else {
        printf("failure.\n");
      }
    }
  }

  // Now need to....:

  //~[back to depth script:] Use libkdtree++ octree to make a nearest-neighbor lookup model to get gradients

  return 0;
}
