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

#define REQ_NUM_ARGS (2)

#define BINS_PER_COLOR (12)
// #define COLOR_TO_BIN(c) ((size_t)(c * BINS_PER_COLOR))
#define COLOR_TO_BIN(c) ((size_t)((1+((c)*255/256))/2 * BINS_PER_COLOR))

#define HIST_BINS (129)
#define ERROR_TO_HIST_BIN(e) ((size_t)((e+1.0)/2.0*(HIST_BINS-1)))

#define REF_PT_ROWS 3  // grid resolution of final lookup table (how many lookup locations there are) TODO: CL argument
#define REF_PT_COLS 4
#define REF_GET_IMROW(ptrow, imrows) ( ((1+(ptrow)) * (imrows)) / (1+REF_PT_ROWS) )
#define REF_GET_IMCOL(ptcol, imcols) ( ((1+(ptcol)) * (imcols)) / (1+REF_PT_COLS) )
#define SQDIST(x, y) ((x)*(x) + (y)*(y))

// RGB->Grad lookup arrays
float grad_r_lookup[BINS_PER_COLOR][BINS_PER_COLOR][BINS_PER_COLOR];
float grad_c_lookup[BINS_PER_COLOR][BINS_PER_COLOR][BINS_PER_COLOR];
int count_lookup[BINS_PER_COLOR][BINS_PER_COLOR][BINS_PER_COLOR];

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

int main(int argc, const char *argv[])
{
  long histogram[HIST_BINS];

  // Input argument parsing stuff:

  ezOptionParser opt;

  opt.overview = "Takes two corresponding heightmaps (ground truth & reconstruction), both 16-bit + 1-channel, "
                    "and produces comparison statistics.";
  opt.syntax = "lookup_gen [OPTIONS] source1 source2 [OPTIONS]";
  opt.example = "lookup_gen [OPTIONS] path_to_gt_folder/img_%07d.png path_to_recon_folder/img_%07d.png [OPTIONS]\n\n";
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

  if (totalNumArgs != REQ_NUM_ARGS + 1) {  // totalNumArgs includes program name argument.
    std::cerr << "ERROR: Expected 2 arguments.\n\n";
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

  // First non-option argument is source of ground truth.
  string gtSource = allArgs[1];

  // Second non-option argument is source of images.
  string reconSouce = allArgs[2];

  bool visualize = true;
  if (opt.isSet("-v")) {
    int boolAsNum = 1;
    opt.get("-v")->getInt(boolAsNum);
    visualize = (boolAsNum != 0);
  }

  for (int r=0; r<BINS_PER_COLOR; r++) {
    for (int g=0; g<BINS_PER_COLOR; g++) {
      for (int b=0; b<BINS_PER_COLOR; b++) {
        count_lookup[r][g][b] = 0;   } } }

  VideoCapture captureGt;
  captureGt.open(gtSource);
  
  VideoCapture captureRecon;
  captureRecon.open(reconSouce);
  
  Mat GtImage;
  Mat ReconImage;
  
  captureRecon >> ReconImage;
  captureGt >> GtImage;
  
  if ((ReconImage.empty() || GtImage.empty())) {
    printf("Error: Input images stream empty.");
    return 1;
  }
  
  // Write the entries of the table out to files
  ofstream table_file;
  table_file.open("comparison_results.csv");
  if (table_file) {
    
    while(!(ReconImage.empty() || GtImage.empty())) {
      for (int i=0; i<HIST_BINS; i++)
        histogram[i] = 0;
    
      ReconImage.convertTo(ReconImage, CV_32FC1);
      ReconImage /= 255.0;
      GtImage.convertTo(GtImage, CV_32FC1);
      GtImage /= 255.0;
      printf("About to access some floats!\n");
      // At this point, we have a depth image stored in GtImage and
      // a Gelsight observed image stored in ReconImage. Their pixels
      // should correspond. Thus, we can loop over these pixels and
      // add them as entries into the gradient->RGB lookup table.
      double MSE = 0;
      int count = 0;
      
      float reconMaxDepth = 0;
      float gtMaxDepth = 0;
      
      for (int i=0; i<ReconImage.rows; i++) {
        for (int j=0; j<ReconImage.cols; j++) {
          /*double reconDepth = (double)ReconImage.at<float>(i,j);
          double gtDepth = (double)GtImage.at<float>(i,j);*/

          /*Vec3f reconDepth = ReconImage.at<Vec3f>(i,j);
          if (i==ReconImage.rows-1 && j==ReconImage.cols-1) {
            printf("hey.\n");
            printf("%f\n",reconDepth[0]);
            printf("%f\n",reconDepth[1]);
            printf("%f\n",reconDepth[2]);
          }*/
          // Do some stuff with the errors.
          //double error = reconDepth - gtDepth;
          
          Vec3f reconDepth = ReconImage.at<Vec3f>(i,j);
          Vec3f gtDepth = GtImage.at<Vec3f>(i,j);
          
          if (gtDepth[0] > 0.01) {
            double error = reconDepth[0] - gtDepth[0];

            histogram[ERROR_TO_HIST_BIN(error)] += 1;
            
            MSE += (error > 0) ? error : -error ; //error*error;
            count += 1;
          }
          
          if (reconMaxDepth < reconDepth[0]) {
            reconMaxDepth = reconDepth[0];
          }
          if (gtMaxDepth < gtDepth[0]) {
            gtMaxDepth = gtDepth[0];
          }
        }
      }
      
      MSE /= count;
      
      double RMSE = MSE; //sqrt(MSE);
      for (int i=0; i<HIST_BINS-1; i++)
        table_file << histogram[i] << " ";
      table_file << HIST_BINS-1 << "\n";
      table_file << RMSE << "\n";
      
      printf("  recon max: %f\n", reconMaxDepth);
      printf("  gt max: %f\n", gtMaxDepth);
      
      captureRecon >> ReconImage;
      captureGt >> GtImage;
    }
    
    
    table_file.close();
  } else {
    printf("Error opening table file for writing.\n");
  }

}
