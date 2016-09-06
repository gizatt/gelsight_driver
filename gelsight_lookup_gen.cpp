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

#define REQ_NUM_ARGS (3)
#define BINS_PER_COLOR (12)
// #define COLOR_TO_BIN(c) ((size_t)(c * BINS_PER_COLOR))
#define COLOR_TO_BIN(c) ((size_t)((1+((c)*255/256))/2 * BINS_PER_COLOR))

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

  // Input argument parsing stuff:

  ezOptionParser opt;

  opt.overview = "Takes ground truth data and resultant images and "
                    "produces a lookup table mapping RGB -> Gradient. ";
  opt.syntax = "lookup_gen [OPTIONS] path_to_gt_folder/img_%07d.jpg path_to_image_folder/img_%07d.jpg background_image.jpg [OPTIONS]";
  opt.example = "lookup_gen groundtruth/depth groundtruth/raw\n\n";
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
    std::cerr << "ERROR: Expected 3 (?) arguments.\n\n";
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
  string rawSource = allArgs[2];

  // Third non-option argument is background image.
  string bgImageName = allArgs[3];

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


  for (int r=0; r<BINS_PER_COLOR; r++) {
    for (int g=0; g<BINS_PER_COLOR; g++) {
      for (int b=0; b<BINS_PER_COLOR; b++) {
        count_lookup[r][g][b] = 0;   } } }
  
  Mat BgImage;
  BgImage = imread(bgImageName.c_str());
  BgImage.convertTo(BgImage, CV_32FC3);
  BgImage /= 255.0;
  if (BgImage.empty()) {
    printf("Error reading background image\n");
    return 1;
  }

  VideoCapture captureGt;
  captureGt.open(gtSource);
  
  VideoCapture captureRaw;
  captureRaw.open(rawSource);
  
  Mat GtImage;
  Mat RawImage;
  
  captureRaw >> RawImage;
  captureGt >> GtImage;
  
  if ((RawImage.empty() || GtImage.empty())) {
    printf("Error: Input images streams empty.");
    return 1;
  }
  
  while(!(RawImage.empty() || GtImage.empty())) {
    RawImage.convertTo(RawImage, CV_32FC3);
    RawImage /= 255.0;
    RawImage = RawImage - BgImage;
    GtImage.convertTo(GtImage, CV_32FC3);
    GtImage /= 255.0;
    // At this point, we have a depth image stored in GtImage and
    // a Gelsight observed image stored in RawImage. Their pixels
    // should correspond. Thus, we can loop over these pixels and
    // add them as entries into the gradient->RGB lookup table.
    for (int i=0; i<RawImage.rows - 1; i++) {
      for (int j=0; j<RawImage.cols - 1; j++) {
        Vec3f depthColor = GtImage.at<Vec3f>(i,j);
        float depth = depthColor[0];
        Vec3f depthColorR = GtImage.at<Vec3f>(i+1,j);
        float depthR = depthColorR[0];
        Vec3f depthColorC = GtImage.at<Vec3f>(i,j+1);
        float depthC = depthColorC[0];
        
        float gradR = depthR - depth;
        float gradC = depthC - depth;

        Vec3f rawColor = RawImage.at<Vec3f>(i,j);
        
        size_t r = COLOR_TO_BIN(rawColor[0]);
        size_t g = COLOR_TO_BIN(rawColor[1]);
        size_t b = COLOR_TO_BIN(rawColor[2]);
        
        grad_r_lookup[r][g][b] += gradR;
        grad_c_lookup[r][g][b] += gradC;
        
        // Note that we have actually observed a value at this rgb
        count_lookup[r][g][b] += 1;
      }
    }
    
    captureRaw >> RawImage;
    captureGt >> GtImage;
  }
  
  for (int r=0; r<BINS_PER_COLOR; r++) {
    for (int g=0; g<BINS_PER_COLOR; g++) {
      for (int b=0; b<BINS_PER_COLOR; b++) {
        if (count_lookup[r][g][b] != 0) {
          grad_r_lookup[r][g][b] /= count_lookup[r][g][b];
          grad_c_lookup[r][g][b] /= count_lookup[r][g][b];
        }
      }
    }
  }

  // There will be holes in our histogram; fill them in by growing (potentially pathologically)
  // out from the known entries
  for (int pass_no=1; pass_no<=BINS_PER_COLOR+BINS_PER_COLOR+BINS_PER_COLOR; pass_no++) {
    for (int r=0; r<BINS_PER_COLOR; r++) {
      for (int g=0; g<BINS_PER_COLOR; g++) {
        for (int b=0; b<BINS_PER_COLOR; b++) {
          if (count_lookup[r][g][b] == 0) {
            grad_r_lookup[r][g][b] /= count_lookup[r][g][b];
            grad_c_lookup[r][g][b] /= count_lookup[r][g][b];
            for (int dr=-1; dr<=1; dr+=2) {
              for (int dg=-1; dg<=1; dg+=2) {
                for (int db=-1; db<=1; db+=2) {
                  if ((r+dr > 0 && r+dr < BINS_PER_COLOR) && (g+dg > 0 && g+dg < BINS_PER_COLOR)
                      && (b+db > 0 && b+db < BINS_PER_COLOR)) { // bounds-checking neighbor
                    if (count_lookup[r+dr][g+dg][b+db] != 0 && count_lookup[r+dr][g+dg][b+db] != -pass_no) {
                      grad_r_lookup[r][g][b] = .9*grad_r_lookup[r+dr][g+dg][b+db];
                      grad_c_lookup[r][g][b] = .9*grad_c_lookup[r+dr][g+dg][b+db];
                      count_lookup[r][g][b] = -pass_no;
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  // Write the entries of the table out to files
  ofstream table_file;
  table_file.open("trained_lookup.dat");
  if (table_file) {
    for (int i=0; i < BINS_PER_COLOR; i++) {
      for (int j=0; j < BINS_PER_COLOR; j++) {
        for (int k=0; k < BINS_PER_COLOR; k++) {
          table_file << grad_r_lookup[i][j][k] << " ";
        }
      }
    }
    for (int i=0; i < BINS_PER_COLOR; i++) {
      for (int j=0; j < BINS_PER_COLOR; j++) {
        for (int k=0; k < BINS_PER_COLOR; k++) {
          table_file << grad_c_lookup[i][j][k] << " ";
        }
      }
    }
    table_file.close();
  } else {
    printf("Error opening table file for writing.\n");
  }
  
  
  // Make sure entries can be read properly
  ifstream table_read_file;
  table_read_file.open("trained_lookup.dat");
  if (table_read_file) {
    for (int i=0; i < BINS_PER_COLOR; i++) {
      for (int j=0; j < BINS_PER_COLOR; j++) {
        for (int k=0; k < BINS_PER_COLOR; k++) {
          float read_float;
          table_read_file >> read_float;
          float error = read_float - grad_r_lookup[i][j][k];
          error = (error > 0) ? error : -error;
          if ((read_float != 0.0 && error/read_float > .00001) || (read_float == 0.0 && error > .00001)) {
            printf("DIFFERENCE: %f , %f , %f , %f\n", read_float, grad_r_lookup[i][j][k], error, error/read_float);
            return 1;
          }
        }
      }
    }
    for (int i=0; i < BINS_PER_COLOR; i++) {
      for (int j=0; j < BINS_PER_COLOR; j++) {
        for (int k=0; k < BINS_PER_COLOR; k++) {
          float read_float;
          table_read_file >> read_float;
          float error = read_float - grad_c_lookup[i][j][k];
          error = (error > 0) ? error : -error;
          if ((read_float != 0.0 && error/read_float > .00001) || (read_float == 0.0 && error > .00001)) {
            printf("DIFFERENCE: %f , %f , %f , %f\n", read_float, grad_c_lookup[i][j][k], error, error/read_float);
            return 1;
          }
        }
      }
    }
  }
  
  printf("Confirmed: Grad_r and Grad_c written successfully!");

  for (int i=0; i < BINS_PER_COLOR; i++) {
    for (int j=0; j < BINS_PER_COLOR; j++) {
      for (int k=0; k < BINS_PER_COLOR; k++) {
        cout << count_lookup[i][j][k] << " ";
      }
    }
  }

  cout << "\n\n\n\nGrads:\n\n";
  

  for (int i=0; i < BINS_PER_COLOR; i++) {
    for (int j=0; j < BINS_PER_COLOR; j++) {
      for (int k=0; k < BINS_PER_COLOR; k++) {
        cout << grad_r_lookup[i][j][k] << " ";
      }
    }
  }

  cout << "\n\n";

}
