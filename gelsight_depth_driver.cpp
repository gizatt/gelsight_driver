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
#include <opencv2/highgui.hpp>
#include <math.h>

// 3D vis via VTK
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkDelaunay2D.h>
#include <vtkLookupTable.h>
#include <vtkMath.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkXMLPolyDataWriter.h>
#include "vtkCamera.h"
#include "vtkCylinderSource.h"

#include "../eigen/Eigen/IterativeLinearSolvers" // for least squares solving
#include "lib/libkdtree/kdtree++/kdtree.hpp" //
#include "rgbToGradientOctNode.hpp"          // for rgb octree
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
#define FILTER_IMCOLS 128

#define BALL_RADIUS_GUESS 45  // radius of detected ball. TODO: Make this a CL argument
#define BALL_RADIUS_GUESS_MARGIN (BALL_RADIUS_GUESS + 20)  // margin of error on ball radius, for HoughCircle
#define SNAPSHOT_WIDTH (2*BALL_RADIUS_GUESS_MARGIN)
#define BALL_RADIUS_TIGHT (.85 * BALL_RADIUS_GUESS)
#define REF_PT_ROWS 4  // grid resolution of final lookup table (how many lookup locations there are) TODO: CL argument
#define REF_PT_COLS 3
#define REF_GET_IMROW(ptrow, imrows) ( ((1+(ptrow)) * (imrows)) / (1+REF_PT_ROWS) )
#define REF_GET_IMCOL(ptcol, imcols) ( ((1+(ptcol)) * (imcols)) / (1+REF_PT_COLS) )
#define REF_GET_PTROW(imrow, imrows) ( ( (imrow) * (REF_PT_ROWS+1) ) / (imrows) )
#define REF_GET_PTCOL(imcol, imcols) ( ( (imcol) * (REF_PT_COLS+1) ) / (imcols) )
#define SQDIST(x, y) ((x)*(x) + (y)*(y))

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

    enum GradMode { kUseLookupTable, kUseConvFilter, kUseOtherMethod };

    bool save_images = atoi(argv[1]);
    bool visualize = atoi(argv[2]);
    GradMode gradMode = kUseConvFilter;
    if (save_images)
        mkdir("output", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    lcm_t * lcm = lcm_create("udpm://239.255.76.67:7667?ttl=0");
    pthread_t lcmThread;
    pthread_create(&lcmThread, NULL, lcmMonitor, lcm);

    VideoCapture capture(0);   // Using -1 tells OpenCV to grab whatever camera is available.
    if(!capture.isOpened()){
        std::cout << "Failed to connect to the camera." << std::endl;
        return(1);
    }
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);

    Mat temp_image; capture >> temp_image; // warm up capture

    //capture.set(CV_CAP_PROP_EXPOSURE, -2);
    // use a command line tool to do this instead:
    char buf[300];
    int ret = 1;
    int vidi=0;
    // in case we're not on video0, loop through...

    struct timespec tim, tim2; // use awkward c convention to
    tim.tv_sec = 0;            // set up half-second duration
    tim.tv_nsec = 250000000L;

    /*
    while (ret != 0){
      nanosleep(&tim, &tim2);
      sprintf(buf, "v4l2-ctl --device=/dev/video%d --set-ctrl=white_balance_temperature_auto=0,backlight_compensation=0,exposure_auto=1,exposure_absolute=30,exposure_auto_priority=0,gain=50", vidi);
      printf("Trying %s\n", buf);
      ret = system(buf);
      vidi++;
    }
*/
    if (visualize){
      namedWindow( "RawImage", cv::WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
      moveWindow( "RawImage", 0, 0);
      namedWindow( "GradientVisImage", cv::WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
      moveWindow( "GradientVisImage", 0, 640);
      namedWindow( "DepthImage", cv::WINDOW_NORMAL );
      moveWindow( "DepthImage", 480, 0);
      startWindowThread();
    }

    // Create a grid of points (height/terrian map)
    vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
   
    unsigned int GridSize = 20;
    double xx, yy, zz;
    std::map<int, vtkIdType> point_index_map;
    for(unsigned int x = 0; x < GridSize; x++)
    {
      for(unsigned int y = 0; y < GridSize; y++)
      {
      xx = x + vtkMath::Random(-.2, .2);
      yy = y + vtkMath::Random(-.2, .2);
      zz = vtkMath::Random(-.5, .5);
      point_index_map[x*GridSize+y] = points->InsertNextPoint(xx / GridSize + 1, yy / GridSize + 1, zz);
      }
    }

// This creates a polygonal cylinder model with eight circumferential facets
// (i.e, in practice an octagonal prism).
vtkSmartPointer<vtkCylinderSource> cylinder =
  vtkSmartPointer<vtkCylinderSource>::New();
cylinder->SetResolution(8);

// The mapper is responsible for pushing the geometry into the graphics library.
// It may also do color mapping, if scalars or other attributes are defined.
vtkSmartPointer<vtkPolyDataMapper> cylinderMapper =
  vtkSmartPointer<vtkPolyDataMapper>::New();
cylinderMapper->SetInputConnection(cylinder->GetOutputPort());

// The actor is a grouping mechanism: besides the geometry (mapper), it
// also has a property, transformation matrix, and/or texture map.
// Here we set its color and rotate it around the X and Y axes.
vtkSmartPointer<vtkActor> cylinderActor =
  vtkSmartPointer<vtkActor>::New();
cylinderActor->SetMapper(cylinderMapper);
cylinderActor->GetProperty()->SetColor(1.0000, 0.3882, 0.2784);
cylinderActor->RotateX(30.0);
cylinderActor->RotateY(-45.0);

    // Add the grid points to a polydata object
    vtkSmartPointer<vtkPolyData> inputPolyData = 
      vtkSmartPointer<vtkPolyData>::New();
    inputPolyData->SetPoints(points);
   
    // Triangulate the grid points
    vtkSmartPointer<vtkDelaunay2D> delaunay = 
      vtkSmartPointer<vtkDelaunay2D>::New();
    delaunay->SetInput(inputPolyData);
    delaunay->Update();
    vtkPolyData* outputPolyData = delaunay->GetOutput();

    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(outputPolyData->GetProducerPort()); 
    vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    // Create a VTK renderer, render window, and interactor
    vtkSmartPointer<vtkRenderer> renderer =
      vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

renderer->AddActor(cylinderActor);
  
    renderer->SetBackground(.1, .2, .3);
    renderer->AddActor(actor);
    renderWindow->Render();
    renderWindowInteractor->Initialize();



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
        flip(tempMat, tempMat, -1);
        tempMat.copyTo(conv_kernel[2-c1][c2]);
      }
    }
    printf("%f, %f, %f\n",conv_kernel[2][2].at<float>(0,0),conv_kernel[2][2].at<float>(0,1),conv_kernel[2][2].at<float>(1,0));
    printf("%f, %f, %f\n",conv_kernel[0][0].at<float>(0,0),conv_kernel[0][0].at<float>(0,1),conv_kernel[0][0].at<float>(1,0));
    printf("%f, %f, %f\n",conv_kernel[1][1].at<float>(0,0),conv_kernel[1][1].at<float>(0,1),conv_kernel[1][1].at<float>(1,0));


    // populate octrees for RGB-to-Gradient lookup-based conversion
    typedef KDTree::KDTree<3,rgbToGradientOctNode> RGBGradOctree;
    int refPtRows = 4;
    int refPtCols = 3;
    RGBGradOctree lookupTrees[refPtRows*refPtCols];
    for (int refr=0; refr<refPtRows; refr++) {
      for (int refc=0; refc<refPtCols; refc++) {
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

        RGBGradOctree * currentTree = &(lookupTrees[refr*refPtCols + refc]);

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
              node.rcgradient[0] = -(imc - (RefPtImage.cols/2))/BALL_RADIUS_TIGHT;
              node.rcgradient[1] = -(imr - (RefPtImage.rows/2))/BALL_RADIUS_TIGHT;

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

    // make subsampled version of RawImage
    unsigned int drows = FILTER_IMROWS;
    unsigned int dcols = FILTER_IMCOLS;
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
      A_coeffs.push_back(T(a_row, i, 1));
      a_row++;
      A_coeffs.push_back(T(a_row, (RawImageSmall.cols-1)*RawImageSmall.rows + i, 1));
      a_row++;
    }
    // borders -- top and bottom border
    for (int i=0; i<RawImageSmall.cols; i++){
      A_coeffs.push_back(T(a_row, i*RawImageSmall.rows + 0, 1));
      a_row++;
      A_coeffs.push_back(T(a_row, (i+1)*RawImageSmall.rows - 1, 1));
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
    while (1) {
        // wasteful but lower latency.
        Mat RawImage;
        Mat RawImageWithBG;
        capture >> RawImageWithBG;
        if (getUnixTime() - last_send_time > 0.0333){
            last_send_time = getUnixTime();
            if(!RawImageWithBG.empty())
            {
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


                Mat RawImageDotsMap;
                cvtColor(RawImage, RawImageDotsMap, CV_RGB2GRAY);
                cv::threshold(RawImageDotsMap, RawImageDotsMap, DOT_THRESHOLD, 1.0, CV_THRESH_BINARY);
                dilate(RawImageDotsMap, RawImageDotsMap, dilateElement);
                erode(RawImageDotsMap, RawImageDotsMap, erodeElement);

                resize(RawImage, RawImageSmall, dsize);
                resize(RawImage, RawImageWithBGSmall, dsize);
                GaussianBlur(RawImageSmall,RawImageSmall,Size(19,19),1.0);
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
                  case kUseLookupTable:
                    {
                      printf("Readying lookup process...");

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

                          std::pair<RGBGradOctree::const_iterator,double> found = currentTree->find_nearest(testNode);
                          rgbToGradientOctNode nearestNode = *found.first;
                          if (found.second > .2) { // if the match is too far from any reference color take null hypothesis of flat slope
                            nearestNode.rcgradient[0] = 0.0001;
                            nearestNode.rcgradient[1] = 0.0001;
                          }

                          // assign gradient to RCGradientImageChannels
                          RCGradientImageChannels[0].at<float>(imr,imc) = nearestNode.rcgradient[0];
                          RCGradientImageChannels[1].at<float>(imr,imc) = nearestNode.rcgradient[1];
                        }
                      }
                    }
                    break;
                  default:
                    std::cerr << "ERROR: Unexpected gradient finder mode encountered.\n";
                    return 1;
                }

                Mat GradientVisImage(RawImageSmall.rows, RawImageSmall.cols, CV_32FC3);
                Mat GradientVisImageChannels[3];
                GradientVisImageChannels[0] = RCGradientImageChannels[0];
                GradientVisImageChannels[1] = RCGradientImageChannels[1];
                GradientVisImageChannels[2] = 0*RCGradientImageChannels[0]; // Don't need a third channel for this vis
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

                // aaaaand done
                if (isnan(x(0)))
                  x = solver.solve(b);
                else
                  x = solver.solveWithGuess(b, x);

                Mat DepthImage(RawImageSmall.rows, RawImageSmall.cols, CV_32FC1);
                for (int u=0; u<RawImageSmall.cols; u++){
                  for (int v=0; v<RawImageSmall.rows; v++){
                    DepthImage.at<float>(v, u) = x(u*RawImageSmall.rows + v);
                    //if (u < 10 && v < 10){
                      //cout << x(u*RawImageSmall.rows + v) << ",";
                    //}
                  }
                  //if (u < 11)
                    //cout << "\n";
                }

                // Also do VTK vis
                if (points->GetNumberOfPoints() != x.rows()){
                  points->Initialize();
                  printf("Resetting VTK\n");
                  point_index_map.clear();
                  for (int u=0; u<RawImageSmall.cols; u++){
                    for (int v=0; v<RawImageSmall.rows; v++){
                      double z = x(u*RawImageSmall.rows + v);
                      point_index_map[u*RawImageSmall.rows + v] = 
                        points->InsertNextPoint(((double)u)/RawImageSmall.cols,
                                                ((double)v)/RawImageSmall.rows, 
                                                z);
                    }
                  }
                } else {
                  for (int u=0; u<RawImageSmall.cols; u++){
                    for (int v=0; v<RawImageSmall.rows; v++){
                      double z = x(u*RawImageSmall.rows + v);
                      points->SetPoint(point_index_map[u*RawImageSmall.rows + v], 
                                                ((double)u)/RawImageSmall.cols,
                                                ((double)v)/RawImageSmall.rows, 
                                                z);
                    }
                  }
                }

                // propagate everything through
                //inputPolyData->Modified();
                //renderWindow->Render();

                // do some compression
                vector<uchar> buf;
                Mat DepthImageEnc;
                DepthImage.copyTo(DepthImageEnc);
                DepthImageEnc *= 255.0;
                DepthImageEnc.convertTo(DepthImageEnc, CV_8UC1);
                bool success = imencode(".jpg", DepthImageEnc, buf);
                if (success){
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
                  Mat GradientVisImageBigger;
                  Mat DepthImageBigger;
                  cv::resize(GradientVisImage, GradientVisImageBigger, cv::Size(640, 480));
                  cv::resize(DepthImage, DepthImageBigger, cv::Size(640, 480));
                  cv::imshow("RawImage", RawImageWithBG);
                  cv::imshow("GradientVisImage", GradientVisImageBigger);
                  cv::imshow("DepthImage", .25 * DepthImageBigger);
                }
            }
        }
    }
    return 0;
}
