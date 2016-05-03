//
//  MarkerTrack.h
//  GelSight1
//
//  Created by Wenzhen on 10/6/15.
//  Copyright (c) 2015 MIT Persci Lab. All rights reserved.
//

#ifndef __GelSight1__MarkerTrack__
#define __GelSight1__MarkerTrack__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


#include "CaptureFrm.h"

class MarkerTrack
{
public:
    bool MarkerAvailable;
    bool MotionCaled;
    int MarkerCount;
    int LastMarkerCount;
    int MarkerIniCount;

    double* MarkerCenter;   // Store the center of the markers; x&y&area
    double* MarkerIniCenter;
    double* MarkerLastCenter;
    double* MarkerMoveDis;  // Store the moving distance of the markers in the current frame;
private:
    bool isMarkerInied;
    
    
    //local virables
    int MinMarkerNum;
    double MarkerThresh;    //Threshold for marker brightness
    double MarkerAreaThresh;
    int maxarea;
    int *LabelMap;
    int sizex,sizey;
    int *LabelCount;int* LabelNum;
    int *sumX;int* sumY;
    double *PixSum;
    int *no_seq;    //local virable for matching markers
    //local virables for display
    cv::Scalar DisplayColor;
    int DisplayThick;
    double QuiverScale;
    
public:
    MarkerTrack();
    MarkerTrack(const CaptureFrm& input);
    void UpdateMarkerMotion(const Mat& Frame);  //the finding marker part is included
    void UpdateMarkerMotion(FrameConvert& input);
    ~MarkerTrack(){DeleteArrays();}
    void QuiverDisplayField(Mat& Frame);
    void QuiverDisplayField(FrameConvert& input){QuiverDisplayField(input.ShowIm);}
    
    void FindMarkerCenter(const Mat& Frame);    // the input Mat should be the gray image
    void FindMarkerCenter(const FrameConvert& input){FindMarkerCenter(input.GrayImg);}
    void FindIniMarkerCenter(const Mat& Frame);
    void ReNew(const CaptureFrm& input);

    
private:
    void CalMarkerMovement();       //calculate the moving distance of the markers, given that the centers of the current frame and the last frame is known;
    void ParaIni();
    void ParaIni(const CaptureFrm& input);
    void DisplayParaIni();
    void DeleteArrays();
    
    int LabelConnection(const Mat& Frame);      //calculate the connection map of the current frame
    void FindCurrentCenterFromLablemap(int count);
    double MarkerCenterDisDefine(double* a, double*b);  //local function; calculate the self-defined distance betweeen two markers
    
};




#endif /* defined(__GelSight1__MarkerTrack__) */
