//
//  CaptureFrm.h
//  GelSight1
//
//  Created by Wenzhen on 10/4/15.
//  Copyright (c) 2015 MIT Persci Lab. All rights reserved.
//

#ifndef __GelSight1__CaptureFrm__
#define __GelSight1__CaptureFrm__

#define __AAAA___ 10 

#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

class CaptureFrm
// This class get the frames from the camera, and crop the frame
{
    friend class FrameConvert;
    VideoCapture Cam;
    int borderX, borderY;
    Range RangeX, RangeY;
    int FrmCount;
    int SourceNum;
public:
    bool isDeviceOpen;
    Mat FrameO, Frame;
    int sizeX0, sizeY0, sizeX, sizeY;   //Size X and SizeY are the final sizes
    
public:
    void setBorder(int a, int b){borderX=a; borderY=b;sizeX=sizeX0-borderX*2; sizeY=sizeY0-borderY*2;RangeX=Range(borderX,sizeX0-borderX);RangeY=Range(borderY,sizeY0-borderY);
    }
    void getiniFrm(){Cam>>FrameO; sizeX0=FrameO.cols;sizeY0=FrameO.rows;FrmCount=0;}
    CaptureFrm(int Device){isDeviceOpen=Cam.open(Device);getiniFrm();setBorder(0,0);SourceNum=Device;}
    CaptureFrm(){isDeviceOpen=Cam.open(1);getiniFrm();setBorder(0,0);SourceNum=-1;}
    CaptureFrm(int Device, int bx, int by){isDeviceOpen=Cam.open(Device);
		Cam.set(CV_CAP_PROP_FRAME_WIDTH, 960);
		Cam.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
        getiniFrm();setBorder(bx,by);SourceNum=Device;}
    CaptureFrm(string filename, int bx, int by){isDeviceOpen=Cam.open(filename);getiniFrm();setBorder(bx,by);SourceNum=-1;}
    void updateFrm(){Cam>>FrameO;if(! FrameO.rows) return; Frame=FrameO(RangeY,RangeX);if(FrmCount<10000)FrmCount++;}
    bool isOpen(){return (Cam.isOpened() && FrameO.rows);}
    void ChangeSoruce();

};


class FrameConvert
{
    double KernelSize;
    
    //local varables for detecting touch
    double chopsNum;
    int steps;
    double touchTresh;
public:
    Mat BackGround;
    Mat GrayImg;
    Mat ShowIm; //the original display image;
    Mat MinImg;
    Mat BImg;
    
    bool InTouch;
    
    //For Test
    Mat DiffR, DiffG, DiffB;
    Mat DiffMax;
    
public:
    FrameConvert(){KernelSize=50;}
    void FrameUpdate(const CaptureFrm& input);
    void getBackGround(const CaptureFrm& input);
    void getBackSubtrGray(const CaptureFrm& input){cvtColor(BackGround-input.Frame, GrayImg, CV_RGB2GRAY);}
    
    void detectContact();
    void getMinImg();
    void calcBImg();
    
    //For Test
    void TestDiffImg();
private:
    void ini();
    
};


#endif /* defined(__GelSight1__CaptureFrm__) */
