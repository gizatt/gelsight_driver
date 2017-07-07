//
//  WindowDisplay.h
//  GelSight1
//
//  Created by Wenzhen on 10/8/15.
//  Copyright (c) 2015 MIT Persci Lab. All rights reserved.
//

#ifndef __GelSight1__WindowDisplay__
#define __GelSight1__WindowDisplay__

#include <iostream>
#include<stdarg.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "CaptureFrm.h"
#include "VideoRecord.h"

using namespace std;
using namespace cv;

//class FrameConvert;

class SubWindowRange
{
    int x1, x2, y1,y2;
public:
    int WindowLabel;
    void set(int x1_, int y1_, int x2_, int y2_, int label){x1=x1_;y1=y1_;x2=x2_;y2=y2_;WindowLabel=label;}
    bool Within(int x, int y)
    {   if(x>x2) return 0;
        if(x<x1) return 0;
        if(y>y2) return 0;
        if(y<y1) return 0;
        return 1;}
};


#define _WINDOWDISPLAY_CURVEIM_     10
#define _WINDOWDISPLAY_ARROWIM_     1
#define _WINDOWDISPLAY_RAWIM_       5


#define _SENDBACK_STARTREC_         11
#define _SENDBACK_ENDREC_           12
#define _SENDBACK_SAVEIM_           13
#define _SENDBACK_CHANGESOURCE_     21
#define _SENDBACK_RENEW_            25
#define _SENDBACK_EXITPROG_         100

class WindowDisplay
{
    string WindowName;
    Mat DisplayImage;
    Mat DisplayImage2;
    
    int sizex, sizey;
    int sizex0, sizey0;    // the size of the Input raw image
    int DisplaySizeX, DisplaySizeY;     //the size of the display image
    
    int saveNum; string saveName;
    
    // Viralbles for UI
    int SubWindowCount;
    SubWindowRange* subObjPos;
    int DisplayImType;
    int DisplayWindowSizeX,DisplayWindowSizeY;
    VideoRecord* objRecorder;


	VideoWriter objVideoWriter;
	bool OnRec;
	int RecNum;
    
public:
    int ShowImgNumber;
    char returnKey;
    int ReturnStatus;
    
    
    bool UIDisplay2Im;  //Flag to see whether to display UI
    
    
        
public:
    WindowDisplay(string strin){WindowName=strin;ini();}
    WindowDisplay(string strin, const CaptureFrm& scr)
    {   sizex0=scr.sizeX; sizey0=scr.sizeY;
            WindowName=strin;ini();}
    void Display(int nArgs, ...);
    void DisplayRGBDiff(const FrameConvert& scr)
    {
        Mat R,G,B;
        cvtColor(scr.DiffR,R,CV_GRAY2BGR);cvtColor(scr.DiffG,G,CV_GRAY2BGR);cvtColor(scr.DiffB,B,CV_GRAY2BGR);
        Display(4,&(scr.ShowIm), &R, &G, &B);
        //Display(3, &(scr.ShowIm), &(scr.ShowIm), &(scr.ShowIm));
    }
    void DisplayCurv(const FrameConvert& scr)
    {
        Mat DM;cvtColor(scr.DiffMax,DM,CV_GRAY2BGR);
        Display(2,&(scr.ShowIm), &DM);
    }
    
    void Display_UI2Im(const FrameConvert& scr);
    void Display_UI2Im_renew(const CaptureFrm& scr);
    WindowDisplay(string strin, const CaptureFrm& scr, VideoRecord& objRec)
    {
        sizex0=scr.sizeX; sizey0=scr.sizeY;
        WindowName=strin;ini();UIDisplay2Im=1;
        objRec.FrameSize=Size(sizex0, sizey0);
        ini_DisplayMat_UI2Im();
        objRecorder=&objRec;
    }
    void reNew(const CaptureFrm& scr)
    {
        if(UIDisplay2Im)    //UI
        {
            sizex0=scr.sizeX; sizey0=scr.sizeY;objRecorder->EndWriting();objRecorder->FrameSize=Size(sizex0, sizey0);
            ini_DisplayMat_UI2Im();
        }
    }
    
    ~WindowDisplay() {delete[] subObjPos;}
private:
    void ini();
    void Interact();
    void ini_DisplayMat();
    void ini_DisplayMat_UI2Im();        //ini for the UI display when there are 2 display windows
    void ButtonPress(int x, int y);

    static void onMouse(int event, int x, int y,  int, void* obj);
};

#endif /* defined(__GelSight1__WindowDisplay__) */
