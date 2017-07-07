//
//  main.cpp
//  GelSight1
//
//  Created by Wenzhen on 10/4/15.
//  Copyright (c) 2015 MIT Persci Lab. All rights reserved.
//

#include <iostream>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

#include "CaptureFrm.h"
#include "MarkerTrack.h"
#include "WindowDisplay.h"
#include "VideoRecord.h"

using namespace cv;

int main()
{
    CaptureFrm CapFrm(1,40,40);
//    CaptureFrm CapFrm("BallRoll_1.mp4", 80,80);
    VideoRecord objVideoWriter;
  
    if(!CapFrm.isDeviceOpen)
    {
        std::cout<<"No Device available"<<std::endl;;
        exit(0);
    }
    FrameConvert ConvFrm;
    ConvFrm.getBackGround(CapFrm);
    MarkerTrack TrackMarkers(CapFrm);
    WindowDisplay Display("show", CapFrm, objVideoWriter);
    while(CapFrm.isOpen()){
        CapFrm.updateFrm();
        ConvFrm.FrameUpdate(CapFrm);
        ConvFrm.detectContact();
        
        ConvFrm.TestDiffImg();//////////For Test
        //ConvFrm.calcBImg();
        TrackMarkers.UpdateMarkerMotion(ConvFrm);
        TrackMarkers.QuiverDisplayField(ConvFrm);
//        Display.Display(1,&(ConvFrm.ShowIm));
//        Display.DisplayRGBDiff(ConvFrm);
//        Display.DisplayCurv(ConvFrm);
        Display.Display_UI2Im(ConvFrm);
//        objVideoWriter.Update(CapFrm.Frame);
        if(Display.ReturnStatus == _SENDBACK_EXITPROG_) break;
        else if(Display.ReturnStatus == _SENDBACK_CHANGESOURCE_)
        {
            CapFrm.ChangeSoruce();Display.ReturnStatus=0;
            ConvFrm.getBackGround(CapFrm);
            TrackMarkers.ReNew(CapFrm);
			Display.Display_UI2Im_renew(CapFrm);
        }
        else if(Display.ReturnStatus == _SENDBACK_RENEW_)
        {
            cout<<"INI"<<endl;
            Display.ReturnStatus=0;
            ConvFrm.getBackGround(CapFrm);
            TrackMarkers.ReNew(CapFrm);
        }
    }
    objVideoWriter.EndWriting();
}