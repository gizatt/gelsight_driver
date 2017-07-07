//
//  VideoRecord.h
//  GelSight3
//
//  Created by Wenzhen on 11/18/15.
//  Copyright (c) 2015 MIT Persci Lab. All rights reserved.
//

#ifndef __GelSight3__VideoRecord__
#define __GelSight3__VideoRecord__

#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

class VideoRecord
{
    int NameCount;
    string WriteNameBase;
    VideoWriter outputVideo;
    bool OnWriting;     // The flag of whether is in writing state
public:
    Size FrameSize;
    
public:
    VideoRecord(){NameCount=0;OnWriting=0;WriteNameBase="Rec";}
    
    void StartWriting();
    void EndWriting(){OnWriting=0;}
    void Update(const Mat& WriteFrame);
    
};


#endif /* defined(__GelSight3__VideoRecord__) */
