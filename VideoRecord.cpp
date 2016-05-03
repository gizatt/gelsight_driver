//
//  VideoRecord.cpp
//  GelSight3
//
//  Created by Wenzhen on 11/18/15.
//  Copyright (c) 2015 MIT Persci Lab. All rights reserved.
//

#include "VideoRecord.h"
#define VIDEOWRITECODE   CV_FOURCC('F','M','P','4')

void VideoRecord::StartWriting()
{
    NameCount++;
    OnWriting=1;
    ostringstream NameStream;
    NameStream<<WriteNameBase<<NameCount;
    outputVideo.open(NameStream.str(), VIDEOWRITECODE, 25, FrameSize, 3);
    
}

void VideoRecord::Update(const Mat& WriteFrame)
{
    if(OnWriting)
    {
        outputVideo<<WriteFrame;
    }
}



