//
//  CaptureFrm.cpp
//  GelSight1
//
//  Created by Wenzhen on 10/4/15.
//  Copyright (c) 2015 MIT Persci Lab. All rights reserved.
//

#include "CaptureFrm.h"

void CaptureFrm::ChangeSoruce()
{
    if(SourceNum>=0)
    {
        SourceNum++;
        if(!Cam.open(SourceNum))
        {
            if(SourceNum==1)
            {
                std::cout<<"No other cameras available"<<std::endl;
            }
            SourceNum=0;Cam.open(SourceNum);
        }
       getiniFrm();setBorder(borderX,borderY);
        Frame=Mat(sizeY,sizeX, CV_8UC3);
    }
}


void FrameConvert::getBackGround(const CaptureFrm& input)
{
    BackGround=Mat(input.sizeY, input.sizeX, CV_8UC3);
    Mat BlurredFrm;
    GaussianBlur(input.FrameO, BlurredFrm, Size(KernelSize*2+1, KernelSize*2+1), KernelSize);

	uchar* pt0 = input.FrameO.data;
	uchar* pt = BlurredFrm.data;
	int size = input.FrameO.rows*input.FrameO.cols*2;
	for (int i = 0; i < size; i++)
	{
		if (*pt> *pt0 + 10)	//black
			*pt = round(*pt*1.1);
		else
			*pt = round(*pt*0.15 + *pt0*0.85);
		pt++; pt0++;
	}
    BlurredFrm(input.RangeY, input.RangeX).copyTo(BackGround);
    
    ini();
    FrameUpdate(input);
}

void FrameConvert::FrameUpdate(const CaptureFrm& input)
{
    input.Frame.copyTo(ShowIm);
    cvtColor(BackGround-ShowIm, GrayImg, CV_RGB2GRAY);}

void FrameConvert::ini()
{
    MinImg=Mat(BackGround.size(),CV_32FC1);
    GrayImg=Mat(BackGround.size(),CV_8UC1);
    ShowIm=Mat(BackGround.size(),CV_8UC3);
    
    chopsNum=15;
    steps=round(BackGround.cols/chopsNum);
    touchTresh=-8;
    
    ///////FOR TEST
    DiffR=Mat(BackGround.size(),CV_8UC1);
    DiffG=Mat(BackGround.size(),CV_8UC1);
    DiffB=Mat(BackGround.size(),CV_8UC1);
    DiffMax=Mat(BackGround.size(),CV_8UC1);
}


void FrameConvert::detectContact()
{
    InTouch=1;return;  // Add this line to close touch detection
    
    getMinImg();
    InTouch=0;
    int i,i1,j,j1;
    Scalar temp;
    for(i=0;i<MinImg.cols-1;i=i1+1)
    {
        i1=min(i+steps, MinImg.cols);
        for(j=0;j<MinImg.rows-1;j=j1+1)
        {
            j1=min(j+steps, MinImg.rows);
            temp=mean(MinImg(Range(j,j1),Range(i,i1)));
            if(temp[0]<touchTresh)
            {
                InTouch=1;return;
            }
        }
    }
}

void FrameConvert::calcBImg()
{   
    std::cout << "here" << std::endl;
    BImg = ShowIm;

    for(int i=0;i<BImg.cols;i+=1)
    {
        for(int j=0;j<BImg.rows;j+=1)
        {
            Vec3f color = BImg.at<Vec3f>(i, j);
            Vec3f bg_color = BackGround.at<Vec3f>(i, j);
            for (int k=0; k<3; k++) color[k] /= bg_color[k];
            // sort in descending order with bubblesort
            for (int k=1; k>=0; k--){
                float lesser = fmin(color[k], color[k+1]);
                float greater = fmax(color[k], color[k+1]);
                color[k] = greater; color[k+1] = lesser;
            }
            color[0] = color[0]*4.4 + color[1]*2.2 + color[2]*0.4;
            BImg.at<Vec3f>(i, j) = color;
            //std::cout << i << ", " << j << std::endl;
        }
    }
    std::cout << "ret" << std::endl;
}

void FrameConvert::getMinImg()
{
    uchar* ptOri2=BackGround.data;
    uchar* ptOri1=ShowIm.data;
    float* ptMinIm=(float*)(MinImg.data);
    int size=MinImg.rows*MinImg.cols;
    for(int i=0;i<size;i++)
    {
        *ptMinIm=min(min(ptOri2[0]-ptOri1[0],ptOri2[1]-ptOri1[1]),ptOri2[2]-ptOri1[2]);
     //   *ptMinIm=min(min(ptOri1[0],ptOri1[1]),ptOri1[2]);
   //     if(*ptMinIm<-20) *ptMinIm=-20;
        ptMinIm++;ptOri1+=3;ptOri2+=3;
    }
}

void FrameConvert::TestDiffImg()
{
    uchar* ptOri2=BackGround.data;
    uchar* ptOri1=ShowIm.data;
    uchar* pt1=DiffB.data;
    uchar* pt2=DiffG.data;
    uchar* pt3=DiffR.data;
    uchar* pt4=DiffMax.data;
    int size=MinImg.rows*MinImg.cols;
 /*   double base=30;
    double timecofR=2.2;
    double timecofB=2.4;
    double timecofG=2.5;
	*/
	double base = 12;
	double timecofR = 1.1;
	double timecofB = 1.3;
	double timecofG = 1.25;
	double t;
    for(int i=0;i<size;i++)
    {
        if(-ptOri2[0]+ptOri1[0]<-5)        //Marker area
        {
            *pt1=base;*pt2=base;*pt3=base;*pt4=base;
        }
        else
        {
			t = MIN(base + timecofB*(-ptOri2[0] + ptOri1[0]), 128); if (t < 0)t = 0;
			*pt1=t;
			t = MIN(base + timecofG*(-ptOri2[1] + ptOri1[1]), 128); if (t < 0)t = 0;
			*pt2 = t;
            t= MIN (base+timecofR*(-ptOri2[2]+ptOri1[2]),128);
			if (t < 0)t = 0; *pt3 = t;
            //*pt4=MAX(*pt1,MAX(*pt2,*pt3));
			t = *pt1 + *pt2 + *pt3 - MIN(*pt1, MIN(*pt2, *pt3));//not calculating the max value, but the sum of the max two values
			t = (t - 10)*1.5; if (t < 0) t = 0; if (t>255)t = 255;
			*pt4 = t;
			//*pt4 = *pt3;
        }
        ptOri1+=3;ptOri2+=3;
        pt1++;pt2++;pt3++;pt4++;
    }
}
