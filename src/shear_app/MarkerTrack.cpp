
//
//  MarkerTrack.cpp
//  GelSight1
//
//  Created by Wenzhen on 10/6/15.
//  Copyright (c) 2015 MIT Persci Lab. All rights reserved.
//

#include "MarkerTrack.h"

using namespace std;
using namespace cv;

void MarkerTrack::DeleteArrays()
{
    delete []MarkerCenter;
    delete []MarkerMoveDis;
    delete []LabelCount;delete []LabelNum;
    delete []LabelMap;
    delete []MarkerLastCenter;
    delete []MarkerIniCenter;
    delete []sumX; delete []sumY;
    delete []PixSum;
    delete []no_seq;
}

MarkerTrack::MarkerTrack()
{
    ParaIni();
}

MarkerTrack::MarkerTrack(const CaptureFrm& input)
{
    ParaIni(input);
}

void MarkerTrack::ReNew(const CaptureFrm &input)
{
    DeleteArrays();ParaIni(input);
}

void MarkerTrack::UpdateMarkerMotion(FrameConvert& input)
{
    if(!input.InTouch)  //not in touch
    {FindIniMarkerCenter(input.GrayImg);return;}
 //   input.MinImg.convertTo(input.GrayImg, CV_8UC1);
    UpdateMarkerMotion(input.GrayImg);
}

void MarkerTrack::UpdateMarkerMotion(const Mat& Frame)
{
    MotionCaled=0;
   if(!isMarkerInied)
    {
        FindIniMarkerCenter(Frame);return;
    }
    FindMarkerCenter(Frame);
    if(MarkerAvailable)
    {
        CalMarkerMovement();
        MotionCaled=1;
    }
}

void MarkerTrack::DisplayParaIni()
{
    DisplayColor=Scalar(0,255,255); //Display color: Yellow
    DisplayThick=3;
    QuiverScale=8;
}

void MarkerTrack::FindMarkerCenter(const Mat& Frame)
// the input Frame should be the grayImg in the FrameConvert class
{
    MarkerAvailable=1;
    int count=LabelConnection(Frame);
    FindCurrentCenterFromLablemap(count);
}

void MarkerTrack::FindIniMarkerCenter(const Mat& Frame)
{
    MarkerAvailable=1;
    int count=LabelConnection(Frame);
    int i;
    FindCurrentCenterFromLablemap(count);
    if(MarkerAvailable)
    {
        isMarkerInied=1;
        MotionCaled=1;
        MarkerIniCount=MarkerCount;
        for(i=0;i<MarkerCount*2;i++)
        {
            MarkerIniCenter[i]=MarkerCenter[i];
            MarkerLastCenter[i]=MarkerCenter[i];
            MarkerMoveDis[i]=0;
        }
        for(;i<MarkerCount*3;i++)
        {
            MarkerIniCenter[i]=MarkerCenter[i];
            MarkerLastCenter[i]=MarkerCenter[i];
        }
    }
}

void MarkerTrack::ParaIni()
{
    MarkerThresh=30;
    MarkerCount=0;
    MarkerAreaThresh=100;
    isMarkerInied=0;
    MinMarkerNum=15;
    DisplayParaIni();
}

void MarkerTrack::ParaIni(const CaptureFrm& input)
{
    ParaIni();
    sizex=input.sizeX;sizey=input.sizeY;
    int sumsize=sizex*sizey;
    maxarea=sumsize/100;
    LabelMap=new int[sumsize];
    LabelCount=new int[maxarea];
    LabelNum=new int[maxarea];
    MarkerCenter=new double[maxarea*3];
    MarkerLastCenter=new double[maxarea*3];
    MarkerMoveDis=new double[maxarea*3];
    sumX=new int[maxarea];
    sumY=new int[maxarea];
    PixSum=new double[maxarea];
    no_seq=new int[maxarea];
	MarkerIniCenter = new double[maxarea];
    maxarea--;
}

void MarkerTrack::QuiverDisplayField(Mat& Frame)
{
    if(!MotionCaled) return;
    double* ptCenter=MarkerIniCenter;
    double* ptMove=MarkerMoveDis;
    Point pt1,pt2;
    
    for(int i=0;i<MarkerIniCount;i++)
    {
        pt1=Point(ptCenter[0], ptCenter[1]);
        ptCenter+=3;
        pt2=pt1+Point(ptMove[0],ptMove[1])*QuiverScale;
        ptMove+=2;
        line(Frame,pt1, pt2, DisplayColor, DisplayThick);
    }
}

void MarkerTrack::CalMarkerMovement()
{
    if(!MarkerAvailable) return;
    
    double* ptCurMarker=MarkerCenter;
    double* ptLstMarker;
    int i,j;
    double tempDis, minDis;int minInd;
    for(i=0;i<MarkerCount;i++)
    {
        minDis=1e9;minInd=0;
        ptLstMarker=MarkerLastCenter;
        for(j=0;j<MarkerIniCount;j++)
        {
            tempDis=MarkerCenterDisDefine(ptCurMarker, ptLstMarker);
            ptLstMarker+=3;
            if(tempDis<minDis)
            {   minDis=tempDis;minInd=j;
            }
        }
        no_seq[i]=minInd;
        ptCurMarker+=3;
    }
    
    double* ptIniMarker=MarkerIniCenter;
    double* ptMovDis=MarkerMoveDis;
    ptLstMarker=MarkerLastCenter;
    int tempInt;
    for(i=0;i<MarkerIniCount;i++)
    {
        minDis=1e9;minInd=0;
        ptCurMarker=MarkerCenter;
        for(j=0;j<MarkerCount;j++)
        {
            tempDis=MarkerCenterDisDefine(ptCurMarker, ptLstMarker);
            ptCurMarker+=3;
            if(tempDis<minDis)
            {   minDis=tempDis;minInd=j;
            }
        }
        
        minDis/=100;
        if(ptLstMarker[2]<minDis)   //for small area
        {
            *ptMovDis=0;ptMovDis++;*ptMovDis=0;ptMovDis++;
            *ptLstMarker=*ptIniMarker;ptLstMarker++;ptIniMarker++;
            *ptLstMarker=*ptIniMarker;ptLstMarker++;ptIniMarker++;
            *ptLstMarker=*ptIniMarker;ptLstMarker++;ptIniMarker++;
        }
        else if(i==no_seq[minInd])  //Match
        {
            tempInt=minInd*3;
            *ptMovDis=MarkerCenter[tempInt]-*ptIniMarker;ptMovDis++;ptIniMarker++;
            *ptLstMarker=MarkerCenter[tempInt];ptLstMarker++;
            *ptMovDis=MarkerCenter[tempInt+1]-*ptIniMarker;ptMovDis++;ptIniMarker++;
            *ptLstMarker=MarkerCenter[tempInt+1];ptLstMarker++;
            *ptLstMarker=MarkerCenter[tempInt+2];ptLstMarker++;
            ptIniMarker++;
        }
        else
        {
            *ptMovDis=0;ptMovDis++;*ptMovDis=0;ptMovDis++;
            *ptLstMarker=*ptIniMarker;ptLstMarker++;ptIniMarker++;
            *ptLstMarker=*ptIniMarker;ptLstMarker++;ptIniMarker++;
            *ptLstMarker=*ptIniMarker;ptLstMarker++;ptIniMarker++;
        }
    }
}


int MarkerTrack::LabelConnection(const Mat& Frame)  //find the connection map; return the max number to serach
{
    int count=0;
    int isizex,isizex1;
    int t,t1;
    for(int i=0;i<sizey;i++)
    {
        isizex=i*sizex;isizex1=isizex-sizex;
        for(int j=0;j<sizex;j++)
        {
            LabelMap[isizex+j]=0;
            if(Frame.at<uchar>(i,j)>MarkerThresh)
            {
                if(i>0 && LabelMap[isizex1+j])
                {
                    t=LabelMap[isizex1+j];
                    if(j>0 && LabelMap[isizex+j-1])
                    {
                        t1=LabelMap[isizex+j-1];
                        if(LabelNum[t1]<LabelNum[t])
                        {
                            LabelNum[t]=LabelNum[t1];
                            t=LabelNum[t1];
                        }
                        else if(LabelNum[t1]>LabelNum[t])
                        {
                            LabelNum[t1]=LabelNum[t];
                        }
                    }
                    LabelMap[isizex+j]=t;
                    LabelCount[t]++;
                }
                else if(j>0 && LabelMap[isizex+j-1])
                {
                    t=LabelMap[isizex+j-1];
                    LabelCount[t]++;
                    LabelMap[isizex+j]=t;
                }
                else    //no connection, build new area
                {
                    count++;
                    LabelMap[isizex+j]=count;
                    LabelCount[count]=1;LabelNum[count]=count;
                }
                if(count>=maxarea)
                {
                    MarkerAvailable=0;return -1;
                }
            }//if marker pix
        }
    }//marker the connections
    if(count<MinMarkerNum)
    {
        MarkerAvailable=0;return -1;
    }
//    int countres=0;
    for(int i=1;i<=count;i++)
    {
        if(LabelNum[i]!=i)
        {
            LabelNum[i]=LabelNum[LabelNum[i]];
            LabelCount[LabelNum[i]]+=LabelCount[i];
        }
        else
        {
            sumX[i]=0;sumY[i]=0;
            PixSum[i]=0;
        }
    }
    for(int i=0;i<sizey;i++)
    {
        isizex=i*sizex;
        for(int j=0;j<sizex;j++) if(LabelMap[isizex+j])
        {
            t=LabelMap[isizex+j];t1=LabelNum[t];
            if(!t1 || !LabelNum[t1])
            {
                LabelMap[isizex+j]=0;continue;
            }
            if(LabelCount[t1]<MarkerAreaThresh)
            {
                LabelMap[isizex+j]=0;
                LabelNum[t]=0;LabelNum[t1]=0;
            }
            else //effective;
            {
                LabelMap[isizex+j]=t1;
                t=Frame.at<uchar>(i,j);
                PixSum[t1]+=t;
                sumX[t1]+=j*t;
                sumY[t1]+=i*t;
            }
        }
    }
    
    return count;
}

void MarkerTrack::FindCurrentCenterFromLablemap(int count)
{
    if(!MarkerAvailable) return;
    MarkerCount=0;double* ptCenter=MarkerCenter;
    for(int i=1;i<=count;i++) if(LabelNum[i]==i)
    {
        *ptCenter=double(sumX[i])/PixSum[i];ptCenter++;
        *ptCenter=double(sumY[i])/PixSum[i];ptCenter++;
        *ptCenter=LabelCount[i];ptCenter++;
        MarkerCount++;
    }
    if(MarkerCount<MinMarkerNum) MarkerAvailable=0;
}

double MarkerTrack::MarkerCenterDisDefine(double* a, double*b)
{
    return (abs(a[0]-b[0])+abs(a[1]-b[1]))*(100+abs(a[2]-b[2]));
}