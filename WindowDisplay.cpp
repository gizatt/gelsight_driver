
//
//  WindowDisplay.cpp
//  GelSight1
//
//  Created by Wenzhen on 10/8/15.
//  Copyright (c) 2015 MIT Persci Lab. All rights reserved.
//

#include "WindowDisplay.h"

void WindowDisplay::ini()
{
    namedWindow(WindowName, WINDOW_AUTOSIZE);
    saveNum=0;
    saveName="Save";
    setMouseCallback( WindowName, WindowDisplay::onMouse, (void*)this );
    
    ShowImgNumber=1;
    DisplaySizeX=1200; DisplaySizeY=700;
    UIDisplay2Im=0;
    DisplayImType=0;
    ReturnStatus=0;

	// about writing the video
	OnRec = 0;
	RecNum = 0;
    
    }

void WindowDisplay::ini_DisplayMat()
{
    if(UIDisplay2Im)    //if display UI
    {
        ini_DisplayMat_UI2Im();
    }
    else if(ShowImgNumber==3 || ShowImgNumber==4)
    {
        if(sizex0<50 || sizex0>1e4)
        {
            sizex0=1276; sizey0=750;
        }
        DisplayImage=Mat(sizey0*2+100, sizex0*2+100, CV_8UC3);
        double a,b;
        a=double(DisplaySizeX)/double(DisplayImage.cols);b=double(DisplaySizeY)/double(DisplayImage.rows);
        if(a<b) DisplaySizeX=round(DisplayImage.cols*b);
        else DisplaySizeY=round(DisplayImage.rows*a);
        DisplayImage2=Mat(DisplaySizeY, DisplaySizeX, CV_8UC3);
    }
    else if(ShowImgNumber==2)
    {
        if(sizex0<50 || sizex0>1e4)
        {
            sizex0=1276; sizey0=750;
        }
        DisplayImage=Mat(sizey0+100, sizex0*2+100, CV_8UC3);
        double a=double(DisplaySizeX)/double(DisplayImage.cols);
        DisplaySizeY=round(DisplayImage.rows*a);
        DisplayImage2=Mat(DisplaySizeY, DisplaySizeX, CV_8UC3);
    }

}

void WindowDisplay::Display_UI2Im_renew(const CaptureFrm& scr)
{
    ShowImgNumber=1;
    DisplaySizeX=1200; DisplaySizeY=700;
    sizex0=scr.sizeX; sizey0=scr.sizeY;
    objRecorder->FrameSize=Size(sizex0, sizey0);
    ini_DisplayMat_UI2Im();
}

void WindowDisplay::ini_DisplayMat_UI2Im()
{
    ShowImgNumber=2;
    if(!DisplayImType) DisplayImType=_WINDOWDISPLAY_CURVEIM_;
    
    if(sizex0<50 || sizex0>1e4)
    {
        sizex0=1276; sizey0=750;
    }
    DisplayWindowSizeX=(DisplaySizeX-30)/2;
    double a=double(DisplayWindowSizeX)/double(sizex0);
    DisplayWindowSizeY=round(sizey0*a);
    DisplaySizeY=round(sizey0*a)+90;
    DisplayImage2=Mat(DisplaySizeY, DisplaySizeX, CV_8UC3, Scalar(0,0,0));
    
    // --------Add buttons-------------
    SubWindowCount=6;
    subObjPos=new SubWindowRange[SubWindowCount];
    
    Scalar FontColor(255,255,255);
    Scalar BottonColor(60,60,60);
    int x1, x2, y1,y2;
    
    x1=50;y1=DisplaySizeY-50;y2=DisplaySizeY-25;x2=x1+110;
    DisplayImage2(Range(y1-5, y2+12), Range(x1-10,x2+10))=BottonColor;
    //rectangle(DisplayImage2, Point(x1-10, y2+12), Point(x2+10,y1-8),BottonColor);
    putText(DisplayImage2, "Start Rec", Point(x1,y2),FONT_HERSHEY_SIMPLEX, 0.7, FontColor);
    subObjPos[0].set(x1, y1, x2, y2, _SENDBACK_STARTREC_);
    
    x1=x2+60;x2=x1+110;
    DisplayImage2(Range(y1-5, y2+12), Range(x1-10,x2+10))=BottonColor;
    putText(DisplayImage2, "Stop Rec", Point(x1,y2),FONT_HERSHEY_SIMPLEX, 0.7, FontColor);
    subObjPos[1].set(x1, y1, x2, y2, _SENDBACK_ENDREC_);
    
    x1=x2+60;x2=x1+140;
    DisplayImage2(Range(y1-5, y2+12), Range(x1-10,x2+10))=BottonColor;
    putText(DisplayImage2, "Save Image", Point(x1,y2),FONT_HERSHEY_SIMPLEX, 0.7, FontColor);
    subObjPos[2].set(x1, y1, x2, y2, _SENDBACK_SAVEIM_);
    
    x1=x2+60;x2=x1+170;
    DisplayImage2(Range(y1-5, y2+12), Range(x1-10,x2+10))=BottonColor;
    putText(DisplayImage2, "Change Source", Point(x1,y2),FONT_HERSHEY_SIMPLEX, 0.7, FontColor);
    subObjPos[3].set(x1, y1, x2, y2, _SENDBACK_CHANGESOURCE_);
    
    x1=x2+60;x2=x1+130;
    DisplayImage2(Range(y1-5, y2+12), Range(x1-10,x2+10))=BottonColor;
    putText(DisplayImage2, "Initialization", Point(x1,y2),FONT_HERSHEY_SIMPLEX, 0.7, FontColor);
    subObjPos[4].set(x1, y1, x2, y2, _SENDBACK_RENEW_);
    
    x1=x2+60;x2=x1+100;
    DisplayImage2(Range(y1-5, y2+12), Range(x1-10,x2+10))=BottonColor;
    putText(DisplayImage2, "Exit", Point(x1,y2),FONT_HERSHEY_SIMPLEX, 0.7, FontColor);
    subObjPos[5].set(x1, y1, x2, y2, _SENDBACK_EXITPROG_);
}

void WindowDisplay::Display(int nArgs, ...)
{
    va_list args;
    va_start(args, nArgs);
    Mat* Im1;
    Im1=va_arg(args, Mat*);
    if(nArgs!=ShowImgNumber)
    {
        ShowImgNumber=nArgs;ini_DisplayMat();
    }
    if(nArgs==1)
    {
        DisplayImage=*Im1;
   //     resize(DisplayImage, DisplayImage2,  Size(DisplaySizeX,DisplaySizeY));
    }
    if(nArgs==3 || nArgs==4)
    {
        int t1,t2;
        t1=Im1->rows+25; t2=Im1->cols+25;
        Im1->copyTo(DisplayImage(Range(25,Im1->rows+25),Range(25,Im1->cols+25)));
        Im1=va_arg(args, Mat*);
        Im1->copyTo(DisplayImage(Range(25,Im1->rows+25),Range(25+t2,Im1->cols+25+t2)));
        Im1=va_arg(args, Mat*);
        Im1->copyTo(DisplayImage(Range(25+t1,t1+Im1->rows+25),Range(25,Im1->cols+25)));
        if(nArgs==4)
        {
            Im1=va_arg(args, Mat*);
            Im1->copyTo(DisplayImage(Range(25+t1,t1+Im1->rows+25),Range(25+t2,t2+Im1->cols+25)));
        }
    }
    if(nArgs==2)
    {
        int t1=Im1->rows+25;
        int t2=Im1->cols+25;
        Im1->copyTo(DisplayImage(Range(25,t1),Range(25,t2)));
        Im1=va_arg(args, Mat*);
        Im1->copyTo(DisplayImage(Range(25,t1),Range(25+t2,t2*2)));
    }
    
 //   resize(DisplayImage, DisplayImage2, Size(DisplaySizeX,DisplaySizeY));
   imshow(WindowName,DisplayImage2);
   returnKey=(char)waitKey(10);
    Interact();
}

void WindowDisplay::Display_UI2Im(const FrameConvert& scr)
{
    if(!UIDisplay2Im) return;
    resize(scr.ShowIm, DisplayImage2(Range(10, DisplayWindowSizeY+10), Range(10, DisplayWindowSizeX+10)), Size(DisplayWindowSizeX,DisplayWindowSizeY));
    
    Mat DM;
    if(DisplayImType==_WINDOWDISPLAY_CURVEIM_)
    {
        cvtColor(scr.DiffMax,DM,CV_GRAY2BGR);
    }
    resize(DM, DisplayImage2(Range(10, DisplayWindowSizeY+10), Range(DisplayWindowSizeX+20, DisplayWindowSizeX*2+20)), Size(DisplayWindowSizeX,DisplayWindowSizeY));
    
    
    imshow(WindowName,DisplayImage2);
//    imshow(WindowName, scr.ShowIm);
    objRecorder->Update(DM);
    if(ReturnStatus==_SENDBACK_SAVEIM_)        //save current image
    {
        ostringstream SaveNameStream;
        saveNum++;
        SaveNameStream<<saveName<<saveNum<<".png";
        imwrite(SaveNameStream.str(),DM);
        ReturnStatus=0;
		ostringstream SaveNameStream2;
		SaveNameStream2 << saveName << saveNum <<"_ori"<< ".png";
		imwrite(SaveNameStream2.str(), scr.ShowIm);
		cout << "Current frame saved" << endl;
    }
	//write in video
	if (OnRec)
	{
		objVideoWriter << DisplayImage2;
	}



    returnKey=(char)waitKey(10);
    Interact();
}

void WindowDisplay::onMouse(int event, int x, int y, int, void* obj)
{
    if( event != EVENT_LBUTTONDOWN )
        return;
    ((WindowDisplay*)obj)->ButtonPress(x,y);
}

void WindowDisplay::Interact()
{
    if(returnKey==' ' || returnKey=='q')
        ReturnStatus=_SENDBACK_EXITPROG_;
}

void WindowDisplay::ButtonPress(int x, int y)
{
//    ReturnStatus=0;
    if(UIDisplay2Im)
    {
        for(int i=0;i<SubWindowCount;i++)
            if(subObjPos[i].Within(x, y))
            {
                ReturnStatus=subObjPos[i].WindowLabel;
           //     cout<<ReturnStatus<<endl;
                break;
            }
        
        if(ReturnStatus==_SENDBACK_STARTREC_)
        {
            objRecorder->StartWriting();ReturnStatus=0;

			// writing video using videowriter
			OnRec = 1;
			RecNum++;
			ostringstream SaveNameStream;
			SaveNameStream << "Rec" << RecNum<<".avi";
			//objVideoWriter.open(SaveNameStream.str(), -1, 25, Size(DisplaySizeX, DisplaySizeY), true);
			objVideoWriter.open(SaveNameStream.str(), CV_FOURCC('M','J', 'P', 'G'), 25, Size(DisplaySizeX, DisplaySizeY));
			if (objVideoWriter.isOpened())
			{
				cout << "Start Writing Video " << RecNum << endl;
			}
			else
			{
				OnRec = 0; RecNum--;
				cout << "Fails to open writing video" << endl;
			}

        }
        else if (ReturnStatus==_SENDBACK_ENDREC_)
        {
            objRecorder->EndWriting();ReturnStatus=0;

			if (OnRec)
				cout << "Stop writing Videos" << endl;
			OnRec = 0;
        }
    }
    if(!ReturnStatus)
    {
        Vec3b Pix;
        Pix=DisplayImage2.at<Vec3b>(y,x);
        cout<<Pix<<endl;
    }    
}