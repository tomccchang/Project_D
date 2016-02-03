// StereoVision.cpp: 主要專案檔。

#include "stdafx.h"

using namespace System;

//OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <highgui.h>

//#include "segmentation.h"
#include "Depth.h"

#include <time.h>

//#include <windows.h>
//#include <dshow.h>

//#include <videoInput.h>
int Job1()
{
	//CvCapture* captureR = cvCreateCameraCapture( 1);
	//CvCapture* captureL = cvCreateCameraCapture( 2 );
	//CvCapture* captureR = cvCreateCameraCapture(CV_CAP_DSHOW);
    //CvCapture* captureL = cvCreateCameraCapture(CV_CAP_DSHOW + 2);
	//videoInput VI;  
	//int numDevices = VI.listDevices();   

    CvCapture* captureL =cvCaptureFromCAM(1); 
	cvWaitKey(100000);
	CvCapture* captureR =cvCaptureFromCAM(0);
	cvWaitKey(100000);

	//Camera Setting
	int w = 320, h = 240;
	cvSetCaptureProperty ( captureL, CV_CAP_PROP_FRAME_WIDTH,  w );  
	cvSetCaptureProperty ( captureL, CV_CAP_PROP_FRAME_HEIGHT, h );
	cvSetCaptureProperty ( captureR, CV_CAP_PROP_FRAME_WIDTH,  w );  
	cvSetCaptureProperty ( captureR, CV_CAP_PROP_FRAME_HEIGHT, h );

	cvNamedWindow( "Camera_L", CV_WINDOW_AUTOSIZE );
	cvNamedWindow( "Camera_R", CV_WINDOW_AUTOSIZE );
	cvNamedWindow( "Segmentation", CV_WINDOW_AUTOSIZE );
	cvNamedWindow( "Disparity", CV_WINDOW_AUTOSIZE );

	//Image buffer
	IplImage *imgL_O, *imgL;
	IplImage *imgR_O, *imgR;

	//segmentation setting
	FastSegmentation Z;
	Z.setting(0,h ,w, 1, 13, 13, 1,6);
	//Depth Setting
	Depth Y;
	Y.setting(h, w, 1, 13, 13, 1, 1, 0);
	Y.ROI(80,150,50,300);
	Y.SMask=Z.FilterMap;

	//debug
	IplImage *Zmap = cvCreateImage(cvSize(Z.width_R,Z.height_R),8,3);
	IplImage *Dmap = cvCreateImage(cvSize(Y.width_R,Y.height_R),8,1);
	
	while(true)
	{
		if( !(imgL= cvQueryFrame(captureL)) ) 
		{
			//printf("\n #Error(Panel_Display):READ_IMAGE_FAIL L"); 
			//getchar();
			//exit(-1);
			continue;
		}

		if( !(imgR= cvQueryFrame(captureR)) ) 
		{
			//printf("\n #Error(Panel_Display):READ_IMAGE_FAIL R"); 
			//getchar();
			//exit(-1);
			continue;
		}
		unsigned char* srcL=(unsigned char* )imgL->imageData;
		unsigned char* srcR=(unsigned char* )imgR->imageData;

		//Z.setting(srcL,h ,w, 1, 13, 13, 1,6);
		Z.Img=srcL;
		Z.filtering();
		Z.clustering2();

		Y.SMask=Z.FilterMap;
		Y.clusters=Z.clusters;
	    Y.ClusterNum=Z.ClusterNum;
        Y.update(srcL,srcR);

		cvShowImage( "Camera_L", imgL );
		cvShowImage( "Camera_R", imgR );
		Zmap->imageData=(char*)Z.FilterMap;
		cvShowImage( "Segmentation", Zmap );
		Dmap->imageData=(char*)Y.DisparityMap;
		cvShowImage( "Disparity", Dmap );

		int key = cvWaitKey(30);
		if( key == 27 ){
			cvSaveImage("L.bmp",imgL, 0);
			cvSaveImage("R.bmp",imgR, 0);
			break;
		}
	}

	cvReleaseCapture( &captureL );
	cvReleaseCapture( &captureR );
	cvDestroyWindow( "Camera_L" );
	cvDestroyWindow( "Camera_R" );

	return 0;
}



int main(array<System::String ^> ^args)
{
	Job1();
	return 0;



	clock_t t1,t2,ta,tb;
	double time;

	IplImage *imgL_O, *imgL;
	IplImage *imgR_O, *imgR;
	//開啟影像內容
	if( !(imgL_O= cvLoadImage("L.bmp", 1)) ) 
	{
		printf("\n #Error(Panel_Display):READ_IMAGE_FAIL1"); 
		getchar();
		exit(-1);
	}
	imgL= cvCreateImage(cvSize(320,240),8,3);
	cvResize(imgL_O,imgL,CV_INTER_LINEAR);//歷史內插

	if( !(imgR_O= cvLoadImage("R.bmp", 1)) ) 
	{
		printf("\n #Error(Panel_Display):READ_IMAGE_FAIL1"); 
		getchar();
		exit(-1);
	}
	imgR= cvCreateImage(cvSize(320,240),8,3);
	cvResize(imgR_O,imgR,CV_INTER_LINEAR);//歷史內插

	IplImage *imgL_=cvCloneImage(imgL);
	unsigned char* srcL=(unsigned char* )imgL->imageData;
	unsigned char* srcL_=(unsigned char* )imgL_->imageData;
	IplImage *imgR_=cvCloneImage(imgR);
	unsigned char* srcR=(unsigned char* )imgR->imageData;
	unsigned char* srcR_=(unsigned char* )imgR_->imageData;

	FastSegmentation Z;
	Z.setting(srcL,imgL->height ,imgL->width, 1, 13, 13, 1,7);
	ta=clock();
	Z.filtering();
	Z.clustering2();
	tb=clock();
	time=double((tb-ta)/CLOCKS_PER_SEC);
	printf("\n time:%f",time);

/*
	IplImage *Zmap = cvCreateImage(cvSize(Z.width_R,Z.height_R),8,3);
	Zmap->imageData=(char*)Z.FilterMap;

	for(int i=0; i<Z.ClusterNum;i++)//Z.ClusterNum
	cvRectangle(Zmap,cvPoint(
		Z.clusters[i].j_start,Z.clusters[i].i_start),cvPoint(Z.clusters[i].j_end,Z.clusters[i].i_end)
		,CV_RGB(255,255,0),1,CV_AA,0);

	cvSaveImage("Zmap.bmp",Zmap, 0);
	cvNamedWindow("Zmap",0);
	cvShowImage("Zmap",Zmap);
	cvNamedWindow("1",0);
	cvShowImage("1",imgL);
	cvNamedWindow("2",0);
	cvShowImage("2",imgR);
	IplImage *clustermap = cvCreateImage(cvSize(Z.width_R,Z.height_R),8,3);
	clustermap->imageData=(char*)Z.ClusterMap;
	cvNamedWindow("3",0);
	cvShowImage("3",clustermap);
	cvWaitKey(0);
	//return 0;
*/

/*
	MeanShiftSegmentation X;
	X.setting(imgL->height ,imgL->width, 1, 13, 13, 1, true);
	ta=clock();
	X.update(srcL);
	tb=clock();
	time=double((tb-ta)/CLOCKS_PER_SEC);
	printf("\n time:%f",time);
	IplImage *map = cvCreateImage(cvSize(X.width_R,X.height_R),8,3);
	//IplImage *map_ = cvCreateImage(cvSize(X.width,X.height),8,3);
	map->imageData=(char*)X.FilterMap;
	cvSaveImage("temp_Seg.bmp",map, 0);
*/


	Depth Y;
	Y.ImgL_=srcL_;//debug
	Y.ImgR_=srcR_;//debug
	Y.setting(imgL->height ,imgL->width, 1, 13, 43, 1, 1, 0);
/*
	//另外輸入分割區
	IplImage *imgL_S;
	if( !(imgL_S= cvLoadImage("testL_Seg.bmp", 1)) ) 
	{
		printf("\n #Error(Panel_Display):READ_IMAGE_FAIL1"); 
		getchar();
		exit(-1);
	}
	Y.SMask=(unsigned char* )imgL_S->imageData;
	IplImage *imgL_S_=cvCloneImage(imgL_S);
	Y.SMask_=(unsigned char* )imgL_S_->imageData;
*/

	//Y.ROI(100,150,50,300);
	Y.SMask=Z.FilterMap;
	Y.clusters=Z.clusters;
	Y.ClusterNum=Z.ClusterNum;

	t1=clock();
	Y.update(srcL,srcR);
	t2=clock();
	time=double((t2-t1)/CLOCKS_PER_SEC);
	printf("\n time:%f",time);

	IplImage *Dmap = cvCreateImage(cvSize(Y.width_R,Y.height_R),8,1);
	Dmap->imageData=(char*)Y.DisparityMap;
	IplImage *DmapN = cvCreateImage(cvSize(Y.width_R,Y.height_R),8,1);
	cvConvertScale(Dmap,DmapN,2,0);
	IplImage *SegMap = cvCreateImage(cvSize(Y.width_R,Y.height_R),8,3);
	SegMap->imageData=(char*)Z.FilterMap;

	cvRectangle(imgL_,cvPoint(Y.Ystart,Y.Xstart),cvPoint(Y.Yend,Y.Xend),CV_RGB(255,255,0),1,CV_AA,0);
	cvRectangle(imgR_,cvPoint(Y.Ystart,Y.Xstart),cvPoint(Y.Yend,Y.Xend),CV_RGB(255,255,0),1,CV_AA,0);

	cvNamedWindow("1",0);
	cvShowImage("1",imgL_);
	cvNamedWindow("2",0);
	cvShowImage("2",imgR_);

	cvNamedWindow("3",CV_WINDOW_NORMAL);
	cvShowImage("3",Dmap);
	cvNamedWindow("4",CV_WINDOW_NORMAL);
	cvShowImage("4",DmapN);
	cvNamedWindow("5",CV_WINDOW_NORMAL);
	cvShowImage("5",SegMap);

	cvWaitKey(0);
	//cvSaveImage("result_s.bmp",img_, 0);


	return 0;
}
