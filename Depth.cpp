#include "stdafx.h"

#include "Depth.h"
#include <math.h>
#define   ROUND(X)     (int)(X+0.5);
#include <stdio.h>

//OPENCV//debug
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <highgui.h>


int Depth::setting(int height ,int width, int ImgSamplePeriod,
	int WindowH, int WindowW, int WindowSamplePeriod, int d_SamplePeriod, unsigned char* SMask)
{
	//image setting
	this->height=height;
	this->width=width;
	this->ImgSamplePeriod=ImgSamplePeriod;
	this->height_R=int(height/ImgSamplePeriod);
	this->width_R=int(width/ImgSamplePeriod);
	this->step=3*width;
	this->patch=3;

	this->Xstart=0;
	this->Xend=239;
	this->Ystart=40;
	this->Yend=279;

	this->d_SamplePeriod=d_SamplePeriod;
	//window setting
	this->WindowH=WindowH;
	this->WindowW=WindowW;
	this->WindowSamplePeriod=WindowSamplePeriod; //(WindowH-1)%WindowSamplePeriod=0
	this->WindowH_R=int(WindowH/WindowSamplePeriod);
	this->WindowW_R=int(WindowW/WindowSamplePeriod);
	this->WindwoL=new bool[this->WindowH_R*this->WindowW_R*this->patch];
	this->WindwoR=new bool[this->WindowH_R*this->WindowW_R*this->patch];
	this->WindowStep=this->WindowW_R*this->patch;

	//this->Is_SMask=Is_SMask;
	if(SMask==0){
		this->Is_SMask=0;
	}
	else{
		this->Is_SMask=1;
		this->SMask=SMask;
	}

	//disparity
	int d_SamplingPtsCenter=int(d_SamplingPtsNum/2);
	d_SamplingPts[d_SamplingPtsCenter]=0;
	int A;
	for(int i=1;i<=d_SamplingPtsCenter;i++){

		A=d_SamplingPts[d_SamplingPtsCenter+(i-1)];
		d_SamplingPts[d_SamplingPtsCenter+i]=A+i;

		d_SamplingPts[d_SamplingPtsCenter-i]=-d_SamplingPts[d_SamplingPtsCenter+i];
	}

	//buffer
	this->cost_buffer=new int[this->width_R];
	this->d_buffer=new int[this->width_R];
	this->dColor_buffer=new int[this->width_R];
	//output setting
	this->DisparityMap=new unsigned char[this->height_R*this->width_R];//disparity

	return 0;
}

int Depth::cost(int x, int y1,int y2)
{
	int d=y2-y1;
	//Window Range
	int i_start=x-(int)(WindowH/2);//search window最左上角
	int j_start=y1-(int)(WindowW/2);//search window最左上角
	int i_end=i_start+WindowH;
	int j_end=j_start+WindowW;
	//範圍超出邊界處理(僅判斷window，中心位置判斷在上一層blockmatch)
	if(i_start<0)
		i_start=0;

	if(i_end>(height-1))
		i_end=height-1;

	if(j_start<0)
		j_start=0;

	if(j_end>(width-1))
		j_end=width-1;

	if(j_end+d>(width-1))
		j_end=width-1-d;

	//
	int dB,dG,dR;
	double dB_sum=0,dG_sum=0,dR_sum=0;
	int k=0;
	for(int i=i_start;i<=i_end;i+=WindowSamplePeriod){
		for(int j=j_start;j<=j_end;j+=WindowSamplePeriod){
			
			dB=(int)ImgL[step*i+patch*j+0]-(int)ImgR[step*i+patch*(j+d)+0];
			dG=(int)ImgL[step*i+patch*j+1]-(int)ImgR[step*i+patch*(j+d)+1];
			dR=(int)ImgL[step*i+patch*j+2]-(int)ImgR[step*i+patch*(j+d)+2];
			dB=abs(dB);
			dG=abs(dG);
			dR=abs(dR);

			dB_sum+=dB;
			dG_sum+=dG;
			dR_sum+=dR;

			k++;
		}}
	double difference=(dB_sum+dG_sum+dR_sum)/(k);
	//double difference=dG_sum/k;
	return (int)difference;
}

int Depth::costS(int x, int y1,int y2)
{
	int d=y2-y1;
	//Window Range
	int i_start=x-(int)(WindowH/2);//search window最左上角
	int j_start=y1-(int)(WindowW/2);//search window最左上角
	int i_end=i_start+WindowH;
	int j_end=j_start+WindowW;

	//中心
	int CenterBS=(int)SMask[step*x+patch*y1+0];
	int CenterGS=(int)SMask[step*x+patch*y1+1];
	int CenterRS=(int)SMask[step*x+patch*y1+2];
	int BS, GS, RS;
	int range=10;
	int BS_MAX=CenterBS+range;
	int BS_MIN=CenterBS-range;
	int GS_MAX=CenterGS+range;
	int GS_MIN=CenterGS-range;
	int RS_MAX=CenterRS+range;
	int RS_MIN=CenterRS-range;

	//範圍超出邊界處理(僅判斷window，中心位置判斷在上一層blockmatch)
	if(i_start<0)
		i_start=0;

	if(i_end>(height-1))
		i_end=height-1;

	if(j_start<0)
		j_start=0;

	if(j_end>(width-1))
		j_end=width-1;

	if(j_end+d>(width-1))
		j_end=width-1-d;
/*
	//debug
	IplImage *temp3 = cvCreateImage(cvSize(width,height),8,3);
	temp3->imageData=(char*)SMask_;
	IplImage *temp2 = cvCreateImage(cvSize(width,height),8,3);
	temp2->imageData=(char*)ImgR_;
	IplImage *temp6 = cvCreateImage(cvSize(width,height),8,3);
*/
	//比對
	int dB,dG,dR;
	double dBi_sum=0,dGi_sum=0,dRi_sum=0;
	//double dBo_sum=0,dGo_sum=0,dRo_sum=0;
	int ki=0;
	int ko=0;
	for(int i=i_start;i<=i_end;i+=WindowSamplePeriod){
		for(int j=j_start;j<=j_end;j+=WindowSamplePeriod){

			//segmentation結果
			BS=(int)SMask[step*i+patch*j+0];
			GS=(int)SMask[step*i+patch*j+1];
			RS=(int)SMask[step*i+patch*j+2];

			//在分割區內
			if(BS<BS_MAX && BS>BS_MIN){
				if(GS<GS_MAX && GS>GS_MIN){
					if(RS<RS_MAX && RS>RS_MIN){//確認是否在同一分割區
						dB=(int)ImgL[step*i+patch*j+0]-(int)ImgR[step*i+patch*(j+d)+0];
						dG=(int)ImgL[step*i+patch*j+1]-(int)ImgR[step*i+patch*(j+d)+1];
						dR=(int)ImgL[step*i+patch*j+2]-(int)ImgR[step*i+patch*(j+d)+2];
						dB=abs(dB);
						dG=abs(dG);
						dR=abs(dR);

						dBi_sum+=dB;
						dGi_sum+=dG;
						dRi_sum+=dR;

						ki++;

						//debug
						//cvCircle(temp3,cvPoint(j,i),1,CV_RGB(255,255,0),1,CV_AA,0);//debug
						//cvCircle(temp2,cvPoint(j+d,i),1,CV_RGB(dR,dG,dB),1,CV_AA,0);//debug
						//cvCircle(temp6,cvPoint(j,i),1,CV_RGB((int)ImgL[step*i+patch*j+2],(int)ImgL[step*i+patch*j+1],(int)ImgL[step*i+patch*j+0]),1,CV_AA,0);//debug

						continue;
				}}}//IF

            //在分割區外
			dB=(int)ImgL[step*i+patch*j+0]-(int)ImgR[step*i+patch*(j+d)+0];
			dG=(int)ImgL[step*i+patch*j+1]-(int)ImgR[step*i+patch*(j+d)+1];
			dR=(int)ImgL[step*i+patch*j+2]-(int)ImgR[step*i+patch*(j+d)+2];
			dB=int(abs(dB)*0.2);
			dG=int(abs(dG)*0.2);
			dR=int(abs(dR)*0.2);

			dBi_sum+=dB;
			dGi_sum+=dG;
			dRi_sum+=dR;

			ko++;

			//cvCircle(temp2,cvPoint(j+d,i),1,CV_RGB(dR,dG,dB),1,CV_AA,0);//debug
			//cvCircle(temp3,cvPoint(j,i),1,CV_RGB(0,0,200),1,CV_AA,0);//debug
		}}
	//debug
	//cvNamedWindow("A",CV_WINDOW_NORMAL);
	//cvShowImage("A",temp6);
	//cvWaitKey(0);

	double difference=(dBi_sum+dGi_sum+dRi_sum)/(ki+ko);

	return (int)difference;
}

int Depth::costSC(int x, int y1,int y2)
{
	int d=y2-y1;
	//Window Range
	int i_start=x-(int)(WindowH/2);//search window最左上角
	int j_start=y1-(int)(WindowW/2);//search window最左上角
	int i_end=i_start+WindowH;
	int j_end=j_start+WindowW;

	//色彩範圍
	int CenterBS=(int)SMask[step*x+patch*y1+0];
	int CenterGS=(int)SMask[step*x+patch*y1+1];
	int CenterRS=(int)SMask[step*x+patch*y1+2];
	int BS, GS, RS;
	int range=10;
	int BS_MAX=CenterBS+range;
	int BS_MIN=CenterBS-range;
	int GS_MAX=CenterGS+range;
	int GS_MIN=CenterGS-range;
	int RS_MAX=CenterRS+range;
	int RS_MIN=CenterRS-range;
	//空間範圍
	int CenterXS=SpaceX[width*x+y1];
	int CenterYS=SpaceY[width*x+y1];
	int XS, YS;
	int XYrange=0;
	int XS_MAX=CenterXS+XYrange;
	int XS_MIN=CenterXS-XYrange;
	int YS_MAX=CenterYS+XYrange;
	int YS_MIN=CenterYS-XYrange;

	//範圍超出邊界處理(僅判斷window，中心位置判斷在上一層blockmatch)
	if(i_start<0)
		i_start=0;

	if(i_end>(height-1))
		i_end=height-1;

	if(j_start<0)
		j_start=0;

	if(j_end>(width-1))
		j_end=width-1;

	if(j_end+d>(width-1))
		j_end=width-1-d;
/*
	//debug
	IplImage *temp3 = cvCreateImage(cvSize(width,height),8,3);
	temp3->imageData=(char*)SMask_;
	IplImage *temp2 = cvCreateImage(cvSize(width,height),8,3);
	temp2->imageData=(char*)ImgR_;
	IplImage *temp6 = cvCreateImage(cvSize(width,height),8,3);
*/
	//比對
	int dB,dG,dR;
	double dBi_sum=0,dGi_sum=0,dRi_sum=0;
	//double dBo_sum=0,dGo_sum=0,dRo_sum=0;
	int ki=0;
	int ko=0;
	for(int i=i_start;i<=i_end;i+=WindowSamplePeriod){
		for(int j=j_start;j<=j_end;j+=WindowSamplePeriod){

			//segmentation結果
			BS=(int)SMask[step*i+patch*j+0];
			GS=(int)SMask[step*i+patch*j+1];
			RS=(int)SMask[step*i+patch*j+2];

			XS=SpaceX[width*i+j];
			YS=SpaceY[width*i+j];

			//確認是否在同一分割區
			if(BS<BS_MAX && BS>BS_MIN){//B
				if(GS<GS_MAX && GS>GS_MIN){//G
					if(RS<RS_MAX && RS>RS_MIN){//R
						if(XS<XS_MAX && XS>XS_MIN){//X
							if(YS<YS_MAX && YS>YS_MIN){//Y
								dB=(int)ImgL[step*i+patch*j+0]-(int)ImgR[step*i+patch*(j+d)+0];
								dG=(int)ImgL[step*i+patch*j+1]-(int)ImgR[step*i+patch*(j+d)+1];
								dR=(int)ImgL[step*i+patch*j+2]-(int)ImgR[step*i+patch*(j+d)+2];
								dB=abs(dB);
								dG=abs(dG);
								dR=abs(dR);

								dBi_sum+=dB;
								dGi_sum+=dG;
								dRi_sum+=dR;

								ki++;

								//debug
								//cvCircle(temp3,cvPoint(j,i),1,CV_RGB(255,255,0),1,CV_AA,0);//debug
								//cvCircle(temp2,cvPoint(j+d,i),1,CV_RGB(dR,dG,dB),1,CV_AA,0);//debug
								//cvCircle(temp6,cvPoint(j,i),1,CV_RGB((int)ImgL[step*i+patch*j+2],(int)ImgL[step*i+patch*j+1],(int)ImgL[step*i+patch*j+0]),1,CV_AA,0);//debug

								continue;
							}}}}}//IF

					//在分割區外
					dB=(int)ImgL[step*i+patch*j+0]-(int)ImgR[step*i+patch*(j+d)+0];
					dG=(int)ImgL[step*i+patch*j+1]-(int)ImgR[step*i+patch*(j+d)+1];
					dR=(int)ImgL[step*i+patch*j+2]-(int)ImgR[step*i+patch*(j+d)+2];
					dB=int(abs(dB)*0.2);
					dG=int(abs(dG)*0.2);
					dR=int(abs(dR)*0.2);

					dBi_sum+=dB;
					dGi_sum+=dG;
					dRi_sum+=dR;

					ko++;

					//cvCircle(temp2,cvPoint(j+d,i),1,CV_RGB(dR,dG,dB),1,CV_AA,0);//debug
					//cvCircle(temp3,cvPoint(j,i),1,CV_RGB(0,0,200),1,CV_AA,0);//debug
				}}
	//debug
	//cvNamedWindow("A",CV_WINDOW_NORMAL);
	//cvShowImage("A",temp6);
	//cvWaitKey(0);

	double difference=(dBi_sum+dGi_sum+dRi_sum)/(ki+ko);

	return (int)difference;
}

int Depth::BlockMatch()
{
	//Matching Range
	int d_end;
	int d_estimate_previous=0;
	//run each pixel
	for(int x=Xstart;x<=Xend;x+=ImgSamplePeriod){

		int yt=-1;
		int yt_=-1;
		int Mulit2OneCount=0;
		bool Mulit2OneFlage=0;
		for(int y=Ystart;y<=Yend;y+=ImgSamplePeriod){
/*
	//debug
	int x=95;
	int y=100;
	IplImage *temp1 = cvCreateImage(cvSize(width,height),8,3);
	temp1->imageData=(char*)ImgL_;
	IplImage *temp2 = cvCreateImage(cvSize(width,height),8,3);
	temp2->imageData=(char*)ImgR_;
	IplImage *temp3 = cvCreateImage(cvSize(width,height),8,3);
	temp3->imageData=(char*)SMask_;
*/


			//searching range
			d_end=50;
			int d_end_=width-y;//搜尋至右畫面邊緣
			if(d_end>d_end_)
				d_end=d_end_;

			//block match in row 
			//d_SamplePeriod=10;//debug
			cost_buffer_count=0;
			TemplateB(x, y);
			for(int d=0;d<d_end;d+=d_SamplePeriod){
				//int d=60;

				cost_buffer[cost_buffer_count]=this->costB(x,y,y+d);
				//cost_buffer[cost_buffer_count]=this->costA(x,y,y+d);
				//cost_buffer[cost_buffer_count]=this->cost(x,y,y+d);
				//cost_buffer[cost_buffer_count]=this->costS(x,y,y+d);

				d_buffer[cost_buffer_count]=d;
				//cvCircle(temp2,cvPoint(y+d,x),1,CV_RGB(0,0,cost_buffer[cost_buffer_count]),1,CV_AA,0);//debug
				cost_buffer_count++;
			}//for(int d=0;d<d_end;d+=d_SamplePeriod){

			//find out MIN cost 
			int cost_min=100000;
			int d_estimate=0;
			for(int k=0;k<cost_buffer_count;k++){
				if(cost_min>cost_buffer[k]){

					//if(dColor_buffer[k]>50)//略過條件
						//continue;

					cost_min=cost_buffer[k];
					d_estimate=d_buffer[k];
				}
				//printf("\n %d %d %d",k,cost_buffer[k],d_estimate);//debug
			}//for(int k=0;k<cost_buffer_count;k++){

			//不可靠處理


			//偵測重複match
			yt=y+d_estimate;
			if(yt==yt_){
				Mulit2OneCount++;
				Mulit2OneFlage=1;
			}
			else if(Mulit2OneFlage==1)//開始修正
			{
				for(int s=1; s<=Mulit2OneCount; s++){
					DisparityMap[width_R*x+y-1-s]=DisparityMap[width_R*x+y-1];
				}
				Mulit2OneCount=0;
				Mulit2OneFlage=0;
			}
			yt_=yt;//紀錄前一筆yt

			//set disparity
			DisparityMap[width_R*x+y]=d_estimate;

			//cvCircle(temp2,cvPoint(y+d_estimate,x),1,CV_RGB(255,0,0),1,CV_AA,0);
			//cvRectangle(temp2,cvPoint(y-6,x-6),cvPoint(y+6,x+6),CV_RGB(255,255,0),1,CV_AA,0);
			//cvRectangle(temp2,cvPoint(y+d_estimate-6,x-6),cvPoint(y+d_estimate+6,x+6),CV_RGB(255,255,0),1,CV_AA,0);
		
		}}//for xy
/*
			//debug
			cvCircle(temp1,cvPoint(y,x),1,CV_RGB(255,255,0),1,CV_AA,0);
			cvRectangle(temp1,cvPoint(y-7,x-7),cvPoint(y+7,x+7),CV_RGB(255,255,0),1,CV_AA,0);
			cvCircle(temp2,cvPoint(y+d_estimate,x),1,CV_RGB(255,0,0),1,CV_AA,0);
			cvRectangle(temp2,cvPoint(y+d_estimate-7,x-7),cvPoint(y+d_estimate+7,x+7),CV_RGB(255,0,0),1,CV_AA,0);
*/
	return 0;

}


int Depth::BlockMatchSeg()
{
	//Matching Range
	int d_end;
	int d_estimate_previous=0;
	//run each pixel

/*
	//debug
	IplImage *temp1 = cvCreateImage(cvSize(width,height),8,3);
	temp1->imageData=(char*)ImgL_;
	IplImage *temp2 = cvCreateImage(cvSize(width,height),8,3);
	temp2->imageData=(char*)ImgR_;
	IplImage *temp3 = cvCreateImage(cvSize(width,height),8,3);
	temp3->imageData=(char*)SMask_;
*/
/*
	int i_start=60;
	int i_end=160;
	int j_start=100;
	int j_end=180;
	int x=(i_start+i_end)/2;
	int y=(j_start+j_end)/2;
*/

	for(int i=0;i<ClusterNum; i++){

		int ClusterIndex=i;
		int i_start=clusters[ClusterIndex].i_start;
		int i_end=clusters[ClusterIndex].i_end;
		int j_start=clusters[ClusterIndex].j_start;
		int j_end=clusters[ClusterIndex].j_end;
		int x=(i_start+i_end)/2;
		int y=(j_start+j_end)/2;


			//searching range
			d_end=50;
			int d_end_=width-y;//搜尋至右畫面邊緣
			if(d_end>d_end_)
				d_end=d_end_;

			//block match in row 
			//d_SamplePeriod=10;//debug
			cost_buffer_count=0;
			for(int d=0;d<d_end;d+=d_SamplePeriod){
				//int d=60;
				
				cost_buffer[cost_buffer_count]=costRS(ClusterIndex,d);
				//cost_buffer[cost_buffer_count]=this->costR(i_start, i_end,j_start,j_end,d);
				d_buffer[cost_buffer_count]=d;
				//cvCircle(temp2,cvPoint(y+d,x),1,CV_RGB(0,0,cost_buffer[cost_buffer_count]),1,CV_AA,0);//debug
				cost_buffer_count++;
			}//for(int d=0;d<d_end;d+=d_SamplePeriod){

			//find out MIN cost 
			int cost_min=100000;
			int d_estimate=0;
			for(int k=0;k<cost_buffer_count;k++){
				if(cost_min>cost_buffer[k]){
					//if(dColor_buffer[k]>50)//略過條件
						//continue;
					cost_min=cost_buffer[k];
					d_estimate=d_buffer[k];
				}
				//printf("\n %d %d %d",k,cost_buffer[k],d_estimate);//debug
			}//for(int k=0;k<cost_buffer_count;k++){

			//Inverse check
			int cost_inv=0;
			for(int d2=0;d2>-1;d2--){
				cost_inv=costRS_inverse(ClusterIndex,d_estimate,d2);
				if(cost_inv<cost_min){
					d_estimate=0;
					break;}
			}//for d2

			//不可靠處理

			//set disparity
			DisparitySegSet(ClusterIndex,d_estimate);
			//DisparityMap[width_R*x+y]=d_estimate;

			//cvCircle(temp2,cvPoint(y+d_estimate,x),1,CV_RGB(255,0,0),1,CV_AA,0);
			//cvRectangle(temp2,cvPoint(y-6,x-6),cvPoint(y+6,x+6),CV_RGB(255,255,0),1,CV_AA,0);
			//cvRectangle(temp2,cvPoint(y+d_estimate-6,x-6),cvPoint(y+d_estimate+6,x+6),CV_RGB(255,255,0),1,CV_AA,0);
		
		}//for i
/*
			//debug
			cvCircle(temp1,cvPoint(y,x),1,CV_RGB(255,0,0),1,CV_AA,0);
			cvRectangle(temp1,cvPoint(j_start,i_start),cvPoint(j_end,i_end),CV_RGB(255,0,0),1,CV_AA,0);
			cvCircle(temp2,cvPoint(y+d_estimate,x),1,CV_RGB(255,0,0),1,CV_AA,0);
			cvRectangle(temp2,cvPoint(j_start+d_estimate,i_start),cvPoint(j_end+d_estimate,i_end),CV_RGB(255,0,0),1,CV_AA,0);
*/
	return 0;

}


int Depth::TemplateB(int x, int y1)
{
	bool B_L,G_L,R_L;
	//Window Range
	int i_start=x-(int)(WindowH/2);//search window最左上角
	int j_start=y1-(int)(WindowW/2);//search window最左上角
	int i_end=i_start+WindowH;
	int j_end=j_start+WindowW;

	//範圍超出邊界處理(僅判斷window，中心位置判斷在上一層blockmatch)
	if(i_start<0)
		i_start=0;

	if(i_end>(height-1))
		i_end=height-1;

	if(j_start<0)
		j_start=0;

	if(j_end>(width-1))
		j_end=width-1;

	//紀錄至globle
	WindowLRangeXstart=i_start-x;
	WindowLRangeYstart=j_start-y1;
    WindowLRangeXend=i_end-x;
	WindowLRangeYend=j_end-y1;

	int CenterB_L=(int)ImgL[step*x+patch*y1+0];
	int CenterG_L=(int)ImgL[step*x+patch*y1+1];
	int CenterR_L=(int)ImgL[step*x+patch*y1+2];

	//Left as Template
	WindwoL_B=0;
    WindwoL_G=0;
	WindwoL_R=0;
	int a=0,b=0;
	for(int i=i_start;i<i_end;i+=WindowSamplePeriod){
		b=0;
		for(int j=j_start;j<j_end;j+=WindowSamplePeriod){//!!
			//中心與其它比較
			B_L=0,G_L=0,R_L=0;
			if(ImgL[step*i+patch*j+0]>=CenterB_L){
				B_L=1;
			}

			if(ImgL[step*i+patch*j+1]>=CenterG_L){
				G_L=1;
			}

			if(ImgL[step*i+patch*j+2]>=CenterR_L){
				R_L=1;
			}

			WindwoL_B+=ImgL[step*i+patch*j+0];
			WindwoL_G+=ImgL[step*i+patch*j+1];
			WindwoL_R+=ImgL[step*i+patch*j+2];

			///存入template資料;
			WindwoL[WindowStep*a+patch*b+0]=B_L;
			WindwoL[WindowStep*a+patch*b+1]=G_L;
			WindwoL[WindowStep*a+patch*b+2]=R_L;
			b++;
			/*
			ImgL_[step*i+patch*j+0]=255*int(B_L);//debug
			ImgL_[step*i+patch*j+1]=255*int(G_L);//debug
			ImgL_[step*i+patch*j+2]=255*int(R_L);//debug
			*/
		}
		a++;
	}//for

	return 0;
}

int Depth::costB(int x, int y1,int y2)
{
	int d=y2-y1;

	//Window Range
	int i_start=x+WindowLRangeXstart;//search window最左上角
	int j_start=y1+WindowLRangeYstart;//search window最左上角
	int i_end=x+WindowLRangeXend;
	int j_end=y1+WindowLRangeYend;

	//色彩範圍
	int CenterBS=(int)SMask[step*x+patch*y1+0];
	int CenterGS=(int)SMask[step*x+patch*y1+1];
	int CenterRS=(int)SMask[step*x+patch*y1+2];
	int BS, GS, RS;
	int range=10;
	int BS_MAX=CenterBS+range;
	int BS_MIN=CenterBS-range;
	int GS_MAX=CenterGS+range;
	int GS_MIN=CenterGS-range;
	int RS_MAX=CenterRS+range;
	int RS_MIN=CenterRS-range;

	int CenterB_R=(int)ImgR[step*x+patch*y2+0];
	int CenterG_R=(int)ImgR[step*x+patch*y2+1];
	int CenterR_R=(int)ImgR[step*x+patch*y2+2];

/*
	//空間範圍
	int CenterXS=SpaceX[width*x+y1];
	int CenterYS=SpaceY[width*x+y1];
	int XS, YS;
	int XYrange=10;
	int XS_MAX=CenterXS+XYrange;
	int XS_MIN=CenterXS-XYrange;
	int YS_MAX=CenterYS+XYrange;
	int YS_MIN=CenterYS-XYrange;
*/
	//範圍超出邊界處理(僅判斷window，中心位置判斷在上一層blockmatch)

	if(j_end+d>(width-1))
		j_end=width-1-d;

	bool B_L,G_L,R_L,B_R,G_R,R_R;
	bool Rc,Gc,Bc;
	//int dB,dG,dR;
	int dBi_sum=0,dGi_sum=0,dRi_sum=0;
	//double dBo_sum=0,dGo_sum=0,dRo_sum=0;
	int ki=0;
	int ko=0;

	//以左圖為準對右圖進行比對
	WindwoR_B=0,WindwoR_G=0,WindwoR_R=0;
	int a=0,b=0;
	int jd;
	for(int i=i_start;i<i_end;i+=WindowSamplePeriod){
		b=0;
		for(int j=j_start;j<j_end;j+=WindowSamplePeriod){//!!
			//右圖
			//中心與其它比較
			B_R=0,G_R=0,R_R=0;

			if(ImgR[step*i+patch*(j+d)+0]>=CenterB_R){
				B_R=1;
			}

			if(ImgR[step*i+patch*(j+d)+1]>=CenterG_R){
				G_R=1;
			}

			if(ImgR[step*i+patch*(j+d)+2]>=CenterR_R){
				R_R=1;
			}

			WindwoR_B+=ImgR[step*i+patch*(j+d)+0];
			WindwoR_G+=ImgR[step*i+patch*(j+d)+1];
			WindwoR_R+=ImgR[step*i+patch*(j+d)+2];

/*
			ImgR_[step*i+patch*(j+d)+0]=255*int(B_R);//debug
			ImgR_[step*i+patch*(j+d)+1]=255*int(G_R);//debug
			ImgR_[step*i+patch*(j+d)+2]=255*int(R_R);//debug
*/

			//比對

			///取出template資料;
			B_L=WindwoL[WindowStep*a+patch*b+0];
			G_L=WindwoL[WindowStep*a+patch*b+1];
			R_L=WindwoL[WindowStep*a+patch*b+2];
			b++;
/*
			ImgR_[step*i+patch*(j)+0]=255*int(B_L);//debug
			ImgR_[step*i+patch*(j)+1]=255*int(G_L);//debug
			ImgR_[step*i+patch*(j)+2]=255*int(R_L);//debug
*/
			///segmentation結果
			BS=(int)SMask[step*i+patch*j+0];
			GS=(int)SMask[step*i+patch*j+1];
			RS=(int)SMask[step*i+patch*j+2];

			///同一分割區
			if(BS<BS_MAX && BS>BS_MIN){//B
				if(GS<GS_MAX && GS>GS_MIN){//G
					if(RS<RS_MAX && RS>RS_MIN){//R
						//左右比較
			            //Bc=(B_L != B_R);
						//Gc=(G_L != G_R);
						//Rc=(R_L != R_R);

						if (B_L != B_R)
							dBi_sum++;

						if (G_L != G_R)
							dGi_sum++;

						if (R_L != R_R)
							dRi_sum++;

						ki++;
/*
						ImgR_[step*(i+20)+patch*(j)+0]=255*int(Bc);//debug
						ImgR_[step*(i+20)+patch*(j)+1]=255*int(Gc);//debug
						ImgR_[step*(i+20)+patch*(j)+2]=255*int(Rc);//debug
*/
						//continue;//忽略下列
					}}}//if

			///在分割區外
			ko++;
		}//for y
		a++;
	}//for x

	//色彩比較
	Windwos_dB=abs(WindwoR_B-WindwoL_B);
	Windwos_dG=abs(WindwoR_G-WindwoL_G);
	Windwos_dR=abs(WindwoR_R-WindwoL_R);

	if(ki>0){
	dColor_buffer[cost_buffer_count]=int((Windwos_dB+Windwos_dG+Windwos_dR)/(ki*3));
	}
	else{
		dColor_buffer[cost_buffer_count]=0;
	}

	int difference=(dBi_sum+dGi_sum+dRi_sum);
	//int difference=(Windwos_dB+Windwos_dG+Windwos_dR);

	return difference;
}

int Depth::costR(int i_start, int i_end, int j_start,int j_end,int d)
{
	//範圍超出邊界處理(僅判斷window，中心位置判斷在上一層blockmatch)
	if(i_start<0)
		i_start=0;

	if(i_end>(height-1))
		i_end=height-1;

	if(j_start<0)
		j_start=0;

	if(j_end>(width-1))
		j_end=width-1;

	if(j_end+d>(width-1))
		j_end=width-1-d;

	//
	int dB,dG,dR;
	double dB_sum=0,dG_sum=0,dR_sum=0;
	int k=0;
	for(int i=i_start;i<=i_end;i+=WindowSamplePeriod){
		for(int j=j_start;j<=j_end;j+=WindowSamplePeriod){
			
			dB=(int)ImgL[step*i+patch*j+0]-(int)ImgR[step*i+patch*(j+d)+0];
			dG=(int)ImgL[step*i+patch*j+1]-(int)ImgR[step*i+patch*(j+d)+1];
			dR=(int)ImgL[step*i+patch*j+2]-(int)ImgR[step*i+patch*(j+d)+2];
			dB=abs(dB);
			dG=abs(dG);
			dR=abs(dR);

			dB_sum+=dB;
			dG_sum+=dG;
			dR_sum+=dR;

			k++;
		}}
	double difference=(dB_sum+dG_sum+dR_sum)/(k);
	//double difference=dG_sum/k;
	return (int)difference;
}

int Depth::costRS(int ClusterIndex,int d)
{
	//region
	int i_start=clusters[ClusterIndex].i_start;
	int i_end=clusters[ClusterIndex].i_end;
	int j_start=clusters[ClusterIndex].j_start;
	int j_end=clusters[ClusterIndex].j_end;


	//範圍超出邊界處理(僅判斷window，中心位置判斷在上一層blockmatch)
	if(i_start<0)
		i_start=0;

	if(i_end>(height-1))
		i_end=height-1;

	if(j_start<0)
		j_start=0;

	if(j_end>(width-1))
		j_end=width-1;

	if(j_end+d>(width-1))
		j_end=width-1-d;

	//
	int dB,dG,dR;
	double dB_sum=0,dG_sum=0,dR_sum=0;
	int k=0;
	unsigned char BS,GS,RS;
	for(int i=i_start;i<=i_end;i+=WindowSamplePeriod){
		for(int j=j_start;j<=j_end;j+=WindowSamplePeriod){
			
			//segmentation結果
			BS=SMask[step*i+patch*j+0];
			GS=SMask[step*i+patch*j+1];
			RS=SMask[step*i+patch*j+2];

			dB=(int)ImgL[step*i+patch*j+0]-(int)ImgR[step*i+patch*(j+d)+0];
			dG=(int)ImgL[step*i+patch*j+1]-(int)ImgR[step*i+patch*(j+d)+1];
			dR=(int)ImgL[step*i+patch*j+2]-(int)ImgR[step*i+patch*(j+d)+2];
			dB=abs(dB);
			dG=abs(dG);
			dR=abs(dR);

			if(BS==clusters[ClusterIndex].B){
				if(GS==clusters[ClusterIndex].G){
					if(RS==clusters[ClusterIndex].R){
				dB_sum+=dB;
				dG_sum+=dG;
				dR_sum+=dR;

				k++;
			}}}//if
		}}
	double difference=(dB_sum+dG_sum+dR_sum)/(k);
	//double difference=dG_sum/k;
	return (int)difference;
}

int Depth::costRS_inverse(int ClusterIndex,int d, int d2)
{
	/* d: ref location in imgR*/
	/* d+d2: the location in imgL to compare with ref*/

	//int d=y2-clusters[ClusterNum].centerY;
	//region
	int i_start=clusters[ClusterIndex].i_start;
	int i_end=clusters[ClusterIndex].i_end;
	int j_start=clusters[ClusterIndex].j_start;
	int j_end=clusters[ClusterIndex].j_end;


	//範圍超出邊界處理(僅判斷window，中心位置判斷在上一層blockmatch)
	if(i_start<0)
		i_start=0;

	if(i_end>(height-1))
		i_end=height-1;

	if(j_start<0)
		j_start=0;

	if(j_end>(width-1))
		j_end=width-1;

	if(j_end+d>(width-1))
		j_end=width-1-d;

	//
	int dB,dG,dR;
	double dB_sum=0,dG_sum=0,dR_sum=0;
	int k=0;
	unsigned char BS,GS,RS;
	for(int i=i_start;i<=i_end;i+=WindowSamplePeriod){
		for(int j=j_start;j<=j_end;j+=WindowSamplePeriod){
			
			//segmentation結果
			BS=SMask[step*i+patch*j+0];
			GS=SMask[step*i+patch*j+1];
			RS=SMask[step*i+patch*j+2];

			dB=(int)ImgL[step*i+patch*(j+d+d2)+0]-(int)ImgR[step*i+patch*(j+d)+0];
			dG=(int)ImgL[step*i+patch*(j+d+d2)+1]-(int)ImgR[step*i+patch*(j+d)+1];
			dR=(int)ImgL[step*i+patch*(j+d+d2)+2]-(int)ImgR[step*i+patch*(j+d)+2];
			dB=abs(dB);
			dG=abs(dG);
			dR=abs(dR);

			if(BS==clusters[ClusterIndex].B){
				if(GS==clusters[ClusterIndex].G){
					if(RS==clusters[ClusterIndex].R){
				dB_sum+=dB;
				dG_sum+=dG;
				dR_sum+=dR;

				k++;
			}}}//if
		}}
	double difference=(dB_sum+dG_sum+dR_sum)/(k);
	//double difference=dG_sum/k;
	return (int)difference;
}



int Depth::DisparitySegSet(int ClusterIndex,int d){

	//region
	int i_start=clusters[ClusterIndex].i_start;
	int i_end=clusters[ClusterIndex].i_end;
	int j_start=clusters[ClusterIndex].j_start;
	int j_end=clusters[ClusterIndex].j_end;

	unsigned char BS,GS,RS;
	for(int i=i_start;i<=i_end;i++){
		for(int j=j_start;j<=j_end;j++){

			//segmentation結果
			BS=SMask[step*i+patch*j+0];
			GS=SMask[step*i+patch*j+1];
			RS=SMask[step*i+patch*j+2];

			if(BS==clusters[ClusterIndex].B){
				if(GS==clusters[ClusterIndex].G){
					if(RS==clusters[ClusterIndex].R){
						DisparityMap[width_R*i+j]=d;
			}}}//if
		}}
	return 0;
}



int Depth::costA(int x, int y1,int y2)
{
	int d=y2-y1;
	//Window Range
	int i_start=x-(int)(WindowH/2);//search window最左上角
	int j_start=y1-(int)(WindowW/2);//search window最左上角
	int i_end=i_start+WindowH;
	int j_end=j_start+WindowW;

	//色彩範圍
	int CenterBS=(int)SMask[step*x+patch*y1+0];
	int CenterGS=(int)SMask[step*x+patch*y1+1];
	int CenterRS=(int)SMask[step*x+patch*y1+2];
	int BS, GS, RS;
	int range=10;
	int BS_MAX=CenterBS+range;
	int BS_MIN=CenterBS-range;
	int GS_MAX=CenterGS+range;
	int GS_MIN=CenterGS-range;
	int RS_MAX=CenterRS+range;
	int RS_MIN=CenterRS-range;

	int CenterB_L=(int)ImgL[step*x+patch*y1+0];
	int CenterG_L=(int)ImgL[step*x+patch*y1+1];
	int CenterR_L=(int)ImgL[step*x+patch*y1+2];
	int CenterB_R=(int)ImgR[step*x+patch*y2+0];
	int CenterG_R=(int)ImgR[step*x+patch*y2+1];
	int CenterR_R=(int)ImgR[step*x+patch*y2+2];

/*
	int CenterB_L=(int)CenterBS;
	int CenterG_L=(int)CenterGS;
	int CenterR_L=(int)CenterRS;
	int CenterB_R=(int)CenterBS;
	int CenterG_R=(int)CenterGS;
	int CenterR_R=(int)CenterRS;
*/

/*
	//空間範圍
	int CenterXS=SpaceX[width*x+y1];
	int CenterYS=SpaceY[width*x+y1];
	int XS, YS;
	int XYrange=10;
	int XS_MAX=CenterXS+XYrange;
	int XS_MIN=CenterXS-XYrange;
	int YS_MAX=CenterYS+XYrange;
	int YS_MIN=CenterYS-XYrange;
*/
	//範圍超出邊界處理(僅判斷window，中心位置判斷在上一層blockmatch)
	if(i_start<0)
		i_start=0;

	if(i_end>(height-1))
		i_end=height-1;

	if(j_start<0)
		j_start=0;

	if(j_end>(width-1))
		j_end=width-1;

	if(j_end+d>(width-1))
		j_end=width-1-d;

	//比對
	bool B_L,G_L,R_L,B_R,G_R,R_R;
	//int dB,dG,dR;
	int dBi_sum=0,dGi_sum=0,dRi_sum=0;
	//double dBo_sum=0,dGo_sum=0,dRo_sum=0;
	int ki=0;
	int ko=0;
	for(int i=i_start;i<i_end;i+=WindowSamplePeriod){
		for(int j=j_start;j<j_end;j+=WindowSamplePeriod){//!!

			//segmentation結果
			BS=(int)SMask[step*i+patch*j+0];
			GS=(int)SMask[step*i+patch*j+1];
			RS=(int)SMask[step*i+patch*j+2];

			//XS=SpaceX[width*i+j];
			//YS=SpaceY[width*i+j];

			//中心與其它比較
			B_L=0,G_L=0,R_L=0,B_R=0,G_R=0,R_R=0;
			//Left
			if(ImgL[step*i+patch*j+0]>=CenterB_L){
				B_L=1;
			}

			if(ImgL[step*i+patch*j+1]>=CenterG_L){
				G_L=1;
			}

			if(ImgL[step*i+patch*j+2]>=CenterR_L){
				R_L=1;
			}

			//Right
			if(ImgR[step*i+patch*(j+d)+0]>=CenterB_R){
				B_R=1;
			}

			if(ImgR[step*i+patch*(j+d)+1]>=CenterG_R){
				G_R=1;
			}

			if(ImgR[step*i+patch*(j+d)+2]>=CenterR_R){
				R_R=1;
			}

			//確認是否在同一分割區
			if(BS<BS_MAX && BS>BS_MIN){//B
				if(GS<GS_MAX && GS>GS_MIN){//G
					if(RS<RS_MAX && RS>RS_MIN){//R
						//if(XS<XS_MAX && XS>XS_MIN){//X
						//if(YS<YS_MAX && YS>YS_MIN){//Y

						//左右比較
						if (B_L != B_R)
							dBi_sum++;

						if (G_L != G_R)
							dGi_sum++;

						if (R_L != R_R)
							dRi_sum++;



						ki++;
						continue;
					}}}//IF

			//在分割區外
			ko++;

		}}//for x y

	int difference=(dBi_sum+dGi_sum+dRi_sum);

	return difference;
}

int Depth::update(unsigned char* ImgL,unsigned char* ImgR)
{
	this->ImgL=ImgL;
	this->ImgR=ImgR;
	
	//BlockMatch();
	BlockMatchSeg();
		return 0;
}

int Depth::ROI(int Xstart, int Xend, int Ystart, int Yend)
{

	this->Xstart=Xstart;
	this->Xend=Xend;
	this->Ystart=Ystart;
	this->Yend=Yend;
	memset (this->DisparityMap,0,height_R*width_R);

	return 0;
}
