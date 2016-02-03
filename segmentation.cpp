#include "stdafx.h"
#include <stdio.h>
#include <string.h>
#include "segmentation.h"
#include <math.h>
#define   ROUND(X)     (int)(X+0.5);
#include <stdlib.h>
#include <time.h>

/*
MeanShiftSegmentation::MeanShiftSegmentation(unsigned char* Img, int height, int width,int ImgSamplePeriod, 
	int WindowH, int WindowW, int WindowSamplePeriod, bool Is_SMask)
{


}
*/



void MeanShiftSegmentation::segmentation(char* src ,char* dst ,char* seg_map 
	,int rows ,int cols)
{



}

int MeanShiftSegmentation::shift(unsigned char* src ,int rows ,int cols
	, int x, int y, int WindowH, int WindowW, int SamplePeriod,double *SMask
	, int& x_new, int& y_new, double& B_avg, double& G_avg, double& R_avg)
{
	//確保視窗平衡
	if(WindowH%2==0){
		WindowH++;
	}
	if(WindowW%2==0){
		WindowW++;
	}
	//Window Range
	int x_start=x-(int)(WindowH/2);//search window最左上角
	int y_start=y-(int)(WindowW/2);//search window最左上角
	int x_end=x_start+WindowH;
	int y_end=y_start+WindowW;

	//範圍超出邊界處理
	if(x>(rows-1) && x<0 && y>(cols-1) && y<0)
		return 0;

	if(x_start<0)
		x_start=0;

	if(x_end>(rows-1))
		x_end=rows-1;

	if(y_start<0)
		y_start=0;

	if(y_end>(cols-1))
		y_end=cols-1;

	//圖片解析格式
	int step=3*cols;
	int patch=3;

	//取色彩平均
	int B=0,G=0,R=0;
	int B_sum=0, G_sum=0, R_sum=0;
	//double B_avg=0, G_avg=0, R_avg=0;
	B_avg=0, G_avg=0, R_avg=0;
	int PixelNumber=0;
	for(int i=x_start;i<=x_end;i=i+SamplePeriod){
		for(int j=y_start;j<=y_end;j=j+SamplePeriod){
			B=(int)src[step*i+patch*j+0];//B
			G=(int)src[step*i+patch*j+1];//G
			R=(int)src[step*i+patch*j+2];//R

			B_sum+=B;
			G_sum+=G;
			R_sum+=R;

			PixelNumber++;
		}}

	B_avg=B_sum/PixelNumber;
	G_avg=G_sum/PixelNumber;
	R_avg=R_sum/PixelNumber;



	//計算各點與平均色度的差異
	double dB=0, dG=0, dR=0;
	double g=0,g_sum=0;
	double xg_sum=0,yg_sum=0;
	if(SMask==0)
	{
		for(int i=x_start;i<x_end;i+=SamplePeriod){
			for(int j=y_start;j<y_end;j+=SamplePeriod){
				B=(int)src[step*i+patch*j+0];//B
				G=(int)src[step*i+patch*j+1];//G
				R=(int)src[step*i+patch*j+2];//R

				//與平均色彩的差異
				dB=double(B)-B_avg;
				dG=double(G)-G_avg;
				dR=double(R)-R_avg;

				dB=abs(dB);
				dG=abs(dG);
				dR=abs(dR);

				//weighting
				dB=exp(-sqrt(dB/32));
				dG=exp(-sqrt(dG/32));
				dR=exp(-sqrt(dR/32));
				g=dB*dG*dR;
	
				g_sum+=g;
				xg_sum+=g*i;
				yg_sum+=g*j;
			}}
	}
	else
	{
		int a=0,b=0;//mask index
		for(int i=x_start;i<x_end;i+=SamplePeriod){
			b=0;
			for(int j=y_start;j<y_end;j+=SamplePeriod){
				B=(int)src[step*i+patch*j+0];//B
				G=(int)src[step*i+patch*j+1];//G
				R=(int)src[step*i+patch*j+2];//R

				//與平均色彩的差異
				dB=double(B)-B_avg;
				dG=double(G)-G_avg;
				dR=double(R)-R_avg;

				dB=abs(dB);
				dG=abs(dG);
				dR=abs(dR);

				//Color Weighting 
				dB=exp(-sqrt(dB/32));
				dG=exp(-sqrt(dG/32));
				dR=exp(-sqrt(dR/32));
				//Space Weighting 
				g=dB*dG*dR*SMask[a*WindowW+b];
				b+=SamplePeriod;

				g_sum+=g;
				xg_sum+=g*i;
				yg_sum+=g*j;
			}
			a+=SamplePeriod;
		}
	}
	//新的中心位置
	double x_new_=xg_sum/g_sum;
	double y_new_=yg_sum/g_sum;

	//防止最小取樣下的偏移
	x_new=ROUND(x_new_);
	y_new=ROUND(y_new_);

return 0;
}

int MeanShiftSegmentation::SpaceMask()
{
	//確保視窗平衡
	if(WindowH%2==0){
		WindowH++;
	}
	if(WindowW%2==0){
		WindowW++;
	}

	//window在圖片中的範圍
	int m_start=-int(WindowH/2);
	int n_start=-int(WindowW/2);
	int m_end=abs(m_start);
	int n_end=abs(n_start);

	int lm=int(WindowH/2);
	int ln=int(WindowW/2);
	double l=sqrt(double(lm)*double(lm)+double(ln)*double(ln));//標準化常數

	double hm=0, hn=0, h=0;//在window位置
	int a=0,b=0;//window index
	this->SMask=new double[WindowH*WindowW];

	//double X=0;
	for(int m=m_start;m<=m_end;m++){
		b=0;
		for(int n=n_start;n<=n_end;n++){
			hm=double(m)/double(lm);
			hn=double(n)/double(ln);
			h=hm*hm+hn*hn;
			h=h/4;
			this->SMask[a*WindowH+b]=exp(-h);
 			//printf("\n %d, %d, %d, %d, %f",m,n,a,b,this->SMask[a*WindowH+b]);//debug
			//X+=m*SMask[a*WindowH+b];
			b++;
		}
		a++;
	}
	return 0;
}

int MeanShiftSegmentation::setting(int height ,int width, int ImgSamplePeriod,
	int WindowH, int WindowW, int WindowSamplePeriod, bool Is_SMask)
{
	//image setting
	this->height=height;
	this->width=width;
	this->step=this->width*this->patch;
	this->ImgSamplePeriod=ImgSamplePeriod;
	this->height_R=int(height/ImgSamplePeriod);
	this->width_R=int(width/ImgSamplePeriod);
	

	//window setting
	this->WindowH=WindowH;
	this->WindowW=WindowW;
	this->WindowSamplePeriod=WindowSamplePeriod; //(WindowH-1)%WindowSamplePeriod=0

	this->Is_SMask=Is_SMask;
	if(Is_SMask==true)
		this->SpaceMask();

	//output setting
	this->FilterMap=new unsigned char[this->height_R*this->width_R*3];//BGR
	SpaceX=new int[this->height_R*this->width_R];
	SpaceY=new int[this->height_R*this->width_R];

	return 0;
}

int  MeanShiftSegmentation::update(unsigned char* src)
{
	this->Img=src;

	this->filtering();

	return 0;
}

int  MeanShiftSegmentation::filtering()
{
	double B_avg,G_avg,R_avg;
	int a=0,b=0;
	for(int i=0; i<height; i+=ImgSamplePeriod){
		b=0;
		for(int j=0; j<width; j+=ImgSamplePeriod){
			//debug
			//int i=280;
			//int j=400;

			int x_new=-1;
			int y_new=-1;
			int x=i,y=j;

			//debug
			//cvCircle(img_,cvPoint(y,x),1,CV_RGB(0,0,0),1,CV_AA,0);
			//cvRectangle(img_,cvPoint(y-25,x-25),cvPoint(y+25,x+25),CV_RGB(0,0,0),1,CV_AA,0);
			for (int k=0; k<=30; k++){
				MeanShiftSegmentation::shift(Img ,height ,width
					, x, y, WindowH, WindowW, WindowSamplePeriod, SMask ,x_new, y_new
					,B_avg ,G_avg ,R_avg);

				//cvCircle(img_,cvPoint(y_new,x_new),1,CV_RGB(0,0,255),1,CV_AA,0);//debug

				//是否收斂
				if(x==x_new && y==y_new){
					break;
				}

				//移動限定
				if(abs(i-x_new)>30 || abs(j-y_new)>30){
					break;
				}

				//更新
				x=x_new;
				y=y_new;
			}

			//debug
			//cvCircle(img_,cvPoint(y,x),4,CV_RGB(255,255,0),1,CV_AA,0);
			//cvRectangle(img_,cvPoint(y-25,x-25),cvPoint(y+25,x+25),CV_RGB(255,255,0),1,CV_AA,0);

			//設定新像素值
			FilterMap[3*(width_R*a+b)+0]=(unsigned char)B_avg;
			FilterMap[3*(width_R*a+b)+1]=(unsigned char)G_avg;
			FilterMap[3*(width_R*a+b)+2]=(unsigned char)R_avg;
			//FilterMap[3*(width_R*a+b)+0]=(unsigned char)Img[step*x+patch*y+0];
			//FilterMap[3*(width_R*a+b)+1]=(unsigned char)Img[step*x+patch*y+1];
			//FilterMap[3*(width_R*a+b)+2]=(unsigned char)Img[step*x+patch*y+2];


			//紀錄最終位置
		    SpaceX[width_R*a+b]=x;
	        SpaceY[width_R*a+b]=y;

			b++;
		}
		a++;
	}

	return 0;
}

int FastSegmentation::setting(unsigned char* src, int height ,int width, int ImgSamplePeriod,
	int WindowH, int WindowW, int WindowSamplePeriod, int HistoLevelNum)
{
	//image setting
	this->Img=src;
	this->height=height;
	this->width=width;
	this->step=this->width*this->patch;
	this->ImgSamplePeriod=ImgSamplePeriod;
	this->height_R=int(height/ImgSamplePeriod);
	this->width_R=int(width/ImgSamplePeriod);
	this->step_R=this->width_R*this->patch;

	//window setting
	///確保視窗平衡
	if(WindowH%2==0){
		WindowH++;
	}
	if(WindowW%2==0){
		WindowW++;
	}

	this->WindowH=WindowH;
	this->WindowW=WindowW;
	this->WindowSamplePeriod=WindowSamplePeriod; //(WindowH-1)%WindowSamplePeriod=0

	//Histogram Setting
	this->HistoLevelNum=HistoLevelNum;
	int HistoL=int(255/HistoLevelNum);
	HistoRange=new int[HistoLevelNum];
	HistoR=new int[HistoLevelNum];
	HistoG=new int[HistoLevelNum];
	HistoB=new int[HistoLevelNum];

	for(int i=0;i<HistoLevelNum-1;i++){
		HistoRange[i]=(i+1)*(HistoL);
	}
	HistoRange[HistoLevelNum-1]=255;

	SizeofHist=sizeof(HistoR)*HistoLevelNum;
	//output setting
	this->FilterMap=new unsigned char[this->height_R*this->width_R*this->patch];//BGR

	//cluster
	this->VisitMap=new bool[this->height_R*this->width_R];
	this->ClusterMap=new unsigned char[this->height_R*this->width_R*this->patch];//BGR

	return 0;
}

int FastSegmentation::classifiy(int x, int y)
{
	//Window Range
	int x_start=x-(int)(WindowH/2);//search window最左上角
	int y_start=y-(int)(WindowW/2);//search window最左上角
	int x_end=x_start+WindowH;
	int y_end=y_start+WindowW;

	//範圍超出邊界處理
	if(x>(width-1) && x<0 && y>(height-1) && y<0)
		return 0;

	if(x_start<0)
		x_start=0;

	if(x_end>=height)
		x_end=height-1;

	if(y_start<0)
		y_start=0;

	if(y_end>=width)
		y_end=width-1;

	//重置histogram;
	memset (HistoR,0,SizeofHist);
	memset (HistoG,0,SizeofHist);
	memset (HistoB,0,SizeofHist);

	//分類
	int B,G,R;
	for(int i=x_start;i<x_end;i+=WindowSamplePeriod){
		for(int j=y_start;j<y_end;j+=WindowSamplePeriod){
			B=(int)Img[step*i+patch*j+0];//B
			G=(int)Img[step*i+patch*j+1];//G
			R=(int)Img[step*i+patch*j+2];//R

			for(int i=0;i<HistoLevelNum;i++){
				if(B<=HistoRange[i]){
					HistoB[i]++;
					break;
				}
			}//for i

			for(int i=0;i<HistoLevelNum;i++){
				if(G<=HistoRange[i]){
					HistoG[i]++;
					break;
				}
			}//for i

			for(int i=0;i<HistoLevelNum;i++){
				if(R<=HistoRange[i]){
					HistoR[i]++;
					break;
				}
			}//for i
		}}//for xy

	//中心加權

	B=(int)Img[step*x+patch*y+0];//B
	G=(int)Img[step*x+patch*y+1];//G
	R=(int)Img[step*x+patch*y+2];//R

	for(int i=0;i<HistoLevelNum;i++){
		if(B<HistoRange[i]){
			HistoB[i]=HistoB[i]*2;
			break;
		}
	}//for i

	for(int i=0;i<HistoLevelNum;i++){
		if(G<HistoRange[i]){
			HistoG[i]=HistoG[i]*2;
			break;
		}
	}//for i

	for(int i=0;i<HistoLevelNum;i++){
		if(R<HistoRange[i]){
			HistoR[i]=HistoR[i]*2;
			break;
		}
	}//for i


	//找出代表RGB
	int HistoMax_R=0 ,HistoMax_G=0,HistoMax_B=0;
	int HistoMax_R_index=0 ,HistoMax_G_index=0,HistoMax_B_index=0;
	for(int s=0;s<HistoLevelNum;s++){
		if(HistoMax_B<HistoB[s]){
			HistoMax_B=HistoB[s];
			HistoMax_B_index=s;
		}

		if(HistoMax_G<HistoG[s]){
			HistoMax_G=HistoG[s];
			HistoMax_G_index=s;
		}

		if(HistoMax_R<HistoR[s]){
			HistoMax_R=HistoR[s];
			HistoMax_R_index=s;
		}
	}
				
	//新的值
	FilterMap[step*x+patch*y+0]=HistoRange[HistoMax_B_index];
	FilterMap[step*x+patch*y+1]=HistoRange[HistoMax_G_index];
	FilterMap[step*x+patch*y+2]=HistoRange[HistoMax_R_index];


	return 0;
}



int  FastSegmentation::filtering()
{
	for(int i=0; i<height; i+=ImgSamplePeriod){
		for(int j=0; j<width; j+=ImgSamplePeriod){
			//debug
			//int i=280;
			//int j=400;
			classifiy(i,j);
		}}

	return 0;
}

int FastSegmentation::clustering()
{
	//Set visited record 0
	//memset(VisitMap,0,sizeof(bool)*this->height_R*this->width_R);
	for(int x=0; x<height_R; x++){
		for(int y=0; y<width_R; y++){
			VisitMap[width_R*x+y]=0;}}


	//visit each pixel
	int B,G,R;
	int Bc,Gc,Rc;
	int ClusterCount=0;
	int LineCount=0;
	int ElementCount=0;
	int i_MIN=100000,i_MAX=0,j_MIN=10000,j_MAX=0;
	for(int x=0; x<height_R; x++){
		for(int y=0; y<width_R; y++){
	        //int x=0,y=0;//debug

			//若拜訪過則跳過
			if(VisitMap[width_R*x+y]!=false)
				continue;

			//取出
			B=(int)FilterMap[step_R*x+patch*y+0];//B
			G=(int)FilterMap[step_R*x+patch*y+1];//G
			R=(int)FilterMap[step_R*x+patch*y+2];//R

			//確定cluster範圍
			i_MIN=1000000,i_MAX=0,j_MIN=1000000,j_MAX=0;
			ElementCount=0;
			for(int i=x; i<height; i++){
				LineCount=0;
				for(int j=y; j<width; j++){
					//取值
					Bc=(int)FilterMap[step_R*i+patch*j+0];//B
					Gc=(int)FilterMap[step_R*i+patch*j+1];//G
					Rc=(int)FilterMap[step_R*i+patch*j+2];//R

					///若非屬同一segment或已拜訪則跳至下一列
					if(VisitMap[width_R*i+j]!=false)//segment
						break;

					if(Bc!=B || Gc!=B || Rc!=R)//已拜訪
						break;

					//設為拜訪過
					VisitMap[width_R*i+j]=1;
					LineCount++;

					//確定範圍
					if(i_MIN>i)
						i_MIN=i;

					if(j_MIN>j)
						j_MIN=j;

					if(i_MAX<i)
						i_MAX=i;

					if(j_MAX<j)
						j_MAX=j;

				}//for j 
				   if(LineCount==0)
					   break;
				   ElementCount+=LineCount;

			}//for i

			//儲存cluster
			clusters[ClusterCount].i_start=i_MIN;
			clusters[ClusterCount].i_end=i_MAX;
			clusters[ClusterCount].j_start=j_MIN;
			clusters[ClusterCount].j_end=j_MAX;
			clusters[ClusterCount].PixelNum=ElementCount+1;

			//新的cluster
			if(ElementCount>1)
				ClusterCount++;

		}}//for xy

	 ClusterNum=ClusterCount;
	return 0;
}

int FastSegmentation::clustering2()
{
	//Set visited record 0
	memset(VisitMap,0,sizeof(bool)*this->height_R*this->width_R);

	//visit each pixel
    srand(int(time(0)));
	int B,G,R;
	int Bc,Gc,Rc;
	int Bs,Gs,Rs;
	int ClusterCount=0;
	int LineCount=0;
	int ElementCount=0;
	int i_MIN=100000,i_MAX=0,j_MIN=10000,j_MAX=0;
	int j_Left,j_Right,j_Center;
	for(int x=0; x<height_R; x++){
		for(int y=0; y<width_R; y++){
	        //int x=0,y=0;//debug

			//若拜訪過則跳過
			if(VisitMap[width_R*x+y]!=false)
				continue;

			//Range Restriction
			int j_Left_limit=y-90;
            int j_Right_limit=y+90;

			if(j_Left_limit<0)
				j_Left_limit=0;

			if(j_Right_limit>width)
				j_Right_limit=width;

			//取出
			B=(int)FilterMap[step_R*x+patch*y+0];//B
			G=(int)FilterMap[step_R*x+patch*y+1];//G
			R=(int)FilterMap[step_R*x+patch*y+2];//R
			
			Bs=rand()%256;//B
			Gs=rand()%256;//G
			Rs=rand()%256;//R

			//確定cluster範圍
			i_MIN=1000000,i_MAX=0,j_MIN=1000000,j_MAX=0;
			ElementCount=0;

			j_Center=y;
			for(int i=x; i<height; i++){
				LineCount=0;
				//向右
				for(j_Right=j_Center; j_Right<j_Right_limit; j_Right++){
					//取值
					Bc=(int)FilterMap[step_R*i+patch*j_Right+0];//B
					Gc=(int)FilterMap[step_R*i+patch*j_Right+1];//G
					Rc=(int)FilterMap[step_R*i+patch*j_Right+2];//R

					///若非屬同一segment或已拜訪則跳至下一列
					if(VisitMap[width_R*i+j_Right]!=false || Bc!=B || Gc!=G || Rc!=R)
						break;
					
					//設為拜訪過
					VisitMap[width_R*i+j_Right]=1;
					LineCount++;

					ClusterMap[step_R*i+patch*j_Right+0]=Bs;//B
					ClusterMap[step_R*i+patch*j_Right+1]=Gs;//G
					ClusterMap[step_R*i+patch*j_Right+2]=Rs;//R
					//debug
					//Img[step_R*i+patch*j_Right+0]=255;//
			        //Img[step_R*i+patch*j_Right+1]=255;//
			        //Img[step_R*i+patch*j_Right+2]=255;//
					
					//確定範圍
					if(i_MIN>i)
						i_MIN=i;

					if(j_MIN>j_Right)
						j_MIN=j_Right;

					if(i_MAX<i)
						i_MAX=i;

					if(j_MAX<j_Right)
						j_MAX=j_Right;
				}//for j 
			
				//向左
				for(j_Left=j_Center-1; j_Left_limit>=0; j_Left--){
					//取值
					Bc=(int)FilterMap[step_R*i+patch*j_Left+0];//B
					Gc=(int)FilterMap[step_R*i+patch*j_Left+1];//G
					Rc=(int)FilterMap[step_R*i+patch*j_Left+2];//R

					///若非屬同一segment或已拜訪則跳至下一列
					if(VisitMap[width_R*i+j_Left]!=false || Bc!=B || Gc!=G || Rc!=R)
						break;

					//設為拜訪過
					VisitMap[width_R*i+j_Left]=1;
					LineCount++;

					ClusterMap[step_R*i+patch*j_Left+0]=Bs;//B
					ClusterMap[step_R*i+patch*j_Left+1]=Gs;//G
					ClusterMap[step_R*i+patch*j_Left+2]=Rs;//R

					//debug
					//Img[step_R*i+patch*j_Left+0]=255;//
			        //Img[step_R*i+patch*j_Left+1]=255;//
			        //Img[step_R*i+patch*j_Left+2]=255;//

					//確定範圍
					if(i_MIN>i)
						i_MIN=i;

					if(j_MIN>j_Left)
						j_MIN=j_Left;

					if(i_MAX<i)
						i_MAX=i;

					if(j_MAX<j_Left)
						j_MAX=j_Left;
				}//for j 

				if(LineCount==0)
					break;

				ElementCount+=LineCount;

				j_Center=int((j_Left+j_Right)*0.5);
			}//for i

			//儲存cluster
			clusters[ClusterCount].i_start=i_MIN;
			clusters[ClusterCount].i_end=i_MAX;
			clusters[ClusterCount].j_start=j_MIN;
			clusters[ClusterCount].j_end=j_MAX;
			clusters[ClusterCount].PixelNum=ElementCount+1;
			clusters[ClusterCount].B=B;
			clusters[ClusterCount].G=G;
			clusters[ClusterCount].R=R;
			clusters[ClusterCount].centerX=int((i_MIN+i_MAX)*0.5);
		    clusters[ClusterCount].centerY=int((j_MIN+j_MAX)*0.5);

			//新的cluster
			if(ElementCount>5)
				ClusterCount++;

		}}//for xy

	 ClusterNum=ClusterCount;
	return 0;
}
