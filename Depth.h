#include "segmentation.h"

class Depth
{
	public:
	//image
	unsigned char* ImgL;
	unsigned char* ImgR;
	unsigned char* ImgL_;
	unsigned char* ImgR_;
	int height;
	int width;
	int ImgSamplePeriod;
	int step;
	int patch;
	int height_R;
	int width_R;
	int Xstart, Xend, Ystart, Yend;
	//window
	unsigned char* SMask;
	int* SpaceX;
	int* SpaceY;
	unsigned char* SMask_;
	bool Is_SMask;
	int WindowH;
	int WindowW;
	int WindowH_R;
	int WindowW_R;
	int WindowStep;
	int WindowSamplePeriod;

	///視窗暫存
    bool* WindwoL;//template
	bool* WindwoR;//
	///視窗可用範圍
	int WindowLRangeXstart,WindowLRangeXend;
	int WindowLRangeYstart,WindowLRangeYend;
    ///色彩特徵
	int  WindwoL_R;
	int  WindwoL_G;
	int  WindwoL_B;
	int  WindwoR_R;
	int  WindwoR_G;
	int  WindwoR_B;
	int Windwos_dR;
	int Windwos_dG;
	int Windwos_dB;

	//disparity
	int d_SamplingPts[31];
	int d_SamplingPtsItr[31];
	static const int d_SamplingPtsNum=31;

	//buffer
	int* cost_buffer;//cost
	int* d_buffer;//diparity
	int* dColor_buffer;//color differnece
	int cost_buffer_count;//number count
	int d_SamplePeriod;
	//result
	unsigned char* DisparityMap;

	//分割區
	Cluster *clusters;
	int ClusterNum;


//設定演算法
int setting(int height ,int width, int ImgSamplePeriod,
	int WindowH, int WindowW, int WindowSamplePeriod,int d_SamplePeriod, unsigned char* SMask);

int BlockMatch();
int BlockMatchSeg();
//
int update(unsigned char* ImgL,unsigned char* ImgR);

//cost
///mean shift segmentation (color)
int costS(int x, int y1,int y2);

///mean shift segmentation (color & space)
int costSC(int x, int y1,int y2);

//最基本
int cost(int x, int y1,int y2);

//以compare方式計算
///先產生樣版
int costB(int x, int y1,int y2);
int TemplateB(int x, int y1);
///不先產生樣版
int costA(int x, int y1,int y2);

int costR(int i_start, int i_end, int j_start,int j_end,int d);
int costRS(int ClusterIndex,int d);
int costRS_inverse(int ClusterIndex,int d, int d2);

int DisparitySegSet(int ClusterIndex,int d);

int ROI(int Xstart, int Xend, int Ystart, int Yend);
};
