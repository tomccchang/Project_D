class MeanShiftSegmentation
{
public:
	unsigned char* Img;
	unsigned char* FilterMap;
	int* SpaceX;
	int* SpaceY;
	int height;
	int width;
	int step;
	static const int patch=3;
	int height_R;
	int width_R;
	double* SMask;
	bool Is_SMask;
	int WindowH;
	int WindowW;
	int ImgSamplePeriod;
	int WindowSamplePeriod;

//MeanShiftSegmentation();
/*
MeanShiftSegmentation(unsigned char* Img, int height, int width,int ImgSamplePeriod, 
	int WindowH, int WindowW, int WindowSamplePeriod, bool Is_SMask);
*/
static void segmentation(char* src ,char* dst ,char* seg_map 
	,int rows ,int cols);

//設定演算法
int setting(int height ,int width, int ImgSamplePeriod,
	int WindowH, int WindowW, int WindowSamplePeriod, bool Is_SMask);

//取得影像
int update(unsigned char* src);

//執行單次mean shift
static int shift(unsigned char* src ,int rows ,int cols
	, int x, int y, int WindowH, int WindowW, int SamplePeorid, double *SMask
	, int& x_new, int& y_new, double& B_avg, double& G_avg, double& R_avg);



int filtering();

int SpaceMask();

};


struct Cluster
{
	int i_start;
	int i_end;
	int j_start;
	int j_end;
	int PixelNum;
	unsigned char B,G,R;
	int centerX;
	int centerY;
};



class FastSegmentation
{
public:
	unsigned char* Img;
	unsigned char* FilterMap;
	//int* SpaceX;
	//int* SpaceY;
	int height;
	int width;
	int step;
	static const int patch=3;
	int step_R;
	int height_R;
	int width_R;
	int WindowH;
	int WindowW;
	int ImgSamplePeriod;
	int WindowSamplePeriod;

	///histogram
	int HistoLevelNum;
	int*  HistoRange;
	int* HistoR;
	int* HistoG;
	int* HistoB;
	int SizeofHist;
	
	//cluster
	Cluster clusters[5000];
	int ClusterNum;
	bool* VisitMap;
	unsigned char*  ClusterMap;

//設定演算法
int setting(unsigned char* src, int height ,int width, int ImgSamplePeriod,
	int WindowH, int WindowW, int WindowSamplePeriod, int HistoLevelNum);

//取得影像
int update();

int filtering();

int classifiy(int x, int y);

int clustering();

int clustering2();

};
