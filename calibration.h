#include "myHeader.h"

class CameraCalibrator{
private:
    vector<string> m_filenames;
    Size m_borderSize;
    vector<vector<Point2f> > m_srcPoints;
    vector<vector<Point3f> > m_dstPoints;
	
public:
	Mat a,extrinsicmatrix;
	Mat cameraMatrix;
	Mat  distCoeffs;
	void calibrate_AR(const Mat &src, Mat &dst);
	void calibrate_AR1(const Mat &src, Mat &dst);
	void calibrate_AR2(const Mat &src, Mat &dst);
	void calibrate_AR3(const Mat &src, Mat &dst);
	void calibrate_AR4(const Mat &src, Mat &dst);
	void extrinsic(vector<string> &b);
	void cube(vector<string> &cubefunction,Mat &dst);
	void onMouse(int Event,int x,int y,int flags,void* param);
	void savedata(Mat &k);
	void printdata();
    void setFilename();
	void setFilename1();
	void setFilename2();
	void setFilename3();
	void setFilename4();
	void setFilename5();
    void setBorderSize(const Size &borderSize);
    void addChessboardPoints();
	 void addChessboardPoints2();
    void addPoints(const vector<Point2f> &imageCorners, const vector<Point3f> &objectCorners);
    void calibrate( Mat &src, Mat &dst);
	void calibrate2( Mat &src, Mat &dst);
	void calibrate3( Mat &src, Mat &dst);
	void calibrate4( Mat &src, Mat &dst);
	void secondperspectivetransform(int &x1,int &y1,int &x2,int &y2,int &x3,int &y3,int &x4,int &y4);
	int x1,y1,x2,y2,x3,y3,x4,y4,a1;
	vector<string> b;
	vector<string> cubefunction;
};
/*
class CameraCalibrator2{
private:
    vector<string> m_filenames;
    Size m_borderSize;
    vector<vector<Point2f> > m_srcPoints;
    vector<vector<Point3f> > m_dstPoints;
public:

    void setFilename();
	void setFilename1();
	void setFilename2();
	void setFilename3();
	void setFilename4();
	void setFilename5();
    void setBorderSize(const Size &borderSize);
    void addChessboardPoints();
	 void addChessboardPoints2();
    void addPoints(const vector<Point2f> &imageCorners, const vector<Point3f> &objectCorners);
    void calibrate( Mat &src, Mat &dst);
	void calibrate2( Mat &src, Mat &dst);
	void calibrate3( Mat &src, Mat &dst);
	
};*/