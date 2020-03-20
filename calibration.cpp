#include "stdafx.h"
#include "calibration.h"



 std::string c[]={"01.jpg","02.jpg","03.jpg","04.jpg","05.jpg","06.jpg","07.jpg","08.jpg","09.jpg","010.jpg","011.jpg","012.jpg","013.jpg","014.jpg","015.jpg"};
 Mat A1 = (Mat_<double>(4,1) << 0,0, 0,1);
 Mat A2 = (Mat_<double>(4,1) << 0,2, 0,1);
 Mat A3 = (Mat_<double>(4,1) << 2, 2, 0,1);
 Mat A4 = (Mat_<double>(4,1) << 2, 0, 0,1);
 Mat A5 = (Mat_<double>(4,1) << 2,2, 2,1);
 Mat A6 = (Mat_<double>(4,1) << 2,0, 2,1);
 Mat A7 = (Mat_<double>(4,1) << 0,0, 2,1);
 Mat A8 = (Mat_<double>(4,1) << 0,2, 2,1);
 double X1,X2,X3,X4,X5,X6,X7,X8;
 double Y1,Y2,Y3,Y4,Y5,Y6,Y7,Y8;
 
 //Mat cam= (Mat_<float>(3,1));
 /*
 void CameraCalibrator::onMouse(int Event,int x,int y,int flags,void* param){
 if(Event==CV_EVENT_LBUTTONDOWN){
     cout<<x<<endl;
    }
   
}
*/
 void CameraCalibrator:: secondperspectivetransform(int &x1,int &y1,int &x2,int &y2,int &x3,int &y3,int &x4,int &y4){
	 
		 Mat image=imread("OriginalPerspective.png",CV_LOAD_IMAGE_UNCHANGED); 
		 Mat image_new =Mat:: zeros(image.rows, image.cols, image.type()) ;
		 Size imageSize = image.size();
		 
		 	Size boardsize= Size(11,8);

	vector<Point2f> srcCandidateCorners; 
	vector<Point3f> dstCandidateCorners;

	vector<vector<Point2f> > perpspective_srcPoints;
    vector<vector<Point3f> > perpspective_dstPoints;

	vector<Point2f> perpspective_4_srcPoints;
    vector<Point3f> perpspective_4_dstPoints;

	 Mat cameraMatrix_perspective, distCoeffs_perspective; 
	 vector<Mat> rvecs_perspective, tvecs_perspective; 



	for(int i=0; i<boardsize.height; i++){ 
        for(int j=0; j<boardsize.width; j++){ 
            dstCandidateCorners.push_back(Point3f(i, j, 0.0f)); 
        } 
    } 



	bool find=findChessboardCorners(image, boardsize, srcCandidateCorners); 		
    TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
    //cornerSubPix(image, srcCandidateCorners, Size(5,5), Size(-1,-1), param);  


	


        if(srcCandidateCorners.size() == boardsize.area()){ 

			 perpspective_srcPoints.push_back(srcCandidateCorners);  
             perpspective_dstPoints.push_back(dstCandidateCorners); 
           
        } 


		for(int i =0 ;i < perpspective_srcPoints.size();i++)
			cout<< perpspective_srcPoints[i]<<endl;
		for(int i =0 ;i < perpspective_dstPoints.size();i++)
			cout<< perpspective_dstPoints[i]<<endl;


	 

	calibrateCamera(perpspective_dstPoints, perpspective_srcPoints, imageSize, cameraMatrix_perspective, distCoeffs_perspective, rvecs_perspective, tvecs_perspective); 

	Mat rvecs_mat;
	Rodrigues(rvecs_perspective[0],rvecs_mat);
	Mat extrinsic_matrix_temp(3,4,CV_64F);

	
	
	extrinsic_matrix_temp.at<double>(0,0) = rvecs_mat.at<double>(0,0);
	extrinsic_matrix_temp.at<double>(1,0) = rvecs_mat.at<double>(1,0);
	extrinsic_matrix_temp.at<double>(2,0) = rvecs_mat.at<double>(2,0);

	extrinsic_matrix_temp.at<double>(0,1) = rvecs_mat.at<double>(0,1);
	extrinsic_matrix_temp.at<double>(1,1) = rvecs_mat.at<double>(1,1);
	extrinsic_matrix_temp.at<double>(2,1) = rvecs_mat.at<double>(2,1);

	extrinsic_matrix_temp.at<double>(0,2) = rvecs_mat.at<double>(0,2);
	extrinsic_matrix_temp.at<double>(1,2) = rvecs_mat.at<double>(1,2);
	extrinsic_matrix_temp.at<double>(2,2) = rvecs_mat.at<double>(2,2);

	extrinsic_matrix_temp.at<double>(0,3) = tvecs_perspective[0].at<double>(0,0);
	extrinsic_matrix_temp.at<double>(1,3) = tvecs_perspective[0].at<double>(1,0);
	extrinsic_matrix_temp.at<double>(2,3) = tvecs_perspective[0].at<double>(2,0);
	
	Mat  new_camera_matrix_perspective = getOptimalNewCameraMatrix(cameraMatrix_perspective,distCoeffs_perspective,imageSize,0,imageSize,0,false);
	
	 vector<Point3f> perspective;

  
  perspective.push_back(Point3f(x1, y1, 0)); 
   perspective.push_back(Point3f(x2,y2, 0)); 
  perspective.push_back(Point3f(x3, y3, 0)); 
 perspective.push_back(Point3f(x4,y4, 0)); 
 cout<<x1<<endl;
  cout<<y1<<endl;
   cout<<x2<<endl;
    cout<<y2<<endl;
	 cout<<x3<<endl;
	  cout<<y3<<endl;
	   cout<<x4<<endl;
	    cout<<y4<<endl;
 
  vector<Point2f> new_perspective;

	for(int count =0;count<perspective.size();count++)
{

	Mat extrinsic_temp(3,1,CV_64F);
    Mat intrinsic_temp(3,1,CV_64F);

	for(int j=0;j<3;j++){
	
		double temp = 0;
	 
	              temp += extrinsic_matrix_temp.at<double>(j,0)* perspective[count].x;
	              temp += extrinsic_matrix_temp.at<double>(j,1)* perspective[count].y;
				  temp += extrinsic_matrix_temp.at<double>(j,2)* perspective[count].z;
				  temp += extrinsic_matrix_temp.at<double>(j,3)* 1;
                  extrinsic_temp.at<double>(j,0) = temp;	
	   }	


	for(int l=0;l<3;l++){
		double temp_2=0;
		for(int k=0;k<3;k++){				
			temp_2 += new_camera_matrix_perspective.at<double>(l,k)* extrinsic_temp.at<double>(k,0);				
		}	
		intrinsic_temp.at<double>(l,0) = temp_2;
	}


		intrinsic_temp.at<double>(0,0)=intrinsic_temp.at<double>(0,0)/intrinsic_temp.at<double>(2,0);
		intrinsic_temp.at<double>(1,0)=intrinsic_temp.at<double>(1,0)/intrinsic_temp.at<double>(2,0);
		intrinsic_temp.at<double>(2,0)=1;
 

	new_perspective.push_back(Point2f(intrinsic_temp.at<double>(0,0),intrinsic_temp.at<double>(1,0)));

}


  
	vector<Point2f> perspective_to_point;

	perspective_to_point.push_back(Point2f(20, 20)); 
	perspective_to_point.push_back(Point2f(450, 20)); 
	perspective_to_point.push_back(Point2f(450, 450)); 
	perspective_to_point.push_back(Point2f(20, 450)); 

	//circle(image, Point(new_perspective[1].x,new_perspective[1].y), 5, Scalar(100,100,100), -1);

	Mat perspective_matrix=getPerspectiveTransform(new_perspective, perspective_to_point);
	warpPerspective(image, image_new, perspective_matrix, imageSize, INTER_LINEAR, BORDER_CONSTANT);
	
	Mat image_show = image_new(Rect(0,0,480,480));
		//imshow("Original",image);
	imshow("Perspective Transform from mouse clicking",image_show);
	
}
void CameraCalibrator::setFilename(){
    m_filenames.clear();
    m_filenames.push_back("1.bmp"); //新增元素至 vector 的尾端
   
    m_filenames.push_back("2.bmp"); 
    m_filenames.push_back("3.bmp"); 
    m_filenames.push_back("4.bmp"); 
    m_filenames.push_back("5.bmp"); 
    m_filenames.push_back("6.bmp"); 
    m_filenames.push_back("7.bmp"); 
    m_filenames.push_back("8.bmp"); 
    m_filenames.push_back("9.bmp"); 
    m_filenames.push_back("10.bmp"); 
    m_filenames.push_back("11.bmp"); 
    m_filenames.push_back("12.bmp"); 
    m_filenames.push_back("13.bmp"); 
    m_filenames.push_back("14.bmp");
	m_filenames.push_back("15.bmp");
} 
void CameraCalibrator::setFilename1(){
  

	
	 m_filenames.push_back("1.bmp");}
 
void CameraCalibrator::setFilename2(){
// m_filenames.clear();

m_filenames.push_back("2.bmp");}
void CameraCalibrator::setFilename3(){
// m_filenames.clear();

m_filenames.push_back("3.bmp");}
void CameraCalibrator::setFilename4(){
 //m_filenames.clear();

m_filenames.push_back("4.bmp");}
void CameraCalibrator::setFilename5(){
 //m_filenames.clear();

	m_filenames.push_back("5.bmp");
}

void CameraCalibrator::setBorderSize(const Size &borderSize){ 
    m_borderSize = borderSize; 
} 

void CameraCalibrator::addChessboardPoints(){  

    vector<Point2f> srcCandidateCorners; //vector可視為會自動擴展容量的陣列 Point2f：2維浮點數點類別,通常用於影像的座標點
    vector<Point3f> dstCandidateCorners; //vector可視為會自動擴展容量的陣列 Point3f：3維浮點數點類別,通常用於影像的座標點
    for(float i=0; i<m_borderSize.height; i++){ 
        for(float j=0; j<m_borderSize.width; j++){ 
            dstCandidateCorners.push_back(Point3f(i, j, 0.0f)); //定義世界座標系
        } 
    } 

    for(int i=0; i<m_filenames.size(); i++){ 
        Mat image=imread(m_filenames[i],CV_LOAD_IMAGE_GRAYSCALE); //用imread()讀取圖片，並將資料寫入Mat
		Mat image2;
		
        bool patternfound =findChessboardCorners(image, m_borderSize, srcCandidateCorners); //找棋盤角點
		if(patternfound){

			TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);//30：迭代次數的最大值,迭代在0.1的準確度下停止
		//要麼達到了最大疊代次數，要麼按達到某個閾值作為收斂結束條件

        cornerSubPix(image, srcCandidateCorners, Size(5,5), Size(-1,-1), param); //找精確角點,winSize：搜尋範圍，假設輸入Size(5,5)，則會搜尋5*2+1的範圍，也就是11×11的尺寸
		cvtColor(image, image2,CV_GRAY2BGR);
		drawChessboardCorners(image2, m_borderSize, srcCandidateCorners, patternfound);
		
		imwrite(c[i],image2);
		Mat Resizeimage;
		Size outSize;
		double fscale = 0.3;
		outSize.width = image2.cols * fscale;
		outSize.height = image2.rows * fscale;
		resize(image2, Resizeimage, outSize, 0, 0, INTER_AREA);
		imshow (c[i],Resizeimage);
		
		}
		
        if(srcCandidateCorners.size() == m_borderSize.area()){ //影像角點數量正確
            addPoints(srcCandidateCorners, dstCandidateCorners); 
        } 
    } 
} 
void CameraCalibrator::addChessboardPoints2(){  
    vector<Point2f> srcCandidateCorners; //vector可視為會自動擴展容量的陣列 Point2f：2維浮點數點類別,通常用於影像的座標點
    vector<Point3f> dstCandidateCorners; //vector可視為會自動擴展容量的陣列 Point3f：3維浮點數點類別,通常用於影像的座標點
    for(float i=0; i<m_borderSize.height; i++){ 
        for(float j=0; j<m_borderSize.width; j++){ 
            dstCandidateCorners.push_back(Point3f(i, j, 0.0f)); //定義世界座標系
        } 
    } 

    for(int i=0; i<m_filenames.size(); i++){ 
        Mat image=imread(m_filenames[i],CV_LOAD_IMAGE_GRAYSCALE); //用imread()讀取圖片，並將資料寫入Mat
		Mat image2;
		
        bool patternfound =findChessboardCorners(image, m_borderSize, srcCandidateCorners); //找棋盤角點
		if(patternfound){

			TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);//30：迭代次數的最大值,迭代在0.1的準確度下停止
		//要麼達到了最大疊代次數，要麼按達到某個閾值作為收斂結束條件

        cornerSubPix(image, srcCandidateCorners, Size(5,5), Size(-1,-1), param); //找精確角點,winSize：搜尋範圍，假設輸入Size(5,5)，則會搜尋5*2+1的範圍，也就是11×11的尺寸
		cvtColor(image, image2,CV_GRAY2BGR);
		drawChessboardCorners(image2, m_borderSize, srcCandidateCorners, patternfound);
		
		imwrite(c[i],image2);
		Mat Resizeimage;
		Size outSize;
		double fscale = 0.3;
		outSize.width = image2.cols * fscale;
		outSize.height = image2.rows * fscale;
		resize(image2, Resizeimage, outSize, 0, 0, INTER_AREA);
		imshow (c[i],Resizeimage);
		}
		
        if(srcCandidateCorners.size() == m_borderSize.area()){ //影像角點數量正確
            addPoints(srcCandidateCorners, dstCandidateCorners); 
        } 
    } 
} 
void CameraCalibrator::addPoints(const vector<Point2f> &srcCorners, const vector<Point3f> &dstCorners){           
    m_srcPoints.push_back(srcCorners);  
    m_dstPoints.push_back(dstCorners); 

} 
void CameraCalibrator::savedata(Mat &k){           
 a=k;

} 
void CameraCalibrator::printdata(){           
	cout << "extrinsic matrix = "<< endl << " "  << a << endl;

} 
void CameraCalibrator::calibrate( Mat &src, Mat &dst){ 
	
    Size imageSize = src.size(); 
    Mat  map1, map2; 
    vector<Mat> rvecs, tvecs; 
    calibrateCamera(m_dstPoints, m_srcPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);//求解出该相机的内参数(cameraMatrix)和每一个视角的外参数(rvecs, tvecs),矯正參數(distCoeffs)  

	
	
	Mat NewCameraMatrix=getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, imageSize,0,0);
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), NewCameraMatrix, imageSize, CV_32F, map1, map2); //Computes the undistortion and rectification transformation map. map1:OutputArray 
	
	
    remap(src, dst, map1, map2, INTER_LINEAR); 
	
	
	Mat rot_mat,tot_mat,A,B;
	  for(int i=0; i<m_filenames.size(); i++){ 
Rodrigues(rvecs[i], rot_mat);
A=Mat(tvecs[i]);

	  }
	
    hconcat(rot_mat,A,B); // 合併rot_mat與tvecs
	savedata(B);
	//printdata();
	Mat cam1=B*A1;//1,1,0
	Mat cam2=B*A2;//1,-1,0
	Mat cam3=B*A3;//-1,-1,0
	Mat cam4=B*A4;//-1,1,0
	Mat cam5=B*A5;//0,0,-2
	Mat cam6=B*A6;//-1,-1,0
	Mat cam7=B*A7;//-1,1,0
	Mat cam8=B*A8;//0,0,-2
	
	Mat Im1=NewCameraMatrix*cam1/cam1.at<double>(2,0);
	Mat Im2=NewCameraMatrix*cam2/cam2.at<double>(2,0);
	Mat Im3=NewCameraMatrix*cam3/cam3.at<double>(2,0);
	Mat Im4=NewCameraMatrix*cam4/cam4.at<double>(2,0);
	Mat Im5=NewCameraMatrix*cam5/cam5.at<double>(2,0);
	
	//std::cout<<"intrinsicMatrix"<<endl<<cameraMatrix<<endl;
	/*
	cout << "distCoeffs = "<< endl << " "  << distCoeffs << endl;
	cout << "rvecs = "<< endl << " "  << rot_mat << endl;
	cout << "tvecs = "<< endl << " "  << A << endl;
	cout << "extrinsic matrix = "<< endl << " "  << B << endl;
	cout << "A1 in image coordinate system = "<< endl << " "  << Im1 << endl;
	cout << "A2 in image coordinate system = "<< endl << " "  << Im2 << endl;
	cout << "A3 in image coordinate system = "<< endl << " "  << Im3 << endl;
	cout << "A4 in image coordinate system = "<< endl << " "  << Im4 << endl;
	cout << "A5 in image coordinate system = "<< endl << " "  << Im5 << endl;
	*/

	  //imshow("Original Image", src);//待矯正的照片
    double k1x=Im1.at<double>(0,0);
	double k1y=Im1.at<double>(1,0);
	double k2x=Im2.at<double>(0,0);
	double k2y=Im2.at<double>(1,0);
	double k3x=Im3.at<double>(0,0);
	double k3y=Im3.at<double>(1,0);
	double k4x=Im4.at<double>(0,0);
	double k4y=Im4.at<double>(1,0);
	double k5x=Im5.at<double>(0,0);
	double k5y=Im5.at<double>(1,0);
	Mat dst2;
	cvtColor(dst, dst2,CV_GRAY2BGR);
    line(dst2,Point(k1x,k1y),Point(k2x,k2y),Scalar(0,0,255),1,CV_AA);
	 line(dst2,Point(k2x,k2y),Point(k3x,k3y),Scalar(0,0,255),1,CV_AA);
	 line(dst2,Point(k3x,k3y),Point(k4x,k4y),Scalar(0,0,255),1,CV_AA);
	  line(dst2,Point(k4x,k4y),Point(k1x,k1y),Scalar(0,0,255),1,CV_AA);
	    line(dst2,Point(k1x,k1y),Point(k5x,k5y),Scalar(0,0,255),1,CV_AA);
	 line(dst2,Point(k2x,k2y),Point(k5x,k5y),Scalar(0,0,255),1,CV_AA);
	 line(dst2,Point(k3x,k3y),Point(k5x,k5y),Scalar(0,0,255),1,CV_AA);
	  line(dst2,Point(k4x,k4y),Point(k5x,k5y),Scalar(0,0,255),1,CV_AA);
	  /*
	imwrite("dst.jpg",dst2);
	
	
	
	imshow("Undistorted Image", dst2);//矯正後的輸出照片
	*/
	
}

void CameraCalibrator::calibrate2( Mat &src, Mat &dst){ 
	
    Size imageSize = src.size(); 
    Mat cameraMatrix, distCoeffs, map1, map2; 
    vector<Mat> rvecs, tvecs; 
    calibrateCamera(m_dstPoints, m_srcPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);//求解出该相机的内参数(cameraMatrix)和每一个视角的外参数(rvecs, tvecs),矯正參數(distCoeffs)  

	
	
	Mat NewCameraMatrix=getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, imageSize,0,0);
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), NewCameraMatrix, imageSize, CV_32F, map1, map2); //Computes the undistortion and rectification transformation map. map1:OutputArray 
	cout << "distCoeffs = "<< endl << " "  << distCoeffs << endl;
	/*
    remap(src, dst, map1, map2, INTER_LINEAR); 
	
	
	Mat rot_mat,tot_mat,A,B;
	  for(int i=0; i<m_filenames.size(); i++){ 
Rodrigues(rvecs[i], rot_mat);
A=Mat(tvecs[i]);

	  }
	
    hconcat(rot_mat,A,B); // 合併rot_mat與tvecs
	Mat cam1=B*A1;//1,1,0
	Mat cam2=B*A2;//1,-1,0
	Mat cam3=B*A3;//-1,-1,0
	Mat cam4=B*A4;//-1,1,0
	Mat cam5=B*A5;//0,0,-2
	Mat cam6=B*A6;//-1,-1,0
	Mat cam7=B*A7;//-1,1,0
	Mat cam8=B*A8;//0,0,-2
	Mat Im1=NewCameraMatrix*cam1/cam1.at<double>(2,0);
	Mat Im2=NewCameraMatrix*cam2/cam2.at<double>(2,0);
	Mat Im3=NewCameraMatrix*cam3/cam3.at<double>(2,0);
	Mat Im4=NewCameraMatrix*cam4/cam4.at<double>(2,0);
	Mat Im5=NewCameraMatrix*cam5/cam5.at<double>(2,0);
	
	*/
	
	
}
void CameraCalibrator::calibrate3( Mat &src, Mat &dst){ 
	
    Size imageSize = src.size(); 
    Mat cameraMatrix, distCoeffs, map1, map2; 
    vector<Mat> rvecs, tvecs; 
    calibrateCamera(m_dstPoints, m_srcPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);//求解出该相机的内参数(cameraMatrix)和每一个视角的外参数(rvecs, tvecs),矯正參數(distCoeffs)  

	
	
	Mat NewCameraMatrix=getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, imageSize,0,0);
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), NewCameraMatrix, imageSize, CV_32F, map1, map2); //Computes the undistortion and rectification transformation map. map1:OutputArray 
	
	
    remap(src, dst, map1, map2, INTER_LINEAR); 
	
	
	Mat rot_mat,tot_mat,A,B;
	  for(int i=0; i<m_filenames.size(); i++){ 
Rodrigues(rvecs[i], rot_mat);
A=Mat(tvecs[i]);

	  }
	
    hconcat(rot_mat,A,B); // 合併rot_mat與tvecs
	Mat cam1=B*A1;//1,1,0
	Mat cam2=B*A2;//1,-1,0
	Mat cam3=B*A3;//-1,-1,0
	Mat cam4=B*A4;//-1,1,0
	Mat cam5=B*A5;//0,0,-2
	Mat cam6=B*A6;//-1,-1,0
	Mat cam7=B*A7;//-1,1,0
	Mat cam8=B*A8;//0,0,-2
	Mat Im1=NewCameraMatrix*cam1/cam1.at<double>(2,0);
	Mat Im2=NewCameraMatrix*cam2/cam2.at<double>(2,0);
	Mat Im3=NewCameraMatrix*cam3/cam3.at<double>(2,0);
	Mat Im4=NewCameraMatrix*cam4/cam4.at<double>(2,0);
	Mat Im5=NewCameraMatrix*cam5/cam5.at<double>(2,0);
	Mat Im6=NewCameraMatrix*cam6/cam6.at<double>(2,0);
	Mat Im7=NewCameraMatrix*cam7/cam7.at<double>(2,0);
	Mat Im8=NewCameraMatrix*cam8/cam8.at<double>(2,0);
	cout << "distCoeffs = "<< endl << " "  << distCoeffs << endl;
	cout << "extrinsic matrix = "<< endl << " "  << B << endl;
	/*
	std::cout<<"intrinsicMatrix"<<endl<<cameraMatrix<<endl;
	cout << "rvecs = "<< endl << " "  << rot_mat << endl;
	cout << "tvecs = "<< endl << " "  << A << endl;
	cout << "extrinsic matrix = "<< endl << " "  << B << endl;
	cout << "A1 in image coordinate system = "<< endl << " "  << Im1 << endl;
	cout << "A2 in image coordinate system = "<< endl << " "  << Im2 << endl;
	cout << "A3 in image coordinate system = "<< endl << " "  << Im3 << endl;
	cout << "A4 in image coordinate system = "<< endl << " "  << Im4 << endl;
	cout << "A5 in image coordinate system = "<< endl << " "  << Im5 << endl;
	*/

	 // imshow("Original Image", src);//待矯正的照片
    double k1x=Im1.at<double>(0,0);
	double k1y=Im1.at<double>(1,0);
	double k2x=Im2.at<double>(0,0);
	double k2y=Im2.at<double>(1,0);
	double k3x=Im3.at<double>(0,0);
	double k3y=Im3.at<double>(1,0);
	double k4x=Im4.at<double>(0,0);
	double k4y=Im4.at<double>(1,0);
	double k5x=Im5.at<double>(0,0);
	double k5y=Im5.at<double>(1,0);
	double k6x=Im6.at<double>(0,0);
	double k6y=Im6.at<double>(1,0);
	double k7x=Im7.at<double>(0,0);
	double k7y=Im7.at<double>(1,0);
	double k8x=Im8.at<double>(0,0);
	double k8y=Im8.at<double>(1,0);
	Mat dst2;
	cvtColor(dst, dst2,CV_GRAY2BGR);
    line(dst2,Point(k1x,k1y),Point(k2x,k2y),Scalar(0,0,255),1,CV_AA);
	 line(dst2,Point(k2x,k2y),Point(k3x,k3y),Scalar(0,0,255),1,CV_AA);
	 line(dst2,Point(k3x,k3y),Point(k4x,k4y),Scalar(0,0,255),1,CV_AA);
	 line(dst2,Point(k4x,k4y),Point(k1x,k1y),Scalar(0,0,255),1,CV_AA);
	  line(dst2,Point(k4x,k4y),Point(k6x,k6y),Scalar(0,0,255),1,CV_AA);
	    line(dst2,Point(k1x,k1y),Point(k7x,k7y),Scalar(0,0,255),1,CV_AA);
	line(dst2,Point(k2x,k2y),Point(k8x,k8y),Scalar(0,0,255),1,CV_AA);
	     line(dst2,Point(k3x,k3y),Point(k5x,k5y),Scalar(0,0,255),1,CV_AA);
	line(dst2,Point(k5x,k5y),Point(k8x,k8y),Scalar(0,0,255),1,CV_AA);
	  line(dst2,Point(k8x,k8y),Point(k7x,k7y),Scalar(0,0,255),1,CV_AA);
	 line(dst2,Point(k7x,k7y),Point(k6x,k6y),Scalar(0,0,255),1,CV_AA);
	 line(dst2,Point(k6x,k6y),Point(k5x,k5y),Scalar(0,0,255),1,CV_AA);
	  
	imwrite("dst.jpg",dst2);
	Mat Resizeimage;
		Size outSize;
		double fscale = 0.3;
		outSize.width = dst2.cols * fscale;
		outSize.height = dst2.rows * fscale;
		resize(dst2, Resizeimage, outSize, 0, 0, INTER_AREA);
	
	
	imshow("Undistorted Image", Resizeimage);//矯正後的輸出照片
	
	
}
void CameraCalibrator::extrinsic(vector<string> &b){
	//char str1[] = {"1.bmp","1.bmp"};
	//, "2.bmp", "3.bmp", "4.bmp", "1.bmp", "1.bmp","1.bmp", "1.bmp", "1.bmp", "1.bmp", "1.bmp", "1.bmp",, "1.bmp", "1.bmp", "1.bmp"};
	//pic.push_back(b);
	 Mat image;
	for(float j=0; j<b.size(); j++){ 
           image=imread(b[j],CV_LOAD_IMAGE_UNCHANGED); 
        } 
   

	Mat image_new =Mat:: zeros(image.rows, image.cols, image.type()) ;
	Size boardsize= Size(11,8);

	vector<Point2f> srcCandidateCorners; 
	vector<Point3f> dstCandidateCorners;

	vector<vector<Point2f> > perpspective_srcPoints;
    vector<vector<Point3f> > perpspective_dstPoints;

	vector<Point2f> perpspective_4_srcPoints;
    vector<Point3f> perpspective_4_dstPoints;

	 Mat cameraMatrix_perspective, distCoeffs_perspective; 
	 vector<Mat> rvecs_perspective, tvecs_perspective; 



	for(int i=0; i<boardsize.height; i++){ 
        for(int j=0; j<boardsize.width; j++){ 
            dstCandidateCorners.push_back(Point3f(i, j, 0.0f)); 
        } 
    } 



	bool find=findChessboardCorners(image, boardsize, srcCandidateCorners); 		
    TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
    //cornerSubPix(image, srcCandidateCorners, Size(5,5), Size(-1,-1), param);  


	


        if(srcCandidateCorners.size() == boardsize.area()){ 

			 perpspective_srcPoints.push_back(srcCandidateCorners);  
             perpspective_dstPoints.push_back(dstCandidateCorners); 
           
        } 

		/*
		for(int i =0 ;i < perpspective_srcPoints.size();i++)
			cout<< perpspective_srcPoints[i]<<endl;
		for(int i =0 ;i < perpspective_dstPoints.size();i++)
			cout<< perpspective_dstPoints[i]<<endl;
*/

	Size imageSize = image.size(); 

	calibrateCamera(perpspective_dstPoints, perpspective_srcPoints, imageSize, cameraMatrix_perspective, distCoeffs_perspective, rvecs_perspective, tvecs_perspective);
	extrinsicmatrix=cameraMatrix_perspective;
	
}

void CameraCalibrator::cube(vector<string> &cubefunction,Mat &dstnew){
		 Mat image;
	for(float j=0; j<cubefunction.size(); j++){ 
           image=imread(cubefunction[j],CV_LOAD_IMAGE_UNCHANGED); 
        } 
   
	
	Mat image_new =Mat:: zeros(image.rows, image.cols, image.type()) ;
	Size boardsize= Size(11,8);

	vector<Point2f> srcCandidateCorners; 
	vector<Point3f> dstCandidateCorners;

	vector<vector<Point2f> > perpspective_srcPoints;
    vector<vector<Point3f> > perpspective_dstPoints;

	vector<Point2f> perpspective_4_srcPoints;
    vector<Point3f> perpspective_4_dstPoints;

	 Mat cameraMatrix_perspective, distCoeffs_perspective; 
	 vector<Mat> rvecs, tvecs; 
	
	 

	for(int i=0; i<boardsize.height; i++){ 
        for(int j=0; j<boardsize.width; j++){ 
            dstCandidateCorners.push_back(Point3f(i, j, 0.0f)); 
        } 
    } 


		
	bool find=findChessboardCorners(image, boardsize, srcCandidateCorners); 		
  TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
    //cornerSubPix(image, srcCandidateCorners, Size(5,5), Size(-1,-1), param);  
	Mat map1,map2;

	
 
        if(srcCandidateCorners.size() == boardsize.area()){ 

			 perpspective_srcPoints.push_back(srcCandidateCorners);  
             perpspective_dstPoints.push_back(dstCandidateCorners); 
           
        } 

	Size imageSize = image.size(); 

	calibrateCamera(perpspective_dstPoints, perpspective_srcPoints, imageSize, cameraMatrix_perspective, distCoeffs_perspective, rvecs, tvecs);
 
	Mat NewCameraMatrix=getOptimalNewCameraMatrix(cameraMatrix_perspective, distCoeffs_perspective, imageSize, 0, imageSize,0,0);
    initUndistortRectifyMap(cameraMatrix_perspective, distCoeffs_perspective, Mat(), NewCameraMatrix, imageSize, CV_32F, map1, map2); //Computes the undistortion and rectification transformation map. map1:OutputArray 
	
	
    remap(image, dstnew, map1, map2, INTER_LINEAR); 

 	
	
	Mat rot_mat,tot_mat,B,A;
	  for(int i=0; i<cubefunction.size(); i++){ 
Rodrigues(rvecs[i], rot_mat);
A= Mat(tvecs[i]);;


	  }
	 hconcat(rot_mat,A,B);
  // 合併rot_mat與tvecs
	Mat cam1=B*A1;//1,1,0
 	Mat cam2=B*A2;//1,-1,0
	Mat cam3=B*A3;//-1,-1,0
	Mat cam4=B*A4;//-1,1,0
	Mat cam5=B*A5;//0,0,-2
	Mat cam6=B*A6;//-1,-1,0
	Mat cam7=B*A7;//-1,1,0
	Mat cam8=B*A8;//0,0,-2
	Mat Im1=NewCameraMatrix*cam1/cam1.at<double>(2,0);
	Mat Im2=NewCameraMatrix*cam2/cam2.at<double>(2,0);
	Mat Im3=NewCameraMatrix*cam3/cam3.at<double>(2,0);
	Mat Im4=NewCameraMatrix*cam4/cam4.at<double>(2,0);
	Mat Im5=NewCameraMatrix*cam5/cam5.at<double>(2,0);
	Mat Im6=NewCameraMatrix*cam6/cam6.at<double>(2,0);
	Mat Im7=NewCameraMatrix*cam7/cam7.at<double>(2,0);
	Mat Im8=NewCameraMatrix*cam8/cam8.at<double>(2,0);
	
    double k1x=Im1.at<double>(0,0);
	double k1y=Im1.at<double>(1,0);
	double k2x=Im2.at<double>(0,0);
	double k2y=Im2.at<double>(1,0);
	double k3x=Im3.at<double>(0,0);
	double k3y=Im3.at<double>(1,0);
	double k4x=Im4.at<double>(0,0);
	double k4y=Im4.at<double>(1,0);
	double k5x=Im5.at<double>(0,0);
	double k5y=Im5.at<double>(1,0);
	double k6x=Im6.at<double>(0,0);
	double k6y=Im6.at<double>(1,0);
	double k7x=Im7.at<double>(0,0);
	double k7y=Im7.at<double>(1,0);
	double k8x=Im8.at<double>(0,0);
	double k8y=Im8.at<double>(1,0);
	Mat dst3;
  	//cvtColor(dst, dst3,CV_GRAY2BGR);
    line(dst3,Point(k1x,k1y),Point(k2x,k2y),Scalar(0,0,255),1,CV_AA);
	 line(dst3,Point(k2x,k2y),Point(k3x,k3y),Scalar(0,0,255),1,CV_AA);
	 line(dst3,Point(k3x,k3y),Point(k4x,k4y),Scalar(0,0,255),1,CV_AA);
	 line(dst3,Point(k4x,k4y),Point(k1x,k1y),Scalar(0,0,255),1,CV_AA);
	  line(dst3,Point(k4x,k4y),Point(k6x,k6y),Scalar(0,0,255),1,CV_AA);
	    line(dst3,Point(k1x,k1y),Point(k7x,k7y),Scalar(0,0,255),1,CV_AA);
	line(dst3,Point(k2x,k2y),Point(k8x,k8y),Scalar(0,0,255),1,CV_AA);
	     line(dst3,Point(k3x,k3y),Point(k5x,k5y),Scalar(0,0,255),1,CV_AA);
	line(dst3,Point(k5x,k5y),Point(k8x,k8y),Scalar(0,0,255),1,CV_AA);
	  line(dst3,Point(k8x,k8y),Point(k7x,k7y),Scalar(0,0,255),1,CV_AA);
	 line(dst3,Point(k7x,k7y),Point(k6x,k6y),Scalar(0,0,255),1,CV_AA);
	 line(dst3,Point(k6x,k6y),Point(k5x,k5y),Scalar(0,0,255),1,CV_AA);
	
	imwrite("newphoto.bmp",dst3);
/*	Mat Resizeimage;
		Size outSize;
		double fscale = 0.3;
		outSize.width = dst2.cols * fscale;
		outSize.height = dst2.rows * fscale;
		resize(dst2, Resizeimage, outSize, 0, 0, INTER_AREA);

	
	imshow("Undistorted Image", Resizeimage);//矯正後的輸出照片
		
	imshow("Image new", dst3);*/
}
void CameraCalibrator::calibrate_AR(const Mat &src, Mat &dst){ 

    Size imageSize = src.size(); 
    Mat cameraMatrix, distCoeffs, map1, map2,newCameraMatrix; 
    vector<Mat> rvecs, tvecs; 
    calibrateCamera(m_dstPoints, m_srcPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);  
	//Mat test = imread(dst);

	std::cout << std::fixed;
//    std::cout << std::setprecision(6);
	//std::cout<< std::setprecision(6)<<"cameraMatrix"<<endl<<cameraMatrix<<endl;
	//std::cout<<"distCoeffs"<<endl<<distCoeffs<<endl;
	//cout<<"rvecs"<<rvecs[0]<<endl;
	//cout<<"rvecs"<<rvecs[1]<<endl;
	for(int i=0; i<m_filenames.size(); i++){
		Rodrigues( rvecs[i],  dst);
		//cout<<"rotation matrix of pic"<<i+1<<endl<<dst<<endl;
		//cout<<"tvecs of pic"<<i+1<<endl << tvecs[i]<<endl;
		Mat H ;
        hconcat(dst, tvecs[i], H);
		//cout << "Extrinsic Matrix  = " <<i+1<< endl << " " << H << endl << endl;
        
	    Mat P1 = (Mat_<double>(4,1) << 0, 0, 0, 1);
		Mat P2 = (Mat_<double>(4,1) << 2, 0, 0, 1);
		Mat P3 = (Mat_<double>(4,1) << 2, 2, 0, 1);
		Mat P4 = (Mat_<double>(4,1) << 0, 2, 0, 1);
		Mat P5 = (Mat_<double>(4,1) << 0, 0, 2, 1);
		Mat P6 = (Mat_<double>(4,1) << 2, 0, 2, 1);
		Mat P7 = (Mat_<double>(4,1) << 2, 2, 2, 1);
		Mat P8 = (Mat_<double>(4,1) << 0, 2, 2, 1);
		//cout<<"test"<<P1<<endl;

		initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix,distCoeffs, imageSize,  0, imageSize, 0, 0 ) , imageSize, CV_32F, map1, map2);

	    Mat a1 = newCameraMatrix*H*P1;
		cout<<a1<<endl;
		Mat A1 = a1/a1.at<double>(2,0);
		cout<<A1<<endl;
		 X1 = A1.at<double>(0,0);
		 Y1 = A1.at<double>(1,0);
		//cout<<A1<<endl<<X1<<endl<<Y1<<endl;
		Mat a2 = newCameraMatrix*H*P2;
		Mat A2 = a2/a2.at<double>(2,0);
		 X2 = A2.at<double>(0,0);
		 Y2 = A2.at<double>(1,0);
		Mat a3 = newCameraMatrix*H*P3;
		Mat A3 = a3/a3.at<double>(2,0);
		 X3 = A3.at<double>(0,0);
		 Y3 = A3.at<double>(1,0);
		Mat a4 = newCameraMatrix*H*P4;
		Mat A4 = a4/a4.at<double>(2,0);
		 X4 = A4.at<double>(0,0);
		Y4 = A4.at<double>(1,0);
		Mat a5 = newCameraMatrix*H*P5;
		Mat A5 = a5/a5.at<double>(2,0);
		 X5 = A5.at<double>(0,0);
		 Y5 = A5.at<double>(1,0);


		Mat a6 = newCameraMatrix*H*P6;
		Mat A6 = a6/a6.at<double>(2,0);
		 X6 = A6.at<double>(0,0);
		 Y6 = A6.at<double>(1,0);

		Mat a7 = newCameraMatrix*H*P7;
		Mat A7 = a7/a7.at<double>(2,0);
		 X7 = A7.at<double>(0,0);
		 Y7 = A7.at<double>(1,0);

		Mat a8 = newCameraMatrix*H*P8;
		Mat A8 = a8/a8.at<double>(2,0);
		 X8 = A8.at<double>(0,0);
		 Y8 = A8.at<double>(1,0);
		
        remap(src, dst, map1, map2, INTER_LINEAR); 
		Mat dst3;
  	cvtColor(dst, dst3,CV_GRAY2BGR);
		if(i==0){
			
	        
				line(dst3, Point2d(X1,Y1), Point2d(X2,Y2), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X2,Y2), Point2d(X3,Y3), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X3,Y3), Point2d(X4,Y4), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X4,Y4), Point2d(X1,Y1), Scalar(0,0,255), 2, 8);

				line(dst3, Point2d(X5,Y5), Point2d(X6,Y6), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X6,Y6), Point2d(X7,Y7), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X7,Y7), Point2d(X8,Y8), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X8,Y8), Point2d(X5,Y5), Scalar(0,0,255), 2, 8);

				line(dst3, Point2d(X1,Y1), Point2d(X5,Y5), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X2,Y2), Point2d(X6,Y6), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X3,Y3), Point2d(X7,Y7), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X4,Y4), Point2d(X8,Y8), Scalar(0,0,255), 2, 8);

				Mat Resizeimage;
				Size outSize;
				double fscale = 0.3;
				outSize.width = dst3.cols * fscale;
				outSize.height = dst3.rows * fscale;
				resize(dst3, Resizeimage, outSize, 0, 0, INTER_AREA);
				
				imshow("AR1", Resizeimage);
			}
	}
}
void CameraCalibrator::calibrate_AR1(const Mat &src, Mat &dst){ 

    Size imageSize = src.size(); 
    Mat cameraMatrix, distCoeffs, map1, map2,newCameraMatrix; 
    vector<Mat> rvecs, tvecs; 
    calibrateCamera(m_dstPoints, m_srcPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);  
	//Mat test = imread(dst);

	std::cout << std::fixed;
//    std::cout << std::setprecision(6);
	//std::cout<< std::setprecision(6)<<"cameraMatrix"<<endl<<cameraMatrix<<endl;
	//std::cout<<"distCoeffs"<<endl<<distCoeffs<<endl;
	//cout<<"rvecs"<<rvecs[0]<<endl;
	//cout<<"rvecs"<<rvecs[1]<<endl;
	for(int i=0; i<m_filenames.size(); i++){
		Rodrigues( rvecs[i],  dst);
		//cout<<"rotation matrix of pic"<<i+1<<endl<<dst<<endl;
		//cout<<"tvecs of pic"<<i+1<<endl << tvecs[i]<<endl;
		Mat H ;
        hconcat(dst, tvecs[i], H);
		//cout << "Extrinsic Matrix  = " <<i+1<< endl << " " << H << endl << endl;
        
	    Mat P1 = (Mat_<double>(4,1) << 0, 0, 0, 1);
		Mat P2 = (Mat_<double>(4,1) << 2, 0, 0, 1);
		Mat P3 = (Mat_<double>(4,1) << 2, 2, 0, 1);
		Mat P4 = (Mat_<double>(4,1) << 0, 2, 0, 1);
		Mat P5 = (Mat_<double>(4,1) << 0, 0, 2, 1);
		Mat P6 = (Mat_<double>(4,1) << 2, 0, 2, 1);
		Mat P7 = (Mat_<double>(4,1) << 2, 2, 2, 1);
		Mat P8 = (Mat_<double>(4,1) << 0, 2, 2, 1);
		//cout<<"test"<<P1<<endl;

		initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix,distCoeffs, imageSize,  0, imageSize, 0, 0 ) , imageSize, CV_32F, map1, map2);

	    Mat a1 = newCameraMatrix*H*P1;
		Mat A1 = a1/a1.at<double>(2,0);
		 X1 = A1.at<double>(0,0);
		 Y1 = A1.at<double>(1,0);
		//cout<<A1<<endl<<X1<<endl<<Y1<<endl;
		Mat a2 = newCameraMatrix*H*P2;
		Mat A2 = a2/a2.at<double>(2,0);
		 X2 = A2.at<double>(0,0);
		 Y2 = A2.at<double>(1,0);
		Mat a3 = newCameraMatrix*H*P3;
		Mat A3 = a3/a3.at<double>(2,0);
		 X3 = A3.at<double>(0,0);
		 Y3 = A3.at<double>(1,0);
		Mat a4 = newCameraMatrix*H*P4;
		Mat A4 = a4/a4.at<double>(2,0);
		 X4 = A4.at<double>(0,0);
		Y4 = A4.at<double>(1,0);
		Mat a5 = newCameraMatrix*H*P5;
		Mat A5 = a5/a5.at<double>(2,0);
		 X5 = A5.at<double>(0,0);
		 Y5 = A5.at<double>(1,0);


		Mat a6 = newCameraMatrix*H*P6;
		Mat A6 = a6/a6.at<double>(2,0);
		 X6 = A6.at<double>(0,0);
		 Y6 = A6.at<double>(1,0);

		Mat a7 = newCameraMatrix*H*P7;
		Mat A7 = a7/a7.at<double>(2,0);
		 X7 = A7.at<double>(0,0);
		 Y7 = A7.at<double>(1,0);

		Mat a8 = newCameraMatrix*H*P8;
		Mat A8 = a8/a8.at<double>(2,0);
		 X8 = A8.at<double>(0,0);
		 Y8 = A8.at<double>(1,0);
		
        remap(src, dst, map1, map2, INTER_LINEAR); 
		Mat dst3;
  	cvtColor(dst, dst3,CV_GRAY2BGR);
		if(i==1){
			
	        
				line(dst3, Point2d(X1,Y1), Point2d(X2,Y2), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X2,Y2), Point2d(X3,Y3), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X3,Y3), Point2d(X4,Y4), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X4,Y4), Point2d(X1,Y1), Scalar(0,0,255), 2, 8);

				line(dst3, Point2d(X5,Y5), Point2d(X6,Y6), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X6,Y6), Point2d(X7,Y7), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X7,Y7), Point2d(X8,Y8), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X8,Y8), Point2d(X5,Y5), Scalar(0,0,255), 2, 8);

				line(dst3, Point2d(X1,Y1), Point2d(X5,Y5), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X2,Y2), Point2d(X6,Y6), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X3,Y3), Point2d(X7,Y7), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X4,Y4), Point2d(X8,Y8), Scalar(0,0,255), 2, 8);

				Mat Resizeimage;
				Size outSize;
				double fscale = 0.3;
				outSize.width = dst3.cols * fscale;
				outSize.height = dst3.rows * fscale;
				resize(dst3, Resizeimage, outSize, 0, 0, INTER_AREA);
				
				imshow("AR2", Resizeimage);
			}
	}
}
void CameraCalibrator::calibrate_AR2(const Mat &src, Mat &dst){ 

    Size imageSize = src.size(); 
    Mat cameraMatrix, distCoeffs, map1, map2,newCameraMatrix; 
    vector<Mat> rvecs, tvecs; 
    calibrateCamera(m_dstPoints, m_srcPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);  
	//Mat test = imread(dst);

	std::cout << std::fixed;
//    std::cout << std::setprecision(6);
	//std::cout<< std::setprecision(6)<<"cameraMatrix"<<endl<<cameraMatrix<<endl;
	//std::cout<<"distCoeffs"<<endl<<distCoeffs<<endl;
	//cout<<"rvecs"<<rvecs[0]<<endl;
	//cout<<"rvecs"<<rvecs[1]<<endl;
	for(int i=0; i<m_filenames.size(); i++){
		Rodrigues( rvecs[i],  dst);
		//cout<<"rotation matrix of pic"<<i+1<<endl<<dst<<endl;
		//cout<<"tvecs of pic"<<i+1<<endl << tvecs[i]<<endl;
		Mat H ;
        hconcat(dst, tvecs[i], H);
		//cout << "Extrinsic Matrix  = " <<i+1<< endl << " " << H << endl << endl;
        
	    Mat P1 = (Mat_<double>(4,1) << 0, 0, 0, 1);
		Mat P2 = (Mat_<double>(4,1) << 2, 0, 0, 1);
		Mat P3 = (Mat_<double>(4,1) << 2, 2, 0, 1);
		Mat P4 = (Mat_<double>(4,1) << 0, 2, 0, 1);
		Mat P5 = (Mat_<double>(4,1) << 0, 0, 2, 1);
		Mat P6 = (Mat_<double>(4,1) << 2, 0, 2, 1);
		Mat P7 = (Mat_<double>(4,1) << 2, 2, 2, 1);
		Mat P8 = (Mat_<double>(4,1) << 0, 2, 2, 1);
		//cout<<"test"<<P1<<endl;

		initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix,distCoeffs, imageSize,  0, imageSize, 0, 0 ) , imageSize, CV_32F, map1, map2);

	    Mat a1 = newCameraMatrix*H*P1;
		Mat A1 = a1/a1.at<double>(2,0);
		 X1 = A1.at<double>(0,0);
		 Y1 = A1.at<double>(1,0);
		//cout<<A1<<endl<<X1<<endl<<Y1<<endl;
		Mat a2 = newCameraMatrix*H*P2;
		Mat A2 = a2/a2.at<double>(2,0);
		 X2 = A2.at<double>(0,0);
		 Y2 = A2.at<double>(1,0);
		Mat a3 = newCameraMatrix*H*P3;
		Mat A3 = a3/a3.at<double>(2,0);
		 X3 = A3.at<double>(0,0);
		 Y3 = A3.at<double>(1,0);
		Mat a4 = newCameraMatrix*H*P4;
		Mat A4 = a4/a4.at<double>(2,0);
		 X4 = A4.at<double>(0,0);
		Y4 = A4.at<double>(1,0);
		Mat a5 = newCameraMatrix*H*P5;
		Mat A5 = a5/a5.at<double>(2,0);
		 X5 = A5.at<double>(0,0);
		 Y5 = A5.at<double>(1,0);


		Mat a6 = newCameraMatrix*H*P6;
		Mat A6 = a6/a6.at<double>(2,0);
		 X6 = A6.at<double>(0,0);
		 Y6 = A6.at<double>(1,0);

		Mat a7 = newCameraMatrix*H*P7;
		Mat A7 = a7/a7.at<double>(2,0);
		 X7 = A7.at<double>(0,0);
		 Y7 = A7.at<double>(1,0);

		Mat a8 = newCameraMatrix*H*P8;
		Mat A8 = a8/a8.at<double>(2,0);
		 X8 = A8.at<double>(0,0);
		 Y8 = A8.at<double>(1,0);
		
        remap(src, dst, map1, map2, INTER_LINEAR); 
		Mat dst3;
  	cvtColor(dst, dst3,CV_GRAY2BGR);
		if(i==2){
			
	        
				line(dst3, Point2d(X1,Y1), Point2d(X2,Y2), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X2,Y2), Point2d(X3,Y3), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X3,Y3), Point2d(X4,Y4), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X4,Y4), Point2d(X1,Y1), Scalar(0,0,255), 2, 8);

				line(dst3, Point2d(X5,Y5), Point2d(X6,Y6), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X6,Y6), Point2d(X7,Y7), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X7,Y7), Point2d(X8,Y8), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X8,Y8), Point2d(X5,Y5), Scalar(0,0,255), 2, 8);

				line(dst3, Point2d(X1,Y1), Point2d(X5,Y5), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X2,Y2), Point2d(X6,Y6), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X3,Y3), Point2d(X7,Y7), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X4,Y4), Point2d(X8,Y8), Scalar(0,0,255), 2, 8);

				Mat Resizeimage;
				Size outSize;
				double fscale = 0.3;
				outSize.width = dst3.cols * fscale;
				outSize.height = dst3.rows * fscale;
				resize(dst3, Resizeimage, outSize, 0, 0, INTER_AREA);
				
				imshow("AR3", Resizeimage);
			}
	}
}
void CameraCalibrator::calibrate_AR3(const Mat &src, Mat &dst){ 

    Size imageSize = src.size(); 
    Mat cameraMatrix, distCoeffs, map1, map2,newCameraMatrix; 
    vector<Mat> rvecs, tvecs; 
    calibrateCamera(m_dstPoints, m_srcPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);  
	//Mat test = imread(dst);

	std::cout << std::fixed;
//    std::cout << std::setprecision(6);
	//std::cout<< std::setprecision(6)<<"cameraMatrix"<<endl<<cameraMatrix<<endl;
	//std::cout<<"distCoeffs"<<endl<<distCoeffs<<endl;
	//cout<<"rvecs"<<rvecs[0]<<endl;
	//cout<<"rvecs"<<rvecs[1]<<endl;
	for(int i=0; i<m_filenames.size(); i++){
		Rodrigues( rvecs[i],  dst);
		//cout<<"rotation matrix of pic"<<i+1<<endl<<dst<<endl;
		//cout<<"tvecs of pic"<<i+1<<endl << tvecs[i]<<endl;
		Mat H ;
        hconcat(dst, tvecs[i], H);
		//cout << "Extrinsic Matrix  = " <<i+1<< endl << " " << H << endl << endl;
        
	    Mat P1 = (Mat_<double>(4,1) << 0, 0, 0, 1);
		Mat P2 = (Mat_<double>(4,1) << 2, 0, 0, 1);
		Mat P3 = (Mat_<double>(4,1) << 2, 2, 0, 1);
		Mat P4 = (Mat_<double>(4,1) << 0, 2, 0, 1);
		Mat P5 = (Mat_<double>(4,1) << 0, 0, 2, 1);
		Mat P6 = (Mat_<double>(4,1) << 2, 0, 2, 1);
		Mat P7 = (Mat_<double>(4,1) << 2, 2, 2, 1);
		Mat P8 = (Mat_<double>(4,1) << 0, 2, 2, 1);
		//cout<<"test"<<P1<<endl;

		initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix,distCoeffs, imageSize,  0, imageSize, 0, 0 ) , imageSize, CV_32F, map1, map2);

	    Mat a1 = newCameraMatrix*H*P1;
		Mat A1 = a1/a1.at<double>(2,0);
		 X1 = A1.at<double>(0,0);
		 Y1 = A1.at<double>(1,0);
		//cout<<A1<<endl<<X1<<endl<<Y1<<endl;
		Mat a2 = newCameraMatrix*H*P2;
		Mat A2 = a2/a2.at<double>(2,0);
		 X2 = A2.at<double>(0,0);
		 Y2 = A2.at<double>(1,0);
		Mat a3 = newCameraMatrix*H*P3;
		Mat A3 = a3/a3.at<double>(2,0);
		 X3 = A3.at<double>(0,0);
		 Y3 = A3.at<double>(1,0);
		Mat a4 = newCameraMatrix*H*P4;
		Mat A4 = a4/a4.at<double>(2,0);
		 X4 = A4.at<double>(0,0);
		Y4 = A4.at<double>(1,0);
		Mat a5 = newCameraMatrix*H*P5;
		Mat A5 = a5/a5.at<double>(2,0);
		 X5 = A5.at<double>(0,0);
		 Y5 = A5.at<double>(1,0);


		Mat a6 = newCameraMatrix*H*P6;
		Mat A6 = a6/a6.at<double>(2,0);
		 X6 = A6.at<double>(0,0);
		 Y6 = A6.at<double>(1,0);

		Mat a7 = newCameraMatrix*H*P7;
		Mat A7 = a7/a7.at<double>(2,0);
		 X7 = A7.at<double>(0,0);
		 Y7 = A7.at<double>(1,0);

		Mat a8 = newCameraMatrix*H*P8;
		Mat A8 = a8/a8.at<double>(2,0);
		 X8 = A8.at<double>(0,0);
		 Y8 = A8.at<double>(1,0);
		
        remap(src, dst, map1, map2, INTER_LINEAR); 
		Mat dst3;
  	cvtColor(dst, dst3,CV_GRAY2BGR);
		if(i==3){
			
	        
				line(dst3, Point2d(X1,Y1), Point2d(X2,Y2), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X2,Y2), Point2d(X3,Y3), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X3,Y3), Point2d(X4,Y4), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X4,Y4), Point2d(X1,Y1), Scalar(0,0,255), 2, 8);

				line(dst3, Point2d(X5,Y5), Point2d(X6,Y6), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X6,Y6), Point2d(X7,Y7), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X7,Y7), Point2d(X8,Y8), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X8,Y8), Point2d(X5,Y5), Scalar(0,0,255), 2, 8);

				line(dst3, Point2d(X1,Y1), Point2d(X5,Y5), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X2,Y2), Point2d(X6,Y6), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X3,Y3), Point2d(X7,Y7), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X4,Y4), Point2d(X8,Y8), Scalar(0,0,255), 2, 8);

				Mat Resizeimage;
				Size outSize;
				double fscale = 0.3;
				outSize.width = dst3.cols * fscale;
				outSize.height = dst3.rows * fscale;
				resize(dst3, Resizeimage, outSize, 0, 0, INTER_AREA);
				
				imshow("AR4", Resizeimage);
			}
	}
}
void CameraCalibrator::calibrate_AR4(const Mat &src, Mat &dst){ 

    Size imageSize = src.size(); 
    Mat cameraMatrix, distCoeffs, map1, map2,newCameraMatrix; 
    vector<Mat> rvecs, tvecs; 
    calibrateCamera(m_dstPoints, m_srcPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);  
	//Mat test = imread(dst);

	std::cout << std::fixed;
//    std::cout << std::setprecision(6);
	//std::cout<< std::setprecision(6)<<"cameraMatrix"<<endl<<cameraMatrix<<endl;
	//std::cout<<"distCoeffs"<<endl<<distCoeffs<<endl;
	//cout<<"rvecs"<<rvecs[0]<<endl;
	//cout<<"rvecs"<<rvecs[1]<<endl;
	for(int i=0; i<m_filenames.size(); i++){
		Rodrigues( rvecs[i],  dst);
		//cout<<"rotation matrix of pic"<<i+1<<endl<<dst<<endl;
		//cout<<"tvecs of pic"<<i+1<<endl << tvecs[i]<<endl;
		Mat H ;
        hconcat(dst, tvecs[i], H);
		//cout << "Extrinsic Matrix  = " <<i+1<< endl << " " << H << endl << endl;
        
	    Mat P1 = (Mat_<double>(4,1) << 0, 0, 0, 1);
		Mat P2 = (Mat_<double>(4,1) << 2, 0, 0, 1);
		Mat P3 = (Mat_<double>(4,1) << 2, 2, 0, 1);
		Mat P4 = (Mat_<double>(4,1) << 0, 2, 0, 1);
		Mat P5 = (Mat_<double>(4,1) << 0, 0, 2, 1);
		Mat P6 = (Mat_<double>(4,1) << 2, 0, 2, 1);
		Mat P7 = (Mat_<double>(4,1) << 2, 2, 2, 1);
		Mat P8 = (Mat_<double>(4,1) << 0, 2, 2, 1);
		//cout<<"test"<<P1<<endl;

		initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix,distCoeffs, imageSize,  0, imageSize, 0, 0 ) , imageSize, CV_32F, map1, map2);

	    Mat a1 = newCameraMatrix*H*P1;
		Mat A1 = a1/a1.at<double>(2,0);
		 X1 = A1.at<double>(0,0);
		 Y1 = A1.at<double>(1,0);
		//cout<<A1<<endl<<X1<<endl<<Y1<<endl;
		Mat a2 = newCameraMatrix*H*P2;
		Mat A2 = a2/a2.at<double>(2,0);
		 X2 = A2.at<double>(0,0);
		 Y2 = A2.at<double>(1,0);
		Mat a3 = newCameraMatrix*H*P3;
		Mat A3 = a3/a3.at<double>(2,0);
		 X3 = A3.at<double>(0,0);
		 Y3 = A3.at<double>(1,0);
		Mat a4 = newCameraMatrix*H*P4;
		Mat A4 = a4/a4.at<double>(2,0);
		 X4 = A4.at<double>(0,0);
		Y4 = A4.at<double>(1,0);
		Mat a5 = newCameraMatrix*H*P5;
		Mat A5 = a5/a5.at<double>(2,0);
		 X5 = A5.at<double>(0,0);
		 Y5 = A5.at<double>(1,0);


		Mat a6 = newCameraMatrix*H*P6;
		Mat A6 = a6/a6.at<double>(2,0);
		 X6 = A6.at<double>(0,0);
		 Y6 = A6.at<double>(1,0);

		Mat a7 = newCameraMatrix*H*P7;
		Mat A7 = a7/a7.at<double>(2,0);
		 X7 = A7.at<double>(0,0);
		 Y7 = A7.at<double>(1,0);

		Mat a8 = newCameraMatrix*H*P8;
		Mat A8 = a8/a8.at<double>(2,0);
		 X8 = A8.at<double>(0,0);
		 Y8 = A8.at<double>(1,0);
		
        remap(src, dst, map1, map2, INTER_LINEAR); 
		Mat dst3;
  	cvtColor(dst, dst3,CV_GRAY2BGR);
		if(i==4){
			
	        
				line(dst3, Point2d(X1,Y1), Point2d(X2,Y2), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X2,Y2), Point2d(X3,Y3), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X3,Y3), Point2d(X4,Y4), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X4,Y4), Point2d(X1,Y1), Scalar(0,0,255), 2, 8);

				line(dst3, Point2d(X5,Y5), Point2d(X6,Y6), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X6,Y6), Point2d(X7,Y7), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X7,Y7), Point2d(X8,Y8), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X8,Y8), Point2d(X5,Y5), Scalar(0,0,255), 2, 8);

				line(dst3, Point2d(X1,Y1), Point2d(X5,Y5), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X2,Y2), Point2d(X6,Y6), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X3,Y3), Point2d(X7,Y7), Scalar(0,0,255), 2, 8);
				line(dst3, Point2d(X4,Y4), Point2d(X8,Y8), Scalar(0,0,255), 2, 8);

				Mat Resizeimage;
				Size outSize;
				double fscale = 0.3;
				outSize.width = dst3.cols * fscale;
				outSize.height = dst3.rows * fscale;
				resize(dst3, Resizeimage, outSize, 0, 0, INTER_AREA);
				
				imshow("AR5", Resizeimage);
			}
	}
}
/*
void CameraCalibrator::calibrate4( Mat &src, Mat &dst){ 
	
    Size imageSize = src.size(); 
    Mat cameraMatrix, distCoeffs, map1, map2; 
    vector<Mat> rvecs, tvecs; 
    calibrateCamera(m_dstPoints, m_srcPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);//求解出该相机的内参数(cameraMatrix)和每一个视角的外参数(rvecs, tvecs),矯正參數(distCoeffs)  

	
	
	Mat NewCameraMatrix=getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, imageSize,0,0);
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), NewCameraMatrix, imageSize, CV_32F, map1, map2); //Computes the undistortion and rectification transformation map. map1:OutputArray 
	
	
    remap(src, dst, map1, map2, INTER_LINEAR); 
	
	
	Mat rot_mat,tot_mat,A,B;
	  for(int i=0; i<m_filenames.size(); i++){ 
Rodrigues(rvecs[i], rot_mat);
A=Mat(tvecs[i]);

	  }
	
    hconcat(rot_mat,A,B); // 合併rot_mat與tvecs
	Mat cam1=B*A1;//1,1,0
	Mat cam2=B*A2;//1,-1,0
	Mat cam3=B*A3;//-1,-1,0
	Mat cam4=B*A4;//-1,1,0
	Mat cam5=B*A5;//0,0,-2
	Mat cam6=B*A6;//-1,-1,0
	Mat cam7=B*A7;//-1,1,0
	Mat cam8=B*A8;//0,0,-2
	Mat Im1=NewCameraMatrix*cam1/cam1.at<double>(2,0);
	Mat Im2=NewCameraMatrix*cam2/cam2.at<double>(2,0);
	Mat Im3=NewCameraMatrix*cam3/cam3.at<double>(2,0);
	Mat Im4=NewCameraMatrix*cam4/cam4.at<double>(2,0);
	Mat Im5=NewCameraMatrix*cam5/cam5.at<double>(2,0);
	Mat Im6=NewCameraMatrix*cam6/cam6.at<double>(2,0);
	Mat Im7=NewCameraMatrix*cam7/cam7.at<double>(2,0);
	Mat Im8=NewCameraMatrix*cam8/cam8.at<double>(2,0);
	cout << "extrinsic matrix = "<< endl << " "  << B << endl;

	//cout << "distCoeffs = "<< endl << " "  << distCoeffs << endl;
	
	/*
	std::cout<<"intrinsicMatrix"<<endl<<cameraMatrix<<endl;
	cout << "rvecs = "<< endl << " "  << rot_mat << endl;
	cout << "tvecs = "<< endl << " "  << A << endl;
	cout << "A1 in image coordinate system = "<< endl << " "  << Im1 << endl;
	cout << "A2 in image coordinate system = "<< endl << " "  << Im2 << endl;
	cout << "A3 in image coordinate system = "<< endl << " "  << Im3 << endl;
	cout << "A4 in image coordinate system = "<< endl << " "  << Im4 << endl;
	cout << "A5 in image coordinate system = "<< endl << " "  << Im5 << endl;
	*/

	 // imshow("Original Image", src);//待矯正的照片
/*    double k1x=Im1.at<double>(0,0);
	double k1y=Im1.at<double>(1,0);
	double k2x=Im2.at<double>(0,0);
	double k2y=Im2.at<double>(1,0);
	double k3x=Im3.at<double>(0,0);
	double k3y=Im3.at<double>(1,0);
	double k4x=Im4.at<double>(0,0);
	double k4y=Im4.at<double>(1,0);
	double k5x=Im5.at<double>(0,0);
	double k5y=Im5.at<double>(1,0);
	double k6x=Im6.at<double>(0,0);
	double k6y=Im6.at<double>(1,0);
	double k7x=Im7.at<double>(0,0);
	double k7y=Im7.at<double>(1,0);
	double k8x=Im8.at<double>(0,0);
	double k8y=Im8.at<double>(1,0);
	Mat dst2;
	cvtColor(dst, dst2,CV_GRAY2BGR);
    line(dst2,Point(k1x,k1y),Point(k2x,k2y),Scalar(0,0,255),1,CV_AA);
	 line(dst2,Point(k2x,k2y),Point(k3x,k3y),Scalar(0,0,255),1,CV_AA);
	 line(dst2,Point(k3x,k3y),Point(k4x,k4y),Scalar(0,0,255),1,CV_AA);
	 line(dst2,Point(k4x,k4y),Point(k1x,k1y),Scalar(0,0,255),1,CV_AA);
	  line(dst2,Point(k4x,k4y),Point(k6x,k6y),Scalar(0,0,255),1,CV_AA);
	    line(dst2,Point(k1x,k1y),Point(k7x,k7y),Scalar(0,0,255),1,CV_AA);
	line(dst2,Point(k2x,k2y),Point(k8x,k8y),Scalar(0,0,255),1,CV_AA);
	     line(dst2,Point(k3x,k3y),Point(k5x,k5y),Scalar(0,0,255),1,CV_AA);
	line(dst2,Point(k5x,k5y),Point(k8x,k8y),Scalar(0,0,255),1,CV_AA);
	  line(dst2,Point(k8x,k8y),Point(k7x,k7y),Scalar(0,0,255),1,CV_AA);
	 line(dst2,Point(k7x,k7y),Point(k6x,k6y),Scalar(0,0,255),1,CV_AA);
	 line(dst2,Point(k6x,k6y),Point(k5x,k5y),Scalar(0,0,255),1,CV_AA);
	  
	imwrite("dst.jpg",dst2);
	Mat Resizeimage;
		Size outSize;
		double fscale = 0.3;
		outSize.width = dst2.cols * fscale;
		outSize.height = dst2.rows * fscale;
		resize(dst2, Resizeimage, outSize, 0, 0, INTER_AREA);
	
	
	imshow("Undistorted Image", Resizeimage);//矯正後的輸出照片
	
}*/
