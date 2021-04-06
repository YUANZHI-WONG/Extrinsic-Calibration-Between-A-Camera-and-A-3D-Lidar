#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include<iostream>
#include <string>
#include <vector>
#include <fstream>  //文件流库函数
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/highgui/highgui_c.h>
#include <assert.h>


using namespace std;
using namespace cv;

double* converto_imgpts(double x, double y, double z, cv::Mat cameramat, cv::Mat distcoeff);
void SplitString(const string& s, vector<string>& v, const string& c);


int main()
{

	//相机内参矩阵
	double tempCameraMatrix[3][3] = { { 431.3602 ,0.000000, 326.6817 }, { 0.000000 ,431.3788, 234.3058 }, { 0.000000, 0.000000, 1.000000 } };
	//畸变系数
	double tempdDistCoeff[5] = { 0.046236, -0.066408, -0.001334, -0.005110, 0.000000 };

	Mat image = imread("C:\\Users\\xunger\\Desktop\\test\\image\\11.jpg");
	if (image.empty())
	{
		cout << "fail to load image !" << endl;
		return -1;
	}

	//棋盘格模式
	int verticiesNum[] = { 8,6 };
	int square_length = 80; //小格子边长，单位：mm

	//标定板尺寸
	int CheckerboardSize[] = { 841, 594 };
	int Checkerboard_offset[] = { 0,0 };

	//标定板四个角点的点云坐标。图像对应点顺序：  绿 黄 蓝 红
	double temp_point[5][3] = { { -2.312, 0.304, 0.278 }, {-2.382, -0.025, 0.777 }, { -2.387, -0.800 , 0.267 }, { -2.317, -0.471, -0.233 } ,{-2.349, -0.248, 0.272} };

	//点云数据文件
	ifstream infile("C:\\Users\\xunger\\Desktop\\test\\pointcloud\\11.txt");

	//点云txt文件分隔符
	string split_sign = " ";

	/////////////////////参数定义与赋值/////////////////////////////////////

	//相机内参
	Mat cameraMatrix(3, 3, CV_64F);
	//畸变系数
	Mat distCoeff(1, 5, CV_64F);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++) {
			cameraMatrix.at<double>(i, j) = tempCameraMatrix[i][j];
		}
	}
	for (int i = 0; i < 5; i++) {
		distCoeff.at<double>(0, i) = tempdDistCoeff[i];
	}

	// 灰度图
	Mat gray;
	

	//像素平面坐标与相机空间坐标的变换矩阵。旋转矩阵、平移矩阵。
	Mat RVEC_img2grid3d, TVEC_img2grid3d;

	//输入图像的尺寸
	cv::Size imgsize;
	//内顶点数
	cv::Size2i patternNum(verticiesNum[0], verticiesNum[1]);
	//小格子边长
	cv::Size2i patternSize(square_length, square_length);
	
	//标定板的尺寸
	std::pair<int, int> board_dimension = std::make_pair(CheckerboardSize[0], CheckerboardSize[1]);
	//棋盘格到标定板的边距 
	std::pair<int, int> cb_translation_error = std::make_pair(Checkerboard_offset[0], Checkerboard_offset[1]);
	//以棋盘中心为原点的棋盘四角坐标。 Board corner coordinates from the centre of the checkerboard 594 841
	std::vector<cv::Point3f> boardcorners;
	
		boardcorners.push_back(cv::Point3f((board_dimension.second - cb_translation_error.second) / 2, (board_dimension.first - cb_translation_error.first) / 2, 0.0));
		boardcorners.push_back(cv::Point3f(-(board_dimension.second + cb_translation_error.second) / 2, (board_dimension.first - cb_translation_error.first) / 2, 0.0));
		boardcorners.push_back(cv::Point3f(-(board_dimension.second + cb_translation_error.second) / 2, -(board_dimension.first + cb_translation_error.first) / 2, 0.0));
		boardcorners.push_back(cv::Point3f((board_dimension.second - cb_translation_error.second) / 2, -(board_dimension.first + cb_translation_error.first) / 2, 0.0));
		boardcorners.push_back(cv::Point3f(-cb_translation_error.second / 2, -cb_translation_error.first / 2, 0.0));//标定板中心点的坐标

	//内角向量
	cv::Mat corner_vectors = cv::Mat::eye(3, 5, CV_64F);

	//识别到的内角
	std::vector<cv::Point2f> corners;


	std::vector<cv::Point2f> Camera_Pixs ;// inDim;
	std::vector<cv::Point3f> Lidar_Poinds;// outDim;

	bool patternfound;

	//激光雷达与摄像机坐标之间的变换矩阵
	Mat RVEC, TVEC;
	//角点的激光点云坐标
	cv::Point3f point;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			//cameraMatrix.at<double>(i, j) = tempMatrix[i][j];
			if (j == 0)
			{
				point.x = temp_point[i][j];
			}
			if (j == 1)
			{
				point.y = temp_point[i][j];
			}
			if (j == 2)
			{
				point.z = temp_point[i][j];
			}
			if (j == 4)
			{
				point.z = temp_point[i][j];
			}
		}
		Lidar_Poinds.push_back(point);

	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*去畸变
	//	去畸变图
	Mat undistort_img;
	
	cv::Size img_size(i_params.image_size.first, i_params.image_size.second);


	cv::Mat undistort_map1;
	cv::Mat undistort_map2;


	imgsize.height = image.rows;
	imgsize.width = image.cols;

	imgsize = image.size();

	cv::initUndistortRectifyMap(cameraMatrix, distCoeff, cv::Mat(),
		cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeff, imgsize, 1, imgsize, 0),
		imgsize, CV_16SC2, undistort_map1, undistort_map2);
	cv::remap(image, undistort_img, undistort_map1, undistort_map2, cv::INTER_LINEAR);

	namedWindow("opencv test1", CV_WINDOW_AUTOSIZE);
	imshow("opencv test1", image);


	namedWindow("opencv test undis", CV_WINDOW_AUTOSIZE);
	imshow("opencv test1 undis", undistort_img);
	waitKey(0);*/



	cv::cvtColor(image, gray, CV_RGB2GRAY);


	patternfound = cv::findChessboardCorners(gray, patternNum, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

	if (patternfound) {
		
		std::cout << "找到角点！！！" << endl;
		
		//以棋盘中心点为原点，每个角点的空间坐标
		cv::Point3f tmpgrid3dpoint;
		std::vector<cv::Point3f> grid3dpoint;

		//角点的像素平面坐标
		cv::Point2f uv;

		// Translation values
		double tx, ty;

		//亚像素优化
		cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		//绘制内角点
		cv::drawChessboardCorners(image, patternNum, corners, patternfound);



		// 板框原点位置为棋盘的左下内角  Location of board frame origin from the bottom left inner corner of the checkerboard
		tx = (patternNum.height - 1) * patternSize.height / 2;
		ty = (patternNum.width - 1) * patternSize.width / 2;

		// 板角与板框的关系  Board corners w.r.t board frame
		for (int i = 0; i < patternNum.height; i++) {
			for (int j = 0; j < patternNum.width; j++) {

				// 将原点从左下角转换到棋盘中心。   Translating origin from bottom left corner to the centre of the checkerboard
				tmpgrid3dpoint.x = i * patternSize.height - tx;
				tmpgrid3dpoint.y = j * patternSize.width - ty;
				tmpgrid3dpoint.z = 0;
				grid3dpoint.push_back(tmpgrid3dpoint);
			}
		}
		//求解像素平面坐标与相机空间坐标的变换矩阵
		cv::solvePnP(grid3dpoint, corners, cameraMatrix, distCoeff, RVEC_img2grid3d, TVEC_img2grid3d);

		//使用一个齐次变换矩阵表示 RVEC_img2grid3d 和 TVEC_img2grid3d
		// chessboardpose是一个4*4的变换矩阵，可以将棋盘框架中的点变换到相机框架中去。   chessboardpose is a 3*4 transform matrix that transforms points in board frame to camera frame | R&T
		cv::Mat chessboardpose = cv::Mat::eye(4, 4, CV_64F);
		cv::Mat chessboard_normal = cv::Mat(1, 3, CV_64F);//标定板平面法向量
		cv::Mat tmprmat = cv::Mat(3, 3, CV_64F); // 旋转矩阵  rotation matrix
		cv::Rodrigues(RVEC_img2grid3d, tmprmat); //欧拉角旋转矩阵 Euler angles to rotation matrix
		
		
		
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				chessboardpose.at<double>(j, k) = tmprmat.at<double>(j, k);
			}
			chessboardpose.at<double>(j, 3) = TVEC_img2grid3d.at<double>(j);
		}
		//相机中标定板平面法向量
		chessboard_normal.at<double>(0) = 0;
		chessboard_normal.at<double>(1) = 0;
		chessboard_normal.at<double>(2) = 1;
		chessboard_normal = chessboard_normal * chessboardpose(cv::Rect(0, 0, 3, 3)).t();



		for (int k = 0; k < boardcorners.size(); k++) {
			// take every point in boardcorners set
			cv::Point3f pt(boardcorners[k]);
			for (int i = 0; i < 3; i++) {
				// Transform it to obtain the coordinates in cam frame
				corner_vectors.at<double>(i, k) = chessboardpose.at<double>(i, 0) * pt.x +
													chessboardpose.at<double>(i, 1) * pt.y +
													chessboardpose.at<double>(i, 3);
			}

			// convert 3D coordinates to image coordinates  
			//获取相机三维坐标系中，四个角点和中心点的坐标
			double* img_coord = converto_imgpts(corner_vectors.at<double>(0, k), corner_vectors.at<double>(1, k), corner_vectors.at<double>(2, k), cameraMatrix, distCoeff);


			// Mark the corners and the board centre
			if (k == 0)
			{
				cv::circle(image, cv::Point(img_coord[0], img_coord[1]), 8, CV_RGB(0, 255, 0), -1); //green
				cout << " green  " << img_coord[0] << "       " << img_coord[1] << endl;
				uv.x = img_coord[0];
				uv.y = img_coord[1];
				Camera_Pixs.push_back(uv);
			}
			else if (k == 1)
			{
				cv::circle(image, cv::Point(img_coord[0], img_coord[1]), 8, CV_RGB(255, 255, 0), -1); //yellow
				cout << " yellow  " << img_coord[0] << "       " << img_coord[1] << endl;
				uv.x = img_coord[0];
				uv.y = img_coord[1];
				Camera_Pixs.push_back(uv);
			}
			else if (k == 2)
			{
				cv::circle(image, cv::Point(img_coord[0], img_coord[1]), 8, CV_RGB(0, 0, 255), -1); //blue
				cout << " blue  " << img_coord[0] << "       " << img_coord[1] << endl;
				uv.x = img_coord[0];
				uv.y = img_coord[1];
				Camera_Pixs.push_back(uv);
			}
			else if (k == 3)
			{
				cv::circle(image, cv::Point(img_coord[0], img_coord[1]), 8, CV_RGB(255, 0, 0), -1); //red
				cout << " red  " << img_coord[0] << "       " << img_coord[1] << endl;
				uv.x = img_coord[0];
				uv.y = img_coord[1];
				Camera_Pixs.push_back(uv);
			}
			else
			{
				cv::circle(image, cv::Point(img_coord[0], img_coord[1]), 8, CV_RGB(255, 255, 255), -1); //white for centre
				cout << " white  " << img_coord[0] << "       " << img_coord[1] << endl;
				uv.x = img_coord[0];
				uv.y = img_coord[1];
				Camera_Pixs.push_back(uv);
				//cout << " cam coordinates center:    " << corner_vectors.at<double>(0, k), corner_vectors.at<double>(1, k), corner_vectors.at<double>(2, k)
			}

			delete[] img_coord;
		}
	}
	else
	{
		std::cout << "未找到内角点" << endl;
	}



	//求解像素平面坐标与激光雷达空间坐标的变换矩阵
	cv::solvePnP(Lidar_Poinds, Camera_Pixs, cameraMatrix, distCoeff, RVEC, TVEC);
	Rodrigues(RVEC, RVEC);


	cout << "\nRVEC：  \n" << RVEC << endl;
	cout << "\nTVEC：  \n" << TVEC << endl;



	/////////////////////点云数据的投影///////////////////////////////

	vector<Point3f> all_points;
	string s;
	vector<string> v;
	vector<double> tmp;
	vector<vector<double>> all_data;

	std::vector<cv::Point2f> projectedPoints;



	if (!infile.is_open())
	{
		cout << "fail to load pointcloud !" << endl;
		return 0;
	}

	while (getline(infile, s))
	{
		//cout << s << endl;
		SplitString(s, v, split_sign); //可按多个字符来分隔;
		for (int i = 0; i < 3; i++)
		{
			tmp.push_back(atof(v[i].c_str()) * 1000);
		}
		all_data.push_back(tmp);
		v.clear();
		tmp.clear();
	}

	infile.close();             //关闭文件输入流 

	cout << "Size of all_points"<<all_data.size() << endl;

	for (int i = 0; i < all_data.size(); i++)
	{
		all_points.push_back(Point3f(all_data[i][0], all_data[i][1], all_data[i][2]));
	}

	cv::projectPoints(all_points, RVEC, TVEC, cameraMatrix, distCoeff, projectedPoints);
	
	

	//绘制投影点
	for (int i = 0; i < projectedPoints.size(); i++)
	{
		Point2f p = projectedPoints[i];
		if (p.y <= 1080)
		{
			circle(image, p, 1, Scalar(255, 255, 0), 1, 8, 0);
		}
	}


	namedWindow("opencv test", CV_WINDOW_AUTOSIZE);
	imshow("opencv test", image);
	waitKey(0);

	return 0;
}


// 在图像框架中，将3D点与相机框架转换为2D像素点。     Convert 3D points w.r.t camera frame to 2D pixel points in image frame    
// 单孔摄像机模式   For pinhole camera model
double* converto_imgpts(double x, double y, double z, cv::Mat cameramat, cv::Mat distcoeff) {
	double tmpxC = x / z;
	double tmpyC = y / z;
	cv::Point2d planepointsC;
	planepointsC.x = tmpxC;
	planepointsC.y = tmpyC;
	double r2 = tmpxC * tmpxC + tmpyC * tmpyC;

	//cout << "\n\n" << distcoeff << endl;
	//cout << "\n\n" << distcoeff.at<double>(0, 0) << endl;


	double tmpdist = 1 + distcoeff.at<double>(0, 0) * r2 + distcoeff.at<double>(0, 1) * r2 * r2 +
		distcoeff.at<double>(0, 4) * r2 * r2 * r2;
	planepointsC.x = tmpxC * tmpdist + 2 * distcoeff.at<double>(0, 2) * tmpxC * tmpyC +
		distcoeff.at<double>(0, 3) * (r2 + 2 * tmpxC * tmpxC);
	planepointsC.y = tmpyC * tmpdist + distcoeff.at<double>(0, 2) * (r2 + 2 * tmpyC * tmpyC) +
		2 * distcoeff.at<double>(0, 3) * tmpxC * tmpyC;
	planepointsC.x = cameramat.at<double>(0, 0) * planepointsC.x + cameramat.at<double>(0, 2);
	planepointsC.y = cameramat.at<double>(1, 1) * planepointsC.y + cameramat.at<double>(1, 2);



	double* img_coord = new double[2];
	*(img_coord) = planepointsC.x;
	*(img_coord + 1) = planepointsC.y;

	return img_coord;
}



void SplitString(const string& s, vector<string>& v, const string& c)
{
	string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while (string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2 - pos1));

		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
		v.push_back(s.substr(pos1));
}


