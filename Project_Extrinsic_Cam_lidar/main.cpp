#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include<iostream>
#include <string>
#include <vector>
#include <fstream>  //�ļ����⺯��
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

	//����ڲξ���
	double tempCameraMatrix[3][3] = { { 431.3602 ,0.000000, 326.6817 }, { 0.000000 ,431.3788, 234.3058 }, { 0.000000, 0.000000, 1.000000 } };
	//����ϵ��
	double tempdDistCoeff[5] = { 0.046236, -0.066408, -0.001334, -0.005110, 0.000000 };

	Mat image = imread("C:\\Users\\xunger\\Desktop\\test\\image\\11.jpg");
	if (image.empty())
	{
		cout << "fail to load image !" << endl;
		return -1;
	}

	//���̸�ģʽ
	int verticiesNum[] = { 8,6 };
	int square_length = 80; //С���ӱ߳�����λ��mm

	//�궨��ߴ�
	int CheckerboardSize[] = { 841, 594 };
	int Checkerboard_offset[] = { 0,0 };

	//�궨���ĸ��ǵ�ĵ������ꡣͼ���Ӧ��˳��  �� �� �� ��
	double temp_point[5][3] = { { -2.312, 0.304, 0.278 }, {-2.382, -0.025, 0.777 }, { -2.387, -0.800 , 0.267 }, { -2.317, -0.471, -0.233 } ,{-2.349, -0.248, 0.272} };

	//���������ļ�
	ifstream infile("C:\\Users\\xunger\\Desktop\\test\\pointcloud\\11.txt");

	//����txt�ļ��ָ���
	string split_sign = " ";

	/////////////////////���������븳ֵ/////////////////////////////////////

	//����ڲ�
	Mat cameraMatrix(3, 3, CV_64F);
	//����ϵ��
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

	// �Ҷ�ͼ
	Mat gray;
	

	//����ƽ������������ռ�����ı任������ת����ƽ�ƾ���
	Mat RVEC_img2grid3d, TVEC_img2grid3d;

	//����ͼ��ĳߴ�
	cv::Size imgsize;
	//�ڶ�����
	cv::Size2i patternNum(verticiesNum[0], verticiesNum[1]);
	//С���ӱ߳�
	cv::Size2i patternSize(square_length, square_length);
	
	//�궨��ĳߴ�
	std::pair<int, int> board_dimension = std::make_pair(CheckerboardSize[0], CheckerboardSize[1]);
	//���̸񵽱궨��ı߾� 
	std::pair<int, int> cb_translation_error = std::make_pair(Checkerboard_offset[0], Checkerboard_offset[1]);
	//����������Ϊԭ��������Ľ����ꡣ Board corner coordinates from the centre of the checkerboard 594 841
	std::vector<cv::Point3f> boardcorners;
	
		boardcorners.push_back(cv::Point3f((board_dimension.second - cb_translation_error.second) / 2, (board_dimension.first - cb_translation_error.first) / 2, 0.0));
		boardcorners.push_back(cv::Point3f(-(board_dimension.second + cb_translation_error.second) / 2, (board_dimension.first - cb_translation_error.first) / 2, 0.0));
		boardcorners.push_back(cv::Point3f(-(board_dimension.second + cb_translation_error.second) / 2, -(board_dimension.first + cb_translation_error.first) / 2, 0.0));
		boardcorners.push_back(cv::Point3f((board_dimension.second - cb_translation_error.second) / 2, -(board_dimension.first + cb_translation_error.first) / 2, 0.0));
		boardcorners.push_back(cv::Point3f(-cb_translation_error.second / 2, -cb_translation_error.first / 2, 0.0));//�궨�����ĵ������

	//�ڽ�����
	cv::Mat corner_vectors = cv::Mat::eye(3, 5, CV_64F);

	//ʶ�𵽵��ڽ�
	std::vector<cv::Point2f> corners;


	std::vector<cv::Point2f> Camera_Pixs ;// inDim;
	std::vector<cv::Point3f> Lidar_Poinds;// outDim;

	bool patternfound;

	//�����״������������֮��ı任����
	Mat RVEC, TVEC;
	//�ǵ�ļ����������
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
	/*ȥ����
	//	ȥ����ͼ
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
		
		std::cout << "�ҵ��ǵ㣡����" << endl;
		
		//���������ĵ�Ϊԭ�㣬ÿ���ǵ�Ŀռ�����
		cv::Point3f tmpgrid3dpoint;
		std::vector<cv::Point3f> grid3dpoint;

		//�ǵ������ƽ������
		cv::Point2f uv;

		// Translation values
		double tx, ty;

		//�������Ż�
		cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		//�����ڽǵ�
		cv::drawChessboardCorners(image, patternNum, corners, patternfound);



		// ���ԭ��λ��Ϊ���̵������ڽ�  Location of board frame origin from the bottom left inner corner of the checkerboard
		tx = (patternNum.height - 1) * patternSize.height / 2;
		ty = (patternNum.width - 1) * patternSize.width / 2;

		// �������Ĺ�ϵ  Board corners w.r.t board frame
		for (int i = 0; i < patternNum.height; i++) {
			for (int j = 0; j < patternNum.width; j++) {

				// ��ԭ������½�ת�����������ġ�   Translating origin from bottom left corner to the centre of the checkerboard
				tmpgrid3dpoint.x = i * patternSize.height - tx;
				tmpgrid3dpoint.y = j * patternSize.width - ty;
				tmpgrid3dpoint.z = 0;
				grid3dpoint.push_back(tmpgrid3dpoint);
			}
		}
		//�������ƽ������������ռ�����ı任����
		cv::solvePnP(grid3dpoint, corners, cameraMatrix, distCoeff, RVEC_img2grid3d, TVEC_img2grid3d);

		//ʹ��һ����α任�����ʾ RVEC_img2grid3d �� TVEC_img2grid3d
		// chessboardpose��һ��4*4�ı任���󣬿��Խ����̿���еĵ�任����������ȥ��   chessboardpose is a 3*4 transform matrix that transforms points in board frame to camera frame | R&T
		cv::Mat chessboardpose = cv::Mat::eye(4, 4, CV_64F);
		cv::Mat chessboard_normal = cv::Mat(1, 3, CV_64F);//�궨��ƽ�淨����
		cv::Mat tmprmat = cv::Mat(3, 3, CV_64F); // ��ת����  rotation matrix
		cv::Rodrigues(RVEC_img2grid3d, tmprmat); //ŷ������ת���� Euler angles to rotation matrix
		
		
		
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				chessboardpose.at<double>(j, k) = tmprmat.at<double>(j, k);
			}
			chessboardpose.at<double>(j, 3) = TVEC_img2grid3d.at<double>(j);
		}
		//����б궨��ƽ�淨����
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
			//��ȡ�����ά����ϵ�У��ĸ��ǵ�����ĵ������
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
		std::cout << "δ�ҵ��ڽǵ�" << endl;
	}



	//�������ƽ�������뼤���״�ռ�����ı任����
	cv::solvePnP(Lidar_Poinds, Camera_Pixs, cameraMatrix, distCoeff, RVEC, TVEC);
	Rodrigues(RVEC, RVEC);


	cout << "\nRVEC��  \n" << RVEC << endl;
	cout << "\nTVEC��  \n" << TVEC << endl;



	/////////////////////�������ݵ�ͶӰ///////////////////////////////

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
		SplitString(s, v, split_sign); //�ɰ�����ַ����ָ�;
		for (int i = 0; i < 3; i++)
		{
			tmp.push_back(atof(v[i].c_str()) * 1000);
		}
		all_data.push_back(tmp);
		v.clear();
		tmp.clear();
	}

	infile.close();             //�ر��ļ������� 

	cout << "Size of all_points"<<all_data.size() << endl;

	for (int i = 0; i < all_data.size(); i++)
	{
		all_points.push_back(Point3f(all_data[i][0], all_data[i][1], all_data[i][2]));
	}

	cv::projectPoints(all_points, RVEC, TVEC, cameraMatrix, distCoeff, projectedPoints);
	
	

	//����ͶӰ��
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


// ��ͼ�����У���3D����������ת��Ϊ2D���ص㡣     Convert 3D points w.r.t camera frame to 2D pixel points in image frame    
// ���������ģʽ   For pinhole camera model
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


