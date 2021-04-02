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
	
	Mat RVEC_img2grid3d, TVEC_img2grid3d;//��������תͼ������
	
	
	//����ڲ�
	Mat cameraMatrix(3, 3, CV_64F);
	//431.3602 0.000000 326.6817
	//0.000000 431.3788 234.3058
	//0.000000 0.000000 1.000000
	float tempMatrix[3][3] = { { 431.3602 ,0.000000, 326.6817 }, { 0.000000 ,431.3788, 234.3058 }, { 0.000000, 0.000000, 1.000000 } };
	for (int i = 0; i < 3; i++)
	{	for (int j = 0; j < 3; j++)
		{
			cameraMatrix.at<double>(i, j) = tempMatrix[i][j];
		}
	}

	//����ϵ��
	Mat distCoeff(1, 5, CV_64F);

	double tempdistCoeff[5] = { 0.046236, - 0.066408, - 0.001334, - 0.005110, 0.000000 };


	for (int i = 0; i < 5; i++) {
		distCoeff.at<double>(0, i)= tempdistCoeff[i];
	}
	//cout << cameraMatrix << endl;
	//cout << distCoeff << endl;

	
	
	std::vector<cv::Point2f> inDim;
	std::vector<cv::Point3f> outDim;
	
	//����ͼ��
	Mat image = imread("C:\\Users\\xunger\\Desktop\\test\\image\\11.jpg");//C:\Users\xunger\Desktop\test\image
	Mat gray;
	if (image.empty())
	{
		cout << "fail to load image !" << endl;
		return -1;
	}
	//namedWindow("opencv test", CV_WINDOW_AUTOSIZE);
	//imshow("opencv test", image);
	//waitKey(0);



	cv::Size imgsize;//����ͼ��ĳߴ�

	//cv::Size2i patternNum(i_params.grid_size.first, i_params.grid_size.second);
	cv::Size2i patternNum(8, 6);//�ڽǵ���
	// cv::Size2i patternSize(i_params.square_length, i_params.square_length);
	cv::Size2i patternSize(80, 80);//С���ӱ߳�����λ��mm

	cv::Point3f tmpgrid3dpoint;



	std::vector<cv::Point2f> corners;
	std::vector<cv::Point3f> grid3dpoint;
	double tx, ty; // Translation values


	std::pair<int, int> board_dimension = std::make_pair(841, 594);//�궨��ĳߴ� A1 594 841
	std::pair<int, int> cb_translation_error = std::make_pair(0, 0);//���̸񵽱궨��ı߾� 


	std::vector<cv::Point3f> boardcorners;
	//����������Ϊԭ��������Ľ����ꡣ Board corner coordinates from the centre of the checkerboard 594 841
	//i_params.board_dimension.second �궨��ĳߴ� A1 594 841
	boardcorners.push_back(	cv::Point3f((board_dimension.second - cb_translation_error.second) / 2,	(board_dimension.first - cb_translation_error.first) / 2, 0.0));
	boardcorners.push_back(	cv::Point3f(-(board_dimension.second + cb_translation_error.second) / 2,(board_dimension.first - cb_translation_error.first) / 2, 0.0));
	boardcorners.push_back(	cv::Point3f(-(board_dimension.second + cb_translation_error.second) / 2,-(board_dimension.first + cb_translation_error.first) / 2, 0.0));
	boardcorners.push_back(	cv::Point3f((board_dimension.second - cb_translation_error.second) / 2,	-(board_dimension.first + cb_translation_error.first) / 2, 0.0));
	// Board centre coordinates from the centre of the checkerboard (due to incorrect placement of checkerbord on board)
	boardcorners.push_back(cv::Point3f(-cb_translation_error.second / 2,-cb_translation_error.first / 2, 0.0));


	cv::Mat corner_vectors = cv::Mat::eye(3, 5, CV_64F);


	cv::cvtColor(image, gray, CV_RGB2GRAY);
	
	bool patternfound = cv::findChessboardCorners(gray, patternNum, corners,CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
	if (patternfound) {
		std::cout << "�ҵ��ǵ㣡����" << endl;
		cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		//�����ڽǵ�
		cv::drawChessboardCorners(image, patternNum, corners, patternfound);
		imgsize.height = image.rows;
		imgsize.width = image.cols;

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

		cv::solvePnP(grid3dpoint, corners, cameraMatrix, distCoeff, RVEC_img2grid3d, TVEC_img2grid3d);
		//solvePnP(outDim, inDim, cameraMatrix, distCoeff, rvec, tvec);

			//cout <<"RVEC_img2grid3d" <<RVEC_img2grid3d << endl;
        	//cout <<"RVEC_img2grid3d" <<TVEC_img2grid3d << endl;





			//ʹ��һ����α�ʾRVEC_img2grid3d&TVEC_img2grid3d
			//  chessboardpose��һ��3*4�ı任���󣬿��Խ����̿���еĵ�任����������ȥ��   chessboardpose is a 3*4 transform matrix that transforms points in board frame to camera frame | R&T
			cv::Mat chessboardpose = cv::Mat::eye(4, 4, CV_64F);
			cv::Mat tmprmat = cv::Mat(3, 3, CV_64F); // ��ת����  rotation matrix
			cv::Rodrigues(RVEC_img2grid3d, tmprmat); //ŷ������ת���� Euler angles to rotation matrix
			cv::Mat chessboard_normal = cv::Mat(1, 3, CV_64F);//�궨��ƽ�淨����

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

			cv::Point2f uv;

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
				double* img_coord = converto_imgpts(corner_vectors.at<double>(0, k),corner_vectors.at<double>(1, k),corner_vectors.at<double>(2, k), cameraMatrix, distCoeff);
				
				
			
				
				

				// Mark the corners and the board centre
				if (k == 0)
				{
					cv::circle(image, cv::Point(img_coord[0], img_coord[1]), 8, CV_RGB(0, 255, 0), -1); //green
					cout << " green  " << img_coord[0] << "       " << img_coord[1] << endl;
					uv.x = img_coord[0];
					uv.y = img_coord[1];
					inDim.push_back(uv);
				}					
				else if (k == 1)
				{
					cv::circle(image, cv::Point(img_coord[0], img_coord[1]), 8, CV_RGB(255, 255, 0), -1); //yellow
					cout << " yellow  " << img_coord[0] << "       " << img_coord[1] << endl;
					uv.x = img_coord[0];
					uv.y = img_coord[1];
					inDim.push_back(uv);
				}
				else if (k == 2)
				{
					cv::circle(image, cv::Point(img_coord[0], img_coord[1]), 8, CV_RGB(0, 0, 255), -1); //blue
					cout << " blue  " << img_coord[0] << "       " << img_coord[1] << endl;
					uv.x = img_coord[0];
					uv.y = img_coord[1];
					inDim.push_back(uv);
				}
				else if (k == 3)
				{
					cv::circle(image, cv::Point(img_coord[0], img_coord[1]), 8, CV_RGB(255, 0, 0), -1); //red
					cout << " red  " << img_coord[0] << "       " << img_coord[1] << endl;
					uv.x = img_coord[0];
					uv.y = img_coord[1];
					inDim.push_back(uv);
				}
				else
				{
					cv::circle(image, cv::Point(img_coord[0], img_coord[1]),8, CV_RGB(255, 255, 255), -1); //white for centre
					cout <<" white  " << img_coord[0] << "       " << img_coord[1] << endl;

					//cout << " cam coordinates center:    " << corner_vectors.at<double>(0, k), corner_vectors.at<double>(1, k), corner_vectors.at<double>(2, k)
				}

				delete[] img_coord;
			}

			namedWindow("opencv test", CV_WINDOW_AUTOSIZE);
			imshow("opencv test", image);
			waitKey(0);


	}
	else
	{
		std::cout << "δ�ҵ��ڽǵ�" << endl;
	}

	Mat RVEC, TVEC;

	//std::vector<cv::Point3f> outDim;
	//std::vector<cv::Point2f> inDim;

	
 	cv::Point3f point;
	//point.x = -2.303;
	//point.y = -0.834;
	//point.z = 0.269;
	//outDim.push_back(point);


	//ͼ���Ӧ��˳��  �� �� �� ��
	double temp_point[4][3] = { { -2.312, 0.304, 0.278 }, {-2.382, -0.025, 0.777 }, { -2.387, -0.800 , 0.267 }, { -2.317, -0.471, -0.233 } };
	
	for (int i = 0; i < 4; i++)
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
		}
		outDim.push_back(point);

	}
	cout << "inDim  \n" << inDim << endl;
	cout << "outDim  \n" << outDim << endl;




	//cv::Point2f uv;
	//uv.x = 187.73;
	//uv.y = 142.606;
	//inDim.push_back(uv);

	cv::solvePnP(outDim, inDim, cameraMatrix, distCoeff, RVEC, TVEC);


	cout <<"RVEC  \n" << RVEC << endl;
    cout <<"RVEC  \n" << TVEC << endl;

	Rodrigues(RVEC, RVEC);



	vector<Point3f> all_points;

	//��ȡtxt�ļ�
	ifstream infile("C:\\Users\\xunger\\Desktop\\test\\pointcloud\\11.txt");
	//infile.open("C:\\Users\\xunger\\Desktop\\test\\pointcloud\\11.txt");   //���ļ����������ļ���������   C:\Users\xunger\Desktop\test\pointcloud\11.txt
	//assert(infile.is_open());   //��ʧ��,�����������Ϣ,����ֹ�������� 
	if (! infile.is_open())
	{
		cout << "fail to load pointcloud !" << endl;
		return 0;
	}

	string s;
	vector<string> v;
	vector<double> tmp;
	vector<vector<double>> all_data;
	//	all_data.resize(all_data.max_size() + 1);
	while (getline(infile, s))
	{
		//cout << s << endl;
		SplitString(s, v, " "); //�ɰ�����ַ����ָ�;
		for (int i = 0; i < 3; i++)
		{
			tmp.push_back(atof(v[i].c_str()) * 1000);
		}
		all_data.push_back(tmp);
		v.clear();
		tmp.clear();
	}

	infile.close();             //�ر��ļ������� 
	/*for (int i = 0; i < all_data.size(); i++)
	{
	cout << all_data[i][0] << " " << all_data[i][1] << " " << all_data[i][2] << endl;
	}*/
	cout << all_data.size() << endl;
	//vector<Point3f> all_points;
	for (int i = 0; i < all_data.size(); i++)
	{
		all_points.push_back(Point3f(all_data[i][0], all_data[i][1], all_data[i][2]));
	}



	std::vector<cv::Point2f> projectedPoints;
	cv::projectPoints(all_points, RVEC, TVEC, cameraMatrix, distCoeff, projectedPoints);



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


	double tmpdist = 1 + distcoeff.at<double>(0,0) * r2 + distcoeff.at<double>(0,1) * r2 * r2 +
		distcoeff.at<double>(0,4) * r2 * r2 * r2;
	planepointsC.x = tmpxC * tmpdist + 2 * distcoeff.at<double>(0,2) * tmpxC * tmpyC +
		distcoeff.at<double>(0,3) * (r2 + 2 * tmpxC * tmpxC);
	planepointsC.y = tmpyC * tmpdist + distcoeff.at<double>(0,2) * (r2 + 2 * tmpyC * tmpyC) +
		2 * distcoeff.at<double>(0,3) * tmpxC * tmpyC;
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
