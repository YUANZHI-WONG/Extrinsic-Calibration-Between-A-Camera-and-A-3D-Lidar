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

int main3()
{
	
	
	
	
	
	//输入图像
	//C:\Users\xunger\Desktop\test\image
	

	//namedWindow("opencv test", CV_WINDOW_AUTOSIZE);
	//imshow("opencv test", image);
	//waitKey(0);



	//cv::Size2i patternNum(i_params.grid_size.first, i_params.grid_size.second);







	
	if (patternfound) {

				
		

		
		//solvePnP(outDim, inDim, cameraMatrix, distCoeff, rvec, tvec);

			//cout <<"RVEC_img2grid3d" <<RVEC_img2grid3d << endl;
        	//cout <<"RVEC_img2grid3d" <<TVEC_img2grid3d << endl;





			

			
			



	Mat RVEC, TVEC;

	//std::vector<cv::Point3f> outDim;
	//std::vector<cv::Point2f> inDim;

	
 	cv::Point3f point;
	//point.x = -2.303;
	//point.y = -0.834;
	//point.z = 0.269;
	//outDim.push_back(point);


	//图像对应点顺序：  绿 黄 蓝 红
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

	//读取txt文件
	ifstream infile("C:\\Users\\xunger\\Desktop\\test\\pointcloud\\11.txt");
	//infile.open("C:\\Users\\xunger\\Desktop\\test\\pointcloud\\11.txt");   //将文件流对象与文件连接起来   C:\Users\xunger\Desktop\test\pointcloud\11.txt
	//assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行 
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
		SplitString(s, v, " "); //可按多个字符来分隔;
		for (int i = 0; i < 3; i++)
		{
			tmp.push_back(atof(v[i].c_str()) * 1000);
		}
		all_data.push_back(tmp);
		v.clear();
		tmp.clear();
	}

	infile.close();             //关闭文件输入流 
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
