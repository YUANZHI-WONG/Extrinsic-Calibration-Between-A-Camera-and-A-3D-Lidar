//// ������Χ�����ľ��α߽�.cpp : �������̨Ӧ�ó������ڵ㡣
//#include "stdafx.h"
//#include<opencv2/opencv.hpp>
//#include<opencv2/highgui/highgui.hpp>
//#include<opencv2/imgproc.hpp>
//using namespace cv;
//using namespace std;
//int main()
//{//��ʼ�����������ֵ
//	Mat image(600, 600, CV_8UC3);
//	RNG& rng = theRNG();
//	while (1) {
//		int count = rng.uniform(3, 103);
//		vector<Point>points;
//		for (int i = 0; i < count; i++) {
//			Point point;
//			point.x = rng.uniform(image.cols / 4, image.cols * 3 / 4);
//			point.y = rng.uniform(image.rows / 4, image.rows * 3 / 4);
//			points.push_back(point);
//		}
//		//���������ɢ��
//		image = Scalar::all(0);
//		for (int i = 0; i < count; i++) {
//			circle(image, points[i], 3, Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), FILLED, LINE_AA);
//
//		}
//
//		//Ѱ����С���εİ�Χ���
//		RotatedRect box = minAreaRect(Mat(points));
//		Point2f vec[4];
//		box.points(vec);
//		//������С����İ�Χ���
//		for (int i = 0; i < 4; i++) {
//			line(image, vec[i], vec[(i + 1) % 4], Scalar(100, 200, 211), 2, LINE_AA);
//		}
//		imshow("���ΰ�Χ", image);
//		char key = (char)waitKey();
//		if (key == 27)break;
//
//	}
//	return 0;
//}
//��������������������������������
//��Ȩ����������ΪCSDN���������ɹ��ײˡ���ԭ�����£���ѭCC 4.0 BY - SA��ȨЭ�飬ת���븽��ԭ�ĳ������Ӽ���������
//ԭ�����ӣ�https ://blog.csdn.net/u013591306/article/details/78086050