//// 创建包围轮廓的矩形边界.cpp : 定义控制台应用程序的入口点。
//#include "stdafx.h"
//#include<opencv2/opencv.hpp>
//#include<opencv2/highgui/highgui.hpp>
//#include<opencv2/imgproc.hpp>
//using namespace cv;
//using namespace std;
//int main()
//{//初始化变量和随机值
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
//		//生成随机离散点
//		image = Scalar::all(0);
//		for (int i = 0; i < count; i++) {
//			circle(image, points[i], 3, Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), FILLED, LINE_AA);
//
//		}
//
//		//寻找最小矩形的包围面积
//		RotatedRect box = minAreaRect(Mat(points));
//		Point2f vec[4];
//		box.points(vec);
//		//绘制最小面积的包围面积
//		for (int i = 0; i < 4; i++) {
//			line(image, vec[i], vec[(i + 1) % 4], Scalar(100, 200, 211), 2, LINE_AA);
//		}
//		imshow("矩形包围", image);
//		char key = (char)waitKey();
//		if (key == 27)break;
//
//	}
//	return 0;
//}
//————————————————
//版权声明：本文为CSDN博主「大郎拱白菜」的原创文章，遵循CC 4.0 BY - SA版权协议，转载请附上原文出处链接及本声明。
//原文链接：https ://blog.csdn.net/u013591306/article/details/78086050