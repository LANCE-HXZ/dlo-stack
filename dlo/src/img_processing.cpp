#include "img_processing.h"

using namespace cv;


/*  打开图片并检查是否打开成功
    输入: 图片路径
	输出: 图像	*/
cv::Mat readImg(string addr){
  Mat img = cv::imread(addr);
  if(!img.cols){
    cout << "\n\n===== 【ERROR ADDRESS】 " << addr << " =====\n\n\n";
    return  cv::Mat::zeros(10, 10, CV_8UC3);	//	若打开图像失败则返回一张黑色图像
  }
  return img;
}

/*  图像腐蚀, 扩大黑色区域, 减小白色区域
    输入: 图像, 图像处理核的尺寸size(越大腐蚀越多), 重复操作次数times	*/
void pre_dilate(Mat & srcImg, int size, int times){
	Mat element = getStructuringElement(MORPH_CROSS, Size(size, size));
	for (size_t i = 0; i < times; i++)
	{
		dilate(srcImg, srcImg, element);
	}
}

/*  图像膨胀, 扩大白色区域, 减小黑色区域
    输入: 图像, 图像处理核的尺寸size(越大膨胀越多), 重复操作次数times	*/
void pre_erode(Mat & srcImg, int size, int times){
	Mat element = getStructuringElement(MORPH_CROSS, Size(size, size));
	for (size_t i = 0; i < times; i++)
	{
		erode(srcImg, srcImg, element);
	}
}

/*  黑白反置
    输入: 一张二值图像	*/
void bw_change(Mat &src){
	int r = src.rows;
	int c = src.cols;
	for(int i = 0; i < r; ++i){
		for(int j = 0; j < c; ++j){
			src.at<uchar>(i, j) = !src.at<uchar>(i, j);
		}
	}
}


/*	根据函数规则将 rgb 图像转换为二值黑白图像
	输入: 原图src, 输出图dst	*/
void rgb2binary(cv::Mat &src, cv::Mat &dst){
	int c = src.cols, r = src.rows;
	for(int ic = 0; ic < c; ++ic){
		for(int ir = 0; ir < r; ++ir){
			if(src.at<Vec3b>(ir, ic)[2] > 128 && src.at<Vec3b>(ir, ic)[0] < 128)
				dst.at<Vec3b>(ir, ic)[0] = dst.at<Vec3b>(ir, ic)[1] = dst.at<Vec3b>(ir, ic)[2] = 0;
			else
				dst.at<Vec3b>(ir, ic)[0] = dst.at<Vec3b>(ir, ic)[1] = dst.at<Vec3b>(ir, ic)[2] = 255;
		}
	}
}

/*	检索离群点, 设置在当前点的搜索半径为radius的范围内，如果相邻点个数少于k个，则该点为离群点
	输入: 传入的需要检查的点inData, 搜索半径radius, 阈值k, 输出点outData	*/
void removeOutlier(vector<Point> inData, int radius, int k, vector<Point> &outData)
{
	outData.clear();
	int cnt = 0;
	int n = 0;
	for (int m = 0; m < inData.size(); m++)
	{
		cnt = 0;
		for (n = 0; n < inData.size(); n++)
		{
			if (n == m)
				continue;
			if (sqrt(pow(inData[m].x - inData[n].x,2) + pow(inData[m].y - inData[n].y,2)) <= radius)
			{
				cnt++;
				if (cnt >= k)
				{
					outData.push_back(inData[m]);
					break;
				}
			}
		}
	}
}

/*	基于半径距离去除离群点
	输入: 原图src, 搜索半径nRadius, 阈值nMin
	输出: 去除离群点后的图	*/
Mat removeSinglePoint(cv::Mat &src, int nRadius, int nMin){
	Mat dst = cv::Mat::zeros(640, 800, CV_8UC3);
	vector<Point> ptWhite, ptOutlier;
	for(int ir = 0; ir < src.rows; ++ir){
		for(int ic = 0; ic < src.cols; ++ic){
			if(src.at<Vec3b>(ir, ic)[0] == 255)
				ptWhite.push_back(Point(ic, ir));
		}
	}
	removeOutlier(ptWhite, nRadius, nMin, ptOutlier);
	for(Point ptOpt:ptOutlier){
		dst.at<Vec3b>(ptOpt.y, ptOpt.x)[0] = dst.at<Vec3b>(ptOpt.y, ptOpt.x)[1] = dst.at<Vec3b>(ptOpt.y, ptOpt.x)[2] = 255;
		// dst.at<uchar>(ptOpt.y, ptOpt.x) = 255;
	}
	ptOutlier.clear();
	ptWhite.clear();
	return dst;
}

/*	计算两条相交线段的夹角, 用于 crop.cpp & strategy.cpp
	输入: 第一条线段的非交点端点， 第二条线段的非交点端点， 两条线段的交点
	输出: 角度制夹角值，°	*/
double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0){
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	double angle_line = (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1) * (dx2*dx2 + dy2*dy2) + 1e-10);
	return acos(angle_line) * 180 / 3.141592653;
}


// 计算 |p1 p2| X |p1 p|
float GetCross(Point& p1, Point& p2,Point& p)
{
    return (p2.x - p1.x) * (p.y - p1.y) -(p.x - p1.x) * (p2.y - p1.y);
}
//判断点是否在矩形内(含边界), 是则返回true, 否返回false
bool IsPointInMatrix(Point& p, vector<Point> vptMatrix)
{
    Point p1 = vptMatrix[0];
    Point p2 = vptMatrix[1];
    Point p3 = vptMatrix[2];
    Point p4 = vptMatrix[3];
    return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

string nameWithTime(){
	time_t timep;
    struct tm *p;
    time(&timep); //获取从1970至今过了多少秒，存入time_t类型的timep
    p = localtime(&timep);//用localtime将秒数转化为struct tm结构体
	string y = to_string(1900 + p->tm_year);
	string m = (1+ p->tm_mon) < 10 ? "0" + to_string(1+ p->tm_mon) : to_string(1+ p->tm_mon);
	string d = (p->tm_mday) < 10 ? "0" + to_string(p->tm_mday) : to_string(p->tm_mday);
	string h = (p->tm_hour) < 10 ? "0" + to_string(p->tm_hour) : to_string(p->tm_hour);
	string min = (p->tm_min) < 10 ? "0" + to_string(p->tm_min) : to_string(p->tm_min);
	string s = (p->tm_sec) < 10 ? "0" + to_string(p->tm_sec) : to_string(p->tm_sec);
	return y+m+d+h+min+s;
}