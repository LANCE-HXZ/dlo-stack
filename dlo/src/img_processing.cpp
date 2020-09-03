#include "img_processing.h"
using namespace cv;


// ==========    黑白反置    ==========
void bw_change(Mat &src){
	int r = src.rows;
	int c = src.cols;
	for(int i = 0; i < r; ++i){
		for(int j = 0; j < c; ++j){
			src.at<uchar>(i, j) = !src.at<uchar>(i, j);
		}
	}
}

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

//设置在当前点的搜索半径为radius的范围内，如果相邻点个数少于k个，则该点为离群点
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

//去除离群点
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
	}
	ptOutlier.clear();
	ptWhite.clear();
	return dst;
}