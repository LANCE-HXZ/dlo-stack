#ifndef IMG_PROCESSING
#define IMG_PROCESSING
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

cv::Mat readImg(string addr);

void pre_dilate(cv::Mat & srcImg, int size, int times);
void pre_erode(cv::Mat & srcImg, int size, int times);

void bw_change(cv::Mat &src);

void rgb2binary(cv::Mat &src, cv::Mat &dst);
void removeOutlier(vector<cv::Point> inData, int radius, int k, vector<cv::Point> &outData);
cv::Mat removeSinglePoint(cv::Mat &src, int nRadius, int nMin);

double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);

float GetCross(cv::Point& p1, cv::Point& p2,cv::Point& p);
bool IsPointInMatrix(cv::Point& p, vector<cv::Point> vptMatrix = {cv::Point(120, 120), cv::Point(120, 520), cv::Point(680, 120), cv::Point(680, 520)});

#endif