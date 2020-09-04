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