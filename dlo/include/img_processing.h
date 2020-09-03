#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

cv::Mat readImg(string addr);

void bw_change(cv::Mat &src);

void rgb2binary(cv::Mat &src, cv::Mat &dst);
void removeOutlier(vector<cv::Point> inData, int radius, int k, vector<cv::Point> &outData);
cv::Mat removeSinglePoint(cv::Mat &src, int nRadius, int nMin);