#ifndef PCH_H
#define PCH_H

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

// #include <opencv2/highgui/highgui_c.h> // 解决"未定义的CV_EVENT_LBUTTONDOWN"问题
// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <string>

// #include <math.h>

int test();

void skeleton(cv::Mat src, string save_address);
void pre_erode(cv::Mat & srcImg, int size, int times);
void thinImage(cv::Mat & srcImg);
void thinningIteration(cv::Mat& img, int iter);
void thinning(const cv::Mat& src, cv::Mat& dst);

void traversal(cv::Mat src, string img_num);
void traversal(cv::Mat src);
void traversal_callback(int event, int x, int y, int flags, void* ustc);
int dxdy2currentpoint(cv::Point curr_pt, int dx, int dy);
bool in_square(cv::Point center, int nx, int ny, int sqr_size);
cv::Point get_first_current_point(cv::Point pre_pt, int per_step);
cv::Point get_next_point(cv::Point curr_pt, int dx, int dy, int per_step);
cv::Point get_opposite_point(cv::Point center, int nx, int ny);
void get_point(int event, int x, int y, int flags, void* ustc);

void crop(cv::Mat src);
void crop_callback(int event, int x, int y, int flags, void* ustc);
cv::Mat RotateImage(cv::Mat src, double angle, cv::Point center);


#endif //PCH_H