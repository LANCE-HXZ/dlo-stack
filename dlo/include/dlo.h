#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <vector>

#include "img_processing.h"

// darknet_ros_msgs
#include "dlo/BoundingBoxes.h"
#include "dlo/BoundingBox.h"

using namespace std;

#define EDGE 80     // 图像四边增加边框的宽度

// #include <opencv2/highgui/highgui_c.h> // 解决"未定义的CV_EVENT_LBUTTONDOWN"问题
// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <string>

// #include <math.h>
// extern ros::NodeHandle nh;
extern cv::Mat new_bw;
extern vector<cv::Point> pt, endpoint, cross, dir;
extern vector<int64> g_vnClassList, g_vnCrossList;
extern vector<int> cpt, ept, start;
extern int point_num, cpt_num, ept_num, line_num;
extern cv::Scalar red, green, blue, yellow,purple, cyan;

// test_function
int test();
int* test(int input);
void thinImage(cv::Mat & src, cv::Mat & dst);
void endPointAndintersectionPointDetection(cv::Mat & src, vector<cv::Point> &endpoint);
void fix_cross_error(cv::Mat & src);
// int glo_xy(int x, int y);
int d(cv::Point opt, cv::Point dir);
void move(cv::Point opt_p, cv::Point opt, int step, uchar *pointer, cv::Point dirflag, int xdir, int ydir);
void new_ske(cv::Mat &src, cv::Mat &dst, cv::Mat &bw, vector<cv::Point> &endpoint);
void DrawArc(cv::Mat &src, cv::Mat &bw, cv::Point ArcCenter, cv::Point StartPoint, cv::Point EndPoint, int Fill);


// skeleton
cv::Mat skeleton(cv::Mat input, string save_address, int channels);
void pre_dilate(cv::Mat & srcImg, int size, int times);
void pre_erode(cv::Mat & srcImg, int size, int times);
void thinImage(cv::Mat & srcImg);
void thinningIteration(cv::Mat& img, int iter);
void thinning(const cv::Mat& src, cv::Mat& dst);
cv::Mat ImgSkeletonization(cv::Mat &input_src,cv::Mat & output_dst, int number);
cv::Mat ImgSkeletonization_H(cv::Mat &input_src, int *search_arr);
cv::Mat ImgSkeletonization_V(cv::Mat &input_src, int *search_arr);

// traversal
// void traversal(cv::Mat thin_img, cv::Mat RGBimg);
void traversal(string thin_img_address, cv::Mat rgb_img);
vector<string> traversal(string s_img_address, cv::Mat rgb_img, dlo::BoundingBoxes::ConstPtr boxes);
// void traversal(cv::Mat src, string imgnum);
void traversal(cv::Mat src);
void traversal_callback(int event, int x, int y, int flags, void* ustc);
int dxdy2currentpoint(cv::Mat src_, cv::Point curr_pt, int dx, int dy, int channel = 3);
bool in_square(cv::Point center, int nx, int ny, int sqr_size);
cv::Point get_first_current_point(cv::Mat srcf, cv::Point pre_pt, int per_step, int round, int channel = 3);
cv::Point get_next_point(cv::Point curr_pt, int dx, int dy, int per_step);
cv::Point get_opposite_point(cv::Point center, int nx, int ny);
void get_point(int event, int x, int y, int flags, void* ustc);
cv::Mat convertTo3Channels(const cv::Mat& binImg);
cv::Point per_dir(cv::Point pre_pt, cv::Point curr_pt, int per_step);

// crop
void crop(cv::Mat src);
void crop_callback(int event, int x, int y, int flags, void* ustc);
cv::Mat RotateImage(cv::Mat src, double angle, cv::Point center);
double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);

// remove_background
cv::Mat remove_background(cv::Mat rgb_img, cv::Mat bw_img);
cv::Mat change_background(cv::Mat rgb_img, cv::Mat bg_img, cv::Mat bw_img, int y);
cv::Mat change_background(cv::Mat rgb_img, cv::Mat bg_img, cv::Mat bw_img);

// visualization
cv::Mat traversal_visualization(cv::Mat src);
void visualization();
cv::Scalar getcolor(int n, int ept_index, int line_index);

// strategy
int strategy();
double draw_grip_direction(int opt_index);
double draw_grip_direction(cv::Point ptTarget, cv::Point ptTargetDir);
void draw_point(cv::Point pt, string pt_text, cv::Scalar color, float text_size = 0.5, int text_thick = 1);
void cout_cross(int i);
int checklist();

// void MoveitKuka();
