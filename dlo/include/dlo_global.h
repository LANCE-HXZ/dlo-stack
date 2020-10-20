#ifndef DLO_GLOBAL
#define DLO_GLOBAL

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

// #include "kuka_moveit.h"

using namespace std;

#define EDGE 80     // 图像四边增加边框的宽度

extern cv::Mat new_bw;
extern vector<cv::Point> pt, cross, dir;
extern vector<int64> g_vnClassList, g_vnCrossList;
extern vector<int> cpt, ept, start;
extern int point_num, cpt_num, ept_num, line_num, nCheckReturn;
extern cv::Scalar red, green, blue, yellow,purple, cyan;
extern vector<int> c0, c1;

struct SOperation{
    string strOperationType;        //  操作类型
    vector<cv::Point> vptPoint;        //  移动抓取点
    vector<double> vdGripperDir;    //  夹爪抓取方向
    vector<double> vdAddInfo;        //  其他需要添加的信息[是否选择距离更远的角度]
};

cv::Scalar getcolor(int n, int ept_index, int line_index);  //  strategy.cpp & visualization.cpp
cv::Point per_dir(cv::Point pre_pt, cv::Point curr_pt, int per_step);   //  strategy.cpp & test_function.cpp & traversal.cpp

#endif