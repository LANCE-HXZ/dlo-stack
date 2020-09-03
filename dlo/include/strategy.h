#include "dlo.h"
#include "kuka_moveit.h"
#include "gripper_control.h"
#include <vector>
using namespace cv;

struct SOperation{
    string strOperationType;        //  操作类型
    vector<Point> vptTarget;        //  移动目标点
    vector<double> vdGripperDir;    //  夹爪抓取方向
    vector<string> vstrMoveWay;     //  移动方式, 双臂D_GROUP, 左臂L_GROUP, 右臂R_GROUP
};

int strategy();
double draw_grip_direction(int opt_index);
double draw_grip_direction(Point ptTarget, Point ptTargetDir);
void draw_point(Point pt, string pt_text, Scalar color, float text_size, int text_thick);
void cout_cross(int i);
int checklist();