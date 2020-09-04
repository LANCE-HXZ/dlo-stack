#include "kuka_moveit.h"
#include "gripper_control.h"
#include "img_processing.h"
#include "dlo_global.h"
using namespace cv;

struct SOperation{
    string strOperationType;        //  操作类型
    vector<Point> vptTarget;        //  移动目标点
    vector<double> vdGripperDir;    //  夹爪抓取方向
    vector<string> vstrGroup;     //  移动方式, 双臂D_GROUP, 左臂L_GROUP, 右臂R_GROUP
};

class CStrategy{
    private:
        Mat result_img;
        Point add_text;
        vector<int> c0, c1;
    
    public:
        CStrategy();
        
        double draw_grip_direction(int opt_index);
        double draw_grip_direction(cv::Point ptTarget, cv::Point ptTargetDir);
        void draw_point(cv::Point pt, string pt_text, cv::Scalar color, float text_size = 0.5, int text_thick = 1);
        void cout_cross(int i);
        int checklist();
};
        
