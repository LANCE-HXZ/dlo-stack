#ifndef STRATEGY
#define STRATEGY

#include "kuka_moveit.h"
#include "gripper_control.h"
#include "img_processing.h"
#include "dlo_global.h"
using namespace cv;

class CStrategy{
    private:
        Mat result_img;
        Point add_text;
    
    public:
        CStrategy();
        ~CStrategy();
        // CKukaMoveit km;
	    // CGripperControl gc;
        SOperation strategy();
        double draw_grip_direction(int opt_index);
        double draw_grip_direction(cv::Point ptTarget, cv::Point ptTargetDir);
        Point draw_point(cv::Point pt, string pt_text, cv::Scalar color, float text_size = 0.5, int text_thick = 1);
        void cout_cross(int i);
        int checklist();
};
        
#endif