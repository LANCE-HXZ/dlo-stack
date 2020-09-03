#include "dlo.h"
#include "kuka_moveit.h"
#include "gripper_control.h"
using namespace cv;

// km, pt[leftindex], gc, dLeftGripDir, pt[rightindex], target_left, target_right
void optionI(CKukaMoveit km, CGripperControl gc, Point ptLeft, int dLeftGripDir, Point ptRight, int dRightGripDir, Point target_left, Point target_right){
    km.SetLeftPose(ptLeft-Point(EDGE, EDGE), 0.94, {1.57+dLeftGripDir, 0, 0});
    km.SetRightPose(ptRight-Point(EDGE, EDGE), 0.94, {1.57-0.523-dRightGripDir, 0, 0});
    km.ExecuteGroup();			

    // km.MoveToRightPose(ptRight-Point(EDGE, EDGE), 0.94, {3.14-0.523-dRightGripDir, 0, 0});
    km.MoveDdxdydz(0, 0, 0.1);
    gc.Dual_Gripper_anypose("220", "160");
    km.MoveRdxdydz(0, 0, -0.1);
    km.MoveToRightPose(target_right-Point(EDGE, EDGE), 0.94, {1.57-0.523-dRightGripDir, 0, 0});
    km.MoveRdxdydz(0, 0, 0.1);
    gc.Gripper_anypose('R', "160");
    km.MoveRdxdydz(-0.1, 0, 0);
    km.MoveRdxdydz(-0.1, 0, 0);
    // km.GoHome(R_GROUP);
    // km.MoveLdxdydz(0, 0, -0.1);
    gc.Gripper_anypose('L', "220");
    km.SetLeftPose(target_left-Point(EDGE, EDGE), 0.94, {1.57-dLeftGripDir, 0, 0});
    km.SetRightPose(-0.561, 0.225, 0.769, {1.57, 0, 0});
    // km.MoveToLeftPose(target_left-Point(EDGE, EDGE), 0.94, {1.57-dLeftGripDir, 0, 0});
    km.ExecuteGroup();
    km.MoveLdxdydz(0, 0, 0.1);
    gc.Gripper_anypose('L', "160");
    km.MoveLdxdydz(0.1, 0, 0);
    km.MoveLdxdydz(0.1, 0, 0);
    // gc.Gripper_Open('L');
    km.GoHome(L_GROUP);
}