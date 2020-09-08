#include "manipulation.h"

using namespace cv;


void manipulation(SOperation oprt){
    CKukaMoveit mkm;
    CGripperControl mgc;
    if(oprt.strOperationType == "I")
        optionI(oprt, mkm, mgc);
}

void optionI(SOperation oprt, CKukaMoveit& mkm, CGripperControl& mgc){
    //	执行机械臂和夹爪
    mkm.MoveRdxdydz(0, 0, -0.1);
    mkm.SetLeftPose(oprt.vptPoint[0]-Point(EDGE, EDGE), 0.94, {1.57+oprt.vdGripperDir[0], 0, 0});
    mkm.SetRightPose(oprt.vptPoint[1]-Point(EDGE, EDGE), 0.94, {1.57-0.523-oprt.vdGripperDir[1], 0, 0});
    mkm.ExecuteGroup();			

    mkm.MoveDdxdydz(0, 0, 0.1);
    mgc.Dual_Gripper_anypose("220", "160");
    mkm.MoveRdxdydz(0, 0, -0.1);
    mkm.MoveToRightPose(oprt.vptPoint[3]-Point(EDGE, EDGE), 0.94, {1.57-0.523-oprt.vdGripperDir[3], 0, 0});
    mkm.MoveRdxdydz(0, 0, 0.1);
    mgc.Gripper_anypose('R', "160");
    mkm.MoveRdxdydz(-0.1, 0, 0);
    mkm.MoveRdxdydz(-0.1, 0, 0);

    mgc.Gripper_anypose('L', "220");
    mkm.SetLeftPose(oprt.vptPoint[2]-Point(EDGE, EDGE), 0.94, {1.57-oprt.vdGripperDir[2], 0, 0});
    mkm.SetRightPose(-0.561, 0.225, 0.769, {1.57, 0, 0});

    mkm.ExecuteGroup();
    mkm.MoveLdxdydz(0, 0, 0.1);
    mgc.Gripper_anypose('L', "160");
    mkm.MoveLdxdydz(0.1, 0, 0);
    mkm.MoveLdxdydz(0.1, 0, 0);

    mkm.GoHome(D_GROUP);
}