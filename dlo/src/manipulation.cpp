#include "manipulation.h"

Point ptEdge = Point(EDGE, EDGE);   //  相机视野四周添加了宽度为EDGE的边界, 控制机器人时给的坐标需要将添加的边界去除
vector<vector<double>> g_vvdEndTarget = {{0, -20, 0, -20, 0, 0, 0},
                                        {0, -20, 0, -20, 0, 0, 0}};

void manipulation(SOperation oprt){
    CIiwaServo msv;
    CGripperControl mgc;
    mgc.Dual_Gripper_anypose(OPN, OPN);
    if(oprt.strOperationType == "I")
        optionI(oprt, msv, mgc);
    else if(oprt.strOperationType == "D")
        optionD(oprt, msv, mgc);
    // else if(oprt.strOperationType == "Q")
    //     optionQ(oprt, msv, mgc);
    else if(oprt.strOperationType == "IX")
        optionIX(oprt, msv, mgc);
    // else if(oprt.strOperationType == "T")
    //     optionT(oprt, msv, mgc);
    else{
        cout << "\n\n===== 【ERROR OPERATION TYPE】 " << " =====\n\n\n";
        return;
    }
    mgc.Dual_Gripper_anypose(OPN, OPN);
    double dTime1 = msv.MoveLeftEulerXYZ();
    double dTime2 = msv.MoveRightEulerXYZ();
    ros::Duration(max(dTime1, dTime2)+0.1).sleep();
}

//  左臂抓取点, 右臂抓取点, 左臂目标点, 右臂目标点
void optionI(SOperation oprt, CIiwaServo& msv, CGripperControl& mgc){
    cout << "====== Now In I ======\n";
    if(oprt.vptPoint.size()!=4){
        cout << "\n\n===== 【ERROR oprt.vptPoint.size() IN optionI】 SIZE: " << oprt.vptPoint.size() << " =====\n\n\n";
        return;
    }
    cout << oprt.vdGripperDir[0] << '\t' << oprt.vdGripperDir[1] << endl;
    //	左右臂移动到抓取位置上方
    msv.DloMoveEuler(oprt.vptPoint[0], OPRTZ, oprt.vdGripperDir[0], oprt.vptPoint[1], OPRTZ, oprt.vdGripperDir[1]);

    //  左右臂向下到夹取高度
    msv.DloMoveEulerIncrease('D', 0.1);
    //  左夹爪夹紧, 右夹爪限位
    mgc.Dual_Gripper_anypose(MDL, CLS);
    //  左臂上移, 然后移动到左臂目标点上方, 回家, 关节角控制至释放点
    msv.DloMoveEulerIncrease('L', -0.15);
    double dTime1 = msv.MoveLeftEulerXYZ();
    ros::Duration(dTime1+0.1).sleep();
    msv.MoveLeftToJoint(g_vvdEndTarget[g_nEndTarget++]);
    ros::Duration(10.1).sleep();
    // msv.DloMoveEuler('L', oprt.vptPoint[2], oprt.vdGripperDir[2], oprt.vdAddInfo[0]);
    // msv.DloMoveEulerIncrease('L', 0.1);

    //  松左夹爪松开
    mgc.Gripper_anypose('L', OPN);
    
    //  右夹爪夹紧, 右臂上移, 回家, 关节角控制至释放点
    mgc.Gripper_anypose('R', CLS);
    msv.DloMoveEulerIncrease('R', -0.15);
    dTime1 = msv.MoveRightEulerXYZ();
    ros::Duration(dTime1+0.1).sleep();
    msv.MoveRightToJoint(g_vvdEndTarget[g_nEndTarget++]);   //  操作结束后移动至下一个端点放置点
    ros::Duration(10.1).sleep();
    // msv.DloMoveEuler('R', oprt.vptPoint[3], oprt.vdGripperDir[3], oprt.vdAddInfo[1]);
    // msv.DloMoveEulerIncrease('R', 0.1);
    //  松右夹爪
    mgc.Gripper_anypose('R', OPN);

    //  扔
    msv.MoveLeftToJoint(0, -20, 0, -20, 0, 45, 0);
    msv.MoveRightToJoint(0, -20, 0, -20, 0, 45, 0);
    ros::Duration(10.1).sleep();
}

//  抓取点, 目标点
void optionD(SOperation oprt, CIiwaServo& msv, CGripperControl& mgc){
    if(oprt.vptPoint.size()!=2){
        cout << "\n\n===== 【ERROR oprt.vptPoint.size() IN optionI】 SIZE: " << oprt.vptPoint.size() << " =====\n\n\n";
        return;
    }
    char cSide;
    if(oprt.vptPoint[0].x >= MDLEGE && oprt.vptPoint[1].x >= MDLEGE)
        cSide = 'L';
    else if(oprt.vptPoint[0].x < MDLEGE && oprt.vptPoint[1].x < MDLEGE)
        cSide = 'R';
    else{
        ////////////跨左右区的操作/////////////
        cout << "ACROSS MIDDLE\n";
    }
    cout << "Side: " << cSide << endl;

    //  单臂移动到目标点上方, 向下到夹取高度, 夹取, 上移
    cout << "=1\n";
    msv.DloMoveEuler(cSide, oprt.vptPoint[0], oprt.vdGripperDir[0]);
    cout << "=2\n";
    msv.DloMoveEulerIncrease(cSide, 0.1);
    mgc.Gripper_anypose(cSide, CLS);
    msv.DloMoveEulerIncrease(cSide, -0.1);

    //  移动到目标位置上方, 下移, 松开
    msv.DloMoveEuler(cSide, oprt.vptPoint[1], oprt.vdGripperDir[1]);
    msv.DloMoveEulerIncrease(cSide, 0.1);
    mgc.Gripper_anypose(cSide, OPN);
    msv.DloMoveEulerIncrease(cSide, -0.15);
}

//  抓取点, 旋转参考点; 抓取角度, 旋转参考角度
void optionQ(SOperation oprt, CIiwaServo& msv, CGripperControl& mgc){
    if(oprt.vptPoint.size()!=1){
        cout << "\n\n===== 【ERROR oprt.vptPoint.size() IN optionI】 SIZE: " << oprt.vptPoint.size() << " =====\n\n\n";
        return;
    }
    if(oprt.vdGripperDir.size()!=2){
        cout << "\n\n===== 【ERROR oprt.vdGripperDir.size() IN optionI】 SIZE: " << oprt.vdGripperDir.size() << " =====\n\n\n";
        return;
    }

    if(oprt.vptPoint[0].x >= MDLEGE && oprt.vptPoint[1].x >= MDLEGE){  //  L
        //  左臂移动到目标点上方, 向下到夹取高度, 夹取, 上移
        msv.MoveLeftEulerXYZ(oprt.vptPoint[0]-ptEdge, OPRTZ, oprt.vdGripperDir[0]);
        ros::Duration(10.1).sleep();
        msv.MoveLeftEulerIncrease(0, 0, 0.1);
        ros::Duration(10.1).sleep();
        mgc.Gripper_anypose('L', CLS);
        msv.MoveLeftEulerIncrease(0, 0, -0.1);
        ros::Duration(10.1).sleep();

        //  确定旋转方向
        int nRotationDir = (oprt.vdGripperDir[0]-90)/(abs(oprt.vdGripperDir[0]-90)+1e-10);      //  1或-1
        //  移动到旋转参考点上方, 旋转解耦
        msv.MoveLeftEulerXYZ(oprt.vptPoint[1]-ptEdge, OPRTZ, oprt.vdGripperDir[1]);
        ros::Duration(10.1).sleep();
        msv.MoveLeftJointIncrease(0,0,0,0,0, nRotationDir*3.14);
        ros::Duration(10.1).sleep();

        //  移动到目标位置上方, 下移, 松开, 上移
        msv.MoveLeftEulerXYZ(oprt.vptPoint[0]-ptEdge, OPRTZ, oprt.vdGripperDir[0]);
        ros::Duration(10.1).sleep();
        msv.MoveLeftEulerIncrease(0, 0, 0.1);
        ros::Duration(10.1).sleep();
        mgc.Gripper_anypose('L', OPN);
        msv.MoveLeftEulerIncrease(0, 0, -0.1);
        ros::Duration(10.1).sleep();
    }
    else if(oprt.vptPoint[0].x < MDLEGE && oprt.vptPoint[1].x < MDLEGE){  //  R
        //  右臂移动到目标点上方, 向下到夹取高度, 夹取, 上移
        msv.MoveRightEulerXYZ(oprt.vptPoint[0]-ptEdge, OPRTZ, oprt.vdGripperDir[0]);
        ros::Duration(10.1).sleep();
        msv.MoveRightEulerIncrease(0, 0, 0.1);
        ros::Duration(10.1).sleep();
        mgc.Gripper_anypose('R', CLS);
        msv.MoveRightEulerIncrease(0, 0, -0.1);
        ros::Duration(10.1).sleep();

        //  确定旋转方向
        int nRotationDir = (oprt.vdGripperDir[0]-90)/(abs(oprt.vdGripperDir[0]-90)+1e-10);      //  1或-1
        //  移动到旋转参考点上方, 旋转解耦
        msv.MoveRightEulerXYZ(oprt.vptPoint[1]-ptEdge, OPRTZ, oprt.vdGripperDir[1]);
        ros::Duration(10.1).sleep();
        msv.MoveRightJointIncrease(0,0,0,0,0, nRotationDir*3.14);
        ros::Duration(10.1).sleep();

        //  移动到目标位置上方, 下移, 松开, 上移
        msv.MoveRightEulerXYZ(oprt.vptPoint[0]-ptEdge, OPRTZ, oprt.vdGripperDir[0]);
        ros::Duration(10.1).sleep();
        msv.MoveRightEulerIncrease(0, 0, 0.1);
        ros::Duration(10.1).sleep();
        mgc.Gripper_anypose('R', OPN);
        msv.MoveRightEulerIncrease(0, 0, -0.1);
        ros::Duration(10.1).sleep();
    }
}

//  抓取点, 目标点; 抓取角度*2
void optionIX(SOperation oprt, CIiwaServo& msv, CGripperControl& mgc){
    if(oprt.vptPoint.size()!=2){
        cout << "\n\n===== 【ERROR oprt.vptPoint.size() IN optionI】 SIZE: " << oprt.vptPoint.size() << " =====\n\n\n";
        return;
    }
    if(oprt.vdGripperDir.size()!=2){
        cout << "\n\n===== 【ERROR oprt.vdGripperDir.size() IN optionI】 SIZE: " << oprt.vdGripperDir.size() << " =====\n\n\n";
        return;
    }
    char cSide;
    if(oprt.vptPoint[0].x >= MDLEGE && oprt.vptPoint[1].x >= MDLEGE)
        cSide = 'L';
    else if(oprt.vptPoint[0].x < MDLEGE && oprt.vptPoint[1].x < MDLEGE)
        cSide = 'R';
    else{
        ////////////跨左右区的操作/////////////
        cout << "ACROSS MIDDLE\n";
    }

    //  单臂移动到目标点上方, 向下到夹取高度, 夹取, 上移
    msv.DloMoveEuler(cSide, oprt.vptPoint[0], oprt.vdGripperDir[0]);
    msv.DloMoveEulerIncrease(cSide, 0.1);
    mgc.Gripper_anypose(cSide, CLS);
    msv.DloMoveEulerIncrease(cSide, -0.1);

    //  移动到目标位置上方, 下移, 松开
    msv.DloMoveEuler(cSide, oprt.vptPoint[1], oprt.vdGripperDir[1]);
    msv.DloMoveEulerIncrease(cSide, 0.1);
}

//  找去点, 目标点; 抓取角度
void optionT(SOperation oprt, CIiwaServo& msv, CGripperControl& mgc){
    if(oprt.vptPoint.size()!=2){
        cout << "\n\n===== 【ERROR oprt.vptPoint.size() IN optionI】 SIZE: " << oprt.vptPoint.size() << " =====\n\n\n";
        return;
    }
    if(oprt.vdGripperDir.size()!=1){
        cout << "\n\n===== 【ERROR oprt.vdGripperDir.size() IN optionI】 SIZE: " << oprt.vdGripperDir.size() << " =====\n\n\n";
        return;
    }

    if(oprt.vptPoint[0].x >= MDLEGE && oprt.vptPoint[1].x >= MDLEGE){  //  L
        //  左臂移动到目标点上方, 向下到夹取高度, 夹取, 上移
        msv.MoveLeftEulerXYZ(oprt.vptPoint[0]-ptEdge, OPRTZ, oprt.vdGripperDir[0]);
        ros::Duration(10.1).sleep();
        msv.MoveLeftEulerIncrease(0, 0, 0.1);
        ros::Duration(10.1).sleep();
        mgc.Gripper_anypose('L', CLS);
        msv.MoveLeftEulerIncrease(0, 0, -0.1);
        ros::Duration(10.1).sleep();

        //  移动到目标位置上方, 下移, 松开
        msv.MoveLeftEulerXYZ(oprt.vptPoint[1]-ptEdge, OPRTZ, oprt.vdGripperDir[1]);
        ros::Duration(10.1).sleep();
        msv.MoveLeftEulerIncrease(0, 0, 0.1);
        ros::Duration(10.1).sleep();
        mgc.Gripper_anypose('L', OPN);
    }
    else if(oprt.vptPoint[0].x < MDLEGE && oprt.vptPoint[1].x < MDLEGE){  //  R
        //  右臂移动到目标点上方, 向下到夹取高度, 夹取, 上移
        msv.MoveRightEulerXYZ(oprt.vptPoint[0]-ptEdge, OPRTZ, oprt.vdGripperDir[0]);
        ros::Duration(10.1).sleep();
        msv.MoveRightEulerIncrease(0, 0, 0.1);
        ros::Duration(10.1).sleep();
        mgc.Gripper_anypose('R', CLS);
        msv.MoveRightEulerIncrease(0, 0, -0.1);
        ros::Duration(10.1).sleep();

        //  移动到目标位置上方, 下移, 松开
        msv.MoveRightEulerXYZ(oprt.vptPoint[1]-ptEdge, OPRTZ, oprt.vdGripperDir[1]);
        ros::Duration(10.1).sleep();
        msv.MoveRightEulerIncrease(0, 0, 0.1);
        ros::Duration(10.1).sleep();
        mgc.Gripper_anypose('R', OPN);
    }
}