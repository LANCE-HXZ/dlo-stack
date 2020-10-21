#include "manipulation.h"

Point ptEdge = Point(EDGE, EDGE);   //  相机视野四周添加了宽度为EDGE的边界, 控制机器人时给的坐标需要将添加的边界去除
vector<vector<double>>  g_vvdEndLeft = {{17.59, -10.72, -16.05, -61.49, -17.53, 20.19, 39.88}},
                                        //  {4.46, -3.36, -12, -47.24, -12.83, 19.58, 44.41} 
                                        // {6.08, -8.82, -11.81, -83.11, 29.45, 65, -5.4}},
                            g_vvdEndRight = {{3, -8.87, -5.36, -50.96, 7.61, 33.94, -29.16}};
                                        //  {3, -1.56, 1.05, -39.47, 7.61, 26.85, -29.16},
                                        // {1.39, -6.7, -17.5, -82.63, -10.33, 56.24, -28.64}};

void manipulation(SOperation oprt){
    CIiwaServo msv;
    CGripperControl mgc;
    mgc.Dual_Gripper_anypose(OPN, OPN);
    if(oprt.strOperationType == "I")
        optionI(oprt, msv, mgc);
    else if(oprt.strOperationType == "D" || oprt.strOperationType == "IX" || oprt.strOperationType == "T")
        optionD(oprt, msv, mgc);
    // else if(oprt.strOperationType == "Q")
    //     optionQ(oprt, msv, mgc);
    else if(oprt.strOperationType == "E")
        return;
    else{
        cout << "\n\n\t\t=====  【END】  =====\n\n\n";
        exit(0);
    }
    mgc.Dual_Gripper_anypose(OPN, OPN);
    // double dTime1 = msv.MoveLeftEulerXYZ();
    // double dTime2 = msv.MoveRightEulerXYZ();
    // ros::Duration(max(dTime1, dTime2)+0.1).sleep();
}

//  左臂抓取点, 右臂抓取点
void optionI(SOperation oprt, CIiwaServo& msv, CGripperControl& mgc){
    cout << "optionI\n" << oprt.vptPoint.size() << endl;
    if(oprt.vptPoint[0].x >= MDLEGE && oprt.vptPoint[1].x >= MDLEGE){     //  两个操作点都在左手工作区域
        //  左臂移动到右臂操作点上方，下移，夹取，移动到右臂工作区的范围内, 下移，松开，上移，回家
        msv.DloMoveEuler('L', oprt.vptPoint[1], oprt.vdGripperDir[1]);
        msv.DloMoveEulerIncrease('L', 0.1);
        mgc.Gripper_anypose('L', CLS);
        msv.DloMoveEulerIncrease('L', -0.1);
        msv.DloMoveEuler('L', Point(350, 100), 90);
        msv.DloMoveEulerIncrease('L', 0.1);
        mgc.Gripper_anypose('L', OPN);
        msv.DloMoveEulerIncrease('L', -0.15);
        double dTime = msv.MoveLeftEulerXYZ();
        ros::Duration(dTime+0.1).sleep();
        return;
    }
    else if(oprt.vptPoint[0].x < MDLEGE && oprt.vptPoint[1].x < MDLEGE){     //  两个操作点都在右手工作区域
        //  右臂移动到左臂操作点上方，下移，夹取，移动到左臂工作区的范围内, 下移，松开，上移，回家
        msv.DloMoveEuler('R', oprt.vptPoint[0], oprt.vdGripperDir[0]);
        msv.DloMoveEulerIncrease('R', 0.1);
        mgc.Gripper_anypose('R', CLS);
        msv.DloMoveEulerIncrease('R', -0.1);
        msv.DloMoveEuler('R', Point(450, 100), 90);
        msv.DloMoveEulerIncrease('R', 0.1);
        mgc.Gripper_anypose('R', OPN);
        msv.DloMoveEulerIncrease('R', -0.15);
        double dTime = msv.MoveRightEulerXYZ();
        ros::Duration(dTime+0.1).sleep();
        return;
    }

    cout << oprt.vdGripperDir[0] << '\t' << oprt.vdGripperDir[1] << endl;
    //	左右臂移动到抓取位置上方
    msv.DloMoveEuler(oprt.vptPoint[0], OPRTZ, oprt.vdGripperDir[0], oprt.vdAddInfo[0], oprt.vptPoint[1], OPRTZ, oprt.vdGripperDir[1], oprt.vdAddInfo[1]);

    //  左右臂向下到夹取高度
    msv.DloMoveEulerIncrease('D', 0.1);

    mgc.Dual_Gripper_anypose(CLS, CLS);
    msv.DloMoveEulerIncrease('D', -0.15);
    msv.MoveLeftToHome(10);
    msv.MoveRightToHome(10);
    ros::Duration(10.1).sleep();
    if(g_nEndLeft == 0 && g_nEndRight == 0){
        msv.MoveLeftJointIncrease(0, 0, 0, 0, 0, -45, 0, 10);
        msv.MoveRightJointIncrease(0, 0, 0, 0, 0, -45, 0, 10);
        ros::Duration(10.1).sleep();
    }
    // double dTime1 = msv.MoveLeftEulerXYZ();
    // double dTime2 = msv.MoveRightEulerXYZ();
    // ros::Duration(max(dTime1, dTime2)+0.1).sleep();
    
    //  左夹爪夹紧, 右夹爪限位
    // mgc.Dual_Gripper_anypose(MDL, CLS);
    // //  左臂上移, 然后移动到左臂目标点上方, 回家, 关节角控制至释放点
    // msv.DloMoveEulerIncrease('L', -0.15);
    // double dTime1 = msv.MoveLeftEulerXYZ();
    // ros::Duration(dTime1+0.1).sleep();
    if(g_nEndLeft < g_vvdEndLeft.size())
        msv.MoveLeftToJoint(g_vvdEndLeft[g_nEndLeft++], 10);
    else{
        double dTime = msv.MoveLeftEulerXYZ();
        ros::Duration(dTime+0.1).sleep();
    }
    // //  松左夹爪松开
    // mgc.Gripper_anypose('L', OPN);
    
    // //  右夹爪夹紧, 右臂上移, 回家, 关节角控制至释放点
    // mgc.Gripper_anypose('R', CLS);
    // msv.DloMoveEulerIncrease('R', -0.15);
    // dTime1 = msv.MoveRightEulerXYZ();
    // ros::Duration(dTime1+0.1).sleep();
    if(g_nEndRight < g_vvdEndRight.size())
        msv.MoveRightToJoint(g_vvdEndRight[g_nEndRight++], 10);   //  操作结束后移动至下一个端点放置点
    else{
        double dTime = msv.MoveRightEulerXYZ();
        ros::Duration(dTime+0.1).sleep();
    }
    
    //  松右夹爪
    mgc.Gripper_anypose('L', OPN);
    mgc.Gripper_anypose('R', OPN);

    //  关节角控制模式回家
    msv.MoveLeftToHome(10);
    msv.MoveRightToHome(10);
    ros::Duration(10.1).sleep();
}

//  抓取点, 目标点
void optionD(SOperation oprt, CIiwaServo& msv, CGripperControl& mgc){
    char cSide;
    if(oprt.vptPoint[0].x >= MDLEGE && oprt.vptPoint[1].x >= MDLEGE)
        cSide = 'L';
    else if(oprt.vptPoint[0].x < MDLEGE && oprt.vptPoint[1].x < MDLEGE)
        cSide = 'R';
    else{
        ////////////跨左右区的操作/////////////
        if(oprt.vptPoint[0].x >= MDLEGE){    //  夹取点在左手工作区域
            cSide = 'L';
            oprt.vptPoint[1].x = 350;
        }
        else{
            cSide = 'R';
            oprt.vptPoint[1].x = 450;
        }
        cout << "ACROSS MIDDLE\n";
    }
    cout << "Side: " << cSide << endl;

    //  单臂移动到目标点上方, 向下到夹取高度, 夹取, 上移
    msv.DloMoveEuler(cSide, oprt.vptPoint[0], oprt.vdGripperDir[0]);
    msv.DloMoveEulerIncrease(cSide, 0.1);
    mgc.Gripper_anypose(cSide, CLS);
    msv.DloMoveEulerIncrease(cSide, -0.1);

    //  移动到目标位置上方, 下移, 松开
    msv.DloMoveEuler(cSide, oprt.vptPoint[1], oprt.vdGripperDir[1]);
    msv.DloMoveEulerIncrease(cSide, 0.1);
    mgc.Gripper_anypose(cSide, OPN);
    msv.DloMoveEulerIncrease(cSide, -0.15);

    //  回家
    double dTime1 = msv.MoveLeftEulerXYZ();
    double dTime2 = msv.MoveRightEulerXYZ();
    ros::Duration(max(dTime1, dTime2)+0.1).sleep();
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