#include "manipulation.h"

Point ptEdge = Point(EDGE, EDGE);   //  相机视野四周添加了宽度为EDGE的边界, 控制机器人时给的坐标需要将添加的边界去除
vector<vector<double>>  g_vvdEndLeft = {{5.93, -43.6, -6.95, -76.65, 3.17, 57.03, 21.04}, 
                                        {11.54, -6.65, -3.59, -52.66, -19.09, 15.08, 37.05}, 
                                        {11.54, -11.57, -3.59, -60.19, -19.09, 25.98, 37.05}, 
                                        {11.54, -22.44, -3.59, -88.24, -19.09, 15.08, 37.05}, 
                                        {11.54, -21.32, -3.59, -50.56, -19.09, 3.99, 37.05}},
                        g_vvdEndRight = {{6.29, -43.89, 9.04, -77.96, -13.47, 58.72, -10.03}, 
                                        {3.12, -5.64, -5.59, -52.92, 7.61, 14.77, -29.16}, 
                                        {3.12, -8.84, -5.59, -50.64, 7.61, 36.96, -29.16}, 
                                        {3.12, -21.48, -5.59, -87.73, 7.61, 14.77, -29.16}, 
                                        {3.12, -21.40, -5.59, -55.98, 7.61, -1.12, -29.16}};

void manipulation(SOperation oprt){
    CIiwaServo msv;
    CGripperControl mgc;
    // mgc.Dual_Gripper_anypose(OPN, OPN);
    // if(oprt.strOperationType == "I")
    //     optionI(oprt, msv, mgc);
    // else if(oprt.strOperationType == "D" || oprt.strOperationType == "IX")
    //     optionD(oprt, msv, mgc);
    // else if(oprt.strOperationType == "Q")
    //     // cout << "Q\n";
    //     optionQ(oprt, msv, mgc);
    // else if(oprt.strOperationType == "T")
    //     optionT(oprt, msv, mgc);
    // else if(oprt.strOperationType == "E")
    //     return;
    // else{
    //     cout << "\n\n\t\t=====  【END】  =====\n\n\n";
    //     exit(0);
    // }
    // mgc.Dual_Gripper_anypose(OPN, OPN);
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

    //	左右臂移动到抓取位置上方
    msv.DloMoveEuler(oprt.vptPoint[0], OPRTZ, oprt.vdGripperDir[0], oprt.vdAddInfo[0], oprt.vptPoint[1], OPRTZ, oprt.vdGripperDir[1], oprt.vdAddInfo[1]);
    cout << oprt.vptPoint[0] << '\t' << oprt.vptPoint[1] << endl;
    cout << oprt.vdGripperDir[0] << '\t' << oprt.vdGripperDir[1] << endl;
    cout << oprt.vdAddInfo[0] << '\t' << oprt.vdAddInfo[1] << endl;
    //  左右臂向下到夹取高度
    msv.DloMoveEulerIncrease('D', 0.1);

    mgc.Dual_Gripper_anypose(CLS, CLS);
    msv.DloMoveEulerIncrease('D', -0.15);
    msv.MoveDualToHome(8);
        
    // msv.MoveLeftJointIncrease(0, 0, 0, 0, 0, -35, 0, 5);
    // msv.MoveRightJointIncrease(0, 0, 0, 0, 0, -35, 0, 5);
    // ros::Duration(5.1).sleep();
    // msv.MoveDualToJoint(g_vvdEndLeft[0], g_vvdEndRight[0], 5);

    if(g_nEndLeft < g_vvdEndLeft.size() && g_nEndRight < g_vvdEndRight.size())
        msv.MoveDualToJoint(g_vvdEndLeft[g_nEndLeft++], g_vvdEndRight[g_nEndRight++], 5);
    else{
        // double dTime = max(msv.MoveLeftEulerXYZ(), msv.MoveRightEulerXYZ());
        // ros::Duration(dTime+0.1).sleep();
        msv.MoveDualToJoint(g_vvdEndLeft[g_vvdEndLeft.size()-1], g_vvdEndRight[g_vvdEndRight.size()-1], 5);
    }
    
    //  松夹爪
    mgc.Dual_Gripper_anypose(OPN, OPN);

    //  关节角控制模式回家
    msv.MoveDualToHome(5);
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
    if(cSide == 'L')
        msv.MoveLeftToHome(5);
    else
        msv.MoveRightToHome(5);
}

//  抓取点, 旋转参考点; 抓取角度, 旋转参考角度
void optionQ(SOperation oprt, CIiwaServo& msv, CGripperControl& mgc){
    char cSide;
    if(oprt.vptPoint[0].x >= MDLEGE && oprt.vptPoint[1].x >= MDLEGE)
        cSide = 'L';
    else
        cSide = 'R';
    cout << "Side: " << cSide << endl;

    // 移动到目标点上方, 向下到夹取高度, 夹取, 上移
    msv.DloMoveEuler(cSide, oprt.vptPoint[0], oprt.vdGripperDir[0]);
    msv.DloMoveEulerIncrease(cSide, 0.1);
    mgc.Gripper_anypose(cSide, CLS);
    msv.DloMoveEulerIncrease(cSide, -0.1);

    //  确定旋转方向
    double nRotationDir = -(oprt.vdGripperDir[1])/(abs(oprt.vdGripperDir[1])+1e-10);      //  1或-1
    cout << "nRotationDir: " << nRotationDir << endl;
    //  移动到旋转参考点上方, 旋转解耦
    msv.DloMoveEuler(cSide, oprt.vptPoint[1], oprt.vdGripperDir[0], 0, 5, 0.625);
    msv.DloMoveEulerIncrease(cSide, 0, 0, 0, 0, 0, nRotationDir*45, 1);
    msv.DloMoveEulerIncrease(cSide, 0, 0, 0, 0, 0, nRotationDir*45, 1);
    msv.DloMoveEulerIncrease(cSide, 0, 0, 0, 0, 0, nRotationDir*45, 1);
    msv.DloMoveEulerIncrease(cSide, 0, 0, 0, 0, 0, nRotationDir*45, 1);
    msv.DloMoveEulerIncrease(cSide, 0, 0, 0, 0, 0, nRotationDir*45, 1);
    // if(cSide == 'L')
    //     msv.MoveLeftJointIncrease(0, 0, 0, 0, 0, 0, nRotationDir*180, 5);
    // else
    //     msv.MoveRightJointIncrease(0, 0, 0, 0, 0, 0, nRotationDir*180, 5);
    // ros::Duration(5.1).sleep();
    
    // 移动到目标位置上方, 下移, 松开, 上移
    vector<double> vdDxy1 = msv.PointPixel2CameraFrame(oprt.vptPoint[0]);
    vector<double> vdDxy2 = msv.PointPixel2CameraFrame(oprt.vptPoint[1]);
    cout << "dx: " << vdDxy1[0]-vdDxy2[0] << "     dy: " << vdDxy1[1]-vdDxy2[1] << endl;
    msv.DloMoveEulerIncrease(cSide, 0.1, vdDxy1[0]-vdDxy2[0], vdDxy1[1]-vdDxy2[1]);
    msv.DloMoveEulerIncrease(cSide, 0.1);
    mgc.Gripper_anypose(cSide, OPN);
    msv.DloMoveEulerIncrease(cSide, -0.15);

    //  回家
    if(cSide == 'L')
        msv.MoveLeftToHome(5);
    else
        msv.MoveRightToHome(5);
}

//  抓取点, 目标点
void optionT(SOperation oprt, CIiwaServo& msv, CGripperControl& mgc){
    char cSide;
    if(oprt.vptPoint[0].x >= MDLEGE && oprt.vptPoint[1].x >= MDLEGE)
        cSide = 'L';
    else if(oprt.vptPoint[0].x < MDLEGE && oprt.vptPoint[1].x < MDLEGE)
        cSide = 'R';
    else{
        ////////////跨左右区的操作/////////////
        if(oprt.vptPoint[0].x >= MDLEGE){    //  夹取点在左手工作区域
            cSide = 'L';
            oprt.vptPoint[1].x = MDLEGE;
        }
        else{
            cSide = 'R';
            oprt.vptPoint[1].x = MDLEGE;
        }
        cout << "ACROSS MIDDLE\n";
    }
    cout << "Side: " << cSide << endl;

    //  单臂移动到目标点上方, 向下到夹取高度, 夹取, 上移
    msv.DloMoveEuler(cSide, oprt.vptPoint[0], oprt.vdGripperDir[0]);
    msv.DloMoveEulerIncrease(cSide, 0.1);
    mgc.Gripper_anypose(cSide, CLS);

    for(int i = 0; i < 6; ++i){
        msv.DloMoveEuler(cSide, oprt.vptPoint[1], oprt.vdGripperDir[1], 0, 1, OPRTZ+0.099);
        mgc.Gripper_anypose(cSide, OPN);
        msv.DloMoveEuler(cSide, oprt.vptPoint[0], oprt.vdGripperDir[0], 0, 1, OPRTZ+0.099);
        mgc.Gripper_anypose(cSide, CLS);
    }

    //  移动到目标位置上方, 下移, 松开
    msv.DloMoveEuler(cSide, oprt.vptPoint[1], oprt.vdGripperDir[1], 0, 1, OPRTZ+0.099);
    mgc.Gripper_anypose(cSide, OPN);

    msv.DloMoveEulerIncrease(cSide, -0.15);

    //  回家
    if(cSide == 'L')
        msv.MoveLeftToHome(5);
    else
        msv.MoveRightToHome(5);
}