#include "manipulation.h"

Point ptEdge = Point(EDGE, EDGE);   //  相机视野四周添加了宽度为EDGE的边界, 控制机器人时给的坐标需要将添加的边界去除

void manipulation(SOperation oprt){
    CKukaMoveit mkm;
    CGripperControl mgc;
    if(oprt.strOperationType == "I")
        optionI(oprt, mkm, mgc);
    else if(oprt.strOperationType == "D")
        optionD(oprt, mkm, mgc);
    else if(oprt.strOperationType == "Q")
        optionQ(oprt, mkm, mgc);
    else if(oprt.strOperationType == "IX")
        optionIX(oprt, mkm, mgc);
    else if(oprt.strOperationType == "T")
        optionT(oprt, mkm, mgc);
    else{
        cout << "\n\n===== 【ERROR OPERATION TYPE】 " << " =====\n\n\n";
        return;
    }

    mkm.GoHome(D_GROUP);
}

//  左臂抓取点, 右臂抓取点, 左臂目标点, 右臂目标点
void optionI(SOperation oprt, CKukaMoveit& mkm, CGripperControl& mgc){
    if(oprt.vptPoint.size()!=4){
        cout << "\n\n===== 【ERROR oprt.vptPoint.size() IN optionI】 SIZE: " << oprt.vptPoint.size() << " =====\n\n\n";
        return;
    }

    //	左右臂移动到抓取位置上方
    mkm.SetLeftPose(oprt.vptPoint[0]-ptEdge, 0.94, {1.57+oprt.vdGripperDir[0], 0, 0});
    mkm.SetRightPose(oprt.vptPoint[1]-ptEdge, 0.94, {1.57-0.523-oprt.vdGripperDir[1], 0, 0});
    mkm.ExecuteGroup();			

    //  左右臂向下到夹取高度
    mkm.MoveDdxdydz(0, 0, 0.1);
    //  左夹爪夹紧, 右夹爪限位
    mgc.Dual_Gripper_anypose(MDL, CLS);
    //  左臂上移, 然后移动到左臂目标点上方, 下移
    mkm.MoveLdxdydz(0, 0, -0.1);
    mkm.MoveToLeftPose(oprt.vptPoint[2]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[2], 0, 0});
    mkm.MoveLdxdydz(0, 0, 0.1);
    //  松左夹爪至限位位置
    mgc.Gripper_anypose('L', MDL);
    
    //  右夹爪夹紧, 右臂上移, 移动到右臂目标点上方, 下移
    mgc.Gripper_anypose('R', CLS);
    mkm.MoveRdxdydz(0, 0, -0.1);
    mkm.MoveToRightPose(oprt.vptPoint[3]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[3], 0, 0});
    mkm.MoveRdxdydz(0, 0, 0.1);
    //  松右夹爪至限位位置
    mgc.Gripper_anypose('L', MDL);

    //  ========    增加一步拉直绳子    ========

    mgc.Dual_Gripper_anypose(OPN, OPN);
}

//  抓取点, 目标点
void optionD(SOperation oprt, CKukaMoveit& mkm, CGripperControl& mgc){
    if(oprt.vptPoint.size()!=2){
        cout << "\n\n===== 【ERROR oprt.vptPoint.size() IN optionI】 SIZE: " << oprt.vptPoint.size() << " =====\n\n\n";
        return;
    }

    if(oprt.vptPoint[0].x >= MDLEGE && oprt.vptPoint[1].x >= MDLEGE){  //  L
        //  左臂移动到目标点上方, 向下到夹取高度, 夹取, 上移
        mkm.MoveToLeftPose(oprt.vptPoint[0]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[0], 0, 0});
        mkm.MoveLdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('L', CLS);
        mkm.MoveLdxdydz(0, 0, -0.1);

        //  移动到目标位置上方, 下移, 松开
        mkm.MoveToLeftPose(oprt.vptPoint[1]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[1], 0, 0});
        mkm.MoveLdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('L', OPN);
    }
    else if(oprt.vptPoint[0].x < MDLEGE && oprt.vptPoint[1].x < MDLEGE){  //  R
        //  右臂移动到目标点上方, 向下到夹取高度, 夹取, 上移
        mkm.MoveToRightPose(oprt.vptPoint[0]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[0], 0, 0});
        mkm.MoveRdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('R', CLS);
        mkm.MoveRdxdydz(0, 0, -0.1);

        //  移动到目标位置上方, 下移, 松开
        mkm.MoveToRightPose(oprt.vptPoint[1]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[1], 0, 0});
        mkm.MoveRdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('R', OPN);
    }
}

//  抓取点; 抓取角度, 解耦方向角度
void optionQ(SOperation oprt, CKukaMoveit& mkm, CGripperControl& mgc){
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
        mkm.MoveToLeftPose(oprt.vptPoint[0]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[0], 0, 0});
        mkm.MoveLdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('L', CLS);
        mkm.MoveLdxdydz(0, 0, -0.1);

        //  ========    计算旋转方向    ========
        //  ========    旋转解耦    ========

        //  移动到目标位置上方, 下移, 松开, 上移
        mkm.MoveToLeftPose(oprt.vptPoint[0]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[0], 0, 0});
        mkm.MoveLdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('L', OPN);
        mkm.MoveLdxdydz(0, 0, -0.1);
    }
    else if(oprt.vptPoint[0].x < MDLEGE && oprt.vptPoint[1].x < MDLEGE){  //  R
        //  右臂移动到目标点上方, 向下到夹取高度, 夹取, 上移
        mkm.MoveToRightPose(oprt.vptPoint[0]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[0], 0, 0});
        mkm.MoveRdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('R', CLS);
        mkm.MoveRdxdydz(0, 0, -0.1);

        //  ========    计算旋转方向    ========
        //  ========    旋转解耦    ========

        //  移动到目标位置上方, 下移, 松开, 上移
        mkm.MoveToRightPose(oprt.vptPoint[0]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[0], 0, 0});
        mkm.MoveRdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('R', OPN);
        mkm.MoveRdxdydz(0, 0, -0.1);
    }
}

//  抓取点, 目标点; 抓取角度*2
void optionIX(SOperation oprt, CKukaMoveit& mkm, CGripperControl& mgc){
    if(oprt.vptPoint.size()!=2){
        cout << "\n\n===== 【ERROR oprt.vptPoint.size() IN optionI】 SIZE: " << oprt.vptPoint.size() << " =====\n\n\n";
        return;
    }
    if(oprt.vdGripperDir.size()!=2){
        cout << "\n\n===== 【ERROR oprt.vdGripperDir.size() IN optionI】 SIZE: " << oprt.vdGripperDir.size() << " =====\n\n\n";
        return;
    }

    if(oprt.vptPoint[0].x >= MDLEGE && oprt.vptPoint[1].x >= MDLEGE){  //  L
        //  左臂移动到目标点上方, 向下到夹取高度, 夹取, 上移
        mkm.MoveToLeftPose(oprt.vptPoint[0]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[0], 0, 0});
        mkm.MoveLdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('L', CLS);
        mkm.MoveLdxdydz(0, 0, -0.1);

        //  移动到目标位置上方, 下移, 松开, 上移
        mkm.MoveToLeftPose(oprt.vptPoint[1]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[1], 0, 0});
        mkm.MoveLdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('L', OPN);
        mkm.MoveLdxdydz(0, 0, -0.1);
    }
    else if(oprt.vptPoint[0].x < MDLEGE && oprt.vptPoint[1].x < MDLEGE){  //  R
        //  右臂移动到目标点上方, 向下到夹取高度, 夹取, 上移
        mkm.MoveToRightPose(oprt.vptPoint[0]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[0], 0, 0});
        mkm.MoveRdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('R', CLS);
        mkm.MoveRdxdydz(0, 0, -0.1);

        //  移动到目标位置上方, 下移, 松开, 上移
        mkm.MoveToRightPose(oprt.vptPoint[1]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[1], 0, 0});
        mkm.MoveRdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('R', OPN);
        mkm.MoveRdxdydz(0, 0, -0.1);
    }
}

//  找去点, 目标点; 抓取角度
void optionT(SOperation oprt, CKukaMoveit& mkm, CGripperControl& mgc){
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
        mkm.MoveToLeftPose(oprt.vptPoint[0]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[0], 0, 0});
        mkm.MoveLdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('L', CLS);
        mkm.MoveLdxdydz(0, 0, -0.1);

        //  移动到目标位置上方, 下移, 松开, 上移
        mkm.MoveToLeftPose(oprt.vptPoint[1]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[0], 0, 0});
        mkm.MoveLdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('L', OPN);
        mkm.MoveLdxdydz(0, 0, -0.1);
    }
    else if(oprt.vptPoint[0].x < MDLEGE && oprt.vptPoint[1].x < MDLEGE){  //  R
        //  右臂移动到目标点上方, 向下到夹取高度, 夹取, 上移
        mkm.MoveToRightPose(oprt.vptPoint[0]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[0], 0, 0});
        mkm.MoveRdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('R', CLS);
        mkm.MoveRdxdydz(0, 0, -0.1);

        //  移动到目标位置上方, 下移, 松开, 上移
        mkm.MoveToRightPose(oprt.vptPoint[1]-ptEdge, 0.94, {1.57-oprt.vdGripperDir[0], 0, 0});
        mkm.MoveRdxdydz(0, 0, 0.1);
        mgc.Gripper_anypose('R', OPN);
        mkm.MoveRdxdydz(0, 0, -0.1);
    }
}