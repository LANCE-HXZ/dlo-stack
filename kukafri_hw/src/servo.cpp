#include "kukafri_hw/servo.h"

using namespace std;
 
CIiwaServo::CIiwaServo(){
    LeftClient=nh.serviceClient<kukafri_hw::setMoveMode>("/left/setMoveMode");
    LeftHomeClient=nh.serviceClient<kukafri_hw::moveToHome>("/left/MoveToHome");
    RightClient=nh.serviceClient<kukafri_hw::setMoveMode>("/right/setMoveMode");
    RightHomeClient=nh.serviceClient<kukafri_hw::moveToHome>("/right/MoveToHome");

    LeftEulerXYZ=nh.advertise<kukafri_hw::kukaCmdPosE>("/left/cmdPos_EulerZYX",1000);
    LeftQuaternion=nh.advertise<kukafri_hw::kukaCmdPosQ>("/left/cmdPos_Quaternion",1000);
    LeftJoint=nh.advertise<kukafri_hw::kukaCmdJoint>("/left/cmdJoint",1000);
    RightEulerXYZ=nh.advertise<kukafri_hw::kukaCmdPosE>("/right/cmdPos_EulerZYX",1000);
    RightQuaternion=nh.advertise<kukafri_hw::kukaCmdPosQ>("/right/cmdPos_Quaternion",1000);
    RightJoint=nh.advertise<kukafri_hw::kukaCmdJoint>("/right/cmdJoint",1000);

    LeftKukaPos=nh.subscribe("/left/curState",1,&CIiwaServo::LeftKukaCb,this);
    RightKukaPos=nh.subscribe("/right/curState",1,&CIiwaServo::RightkukaCb,this);

    homeSrv.request.no_use=0;
    SetLeftMoveMode();
    SetRightMoveMode();
    ros::Duration(0.1).sleep();
}

CIiwaServo::~CIiwaServo()
{}

void CIiwaServo::LeftKukaCb(const kukafri_hw::kukaState::ConstPtr& msg){
    m_vdLeftPos=msg->Pos;
    m_vdLeftEuler=msg->EulerZYX;
    m_vdLeftjoint=msg->Joints;
}

void CIiwaServo::RightkukaCb(const kukafri_hw::kukaState::ConstPtr& msg){
    m_vdRightPos=msg->Pos;
    m_vdRightEuler=msg->EulerZYX;
    m_vdRightjoint=msg->Joints;
}


void CIiwaServo::SetLeftMoveMode(int nMoveMode,int nPathMode,double dMoveDuration){
    kukafri_hw::setMoveMode leftsrv;
    leftsrv.request.moveMode=nMoveMode;
    leftsrv.request.pathMode=nPathMode;
    leftsrv.request.moveDuration=dMoveDuration;
    LeftClient.call(leftsrv);
}

void CIiwaServo::SetRightMoveMode(int nMoveMode,int nPathMode,double dMoveDuration){
    kukafri_hw::setMoveMode rightsrv;
    rightsrv.request.moveMode=nMoveMode;
    rightsrv.request.pathMode=nPathMode;
    rightsrv.request.moveDuration=dMoveDuration;
    RightClient.call(rightsrv);
}

/****************************************左臂相关函数****************************************************/
void CIiwaServo::MoveLeftToHome(double dMoveDuration){
    SetLeftMoveMode(0,0,dMoveDuration);
    LeftHomeClient.call(homeSrv);
}

double CIiwaServo::MoveLeftEulerXYZ(double dX,double dY,double dZ,double dOx,double dOy,double dOz,double dMoveDuration,int nPathMode){
    if(dMoveDuration == 10.01) dMoveDuration = UniformTime('L', {dX, dY, dZ, dOx, dOy, dOz});
    SetLeftMoveMode(1,nPathMode,dMoveDuration);
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=dX;
    msg.Y_Axis=dY;
    msg.Z_Axis=dZ;
    msg.alpha=dOx;
    msg.beta=dOy;
    msg.gamma=dOz;
    LeftEulerXYZ.publish(msg);
    return dMoveDuration;
}
double CIiwaServo::MoveLeftEulerXYZ(Point ptTarget,double dZ,double dOz,double dMoveDuration,double dOx,double dOy,int nPathMode){
    vector<double> xy = PointPixel2CameraFrame(ptTarget);
    cout << "X: " << xy[0] << "\tY: " << xy[1] << "\tDir: " << dOz << endl;
    if(dMoveDuration == 10.01) dMoveDuration = UniformTime('L', {xy[0], xy[1], dZ, dOx, dOy, dOz});
    SetLeftMoveMode(1,nPathMode,dMoveDuration);
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=xy[0];
    msg.Y_Axis=xy[1];
    msg.Z_Axis=dZ;
    msg.alpha=dOx;
    msg.beta=dOy;
    msg.gamma=dOz;
    LeftEulerXYZ.publish(msg);
    return dMoveDuration;
}

void CIiwaServo::MoveLeftQuaternion(double dX,double dY,double dZ,double dOx,double dOy,double dOz,double dOw,double dMoveDuration,int nPathMode){
    SetLeftMoveMode(1,nPathMode,dMoveDuration);
    kukafri_hw::kukaCmdPosQ msg;
    msg.X_Axis=dX;
    msg.Y_Axis=dY;
    msg.Z_Axis=dZ;
    msg.x=dOx;
    msg.y=dOy;
    msg.z=dOz;
    msg.w=dOw;
    LeftQuaternion.publish(msg);
}

void CIiwaServo::MoveLeftToJoint(double dJoint1,double dJoint2,double dJoint3,double dJoint4,double dJoint5,double dJoint6,double dJoint7,double dMoveDuration,int nPathMode){
    SetLeftMoveMode(0,nPathMode,dMoveDuration);
    kukafri_hw::kukaCmdJoint msg;
    msg.joint1=dJoint1;
    msg.joint2=dJoint2;
    msg.joint3=dJoint3;
    msg.joint4=dJoint4;
    msg.joint5=dJoint5;
    msg.joint6=dJoint6;
    msg.joint7=dJoint7;
    LeftJoint.publish(msg);
}
void CIiwaServo::MoveLeftToJoint(vector<double> vdJoint,double dMoveDuration,int nPathMode){
    SetLeftMoveMode(0,nPathMode,dMoveDuration);
    kukafri_hw::kukaCmdJoint msg;
    msg.joint1=vdJoint[0];
    msg.joint2=vdJoint[1];
    msg.joint3=vdJoint[2];
    msg.joint4=vdJoint[3];
    msg.joint5=vdJoint[4];
    msg.joint6=vdJoint[5];
    msg.joint7=vdJoint[6];
    LeftJoint.publish(msg);
    ros::Duration(dMoveDuration+0.1).sleep();
}

double CIiwaServo::MoveLeftEulerIncrease(double dDX,double dDY,double dDZ,double dDOx,double dDOy,double dDOz,double dMoveDuration,int nPathMode){
    ros::spinOnce();
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=m_vdLeftPos[0]+dDX;
    msg.Y_Axis=m_vdLeftPos[1]+dDY;
    msg.Z_Axis=m_vdLeftPos[2]+dDZ;
    msg.alpha=m_vdLeftEuler[0]+dDOx;
    msg.beta=m_vdLeftEuler[1]+dDOy;
    msg.gamma=m_vdLeftEuler[2]+dDOz;
    if(dMoveDuration == 10.01) dMoveDuration = UniformTime('L', {msg.X_Axis, msg.Y_Axis, msg.Z_Axis, msg.alpha, msg.beta, msg.gamma});
    SetLeftMoveMode(1,nPathMode,dMoveDuration);
    LeftEulerXYZ.publish(msg);
    return dMoveDuration;
}

void CIiwaServo::MoveLeftQuaternionIncrease(double dDX,double dDY,double dDZ,double dDOx,double dDOy,double dDOz,double dDOw,double dMoveDuration,int nPathMode){
    SetLeftMoveMode(1,nPathMode,dMoveDuration);
    ros::spinOnce();
    kukafri_hw::kukaCmdPosQ msg;
    msg.X_Axis=m_vdLeftPos[0]+dDX;
    msg.Y_Axis=m_vdLeftPos[1]+dDY;
    msg.Z_Axis=m_vdLeftPos[2]+dDZ;
    msg.x=m_vdLeftPos[3]+dDOx;
    msg.y=m_vdLeftPos[4]+dDOy;
    msg.z=m_vdLeftPos[5]+dDOz;
    msg.w=m_vdLeftPos[6]+dDOw;
    LeftQuaternion.publish(msg);
}

void CIiwaServo::MoveLeftJointIncrease(double dDJoint1,double dDJoint2,double dDJoint3,double dDJoint4,double dDJoint5,double dDJoint6,double dDJoint7,double dMoveDuration,int nPathMode){
    SetLeftMoveMode(0,nPathMode,dMoveDuration);
    ros::spinOnce();
    kukafri_hw::kukaCmdJoint msg;
    msg.joint1=m_vdLeftjoint[0]+dDJoint1;
    msg.joint2=m_vdLeftjoint[1]+dDJoint2;
    msg.joint3=m_vdLeftjoint[2]+dDJoint3;
    msg.joint4=m_vdLeftjoint[3]+dDJoint4;
    msg.joint5=m_vdLeftjoint[4]+dDJoint5;
    msg.joint6=m_vdLeftjoint[5]+dDJoint6;
    msg.joint7=m_vdLeftjoint[6]+dDJoint7;
    LeftJoint.publish(msg);
}

/******************************************************************************************************/

/****************************************右臂相关函数****************************************************/
void CIiwaServo::MoveRightToHome(double dMoveDuration){
    SetRightMoveMode(0,0,dMoveDuration);
    RightHomeClient.call(homeSrv);
}

double CIiwaServo::MoveRightEulerXYZ(double dX,double dY,double dZ,double dOx,double dOy,double dOz,double dMoveDuration,int nPathMode){
    if(dMoveDuration == 10.01) dMoveDuration = UniformTime('R', {dX, dY, dZ, dOx, dOy, dOz});
    SetRightMoveMode(1,nPathMode,dMoveDuration);
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=dX;
    msg.Y_Axis=dY;
    msg.Z_Axis=dZ;
    msg.alpha=dOx;
    msg.beta=dOy;
    msg.gamma=dOz;
    RightEulerXYZ.publish(msg);
    return dMoveDuration;
}
double CIiwaServo::MoveRightEulerXYZ(Point ptTarget,double dZ,double dOz,double dMoveDuration,double dOx,double dOy,int nPathMode){
    vector<double> xy = PointPixel2CameraFrame(ptTarget);
    cout << "X: " << xy[0] << "\tY: " << xy[1] << "\tDir: " << dOz << endl;
    if(dMoveDuration == 10.01) dMoveDuration = UniformTime('R', {xy[0], xy[1], dZ, dOx, dOy, dOz});
    SetRightMoveMode(1,nPathMode,dMoveDuration);
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=xy[0];
    msg.Y_Axis=xy[1];
    msg.Z_Axis=dZ;
    msg.alpha=dOx;
    msg.beta=dOy;
    msg.gamma=dOz;
    RightEulerXYZ.publish(msg);
    return dMoveDuration;
}

void CIiwaServo::MoveRightQuaternion(double dX,double dY,double dZ,double dOx,double dOy,double dOz,double dOw,double dMoveDuration,int nPathMode){
    SetRightMoveMode(1,nPathMode,dMoveDuration);
    kukafri_hw::kukaCmdPosQ msg;
    msg.X_Axis=dX;
    msg.Y_Axis=dY;
    msg.Z_Axis=dZ;
    msg.x=dOx;
    msg.y=dOy;
    msg.z=dOz;
    msg.w=dOw;
    RightQuaternion.publish(msg);
}

void CIiwaServo::MoveRightToJoint(double dJoint1,double dJoint2,double dJoint3,double dJoint4,double dJoint5,double dJoint6,double dJoint7,double dMoveDuration,int nPathMode){
    SetRightMoveMode(0,nPathMode,dMoveDuration);
    kukafri_hw::kukaCmdJoint msg;
    msg.joint1=dJoint1;
    msg.joint2=dJoint2;
    msg.joint3=dJoint3;
    msg.joint4=dJoint4;
    msg.joint5=dJoint5;
    msg.joint6=dJoint6;
    msg.joint7=dJoint7;
    RightJoint.publish(msg);
}
void CIiwaServo::MoveRightToJoint(vector<double> vdJoint,double dMoveDuration,int nPathMode){
    SetRightMoveMode(0,nPathMode,dMoveDuration);
    kukafri_hw::kukaCmdJoint msg;
    msg.joint1=vdJoint[0];
    msg.joint2=vdJoint[1];
    msg.joint3=vdJoint[2];
    msg.joint4=vdJoint[3];
    msg.joint5=vdJoint[4];
    msg.joint6=vdJoint[5];
    msg.joint7=vdJoint[6];
    RightJoint.publish(msg);
    ros::Duration(dMoveDuration+0.1).sleep();
}

double CIiwaServo::MoveRightEulerIncrease(double dDX,double dDY,double dDZ,double dDOx,double dDOy,double dDOz,double dMoveDuration,int nPathMode){
    ros::spinOnce();
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=m_vdRightPos[0]+dDX;
    msg.Y_Axis=m_vdRightPos[1]+dDY;
    msg.Z_Axis=m_vdRightPos[2]+dDZ;
    msg.alpha=m_vdRightEuler[0]+dDOx;
    msg.beta=m_vdRightEuler[1]+dDOy;
    msg.gamma=m_vdRightEuler[2]+dDOz;
    if(dMoveDuration == 10.01) dMoveDuration = UniformTime('R', {msg.X_Axis, msg.Y_Axis, msg.Z_Axis, msg.alpha, msg.beta, msg.gamma});
    SetRightMoveMode(1,nPathMode,dMoveDuration);
    RightEulerXYZ.publish(msg);
    return dMoveDuration;
}

void CIiwaServo::MoveRightQuaternionIncrease(double dDX,double dDY,double dDZ,double dDOx,double dDOy,double dDOz,double dDOw,double dMoveDuration,int nPathMode){
    SetRightMoveMode(1,nPathMode,dMoveDuration);
    ros::spinOnce();
    kukafri_hw::kukaCmdPosQ msg;
    msg.X_Axis=m_vdRightPos[0]+dDX;
    msg.Y_Axis=m_vdRightPos[1]+dDY;
    msg.Z_Axis=m_vdRightPos[2]+dDZ;
    msg.x=m_vdRightPos[3]+dDOx;
    msg.y=m_vdRightPos[4]+dDOy;
    msg.z=m_vdRightPos[5]+dDOz;
    msg.w=m_vdRightPos[6]+dDOw;
    RightQuaternion.publish(msg);
}

void CIiwaServo::MoveRightJointIncrease(double dDJoint1,double dDJoint2,double dDJoint3,double dDJoint4,double dDJoint5,double dDJoint6,double dDJoint7,double dMoveDuration,int nPathMode){
    SetRightMoveMode(0,nPathMode,dMoveDuration);
    ros::spinOnce();
    kukafri_hw::kukaCmdJoint msg;
    msg.joint1=m_vdRightjoint[0]+dDJoint1;
    msg.joint2=m_vdRightjoint[1]+dDJoint2;
    msg.joint3=m_vdRightjoint[2]+dDJoint3;
    msg.joint4=m_vdRightjoint[3]+dDJoint4;
    msg.joint5=m_vdRightjoint[4]+dDJoint5;
    msg.joint6=m_vdRightjoint[5]+dDJoint6;
    msg.joint7=m_vdRightjoint[6]+dDJoint7;
    RightJoint.publish(msg);
}

/******************************************************************************************************/

/****************************************双臂相关函数****************************************************/
void CIiwaServo::MoveDualToHome(double dMoveDuration){
    SetRightMoveMode(0,0,dMoveDuration);
    SetRightMoveMode(0,0,dMoveDuration);
    RightHomeClient.call(homeSrv);
    LeftHomeClient.call(homeSrv);
}

/******************************************************************************************************/

/*  将图像的像素坐标(col, row)转换为相机坐标的(x, y), 像素相关参数在 move_kuka.h 文件宏定义中修改
    输入: 像素坐标系下的 cv::Point(col, row)
    输出: 相机坐标系下的 {x, y}  */
vector<double> CIiwaServo::PointPixel2CameraFrame(Point ptPixel){
    double fix_x = 0.01*(8.6/267.33*(ptPixel.x-352.67)-0.5);        //  /*
    double fix_y = 0.01*(5.8/82*(ptPixel.y-298.75)+5);              //      手动测量误差后进行补偿, 相机移动或重新标定后需自行设计补偿方程  */
    return {(ptPixel.x-PAN_X)/PIXELS_OF_1MX+fix_x, (ptPixel.y-PAN_Y)/PIXELS_OF_1MY-fix_y};
}

/*  根据夹角计算机械臂欧拉角度时可能得到相差180的两个角度值，输入较大的一个值，比较从当前角度分别旋转至两个目标角度所需的行程角度，输出所需行程较小的那个目标角度
    输入: cSide左夹爪或右夹爪（‘L'为左，其余任意值为右），dBiggerAngle两个目标角度的较大值，nAxis旋转轴（0 = X，1 = Y，2 = Z，默认为绕Z轴旋转的角度，
    dIfFar是否选择更近的角度，为０则选近角，为１则选远角）
    输出: 末端欧拉角位姿所应到达的nAxis轴所对应的相对于参考坐标系的角度 */
double CIiwaServo::CloserAngle(char cSide, double dBiggerAngle, double dIfFar, int nAxis){
    if(nAxis < 0 || nAxis > 2){
        cout << "\n\n===== 【ERROR AXIS IN CloserAngle()】 nAxis: " << nAxis << " =====\n\n\n";
        return 0;
    }
    if(dBiggerAngle < 0 || dBiggerAngle > 180){
        cout << "\n\n===== 【ERROR BIGGERANGLE IN CloserAngle()】 dBiggerAngle: " << dBiggerAngle << " =====\n\n\n";
        return 0;
    }
    ros::spinOnce();
    double dSmallerAngle = dBiggerAngle - 180;
    double dCurAngle = (cSide == 'L') ? m_vdLeftEuler[nAxis] : m_vdRightEuler[nAxis];
    double dAbsToBigger = abs(dBiggerAngle - dCurAngle);
    double dAbsToSmaller = abs(dSmallerAngle - dCurAngle);
    if(dIfFar)
        return dAbsToSmaller < dAbsToBigger ? dBiggerAngle : dSmallerAngle;
    return dAbsToSmaller < dAbsToBigger ? dSmallerAngle : dBiggerAngle;
}

/*  计算等速运动所需的时间, 根据当前位置xyz和目标xyz计算距离后除以速度得到所需运动周期
    输入: cSide左夹爪或右夹爪（‘L'为左，'R'为右），ptOprt目标位置的像素坐标x和y，z2目标位置的z(m), 运动速度(m/s)
    输出: 所需运动周期(s) */
// double CIiwaServo::UniformTime(char cSide, Point ptOprt, double z2, double dSpeed)
double CIiwaServo::UniformTime(char cSide, vector<double> vdTargetPos, double dSpeed){
    if(dSpeed > 0.1){
        cout << "\n\n===== 【SPEED IS TOO FAST】 dSpeed: " << dSpeed << " =====\n\n\n";
        return 10;
    }
    ros::spinOnce();
    // vector<double> xy = PointPixel2CameraFrame(ptOprt-PTEDGE);
    double x1, y1, z1, ox1, oy1, oz1;
    double x2 = vdTargetPos[0], y2 = vdTargetPos[1], z2 = vdTargetPos[2],
            ox2 = vdTargetPos[3], oy2 = vdTargetPos[4], oz2 = vdTargetPos[5];
    if(cSide == 'L'){
        x1 = m_vdLeftPos[0];
        y1 = m_vdLeftPos[1];
        z1 = m_vdLeftPos[2];
        ox1 = m_vdLeftEuler[0];
        oy1 = m_vdLeftEuler[1];
        oz1 = m_vdLeftEuler[2];
    }
    else if(cSide == 'R'){
        x1 = m_vdRightPos[0];
        y1 = m_vdRightPos[1];
        z1 = m_vdRightPos[2];
        ox1 = m_vdRightEuler[0];
        oy1 = m_vdRightEuler[1];
        oz1 = m_vdRightEuler[2];
    }
    double dSpeedPos = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))/dSpeed;
    double dSpeedRuler = max(max(abs(ox1-ox2), abs(oy1-oy2)), abs(oz1-oz2))/30.0;
    cout << "dSpeedPos: " << dSpeedPos << "     dSpeedRuler: " << dSpeedRuler << endl;
    return max(dSpeedPos, dSpeedRuler);
}
// double CIiwaServo::UniformTime(char cSide, double x2, double y2, double z2, double dSpeed){
//     if(dSpeed > 0.1){
//         cout << "\n\n===== 【SPEED IS TOO FAST】 dSpeed: " << dSpeed << " =====\n\n\n";
//         return 10;
//     }
//     ros::spinOnce();
//     double x1, y1, z1;
//     if(cSide == 'L'){
//         x1 = m_vdLeftPos[0];
//         y1 = m_vdLeftPos[1];
//         z1 = m_vdLeftPos[2];
//     }
//     else if(cSide == 'R'){
//         x1 = m_vdRightPos[0];
//         y1 = m_vdRightPos[1];
//         z1 = m_vdRightPos[2];
//     }
//     return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))/dSpeed;
// }

/*  封装一层, 增加cSide参数以减少左右臂的重复代码, 将一些重复步骤封装进函数内, 简化代码
    输入: cSide左夹爪或右夹爪（‘L'为左，'R'为右），ptOprt像素坐标系的操作点，dOprtZ夹取高度 */
void CIiwaServo::DloMoveEuler(char cSide, Point ptOprt, double dGripperDir, double dIfFar, double dMoveDuration, double dOprtZ){
    if(cSide == 'L')
        dMoveDuration = MoveLeftEulerXYZ(ptOprt-PTEDGE, dOprtZ-L_DZ, CloserAngle(cSide, dGripperDir, dIfFar), dMoveDuration);
    else if(cSide == 'R')
        dMoveDuration = MoveRightEulerXYZ(ptOprt-PTEDGE, dOprtZ-R_DZ, CloserAngle(cSide, dGripperDir, dIfFar), dMoveDuration);
    else{
        cout << "\n\n===== 【ERROR SIDE IN DloMoveEuler()】 cSide: " << cSide << " =====\n\n\n";
        return;
    }
    ros::Duration(dMoveDuration + 0.1).sleep();
}
/*  函数重载, 用在当左右臂需要同时进行移动而移动参数不一样时   */
void CIiwaServo::DloMoveEuler(Point ptOprtL, double dOprtZL, double dGripperDirL, double dIsFarL, Point ptOprtR, double dOprtZR, double dGripperDirR, double dIsFarR, double dMoveDuration){
    double dTime1 = MoveLeftEulerXYZ(ptOprtL-PTEDGE, dOprtZL-L_DZ, CloserAngle('L', dGripperDirL, dIsFarL), dMoveDuration);
    double dTime2 = MoveRightEulerXYZ(ptOprtR-PTEDGE, dOprtZR-R_DZ, CloserAngle('R', dGripperDirR, dIsFarR), dMoveDuration);
    dMoveDuration = max(dTime1, dTime2);
    ros::Duration(dMoveDuration + 0.1).sleep();
}

/*  封装一层, 增加cSide参数以减少左右臂的重复代码, 将一些重复步骤封装进函数内, 简化代码
    输入: cSide左夹爪或右夹爪（‘L'为左，'R'为右, 'D'为双臂同时），其余参数同MoveLeftEulerIncrease() */
void CIiwaServo::DloMoveEulerIncrease(char cSide,double dDZ,double dDX,double dDY,double dDOx,double dDOy,double dDOz,double dMoveDuration,int nPathMode){
    ros::spinOnce();
    if(cSide == 'L'){
        dMoveDuration = MoveLeftEulerIncrease(dDX, dDY, dDZ, dDOx, dDOy, dDOz, dMoveDuration, nPathMode);
        ros::Duration(dMoveDuration + 0.1).sleep();
    }
    else if(cSide == 'R'){
        dMoveDuration = MoveRightEulerIncrease(dDX, dDY, dDZ, dDOx, dDOy, dDOz, dMoveDuration, nPathMode);
        ros::Duration(dMoveDuration + 0.1).sleep();
    }
    else if(cSide == 'D'){
        double dTime1 = MoveLeftEulerIncrease(dDX, dDY, dDZ, dDOx, dDOy, dDOz, dMoveDuration, nPathMode);
        double dTime2 = MoveRightEulerIncrease(dDX, dDY, dDZ, dDOx, dDOy, dDOz, dMoveDuration, nPathMode);
        dMoveDuration = max(dTime1, dTime2);
        ros::Duration(dMoveDuration + 0.1).sleep();
    }
}
/*  函数重载, 用在当左右臂需要同时进行移动而移动参数不一样时
    输入: 两个vector<double>, 第一个表示左臂移动参数, 第二个表示右臂移动参数, 两个vector<double>的内容及其余参数同MoveLeftEulerIncrease()   */
void CIiwaServo::DloMoveEulerIncrease(vector<double> dDLeft, vector<double> dDRight, double dMoveDuration, int nPathMode){
    double dTime1 = MoveLeftEulerIncrease(dDLeft[0], dDLeft[1], dDLeft[2], dDLeft[3], dDLeft[4], dDLeft[5], dMoveDuration, nPathMode);
    double dTime2 = MoveRightEulerIncrease(dDRight[0], dDRight[1], dDRight[2], dDRight[3], dDRight[4], dDRight[5], dMoveDuration, nPathMode);
    dMoveDuration = max(dTime1, dTime2);
    ros::Duration(dMoveDuration + 0.1).sleep();
}

// void CIiwaServo::DloMoveToHome(char cSide){
//     ros::spinOnce();
//     double x2, y2, z2, dTime;
//     if(cSide == 'L'){
//         x2 = m_vdLeftPos[0]+dDX;
//         y2 = m_vdLeftPos[1]+dDY;
//         z2 = m_vdLeftPos[2]+dDZ;
//         dTime = UniformTime(cSide, x2, y2, z2);
//     }
//     else if(cSide == 'R' || cSide == 'D'){
//         x2 = m_vdRightPos[0]+dDX;
//         y2 = m_vdRightPos[1]+dDY;
//         z2 = m_vdRightPos[2]+dDZ;
//         dTime = UniformTime('R', x2, y2, z2);
//     }
//     }
// }