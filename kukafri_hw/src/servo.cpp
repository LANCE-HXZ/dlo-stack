#include "kukafri_hw/servo.h"

using namespace std;
 
CIiwaServo::CIiwaServo(){
    LeftClient=nh.serviceClient<kukafri_hw::setMoveMode>("/left/setMoveMode");
    LeftHomeClient=nh.serviceClient<kukafri_hw::moveToHome>("/left/MoveToHome");
    LeftEulerXYZ=nh.advertise<kukafri_hw::kukaCmdPosE>("/left/cmdPos_EulerZYX",1000);
    LeftQuaternion=nh.advertise<kukafri_hw::kukaCmdPosQ>("/left/cmdPos_Quaternion",1000);
    LeftJoint=nh.advertise<kukafri_hw::kukaCmdJoint>("/left/cmdJoint",1000);

    RightClient=nh.serviceClient<kukafri_hw::setMoveMode>("/right/setMoveMode");
    RightHomeClient=nh.serviceClient<kukafri_hw::moveToHome>("/right/MoveToHome");
    RightEulerXYZ=nh.advertise<kukafri_hw::kukaCmdPosE>("/right/cmdPos_EulerZYX",1000);
    RightQuaternion=nh.advertise<kukafri_hw::kukaCmdPosQ>("/right/cmdPos_Quaternion",1000);
    RightJoint=nh.advertise<kukafri_hw::kukaCmdJoint>("/right/cmdJoint",1000);

    LeftKukaPos=nh.subscribe("/left/curState",1,&CIiwaServo::LeftKukaCb,this);
    RightKukaPos=nh.subscribe("/right/curState",1,&CIiwaServo::RightkukaCb,this);

    homeSrv.request.no_use=0;
    SetLeftMoveMode();
    SetRightMoveMode();
    ros::Duration(3).sleep();
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

void CIiwaServo::MoveLeftEulerXYZ(double dX,double dY,double dZ,double dOx,double dOy,double dOz,double dOz,double dMoveDuration,int nPathMode){
    SetLeftMoveMode(1,nPathMode,dMoveDuration);
    vector<double> xy = PointPixel2CameraFrame(oprt.vptPoint[1]-ptEdge);
    cout << "X: " << xy[0] << "\tY: " << xy[1] << "\tDir: " << dOz << endl;
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=dX;
    msg.Y_Axis=dY;
    msg.Z_Axis=dZ;
    msg.alpha=dOx;
    msg.beta=dOy;
    msg.gamma=dOz;
    LeftEulerXYZ.publish(msg);
}
void CIiwaServo::MoveLeftEulerXYZ(Point ptTarget,double dZ,double dOz,double dOx,double dOy,double dMoveDuration,int nPathMode){
    SetLeftMoveMode(1,nPathMode,dMoveDuration);
    kukafri_hw::kukaCmdPosE msg;
    vector<double> xy = PointPixel2CameraFrame(ptTarget);
    msg.X_Axis=xy[0];
    msg.Y_Axis=xy[1];
    msg.Z_Axis=dZ;
    msg.alpha=dOx;
    msg.beta=dOy;
    msg.gamma=dOz;
    LeftEulerXYZ.publish(msg);
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

void CIiwaServo::MoveLeftEulerIncrease(double dDX,double dDY,double dDZ,double dOx,double dDOy,double dDOz,double dMoveDuration,int nPathMode){
    SetLeftMoveMode(1,nPathMode,dMoveDuration);
    ros::spinOnce();
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=m_vdLeftPos[0]+dDX;
    msg.Y_Axis=m_vdLeftPos[1]+dDY;
    msg.Z_Axis=m_vdLeftPos[2]+dDZ;
    msg.alpha=m_vdLeftEuler[0]+dOx;
    msg.beta=m_vdLeftEuler[1]+dDOy;
    msg.gamma=m_vdLeftEuler[2]+dDOz;
    LeftEulerXYZ.publish(msg);
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
    msg.joint7=m_vdLeftjoint[6]+dDJoint6;
    LeftJoint.publish(msg);
}

/******************************************************************************************************/

/****************************************右臂相关函数****************************************************/
void CIiwaServo::MoveRightToHome(double dMoveDuration){
    SetRightMoveMode(0,0,dMoveDuration);
    RightHomeClient.call(homeSrv);
}

void CIiwaServo::MoveRightEulerXYZ(double dX,double dY,double dZ,double dOx,double dOy,double dOz,double dMoveDuration,int nPathMode){
    SetRightMoveMode(1,nPathMode,dMoveDuration);
    vector<double> xy = PointPixel2CameraFrame(oprt.vptPoint[1]-ptEdge);
    cout << "X: " << xy[0] << "\tY: " << xy[1] << "\tDir: " << dOz << endl;
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=dX;
    msg.Y_Axis=dY;
    msg.Z_Axis=dZ;
    msg.alpha=dOx;
    msg.beta=dOy;
    msg.gamma=dOz;
    RightEulerXYZ.publish(msg);
}
void CIiwaServo::MoveRightEulerXYZ(Point ptTarget,double dZ,double dOz,double dOx,double dOy,double dMoveDuration,int nPathMode){
    SetRightMoveMode(1,nPathMode,dMoveDuration);
    kukafri_hw::kukaCmdPosE msg;
    vector<double> xy = PointPixel2CameraFrame(ptTarget);
    msg.X_Axis=xy[0];
    msg.Y_Axis=xy[1];
    msg.Z_Axis=dZ;
    msg.alpha=dOx;
    msg.beta=dOy;
    msg.gamma=dOz;
    RightEulerXYZ.publish(msg);
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

void CIiwaServo::MoveRightEulerIncrease(double dDX,double dDY,double dDZ,double dOx,double dDOy,double dDOz,double dMoveDuration,int nPathMode){
    SetRightMoveMode(1,nPathMode,dMoveDuration);
    ros::spinOnce();
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=m_vdRightPos[0]+dDX;
    msg.Y_Axis=m_vdRightPos[1]+dDY;
    msg.Z_Axis=m_vdRightPos[2]+dDZ;
    msg.alpha=m_vdRightEuler[0]+dOx;
    msg.beta=m_vdRightEuler[1]+dDOy;
    msg.gamma=m_vdRightEuler[2]+dDOz;
    RightEulerXYZ.publish(msg);
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