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


void CIiwaServo::SetLeftMoveMode(int moveMode,int pathMode,double moveDuration){
    kukafri_hw::setMoveMode leftsrv;
    leftsrv.request.moveMode=moveMode;
    leftsrv.request.pathMode=pathMode;
    leftsrv.request.moveDuration=moveDuration;
    LeftClient.call(leftsrv);
}

void CIiwaServo::SetRightMoveMode(int moveMode,int pathMode,double moveDuration){
    kukafri_hw::setMoveMode rightsrv;
    rightsrv.request.moveMode=moveMode;
    rightsrv.request.pathMode=pathMode;
    rightsrv.request.moveDuration=moveDuration;
    RightClient.call(rightsrv);
}

/****************************************左臂相关函数****************************************************/
void CIiwaServo::MoveLeftToHome(double time){
    SetLeftMoveMode(0,0,time);
    LeftHomeClient.call(homeSrv);
}

void CIiwaServo::MoveLeftEulerXYZ(double X_Axis,double Y_Axis,double Z_Axis,double alpha,double beta,double gamma,double time,int path){
    SetLeftMoveMode(1,path,time);
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=X_Axis;
    msg.Y_Axis=Y_Axis;
    msg.Z_Axis=Z_Axis;
    msg.alpha=alpha;
    msg.beta=beta;
    msg.gamma=gamma;
    LeftEulerXYZ.publish(msg);
}

void CIiwaServo::MoveLeftQuaternion(double X_Axis,double Y_Axis,double Z_Axis,double x,double y,double z,double w,double time,int path){
    SetLeftMoveMode(1,path,time);
    kukafri_hw::kukaCmdPosQ msg;
    msg.X_Axis=X_Axis;
    msg.Y_Axis=Y_Axis;
    msg.Z_Axis=Z_Axis;
    msg.x=x;
    msg.y=y;
    msg.z=z;
    msg.w=w;
    LeftQuaternion.publish(msg);
}

void CIiwaServo::MoveLeftToJoint(double Joint1,double Joint2,double Joint3,double Joint4,double Joint5,double Joint6,double Joint7,double time,int path){
    SetLeftMoveMode(0,path,time);
    kukafri_hw::kukaCmdJoint msg;
    msg.joint1=Joint1;
    msg.joint2=Joint2;
    msg.joint3=Joint3;
    msg.joint4=Joint4;
    msg.joint5=Joint5;
    msg.joint6=Joint6;
    msg.joint7=Joint7;
    LeftJoint.publish(msg);
}

void CIiwaServo::MoveLeftEulerIncrease(double dx,double dy,double dz,double dalpha,double dbeta,double dgamma,double time,int path){
    SetLeftMoveMode(1,path,time);
    ros::spinOnce();
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=m_vdLeftPos[0]+dx;
    msg.Y_Axis=m_vdLeftPos[1]+dy;
    msg.Z_Axis=m_vdLeftPos[2]+dz;
    msg.alpha=m_vdLeftEuler[0]+dalpha;
    msg.beta=m_vdLeftEuler[1]+dbeta;
    msg.gamma=m_vdLeftEuler[2]+dgamma;
    LeftEulerXYZ.publish(msg);
}

void CIiwaServo::MoveDLeftQuaternion(double dX,double dY,double dZ,double dx,double dy,double dz,double dw,double time,int path){
    SetLeftMoveMode(1,path,time);
    ros::spinOnce();
    kukafri_hw::kukaCmdPosQ msg;
    msg.X_Axis=m_vdLeftPos[0]+dX;
    msg.Y_Axis=m_vdLeftPos[1]+dY;
    msg.Z_Axis=m_vdLeftPos[2]+dZ;
    msg.x=m_vdLeftPos[3]+dx;
    msg.y=m_vdLeftPos[4]+dy;
    msg.z=m_vdLeftPos[5]+dz;
    msg.w=m_vdLeftPos[6]+dw;
    LeftQuaternion.publish(msg);
}

void CIiwaServo::MoveDLeftJoint(double dJoint1,double dJoint2,double dJoint3,double dJoint4,double dJoint5,double dJoint6,double dJoint7,double time,int path){
    SetLeftMoveMode(0,path,time);
    ros::spinOnce();
    kukafri_hw::kukaCmdJoint msg;
    msg.joint1=m_vdLeftjoint[0]+dJoint1;
    msg.joint2=m_vdLeftjoint[1]+dJoint2;
    msg.joint3=m_vdLeftjoint[2]+dJoint3;
    msg.joint4=m_vdLeftjoint[3]+dJoint4;
    msg.joint5=m_vdLeftjoint[4]+dJoint5;
    msg.joint6=m_vdLeftjoint[5]+dJoint6;
    msg.joint7=m_vdLeftjoint[6]+dJoint6;
    LeftJoint.publish(msg);
}

/******************************************************************************************************/

/****************************************右臂相关函数****************************************************/
void CIiwaServo::MoveRightToHome(double time){
    SetRightMoveMode(0,0,time);
    RightHomeClient.call(homeSrv);
}

void CIiwaServo::MoveRightEulerXYZ(double X_Axis,double Y_Axis,double Z_Axis,double alpha,double beta,double gamma,double time,int path){
    SetRightMoveMode(1,path,time);
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=X_Axis;
    msg.Y_Axis=Y_Axis;
    msg.Z_Axis=Z_Axis;
    msg.alpha=alpha;
    msg.beta=beta;
    msg.gamma=gamma;
    RightEulerXYZ.publish(msg);
}

void CIiwaServo::MoveRightQuaternion(double X_Axis,double Y_Axis,double Z_Axis,double x,double y,double z,double w,double time,int path){
    SetRightMoveMode(1,path,time);
    kukafri_hw::kukaCmdPosQ msg;
    msg.X_Axis=X_Axis;
    msg.Y_Axis=Y_Axis;
    msg.Z_Axis=Z_Axis;
    msg.x=x;
    msg.y=y;
    msg.z=z;
    msg.w=w;
    RightQuaternion.publish(msg);
}

void CIiwaServo::MoveRightJoint(double Joint1,double Joint2,double Joint3,double Joint4,double Joint5,double Joint6,double Joint7,double time,int path){
    SetRightMoveMode(0,path,time);
    kukafri_hw::kukaCmdJoint msg;
    msg.joint1=Joint1;
    msg.joint2=Joint2;
    msg.joint3=Joint3;
    msg.joint4=Joint4;
    msg.joint5=Joint5;
    msg.joint6=Joint6;
    msg.joint7=Joint7;
    RightJoint.publish(msg);
}

void CIiwaServo::MoveDRightEulerXYZ(double dx,double dy,double dz,double dalpha,double dbeta,double dgamma,double time,int path){
    SetRightMoveMode(1,path,time);
    ros::spinOnce();
    kukafri_hw::kukaCmdPosE msg;
    msg.X_Axis=m_vdRightPos[0]+dx;
    msg.Y_Axis=m_vdRightPos[1]+dy;
    msg.Z_Axis=m_vdRightPos[2]+dz;
    msg.alpha=m_vdRightEuler[0]+dalpha;
    msg.beta=m_vdRightEuler[1]+dbeta;
    msg.gamma=m_vdRightEuler[2]+dgamma;
    RightEulerXYZ.publish(msg);
}

void CIiwaServo::MoveDRightQuaternion(double dX,double dY,double dZ,double dx,double dy,double dz,double dw,double time,int path){
    SetRightMoveMode(1,path,time);
    ros::spinOnce();
    kukafri_hw::kukaCmdPosQ msg;
    msg.X_Axis=m_vdRightPos[0]+dX;
    msg.Y_Axis=m_vdRightPos[1]+dY;
    msg.Z_Axis=m_vdRightPos[2]+dZ;
    msg.x=m_vdRightPos[3]+dx;
    msg.y=m_vdRightPos[4]+dy;
    msg.z=m_vdRightPos[5]+dz;
    msg.w=m_vdRightPos[6]+dw;
    RightQuaternion.publish(msg);
}

void CIiwaServo::MoveDRightJoint(double dJoint1,double dJoint2,double dJoint3,double dJoint4,double dJoint5,double dJoint6,double dJoint7,double time,int path){
    SetRightMoveMode(0,path,time);
    ros::spinOnce();
    kukafri_hw::kukaCmdJoint msg;
    msg.joint1=m_vdRightjoint[0]+dJoint1;
    msg.joint2=m_vdRightjoint[1]+dJoint2;
    msg.joint3=m_vdRightjoint[2]+dJoint3;
    msg.joint4=m_vdRightjoint[3]+dJoint4;
    msg.joint5=m_vdRightjoint[4]+dJoint5;
    msg.joint6=m_vdRightjoint[5]+dJoint6;
    msg.joint7=m_vdRightjoint[6]+dJoint7;
    RightJoint.publish(msg);
}

/******************************************************************************************************/

/****************************************双臂相关函数****************************************************/
void CIiwaServo::MoveDualToHome(double time){
    SetRightMoveMode(0,0,time);
    SetRightMoveMode(0,0,time);
    RightHomeClient.call(homeSrv);
    LeftHomeClient.call(homeSrv);
}

/******************************************************************************************************/

