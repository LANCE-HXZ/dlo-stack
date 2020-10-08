#ifndef SERVO
#define SERVO

#include <ros/ros.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <algorithm> 

#include "kukafri_hw/servo.h"
#include "kukafri_hw/setMoveMode.h"
#include "kukafri_hw/moveToHome.h"
#include "kukafri_hw/kukaCmdPosE.h"
#include "kukafri_hw/kukaCmdPosQ.h"
#include "kukafri_hw/kukaCmdJoint.h"
#include "kukafri_hw/kukaState.h"

using namespace std;

class CIiwaServo{
    private:
        vector<double> m_vdLeftPos;
        vector<double> m_vdLeftEuler;
        vector<double> m_vdLeftjoint;
        vector<double> m_vdRightPos;
        vector<double> m_vdRightEuler;
        vector<double> m_vdRightjoint;

        ros::NodeHandle nh;
        ros::ServiceClient LeftClient;
        ros::ServiceClient RightClient;
        ros::ServiceClient LeftHomeClient;
        ros::ServiceClient RightHomeClient;
                
        ros::Publisher LeftEulerXYZ;
        ros::Publisher LeftQuaternion;
        ros::Publisher LeftJoint;
        ros::Publisher RightEulerXYZ;
        ros::Publisher RightQuaternion;
        ros::Publisher RightJoint;

        ros::Subscriber LeftKukaPos;
        ros::Subscriber RightKukaPos;

        kukafri_hw::moveToHome homeSrv;

        void SetLeftMoveMode(int nMoveMode=0, int nPathMode=0, double dMoveDuration=10);
        void SetRightMoveMode(int nMoveMode=0, int nPathMode=0, double dMoveDuration=10);
        void LeftKukaCb(const kukafri_hw::kukaState::ConstPtr& msg);
        void RightkukaCb(const kukafri_hw::kukaState::ConstPtr& msg);

    public:
        CIiwaServo();
        ~CIiwaServo();

        /*  返回Lefthome位置    */
        void MoveLeftToHome(double dMoveDuration=10);
        /*  以相机坐标系为准，移动机械臂以指定位姿移动到指定点       alpha为沿X轴旋转到的目标角度    beta为沿Y轴旋转到的目标角度   gamma为沿Z轴旋转到的目标角度*/
        void MoveLeftEulerXYZ(double dX=0.6, double dY=0.225, double dZ=0.669, double dOx=0, double dOy=0, double dOz=-90, 
                              double dMoveDuration=10, int nPathMode=0);
        /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       x,y,z,w为目标点四元数*/
        void MoveLeftQuaternion(double dX=0.6, double dY=0.225, double dZ=0.669, double dOx=0, double dOy=0, double dOz=-0.707, double dOw=0.707,
                                double dMoveDuration=10, int nPathMode=0);
        /*指定机械臂七个关节角进行移动*/
        void MoveLeftToJoint(double dJoint1=-23.09, double dJoint2=-29.527, double dJoint3=4.9144, double dJoint4=-72.56458, double dJoint5=25.49335,
                             double dJoint6=46.8971, double dJoint7=9.2455, double dMoveDuration=10, int nPathMode=0);
        /*机械臂相对于当前位置移动一个坐标和位姿,位姿用EulerXYZ表示*/
        void MoveLeftEulerIncrease(double dDX=0,double dDY=0,double dDZ=0,double dDOx=0,double dDOy=0,double dDOz=0,double dMoveDuration=10,int nPathMode=0);
        /*机械臂相对于当前位置移动一个坐标和位姿，位姿用四元数表示*/
        void MoveDLeftQuaternion(double dDX=0,double dDY=0,double dDZ=0,double dDOx=0,double dDOy=0,double dDOz=0,double dDOw=0,
                                 double dMoveDuration=10,int nPathMode=0);
        /*机械臂各关节角相对于当前关节角移动一个关节角,单位为°*/
        void MoveDLeftJoint(double dDJoint1=0,double dDJoint2=0,double dDJoint3=0,double dDJoint4=0,double dDJoint5=0,double dDJoint6=0,double dDJoint7=0,
                            double dMoveDuration=10,int nPathMode=0);

        /*返回Righthome位置*/
        void MoveRightToHome(double dMoveDuration=10);
        /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       alpha为沿X轴旋转到的目标角度    beta为沿Y轴旋转到的目标角度   gamma为沿Z轴旋转到的目标角度*/
        void MoveRightEulerXYZ(double dX=-0.6,double dY=0.225,double dZ=0.7,double dOx=0,double dOy=0,double dOz=-90,double dMoveDuration=10,int nPathMode=0);
        /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       x,y,z,w为目标点四元数*/
        void MoveRightQuaternion(double dX=-0.6,double dY=0.225,double dZ=0.7,double dOx=0,double dOy=0,double dOz=-0.707,double dOw=0.707,
                                 double dMoveDuration=10,int nPathMode=0);
        /*指定机械臂七个关节角进行移动*/
        void MoveRightJoint(double dJoint1=28.6085,double dJoint2=-29.537,double dJoint3=-6.1648,double dJoint4=-72.56458,double dJoint5=-29.92,
                            double dJoint6=48.27,double dJoint7=-8.845,double dMoveDuration=10,int nPathMode=0);
        /*机械臂相对于当前位置移动一个坐标和位姿,位姿用EulerXYZ表示*/
        void MoveDRightEulerXYZ(double dDX=0,double dDY=0,double dDZ=0,double dDOx=0,double dDOy=0,double dDOz=0,double dMoveDuration=10,int nPathMode=0);
        /*机械臂相对于当前位置移动一个坐标和位姿，位姿用四元数表示*/
        void MoveDRightQuaternion(double dDX=0,double dDY=0,double dDZ=0,double dDOx=0,double dDOy=0,double dDOz=0,double dDOw=0,
                                  double dMoveDuration=10,int nPathMode=0);
        /*机械臂各关节角相对于当前关节角移动一个关节角,单位为°*/
        void MoveDRightJoint(double dDJoint1=0,double dDJoint2=0,double dDJoint3=0,double dDJoint4=0,double dDJoint5=0,double dDJoint6=0,double dDJoint7=0,
                             double dMoveDuration=10,int nPathMode=0);

        /*返回Dualhome位置*/
        void MoveDualToHome(double dMoveDuration=10);
        /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       alpha为沿X轴旋转到的目标角度    beta为沿Y轴旋转到的目标角度   gamma为沿Z轴旋转到的目标角度*/
        void MoveDualEulerXYZ(double dX=-0.6,double dY=0.225,double dZ=0.7,double dOx=0,double dOy=0,double dOz=-90,double dMoveDuration=10,int nPathMode=0);
};

#endif