#ifndef SERVO
#define SERVO

#include <ros/ros.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <algorithm> 
#include <opencv2/opencv.hpp>

#include "kukafri_hw/servo.h"
#include "kukafri_hw/setMoveMode.h"
#include "kukafri_hw/moveToHome.h"
#include "kukafri_hw/kukaCmdPosE.h"
#include "kukafri_hw/kukaCmdPosQ.h"
#include "kukafri_hw/kukaCmdJoint.h"
#include "kukafri_hw/kukaState.h"


#define L_EE "L_link_ee"   // 左臂末端执行器名称
#define R_EE "R_link_ee"   // 右臂末端执行器名称
#define D_GROUP "D"  // 双臂规划组名称
#define L_GROUP "L"     // 左臂规划组名称
#define R_GROUP "R"     // 右臂规划组名称
#define CAMERA_FRAME "camera_frame"    // 相机坐标系名称

// 以下注意带小数点, 避免计算时出现整数除法
#define MOV_Z 1.0       // 仅xy平面平移时的高度z的默认值
#define MAX_V 0.05      // 机械臂最大速度
#define MAX_A 0.05      // 机械臂最大加速度

#define PIC_WIDTH 480.0   // 相机获取的图像的宽度(像素), 像素坐标系中的rows, y轴
#define PIC_LENGTH 640.0  // 相机获取的图像的长度(像素), 像素坐标系中的cols, x轴
#define MAPPING_WIDTH 0.835     // 视野中操作平面的宽度(m), y轴
#define MAPPING_LENGTH 1.101   // 视野中操作平面的长度(m), x轴
#define PIXELS_OF_1MY PIC_WIDTH/MAPPING_WIDTH      // 每1m宽度(y轴)对应的像素点个数
#define PIXELS_OF_1MX PIC_LENGTH/MAPPING_LENGTH    // 每1m长度(x轴)对应的像素点个数
#define PAN_Y PIC_WIDTH/2.0      // 图像中心到图像像素cv::Point(0, 0)的距离(像素个数), y轴
#define PAN_X PIC_LENGTH/2.0    // 图像中心到图像像素cv::Point(0, 0)的距离(像素个数), x轴


using namespace std;
using cv::Point;

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
            void MoveLeftEulerXYZ(Point ptTarget,double dZ=0.94,double dOz=0,double dOx=0,double dOy=0,double dMoveDuration=10,int nPathMode=0);
        /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       x,y,z,w为目标点四元数*/
        void MoveLeftQuaternion(double dX=0.6, double dY=0.225, double dZ=0.669, double dOx=0, double dOy=0, double dOz=-0.707, double dOw=0.707,
                                double dMoveDuration=10, int nPathMode=0);
        /*指定机械臂七个关节角进行移动*/
        void MoveLeftToJoint(double dJoint1=-23.09, double dJoint2=-29.527, double dJoint3=4.9144, double dJoint4=-72.56458, double dJoint5=25.49335,
                             double dJoint6=46.8971, double dJoint7=9.2455, double dMoveDuration=10, int nPathMode=0);
        /*机械臂相对于当前位置移动一个坐标和位姿,位姿用EulerXYZ表示*/
        void MoveLeftEulerIncrease(double dDX=0,double dDY=0,double dDZ=0,double dDOx=0,double dDOy=0,double dDOz=0,double dMoveDuration=10,int nPathMode=0);
        /*机械臂相对于当前位置移动一个坐标和位姿，位姿用四元数表示*/
        void MoveLeftQuaternionIncrease(double dDX=0,double dDY=0,double dDZ=0,double dDOx=0,double dDOy=0,double dDOz=0,double dDOw=0,
                                 double dMoveDuration=10,int nPathMode=0);
        /*机械臂各关节角相对于当前关节角移动一个关节角,单位为°*/
        void MoveLeftJointIncrease(double dDJoint1=0,double dDJoint2=0,double dDJoint3=0,double dDJoint4=0,double dDJoint5=0,double dDJoint6=0,double dDJoint7=0,
                            double dMoveDuration=10,int nPathMode=0);

        /*返回Righthome位置*/
        void MoveRightToHome(double dMoveDuration=10);
        /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       alpha为沿X轴旋转到的目标角度    beta为沿Y轴旋转到的目标角度   gamma为沿Z轴旋转到的目标角度*/
        void MoveRightEulerXYZ(double dX=-0.6,double dY=0.225,double dZ=0.7,double dOx=0,double dOy=0,double dOz=-90,double dMoveDuration=10,int nPathMode=0);
        /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       x,y,z,w为目标点四元数*/
            void MoveRightEulerXYZ(Point ptTarget,double dZ=0.94,double dOz=0,double dOx=0,double dOy=0,double dMoveDuration=10,int nPathMode=0);
        void MoveRightQuaternion(double dX=-0.6,double dY=0.225,double dZ=0.7,double dOx=0,double dOy=0,double dOz=-0.707,double dOw=0.707,
                                 double dMoveDuration=10,int nPathMode=0);
        /*指定机械臂七个关节角进行移动*/
        void MoveRightToJoint(double dJoint1=28.6085,double dJoint2=-29.537,double dJoint3=-6.1648,double dJoint4=-72.56458,double dJoint5=-29.92,
                            double dJoint6=48.27,double dJoint7=-8.845,double dMoveDuration=10,int nPathMode=0);
        /*机械臂相对于当前位置移动一个坐标和位姿,位姿用EulerXYZ表示*/
        void MoveRightEulerIncrease(double dDX=0,double dDY=0,double dDZ=0,double dDOx=0,double dDOy=0,double dDOz=0,double dMoveDuration=10,int nPathMode=0);
        /*机械臂相对于当前位置移动一个坐标和位姿，位姿用四元数表示*/
        void MoveRightQuaternionIncrease(double dDX=0,double dDY=0,double dDZ=0,double dDOx=0,double dDOy=0,double dDOz=0,double dDOw=0,
                                  double dMoveDuration=10,int nPathMode=0);
        /*机械臂各关节角相对于当前关节角移动一个关节角,单位为°*/
        void MoveRightJointIncrease(double dDJoint1=0,double dDJoint2=0,double dDJoint3=0,double dDJoint4=0,double dDJoint5=0,double dDJoint6=0,double dDJoint7=0,
                             double dMoveDuration=10,int nPathMode=0);

        /*返回Dualhome位置*/
        void MoveDualToHome(double dMoveDuration=10);
        /*以相机坐标系为准，移动机械臂以指定位姿移动到指定点       alpha为沿X轴旋转到的目标角度    beta为沿Y轴旋转到的目标角度   gamma为沿Z轴旋转到的目标角度*/
        void MoveDualEulerXYZ(double dX=-0.6,double dY=0.225,double dZ=0.7,double dOx=0,double dOy=0,double dOz=-90,double dMoveDuration=10,int nPathMode=0);

        vector<double> PointPixel2CameraFrame(Point ptPixel);
};

#endif