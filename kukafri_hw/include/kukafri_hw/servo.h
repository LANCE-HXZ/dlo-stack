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

#define PTEDGE Point(80, 80)        //  相机视野四周添加了宽度为EDGE的边界, 控制机器人时给的坐标需要将添加的边界去除
#define L_DZ 0.01        //  左夹指夹取位置相对于原配夹指的位移(比原配夹指长为正)
#define R_DZ 0.0        //  右夹指夹取位置相对于原配夹指的位移(比原配夹指长为正)
// 以下注意带小数点, 避免计算时出现整数除法
#define OPRTZ 0.725      // 常用操作平面的Z值
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
            void MoveLeftEulerXYZ(Point ptTarget,double dZ=0.94,double dOz=0,double dMoveDuration=10,double dOx=0,double dOy=0,int nPathMode=0);
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
            void MoveRightEulerXYZ(Point ptTarget,double dZ=0.94,double dOz=0,double dMoveDuration=10,double dOx=0,double dOy=0,int nPathMode=0);
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

        /*  将图像的像素坐标(col, row)转换为相机坐标的(x, y), 像素相关参数在 move_kuka.h 文件宏定义中修改
            输入: 像素坐标系下的 cv::Point(col, row)
            输出: 相机坐标系下的 {x, y}  */
        vector<double> PointPixel2CameraFrame(Point ptPixel);
        /*  根据夹角计算机械臂欧拉角度时可能得到相差180的两个角度值，输入较大的一个值，比较从当前角度分别旋转至两个目标角度所需的行程角度，输出所需行程较小的那个目标角度
            输入: cSide左夹爪或右夹爪（‘L'为左，其余任意值为右），dBiggerAngle两个目标角度的较大值，nAxis旋转轴（0 = X，1 = Y，2 = Z，默认为绕Z轴旋转的角度）
            输出: 末端欧拉角位姿所应到达的nAxis轴所对应的相对于参考坐标系的角度 */
        double CloserAngle(char cSide, double dBiggerAngle, int nAxis = 2);

        /*  计算等速运动所需的时间, 根据当前位置xyz和目标xyz计算距离后除以速度得到所需运动周期
            输入: cSide左夹爪或右夹爪（‘L'为左，'R'为右），ptOprt目标位置的像素坐标x和y，z2目标位置的z(m), 运动速度(m/s)
            输出: 所需运动周期(s) */
        double UniformTime(char cSide, Point ptOprt, double z2 = OPRTZ, double dSpeed = 0.05);
        double UniformTime(char cSide, double x2, double y2, double z2 = OPRTZ, double dSpeed = 0.05);

        /*  封装一层, 增加cSide参数以减少左右臂的重复代码, 将一些重复步骤封装进函数内, 简化代码
            输入: cSide左夹爪或右夹爪（‘L'为左，'R'为右），ptOprt像素坐标系的操作点，dOprtZ夹取高度 */
        void DloMoveEuler(char cSide, Point ptOprt, double dGripperDir, double dMoveDuration=-1, double dOprtZ = OPRTZ);
        /*  函数重载, 用在当左右臂需要同时进行移动而移动参数不一样时   */
        void DloMoveEuler(Point ptOprtL, double dOprtZL, double dGripperDirL, Point ptOprtR, double dOprtZR, double dGripperDirR, double dMoveDuration=-1);
        /*  封装一层, 增加cSide参数以减少左右臂的重复代码, 将一些重复步骤封装进函数内, 简化代码
            输入: cSide左夹爪或右夹爪（‘L'为左，'R'为右, 'D'为双臂同时），其余参数同MoveLeftEulerIncrease() */
        void DloMoveEulerIncrease(char cSide, double dDZ=0,double dDX=0,double dDY=0,double dDOx=0,double dDOy=0,double dDOz=0,double dMoveDuration=-1,int nPathMode=0);
        /*  以上函数的重载, 用在当左右臂需要同时进行移动而移动参数不一样时
            输入: 两个vector<double>, 第一个表示左臂移动参数, 第二个表示右臂移动参数, 两个vector<double>的内容及其余参数同MoveLeftEulerIncrease()   */
        void DloMoveEulerIncrease(vector<double> dDLeft, vector<double> dDRight, double dMoveDuration=10, int nPathMode=0);
};

#endif