#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#define FOR(i, a, b) for(int i(a); i<b; ++i)

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

class CKukaMoveit{
    private:
        string m_strReferenceFrame;
        moveit::planning_interface::MoveGroupInterface D, L, R;
        moveit::planning_interface::MoveItErrorCode success;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        geometry_msgs::PoseStamped poseStampMsg;
        robot_state::JointModelGroup* L_joint_model_group;
        robot_model_loader::RobotModelLoader robot_model_loader;
        robot_model::RobotModelPtr robot_model;
        robot_state::RobotStatePtr robot_state;
        geometry_msgs::Pose L_target_pose, R_target_pose;
        vector<double> L_target_joint, R_target_joint;

        tf::TransformListener listener;
        tf::StampedTransform transform;
        
        void SetPose(geometry_msgs::Pose& target_pose, double x, double y, double z, vector<double> o);
        void Setdxdydz(geometry_msgs::Pose& target_pose, string EE, double dx, double dy, double dz);
        void SetJoint(vector<double>& target_joint, double a6, double a5, double a4, double a3, double a2, double a1, double a0);

    public:
        CKukaMoveit();
        ~CKukaMoveit();

        void SetFrame(string re_frame = CAMERA_FRAME);

        void SetLdxdydz(double dx, double dy = 0, double dz = 0);
        void SetRdxdydz(double dx, double dy = 0, double dz = 0);

        void SetLeftPose(double x, double y, double z = MOV_Z, vector<double> o = {0, 0, 0, 1});
        void SetLeftPose(Point ptTarget, double z = MOV_Z, vector<double> o = {0, 0, 0, 1});
        void SetRightPose(double x, double y, double z = MOV_Z, vector<double> o = {0, 0, 0, 1});
        void SetRightPose(Point ptTarget, double z = MOV_Z, vector<double> o = {0, 0, 0, 1});

        void ExecuteGroup(string strGroup = D_GROUP);
        void ExecuteJointGroup(string strGroup = D_GROUP);

        void MoveDdxdydz(double dx, double dy = 0, double dz = 0);
        void MoveLdxdydz(double dx, double dy = 0, double dz = 0);
        void MoveRdxdydz(double dx, double dy = 0, double dz = 0);

        void MoveToLeftPose(double x, double y, double z = MOV_Z, vector<double> o = {0, 0, 0, 1});
        void MoveToLeftPose(Point ptTarget, double z = MOV_Z, vector<double> o = {0, 0, 0, 1});
        void MoveToRightPose(double x, double y, double z = MOV_Z, vector<double> o = {0, 0, 0, 1});
        void MoveToRightPose(Point ptTarget, double z = MOV_Z, vector<double> o = {0, 0, 0, 1});

        void SetLeftJoint(double a6 = 0, double a5 = 0, double a4 = 0, double a3 = 0, double a2 = 0, double a1 = 0, double a0 = 0);
        void SetRightJoint(double a6 = 0, double a5 = 0, double a4 = 0, double a3 = 0, double a2 = 0, double a1 = 0, double a0 = 0);

        void MoveToLeftJoint(double a6 = 0, double a5 = 0, double a4 = 0, double a3 = 0, double a2 = 0, double a1 = 0, double a0 = 0);
        void MoveToLeftJoint(vector<double> vJoints = {0, 0, 0, 0, 0, 0, 0});
        void MoveToRightJoint(double a6 = 0, double a5 = 0, double a4 = 0, double a3 = 0, double a2 = 0, double a1 = 0, double a0 = 0);
        void MoveToRightJoint(vector<double> vJoints = {0, 0, 0, 0, 0, 0, 0});
        void MoveToDualJoint(vector<double> vLeftJoints = {0, 0, 0, 0, 0, 0, 0}, vector<double> vRightJoints = {0, 0, 0, 0, 0, 0, 0});

        void MoveOneLeftJointToTarget(int nJointID, double dJointValue);
        void MoveOneRightJointToTarget(int nJointID, double dJointValue);
        void MoveOneLeftJointIncrease(int nJointID, double dJointIncrease);
        void MoveOneRightJointIncrease(int nJointID, double dJointIncrease);

        void GoHome(string strGroup = D_GROUP);

        vector<double> PointPixel2CameraFrame(Point ptPixel);
        vector<double> RPY2XYZW(double oz = 0.0, double oy = 0.0, double ox = 0.0);

};
