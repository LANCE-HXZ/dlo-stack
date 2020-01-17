//让机械臂从初始位置变换到现在夹持电缆位置，然后让机械臂末端向x方向运动5cm，起始点与目标点作一次插值，使用
//action的方式运动，运动过程中，机械臂停顿明显

#include <ros/ros.h>
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


//末端旋转矩阵与欧拉角
Eigen::Matrix3d rotation_matrix;
Eigen::Vector3d euler_ZYX;


//将旋转矩阵转变为欧拉角，输出RPY欧拉角的顺序为YPR
void rotm2Eul(Eigen::Vector3d& euler_ZYX, Eigen::Matrix3d rotm)
{
    float sy = sqrt(rotm(0,0)*rotm(0,0) + rotm(1,0)*rotm(1,0));
    bool singular = sy < 1e-6;
    if(!singular)
    {
        euler_ZYX(2) = atan2(rotm(2,1), rotm(2,2));
        euler_ZYX(1) = atan2(-rotm(2,0), sy);
        euler_ZYX(0) = atan2(rotm(1,0), rotm(0,0));
    }
    else
    {
        euler_ZYX(2) = atan2(-rotm(1,2), rotm(1,1));
        euler_ZYX(1) = atan2(-rotm(2,0), sy);
        euler_ZYX(0) = 0;
    }
}

//计算在link_0坐标系下XYZ值
Eigen::MatrixXf xyzworldtocoordinate(Eigen::MatrixXd Curr_pose)
{
    //计算XYZ值
    float a[3];
    for (int i=0;i<3;i++)
    {
        a[i]=Curr_pose(i,0);
    }
    a[0]=a[0]-0.153055;
    a[1]=a[1]-0.297-0.0595;
    a[2]=a[2]-0.92-0.458;
    Eigen::MatrixXf R(3,3);
    R<<0,-0.8660254,0.5,
       0,-0.5,-0.8660254,
       1,0,0;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV );
   float  pinvtoler = 1.e-6; // choose your tolerance wisely
   Eigen::MatrixXf singularValues_inv = svd.singularValues();      
        for ( long i=0; i<R.rows(); i++) 
        {
            if ( singularValues_inv(i) > pinvtoler )
                 singularValues_inv(i)=1.0/singularValues_inv(i);
            else singularValues_inv(i)=0;
        }
    Eigen::MatrixXf A =  svd.matrixV()*singularValues_inv.asDiagonal()*svd.matrixU().transpose();    // 求逆 6*18
    
    Eigen::MatrixXf b(3,1);
    b<<a[0],
        a[1],
        a[2];                        //0.892056,
       //-0.835088,
       //0.59344;
    Eigen::MatrixXf xyz(3,1);
    xyz=A*b;
    return xyz;
    //计算XYZ值
}

Eigen::Vector3d rpyworldtocoordinate(Eigen::MatrixXd Curr_pose, Eigen::Matrix3d &rotation_matrix)
{
    
    Eigen::MatrixXf B(3,3);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            B(i,j)=rotation_matrix(i,j);
        }
    }
    float a[3];
    for (int i=0;i<3;i++)
    {
        a[i]=Curr_pose(i+3,0);
    }
    // a[0]=a[0]-0.153055;
    // a[1]=a[1]-0.297-0.0595;
    // a[2]=a[2]-0.92-0.458;
    Eigen::MatrixXf R(3,3);
    R<<0,0,1,
       -0.8660254,-0.5,0,
       0.5,-0.8660254,0;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV );
   float  pinvtoler = 1.e-6; // choose your tolerance wisely

    
    Eigen::MatrixXf b(3,1);
    b<<a[0],
        a[1],
        a[2];                        //0.892056,
       //-0.835088,
       //0.59344;
    
    Eigen::MatrixXf r(3,3);
    r=B*R;
    Eigen::Matrix3d rpy_rot;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            rpy_rot(i,j)=r(i,j);
        }
    }
    rotm2Eul(euler_ZYX, rpy_rot);    
    return euler_ZYX;
}

//一次插值轨迹
double Calc1JiTraje(double orig, double goal, double freq, double time)
{
    double ref = goal;
    double time_n = freq*time;

    if(time_n <= 1)
        ref = orig + (goal-orig)*time_n;
        // ref = orig + (goal-orig)/freq;
    return ref;
}

//jacobian_RPY用于将RPY欧拉角转变到末端的角速度
Eigen::MatrixXd ur_jacobian_RPY(Eigen::Vector3d rpy)
{
  Eigen::MatrixXd jcbn_RPY = Eigen::MatrixXd::Zero(6,6);
  jcbn_RPY(0,0) = 1.0;
  jcbn_RPY(1,1) = 1.0;
  jcbn_RPY(2,2) = 1.0;
  jcbn_RPY(3,3) = cos(rpy(1)) * cos(rpy(0));
  jcbn_RPY(3,4) = -sin(rpy(0));
  jcbn_RPY(4,3) = sin(rpy(0)) * cos(rpy(1));
  jcbn_RPY(4,4) = cos(rpy(0));
  jcbn_RPY(5,3) = -sin(rpy(1));
  jcbn_RPY(5,5) = 1;
  return jcbn_RPY;
}

//获取当前位姿
Eigen::MatrixXd getcurrent(moveit::core::RobotStatePtr current_state)
{
    Eigen::MatrixXd Curr_pose;
    Curr_pose.resize(6,1);
    auto& end_effector_initial = current_state->getGlobalLinkTransform("L_link_ee");
    rotation_matrix = end_effector_initial.rotation();
    rotm2Eul(euler_ZYX, rotation_matrix);
    Curr_pose(0,0) = end_effector_initial.translation().x();
    Curr_pose(1,0) = end_effector_initial.translation().y();
    Curr_pose(2,0) = end_effector_initial.translation().z();
    Curr_pose(3,0) = euler_ZYX[2];
    Curr_pose(4,0) = euler_ZYX[1];
    Curr_pose(5,0) = euler_ZYX[0];
    return Curr_pose;
}

//打印位姿
void printpose(Eigen::MatrixXd Curr_pose)
{
    for(int i = 0; i < 6 ; i++)
        {
            std::cout << Curr_pose(i,0) << " ";
        }
        std::cout << std::endl;
}

//打印关节角度
void printjoint(const std::vector<std::string>& joint_names, std::vector<double> joint_group_positions)
{
    for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_group_positions[i]);
        }
        std::cout << std::endl;
}

//设定goal
control_msgs::FollowJointTrajectoryGoal getgoal(control_msgs::FollowJointTrajectoryGoal goal, std::vector<double> joint_group_positions)
{
    goal.trajectory.joint_names.push_back("L_joint_1");
    goal.trajectory.joint_names.push_back("L_joint_2");
    goal.trajectory.joint_names.push_back("L_joint_3");
    goal.trajectory.joint_names.push_back("L_joint_4");
    goal.trajectory.joint_names.push_back("L_joint_5");
    goal.trajectory.joint_names.push_back("L_joint_6");
    goal.trajectory.joint_names.push_back("L_joint_7");
    goal.trajectory.points.resize(1);

    goal.trajectory.points[0].positions.resize(7);
    goal.trajectory.points[0].positions[0] = joint_group_positions[0];
    goal.trajectory.points[0].positions[1] = joint_group_positions[1];
    goal.trajectory.points[0].positions[2] = joint_group_positions[2];
    goal.trajectory.points[0].positions[3] = joint_group_positions[3];
    goal.trajectory.points[0].positions[4] = joint_group_positions[4];
    goal.trajectory.points[0].positions[5] = joint_group_positions[5];
    goal.trajectory.points[0].positions[6] = joint_group_positions[6];
    // Velocities
    goal.trajectory.points[0].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)    {goal.trajectory.points[0].velocities[j] = 0.0;}
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[0].time_from_start = ros::Duration(0.8);
    goal.trajectory.header.stamp = ros::Time::now();
    return goal;
}

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kuka_servo_test");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Action client for the joint trajectory action used to trigger the arm movement action
    TrajClient* traj_client_;
    traj_client_ = new TrajClient("iiwa/PositionJointInterface_L_trajectory_controller/follow_joint_trajectory", true);
    // tell the action client that we want to spin a thread by default
    while(!traj_client_->waitForServer(ros::Duration(5.0)))     {ROS_INFO("Waiting for the follow_joint_trajectory server");}

    //加载模型
    static const std::string PLANNING_GROUP = "L";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();//关节角的名称
    std::vector<double> joint_group_positions, joint_group_positions_, total_error_joint_values;

    Eigen::MatrixXd Curr_pose, initial_pose, Delta_pose, delta_x, Ref_pose, delta_joint, aim_pose;//位姿变量
    Curr_pose.resize(6,1);//机械臂末端当前时刻末端位姿
    initial_pose.resize(6,1);//机械臂轨迹规划起点的末端位姿
    Delta_pose.resize(6,1);//末端位移增量
    delta_x.resize(6,1);
    delta_x(2,0) = -0.001;
    Ref_pose.resize(6,1);//机械臂末端运动参考位置
    aim_pose.resize(6,1);//机械臂目标时刻末端位姿
    // aim_pose(0,0) =  0.3;
    // aim_pose(1,0) =  -0.760653;
    // aim_pose(2,0) =  1.43295;
    // aim_pose(3,0) =  -1.32642;
    // aim_pose(4,0) =  -1.1589;
    // aim_pose(5,0) =  -2.86237;
    Eigen::MatrixXf xyz_link;
    Eigen::Vector3d rpy_link;

    // =============================   get the initial state of robot   =============================
    std::cout << "--- initial end effector position --- | ";
    initial_pose = getcurrent(current_state);
    printpose(initial_pose);
    Curr_pose = initial_pose;
    aim_pose = initial_pose;
    // for(int i = 0; i < 6 ; i++)     {aim_pose(i,0) = initial_pose(i,0);}
    aim_pose(1,0) -= 0.1;
    aim_pose(2,0) += 0.1;
    std::cout << "---   aim end effector position   --- | ";
    printpose(aim_pose);
    // std::cout << "--------------------------------------------------------------------------------------" << std::endl;

    current_state = move_group.getCurrentState();
    rotation_matrix = current_state->getGlobalLinkTransform("L_link_ee").rotation();
    xyz_link = xyzworldtocoordinate(aim_pose);
    rpy_link = rpyworldtocoordinate(aim_pose, rotation_matrix);
    for(int i=0; i<3; i++)  {aim_pose(i, 0) = xyz_link(i, 0);}
    for(int i=0; i<3; i++)  {aim_pose(i+3, 0) = rpy_link(i, 0);}
    current_state = move_group.getCurrentState();
    rotation_matrix = current_state->getGlobalLinkTransform("L_link_ee").rotation();
    xyz_link = xyzworldtocoordinate(initial_pose);
    rpy_link = rpyworldtocoordinate(initial_pose, rotation_matrix);
    for(int i=0; i<3; i++)  {initial_pose(i, 0) = xyz_link(i, 0);}
    for(int i=0; i<3; i++)  {initial_pose(i+3, 0) = rpy_link(i, 0);}
    std::cout << aim_pose - initial_pose << std::endl;
    //关节角名称传入joint_group_positions
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);
    current_state->copyJointGroupPositions(joint_model_group, total_error_joint_values);
    for(int i=0; i<7; i++)
    {
        total_error_joint_values[i] = 0.0 ;
        std::cout << total_error_joint_values[i];
    }
    std::cout << " initial joint: " << std::endl;
    printjoint(joint_names, joint_group_positions);

    std::cout << "--------------------beginning trajectory--------------------" <<std::endl;
    // =======================================   trajectory   =======================================
    static double init_sec = ros::Time::now().toSec();
    double curr_sec, timer, cal_time, exe_time, nowtime;
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian, singularValues_inv, pinvmat, jacobian_RPY;
    

    while(ros::ok())
    // for(int k = 0; k < 3; k++)
    {
        curr_sec = ros::Time::now().toSec();
        timer = curr_sec - init_sec;
        std::cout << "--------------------------------------------------------------------------------------" << std::endl;
        std::cout << "init_sec: " << init_sec << "  " << " curr_sec: " <<curr_sec << "  " << "timer: " << timer << std::endl; 
        
        // //当前时刻机械臂末端的位姿
        current_state = move_group.getCurrentState();
        Curr_pose = getcurrent(current_state);
        rotation_matrix = current_state->getGlobalLinkTransform("L_link_ee").rotation();
        xyz_link = xyzworldtocoordinate(Curr_pose);
        rpy_link = rpyworldtocoordinate(Curr_pose, rotation_matrix);
        for(int i=0; i<3; i++)  {Curr_pose(i, 0) = xyz_link(i, 0);}
        for(int i=0; i<3; i++)  {Curr_pose(i+3, 0) = rpy_link(i, 0);}
        std::cout << "--- current end effector position --- | ";
        printpose(Curr_pose);

        //起始时刻末端位置与目标位置作一次插值，获得当前时刻的轨迹点
        std::cout << "---reference end effector position--- | ";
        for(int i = 0; i < 6 ; i++)
        {
            Ref_pose(i,0) = Calc1JiTraje(initial_pose(i,0), aim_pose(i,0), 0.1, timer);
            std::cout << Ref_pose(i,0) << " ";
        }
        std::cout << std::endl;

        //获取雅克比矩阵
        current_state->getJacobian(joint_model_group,
                                  current_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                  reference_point_position, jacobian);     
        //求雅克比矩阵的逆
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV );
        float  pinvtoler = 1.e-6; // choose your tolerance wisely
        singularValues_inv = svd.singularValues();

        for ( int i=0; i<6; i++) {
            if ( singularValues_inv(i) > pinvtoler )
                singularValues_inv(i)=1.0/singularValues_inv(i);
            else singularValues_inv(i)=0;
        }
        pinvmat =  svd.matrixV()*singularValues_inv.asDiagonal()*svd.matrixU().transpose();
        Delta_pose = Ref_pose - Curr_pose;
        // Delta_pose = Curr_pose- Ref_pose;
        // Delta_pose = (aim_pose - initial_pose)/50;

        // for(int i = 0; i < 6; i++)   {Delta_pose(i,0) = fabs(Ref_pose(i,0) - Curr_pose(i,0)) < 1.e-6 ? 0 : (Ref_pose(i,0) - Curr_pose(i,0));}
        // std::cout << std::endl << delta_x << std::endl;
        std::cout << "Error pose:" << std::endl << Delta_pose << std::endl;
        jacobian_RPY = ur_jacobian_RPY(euler_ZYX);//jacobian_RPY用于将RPY欧拉角转变到末端的角速度
        delta_joint = pinvmat * jacobian_RPY * Delta_pose;//关节增量
        // delta_joint = pinvmat * jacobian_RPY * delta_x;

        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        std::cout << " last joint values: " << std::endl;
        printjoint(joint_names, joint_group_positions);

        std::cout << " error joint values: " << std::endl;
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_group_positions[i] - joint_group_positions_[i]);
            total_error_joint_values[i] += joint_group_positions[i] - joint_group_positions_[i];
        }
        std::cout << std::endl;

        //对角位移量加限位
        std::cout << " delta joint values: " << std::endl;
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
            if(delta_joint(i,0) > 0.05)
                delta_joint(i,0) = 0.05;
            else if(delta_joint(i,0) < -0.05)
                delta_joint(i,0) = -0.05;
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), delta_joint(i,0));
        }
        //关节角度 += 角度增量
        std::cout << std::endl << " next joint values: " << std::endl;
        for (std::size_t i = 0; i < joint_names.size(); ++i)
            {
                joint_group_positions_[i] = joint_group_positions[i] + delta_joint(i,0);
            }
        printjoint(joint_names, joint_group_positions_);
        
        //move
        control_msgs::FollowJointTrajectoryGoal goal;
        goal = getgoal(goal, joint_group_positions_);
        traj_client_->sendGoal(goal);

        cal_time = ros::Time::now().toSec();
        std::cout<<"calculation time: "<<cal_time-curr_sec<<std::endl;
        traj_client_->waitForResult();
        exe_time = ros::Time::now().toSec();
        std::cout<<"execution time: "<<exe_time-cal_time<<std::endl;

        ROS_INFO("Action finished");
        if(timer >= 10)
            break;
        Eigen::MatrixXd maxOne = (Curr_pose - aim_pose).colwise().maxCoeff();
        Eigen::MatrixXd minOne = (Curr_pose - aim_pose).colwise().minCoeff();
        if( fabs( maxOne(0,0) ) < 0.001 && fabs( minOne(0,0) ) < 0.001 )
            break;
        // while(1)
        // {
        //     nowtime = ros::Time::now().toSec();
        //     if(nowtime - curr_sec >= 0.2)
        //     {
        //         break;
        //     }
        // }
    }
    std::cout << "--- initial end effector position --- | ";
    printpose(initial_pose);
    std::cout << "---   aim end effector position   --- | ";
    printpose(aim_pose);
    current_state = move_group.getCurrentState();
    Curr_pose = getcurrent(current_state);
    rotation_matrix = current_state->getGlobalLinkTransform("L_link_ee").rotation();
    xyz_link = xyzworldtocoordinate(Curr_pose);
    rpy_link = rpyworldtocoordinate(Curr_pose, rotation_matrix);
    for(int i=0; i<3; i++)  {Curr_pose(i, 0) = xyz_link(i, 0);}
    for(int i=0; i<3; i++)  {Curr_pose(i+3, 0) = rpy_link(i, 0);}
    std::cout << "--- current end effector position --- | ";
    printpose(Curr_pose);
    std::cout << " total error joint values: " << std::endl;
    printjoint(joint_names, total_error_joint_values);
    delete traj_client_;
    ros::shutdown();
    return 0;
}
