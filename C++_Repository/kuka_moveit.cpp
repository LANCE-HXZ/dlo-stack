#include "kuka_moveit.h"

/*  构造函数  */
CKukaMoveit::CKukaMoveit()
: D(D_GROUP), L(L_GROUP), R(R_GROUP),       //  定义双臂规划组D和左/右单臂规划组L/R
robot_state(new robot_state::RobotState(robot_model)),  //  /*
robot_model(robot_model_loader.getModel()),             //     用于获取机械臂状态
robot_model_loader("robot_description")                 //                      */
{
    m_strReferenceFrame = CAMERA_FRAME;
    FOR(i, 0, 7){
        L_target_joint.push_back(0);        //  /*
        R_target_joint.push_back(0);        //      用于存储两个机械臂目标角度, 初始化为7个0(弧度制)    */
    }
}
/*  析构函数  */
CKukaMoveit::~CKukaMoveit()
{}

/*  设置绝对位姿, 私有函数, 仅在类成员函数中调用, 不能单独使用    无输出  */
void CKukaMoveit::SetPose(geometry_msgs::Pose& target_pose, double x, double y, double z, vector<double> o){
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    if(o.size() == 4){                      //  /*
        target_pose.orientation.x = o[0];
        target_pose.orientation.y = o[1];   //      输入为四元数
        target_pose.orientation.z = o[2];
        target_pose.orientation.w = o[3];   //                      */    
    }
    else if(o.size() == 3){                                                                                     //  /*
        target_pose.orientation.x = cos(o[0]/2)*cos(o[1]/2)*sin(o[2]/2)+sin(o[0]/2)*sin(o[1]/2)*cos(o[2]/2);
        target_pose.orientation.y = cos(o[0]/2)*sin(o[1]/2)*cos(o[2]/2)-sin(o[0]/2)*cos(o[1]/2)*sin(o[2]/2);    //      输入为RPY角转换为四元数
        target_pose.orientation.z = sin(o[0]/2)*cos(o[1]/2)*cos(o[2]/2)+cos(o[0]/2)*sin(o[1]/2)*sin(o[2]/2);
        target_pose.orientation.w = cos(o[0]/2)*cos(o[1]/2)*cos(o[2]/2)-sin(o[0]/2)*sin(o[1]/2)*sin(o[2]/2);    //                              */  
    }
    else    cout << "【ERROR】Orientation value in SetPose()";
}

/*  设置相对于当前位置的位移矢量, 私有函数, 仅在类成员函数中调用, 不能单独使用    无输出  */
void CKukaMoveit::Setdxdydz(geometry_msgs::Pose& target_pose, string EE, double dx, double dy, double dz){
    listener.waitForTransform(CAMERA_FRAME, EE, ros::Time(0), ros::Duration(3.0));      //  <tf/transform_listener.h>
    listener.lookupTransform(CAMERA_FRAME, EE,ros::Time(0), transform);                 //  直接获取EE相对于CAMERA_FRAME的坐标
    target_pose.position.x = transform.getOrigin().x() + dx;
    target_pose.position.y = transform.getOrigin().y() + dy;
    target_pose.position.z = transform.getOrigin().z() + dz;
}

/*  设置关节角, 私有函数, 仅在类成员函数中调用, 不能单独使用    无输出  */
void CKukaMoveit::SetJoint(vector<double>& target_joint, double a6, double a5, double a4, double a3, double a2, double a1, double a0){
    target_joint[0] = a0;
    target_joint[1] = a1;
    target_joint[2] = a2;
    target_joint[3] = a3;
    target_joint[4] = a4;
    target_joint[5] = a5;
    target_joint[6] = a6;
}




/*  设置位姿操作指令的相对坐标系
    输入: 相对坐标系的名称      参数可缺省, 默认为相机坐标系, 相机坐标系名称在 move_kuka.h 文件宏定义中修改  */
void CKukaMoveit::SetFrame(string re_frame){
    m_strReferenceFrame = re_frame;
    D.setPoseReferenceFrame(m_strReferenceFrame);
    L.setPoseReferenceFrame(m_strReferenceFrame);
    R.setPoseReferenceFrame(m_strReferenceFrame);
}


/*  设置左/右臂相对于当前位置的位移矢量, 
    输入: 三个坐标轴方向上的相对位移量(单位m)       dy,dz参数可缺省, 默认缺省坐标方向不移动  */
void CKukaMoveit::SetLdxdydz(double dx, double dy, double dz){
    Setdxdydz(L_target_pose, L_EE, dx, dy, dz);
}
void CKukaMoveit::SetRdxdydz(double dx, double dy, double dz){
    Setdxdydz(R_target_pose, R_EE, dx, dy, dz);
}


/*  设置左/右臂位姿
    输入: x, y, z, {ox, oy, oz, ow}或{r, p, y}      
        后五个参数可缺省, 默认为 MOV_Z(在 move_kuka.h 文件宏定义中修改), {0, 0, 0, 1}(沿z轴正方向)  */
void CKukaMoveit::SetLeftPose(double x, double y, double z, vector<double> o){
    SetPose(L_target_pose, x, y, z, o);
}
void CKukaMoveit::SetRightPose(double x, double y, double z, vector<double> o){
    SetPose(R_target_pose, x, y, z, o);
}
/*  设置左/右臂位姿(函数重载)
    输入: cv::Point(x, y)(相机视野的像素坐标), {ox, oy, oz, ow}或{r, p, y}
        后五个参数可缺省, 默认为 MOV_Z(在 move_kuka.h 文件宏定义中修改), {0, 0, 0, 1}(沿z轴正方向)  */
void CKukaMoveit::SetLeftPose(Point ptTarget, double z, vector<double> o){
    if(o.size() != 3 && o.size() != 4)          cout << "【ERROR】Orientation value in MoveToLeftPose()\n";
    vector<double> ptTargetInFrame = PointPixel2CameraFrame(ptTarget);      // 像素坐标转换为相机坐标
    SetPose(L_target_pose, ptTargetInFrame[0], ptTargetInFrame[1], z, o);
}
void CKukaMoveit::SetRightPose(Point ptTarget, double z, vector<double> o){
    if(o.size() != 3 && o.size() != 4)          cout << "【ERROR】Orientation value in MoveToLeftPose()\n";
    vector<double> ptTargetInFrame = PointPixel2CameraFrame(ptTarget);      // 像素坐标转换为相机坐标
    SetPose(R_target_pose, ptTargetInFrame[0], ptTargetInFrame[1], z, o);
}


/*  设置左/右臂关节角
    输入: a6, a5, a4, a3, a2, a1, a0(注意末端关节在第一位)      参数可缺省, 默认为(0, 0, 0, 0, 0, 0, 0)  */
void CKukaMoveit::SetLeftJoint(double a6, double a5, double a4, double a3, double a2, double a1, double a0){
    SetJoint(L_target_joint, a6, a5, a4, a3, a2, a1, a0);
}
void CKukaMoveit::SetRightJoint(double a6, double a5, double a4, double a3, double a2, double a1, double a0){
    SetJoint(R_target_joint, a6, a5, a4, a3, a2, a1, a0);
}


/*  执行机械臂姿态移动
    输入: 规划组名称        参数可缺省, 默认执行双臂规划组"D"  */
void CKukaMoveit::ExecuteGroup(string strGroup){
    moveit::planning_interface::MoveGroupInterface GROUP(strGroup);
    GROUP.setPoseReferenceFrame(m_strReferenceFrame);
    GROUP.setMaxVelocityScalingFactor(MAX_V);        //  /*
    GROUP.setMaxAccelerationScalingFactor(MAX_A);
    if(strGroup == D_GROUP || strGroup == L_GROUP)
        GROUP.setPoseTarget(L_target_pose, L_EE);
    if(strGroup == D_GROUP || strGroup == R_GROUP)
        GROUP.setPoseTarget(R_target_pose, R_EE);
    success = GROUP.plan(my_plan);
    ROS_INFO("PLANNING %s IN KukaMoveit::ExecuteGroup()", success ? "SUCCESS" : "FAILED");
    if(success)
        GROUP.execute(my_plan);
}

/*  执行机械臂关节角移动
    输入: 规划组名称        参数可缺省, 默认执行双臂规划组"D"  */
void CKukaMoveit::ExecuteJointGroup(string strGroup){
    moveit::planning_interface::MoveGroupInterface GROUP(strGroup);
    if(strGroup == D_GROUP){
        vector<double> D_target_joint;        // 将L_target_joint, R_target_joint合并存储到D_target_joint
        D_target_joint.insert(D_target_joint.end(), L_target_joint.begin(), L_target_joint.end());
        D_target_joint.insert(D_target_joint.end(), R_target_joint.begin(), R_target_joint.end());
        GROUP.setJointValueTarget(D_target_joint);
    }
    else if(strGroup == L_GROUP)
        GROUP.setJointValueTarget(L_target_joint);
    else if(strGroup == R_GROUP)
        GROUP.setJointValueTarget(R_target_joint);
    success = GROUP.plan(my_plan);
    ROS_INFO("PLANNING %s IN KukaMoveit::ExecuteGroup()", success ? "SUCCESS" : "FAILED");
    if(success)
        GROUP.execute(my_plan);
}


/*  执行左/右/双臂相对于当前位置移动一个位移矢量(单位m)
    输入: 三个坐标轴方向上的相对位移量      dy,dz参数可缺省, 默认缺省坐标方向不移动  */
void CKukaMoveit::MoveLdxdydz(double dx, double dy, double dz){
    Setdxdydz(L_target_pose, L_EE, dx, dy, dz);
    ExecuteGroup(L_GROUP);
}
void CKukaMoveit::MoveRdxdydz(double dx, double dy, double dz){
    Setdxdydz(R_target_pose, R_EE, dx, dy, dz);
    ExecuteGroup(R_GROUP);
}
void CKukaMoveit::MoveDdxdydz(double dx, double dy, double dz){
    Setdxdydz(L_target_pose, L_EE, dx, dy, dz);
    Setdxdydz(R_target_pose, R_EE, dx, dy, dz);
    ExecuteGroup(D_GROUP);
}


/*  执行移动左/右臂至目标位姿
    输入: x, y, z, {ox, oy, oz, ow}或{r, p, y}      
        后五个参数可缺省, 默认为 MOV_Z(在 move_kuka.h 文件宏定义中修改), {0, 0, 0, 1}(沿z轴正方向)  */
void CKukaMoveit::MoveToLeftPose(double x, double y, double z, vector<double> o){
    if(o.size() != 3 && o.size() != 4)          cout << "【ERROR】Orientation value in MoveToLeftPose()\n";
    SetPose(L_target_pose, x, y, z, o);
    ExecuteGroup(L_GROUP);
}
void CKukaMoveit::MoveToRightPose(double x, double y, double z, vector<double> o){
    if(o.size() != 3 && o.size() != 4)          cout << "【ERROR】Orientation value in MoveToRightPose()\n";
    SetPose(L_target_pose, x, y, z, o);
    ExecuteGroup(R_GROUP);
}
/*  执行移动左/右臂至目标位姿(函数重载)
    输入: cv::Point(x, y)(相机视野的像素坐标), {ox, oy, oz, ow}或{r, p, y}
        后五个参数可缺省, 默认为 MOV_Z(在 move_kuka.h 文件宏定义中修改), {0, 0, 0, 1}(沿z轴正方向)  */
void CKukaMoveit::MoveToLeftPose(Point ptTarget, double z, vector<double> o){
    if(o.size() != 3 && o.size() != 4)          cout << "【ERROR】Orientation value in MoveToLeftPose()\n";
    vector<double> ptTargetInFrame = PointPixel2CameraFrame(ptTarget);      // 像素坐标转换为相机坐标
    SetPose(L_target_pose, ptTargetInFrame[0], ptTargetInFrame[1], z, o);
    ExecuteGroup(L_GROUP);
}
void CKukaMoveit::MoveToRightPose(Point ptTarget, double z, vector<double> o){
    if(o.size() != 3 && o.size() != 4)          cout << "【ERROR】Orientation value in MoveToRightPose()\n";
    vector<double> ptTargetInFrame = PointPixel2CameraFrame(ptTarget);      // 像素坐标转换为相机坐标
    SetPose(R_target_pose, ptTargetInFrame[0], ptTargetInFrame[1], z, o);
    ExecuteGroup(R_GROUP);
}


/*  执行移动左/右臂至目标关节角
    输入: a6, a5, a4, a3, a2, a1, a0(注意末端关节在第一位)      参数可缺省, 默认为(0, 0, 0, 0, 0, 0, 0)  */
void CKukaMoveit::MoveToLeftJoint(double a6, double a5, double a4, double a3, double a2, double a1, double a0){
    SetJoint(L_target_joint, a6, a5, a4, a3, a2, a1, a0);
    ExecuteJointGroup(L_GROUP);
}
void CKukaMoveit::MoveToRightJoint(double a6, double a5, double a4, double a3, double a2, double a1, double a0){
    SetJoint(R_target_joint, a6, a5, a4, a3, a2, a1, a0);
    ExecuteJointGroup(R_GROUP);
}
/*  执行移动左/右臂至目标关节角(函数重载)
    输入: {a0, a1, a2, a3, a4, a5, a6}, 注意末端关节在最后一位      参数可缺省, 默认为{0, 0, 0, 0, 0, 0, 0}  */
void CKukaMoveit::MoveToLeftJoint(vector<double> vJoints){
    if(vJoints.size() > 7)        cout << "【ERROR】vJoints.size()>7 in MoveToLeftJoint()\n";
    L_target_joint = {0, 0, 0, 0, 0, 0, 0};
    FOR(i, 0, vJoints.size())       L_target_joint[i] = vJoints[i];
    ExecuteJointGroup(L_GROUP);
}
void CKukaMoveit::MoveToRightJoint(vector<double> vJoints){
    if(vJoints.size() > 7)        cout << "【ERROR】vJoints.size()>7 in MoveToRightJoint()\n";
    R_target_joint = {0, 0, 0, 0, 0, 0, 0};
    FOR(i, 0, vJoints.size())       R_target_joint[i] = vJoints[i];
    ExecuteJointGroup(R_GROUP);
}

/*  执行移动双臂至目标关节角
    输入: L{a0, a1, a2, a3, a4, a5, a6}, R{a0, a1, a2, a3, a4, a5, a6}, 注意末端关节在最后一位
        参数可缺省, 默认为{0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}  */
void CKukaMoveit::MoveToDualJoint(vector<double> vLeftJoints, vector<double> vRightJoints){
    if(vLeftJoints.size() > 7)      cout << "【ERROR】vLeftJoints.size()>7 in MoveToDualJoint()\n";
    if(vRightJoints.size() > 7)     cout << "【ERROR】vRightJoints.size()>7 in MoveToDualJoint()\n";
    L_target_joint = {0, 0, 0, 0, 0, 0, 0};
    R_target_joint = {0, 0, 0, 0, 0, 0, 0};
    FOR(i, 0, vLeftJoints.size())       L_target_joint[i] = vLeftJoints[i];
    FOR(i, 0, vRightJoints.size())      R_target_joint[i] = vRightJoints[i];
    ExecuteJointGroup(D_GROUP);
}


/*  执行旋转左/右臂某个关节至目标角度
    输入: nJointID(关节序号, 机械臂首个关节序号为0), dJointValue(目标角度)  */
void CKukaMoveit::MoveOneLeftJointToTarget(int nJointID, double dJointValue){
    if(nJointID>7 || nJointID<0)        cout << "【ERROR】nJointID in MoveOneLeftJointToTarget()\n";
    L_target_joint[nJointID] = dJointValue;
    ExecuteJointGroup(L_GROUP);
}
void CKukaMoveit::MoveOneRightJointToTarget(int nJointID, double dJointValue){
    if(nJointID>7 || nJointID<0)        cout << "【ERROR】nJointID in MoveOneRightJointToTarget()\n";
    R_target_joint[nJointID] = dJointValue;
    ExecuteJointGroup(R_GROUP);
}


/*  执行左/右臂某个关节相对当前角度旋转一个增量
    输入: nJointID(关节序号, 机械臂首个关节序号为0), dJointIncrease(角度增量)  */
void CKukaMoveit::MoveOneLeftJointIncrease(int nJointID, double dJointIncrease){
    robot_state = L.getCurrentState();
    robot_state->copyJointGroupPositions(robot_state->getJointModelGroup("L"), L_target_joint);
    L_target_joint[nJointID] += dJointIncrease;
    ExecuteJointGroup(L_GROUP);
}
void CKukaMoveit::MoveOneRightJointIncrease(int nJointID, double dJointIncrease){
    robot_state = R.getCurrentState();
    robot_state->copyJointGroupPositions(robot_state->getJointModelGroup("R"), R_target_joint);
    R_target_joint[nJointID] += dJointIncrease;
    ExecuteJointGroup(R_GROUP);
}


/*  执行机械臂回到 HOME 关节位置
    输入: strGroup(规划组名称)      参数可缺省, 默认执行双臂规划组"D"  */
void CKukaMoveit::GoHome(string strGroup){
    if(strGroup == D_GROUP || strGroup == L_GROUP)
        SetLeftJoint(3.14, 1.1, 0, -1.1, 0, -0.6);
    if(strGroup == D_GROUP || strGroup == R_GROUP)
        SetRightJoint(3.14, 1.1, 0, -1.1, 0, -0.6);
    ExecuteJointGroup(strGroup);
}

/*  将图像的像素坐标(col, row)转换为相机坐标的(x, y), 像素相关参数在 move_kuka.h 文件宏定义中修改
    输入: 像素坐标系下的 cv::Point(col, row)
    输出: 相机坐标系下的 {x, y}  */
vector<double> CKukaMoveit::PointPixel2CameraFrame(Point ptPixel){
    double fix_x = 0.01*(8.6/267.33*(ptPixel.x-352.67)-0.5);        //  /*
    double fix_y = 0.01*(5.8/82*(ptPixel.y-298.75)+5);              //      手动测量误差后进行补偿, 相机移动或重新标定后需自行设计补偿方程  */
    return {(ptPixel.x-PAN_X)/PIXELS_OF_1MX+fix_x, (ptPixel.y-PAN_Y)/PIXELS_OF_1MY-fix_y};
}


/*  将RPY转换为四元数XYZW
    输入: r, p, y
    输出: {x, y, z, w}  */
vector<double> CKukaMoveit::RPY2XYZW(double oz, double oy, double ox){
    vector<double> vXYZW;
    vXYZW.push_back(cos(oz/2)*cos(oy/2)*sin(ox/2)+sin(oz/2)*sin(oy/2)*cos(ox/2));
    vXYZW.push_back(cos(oz/2)*sin(oy/2)*cos(ox/2)-sin(oz/2)*cos(oy/2)*sin(ox/2));
    vXYZW.push_back(sin(oz/2)*cos(oy/2)*cos(ox/2)+cos(oz/2)*sin(oy/2)*sin(ox/2));
    vXYZW.push_back(cos(oz/2)*cos(oy/2)*cos(ox/2)-sin(oz/2)*sin(oy/2)*sin(ox/2));
    return vXYZW;
}