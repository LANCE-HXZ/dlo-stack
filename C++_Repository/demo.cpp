#include "kuka_moveit.h"
#include "gripper_control.h"

int main(int argc, char *argv[]){
    ros::init(argc, argv, "Node_Name");
    ros::NodeHandle nh;

    /*
        ros::init(argc, argv, "Node_Name");
        ros::NodeHandle nh;
        确保定义类 CKukaMoveit 的对象前有以上两行, 创建 ROS 句柄
        在头文件中查看相关宏定义，根据自己实际情况修改和使用
        函数的参数大部分可以缺省，具体查看头文件中的参数默认值
        使用 Set 函数后需要使用 ExecuteGroup 或 ExecuteJointGroup 函数才能执行机械臂
        使用 Move 函数后直接执行
                                                                                    */

    CKukaMoveit km;
    km.GoHome(D_GROUP);             //  双臂回 home 点

    km.SetFrame(CAMERA_FRAME);      //  设置参考坐标系，参数为 string 类型，默认为头文件宏定义的相机坐标系

    //  位姿控制

    km.SetLeftPose(cv::Point(240, 320), 0.94, {1.57, 0, 0});    //  左臂移动到相机视野的 (240, 320) 坐标，ｚ轴高度为 0.94，姿态沿ｚ轴旋转 90 度，姿态参数定义为绕 {z, y, x} 轴旋转的角度
    km.ExecuteGroup("L");                                       //  setpose 设定目标位姿后使用, 执行到目标位姿, 参数填规划组名称 (string)，也可使用宏定义

    km.SetLeftPose(0, 0, 0.94, {0, 0, 0, 1});                   //  位置可以直接使用相对于参考坐标系的 x, y, z; 姿态可以使用四元素 {ox, oy, oz, ow}
    km.SetRdxdydz(0.1, 0, 0.2);                                 //  设置右臂目标位置相对与当前沿ｘ轴移动 0.1m，沿ｚ轴移动 -0.2m
    km.ExecuteGroup();                                          //  左右臂的目标位姿设定后可同时执行双臂，参数缺省则默认为执行双臂规划组"D"

    km.MoveRdxdydz(0, 0, -0.1);                                 //  右臂沿ｚ轴移动-0.1米

    km.MoveToLeftPose(cv::Point(240, 320), 0.94, {1.57, 0, 0}); //  左臂直接移动到某个位姿
    km.MoveToRightPose(0, 0, 0.94, {0, 0, 0, 1});               //  右臂直接移动到某个位姿

    //  关节控制

    km.SetLeftJoint(0, 1.57, 0, -1.57, 0.3);                    //  设置左臂的末端五个关节的目标角度，缺省的参数关节角默认为0
    km.SetRightJoint(0, 0, 0, 0, 0, 0, 0.1);                    //  设置右臂的第一关节的目标角度为0.1弧度
    km.ExecuteJointGroup("D");                                  //  设置好目标关节角后执行

    km.MoveToLeftJoint(0, 0, 0, 0, 0, 0, 0.1);                              //  直接移动到
    km.MoveToRightJoint({0, 0, 0, 0, 0, 0, 0});                             //  参数可以传入vector<double>
    km.MoveToDualJoint({0, 0, 0, 0, 0, 0, 0}, {0.3, 0, 0, 0, 0, 0, 0.1});   //  直接移动左右臂的关节

    km.MoveOneLeftJointToTarget(6, 1.57);                       //  末端关节(6)移动到绝对角度1.57
    km.MoveOneRightJointIncrease(0, -0.1);                      //  第一关节(0)相对于当前角度旋转-0.1



    /*
        用于 Robotiq 夹爪的控制, 用到以下文件, 官方提供的功能包不包含，注意自行添加
        robotiq/robotiq_2f_gripper_control/launch 中 robotiq_2f_dual_gripper_control.launch
        robotiq/robotiq_2f_gripper_control/nodes 中 L_Robotiq2FGripperRtuNode.py, R_Robotiq2FGripperRtuNode.py

        使用单个夹爪，请先
        "rosrun robotiq_2f_gripper_control X_Robotiq2FGripperRtuNode.py  /dev/ttyUSB(num) X_  "
        X为"R"或"L",num为当前USB号

        同时使用两只夹爪时，运行
        "roslaunch robotiq_2f_gripper_control robotiq_2f_dual_gripper_control.launch "
        默认右手为ttyUSB0，左手为ttyUSB1,若不符合，则运行
        "roslaunch robotiq_2f_gripper_control robotiq_2f_dual_gripper_control.launch device0:=/dev/ttyUSB(num) X_  device1:=/dev/ttyUSB(num) X_"

        ros::init(argc, argv, "Node_Name");
        ros::NodeHandle nh;
        确保定义类 CGripperControl 的对象前有以上两行, 创建 ROS 句柄
                                                                                                                                                        */

    CGripperControl gc;
    gc.Dual_Gripper_anypose("220", "160");                      //  左右夹爪分别闭合至　220, 160, 默认值是 0（open）
    gc.Gripper_anypose('R', "160");                             //  设置任意夹爪至任意值

    gc.Gripper_Open('R');                                       //  右夹爪全开
    gc.Gripper_Close('D');                                      //  左夹爪全关

    gc.SetClose("200");                                         //  用于自定义关夹爪的行程值，此后使用NewClose函数关至　200
    gc.SetOpen("100");                                          //  用于自定义开夹爪的行程值，此后使用NewOpen函数关至　100
    gc.NewClose('R');                                           //  自定义行程的关右夹爪
    gc.NewOpen('L');                                            //  自定义行程的开左夹爪
}