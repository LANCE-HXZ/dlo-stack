#include "gripper_control.h"
/*使用单个夹爪，请先
"rosrun robotiq_2f_gripper_control X_Robotiq2FGripperRtuNode.py  /dev/ttyUSB(num) X_  "
X为"R"或"L",num为当前USB号

同时使用两只夹爪时，运行
"roslaunch robotiq_2f_gripper_control robotiq_2f_dual_gripper_control.launch "
默认右手为ttyUSB0，左手为ttyUSB1,若不符合，则运行
"roslaunch robotiq_2f_gripper_control robotiq_2f_dual_gripper_control.launch device0:="/dev/ttyUSB(num) X_"  device1:="/dev/ttyUSB(num) X_"""
*/

CGripperControl::CGripperControl()//ros::NodeHandle &nh):nh_(nh)
{
    pub=nh_.advertise<dlo::Robotiq2FGripper_robot_output>("R_Robotiq2FGripperRobotOutput",1);
    pub_=nh_.advertise<dlo::Robotiq2FGripper_robot_output>("L_Robotiq2FGripperRobotOutput",1);
    Gripper_Reset();
}


CGripperControl::~CGripperControl()
{}

void CGripperControl::genCommand(string a){
    if(a=="a")
    {
        msg.rACT = 1;
        msg.rGTO = 1;
        msg.rSP  = 255;
        msg.rFR  = 150;
    }
    else if(a=="c")
    {
         msg.rPR = 255;
    }
    else if(a=="o")
    {
        msg.rPR = 0  ; 
    }
    else if(a=="r")
    {
        msg.rACT = 0;
    }
    else 
    {
        int num=stoi(a);
        if(num>255)
            msg.rPR=255;
        else if(num<0)
            msg.rPR=0;
        else
            msg.rPR=num;
    }
}

//输入需要哪只手open，默认为右，可输入"R"" "L" "D"
void CGripperControl::Gripper_Open(char which){
    genCommand("o");
    switch (which){
    case 'R':
        pub.publish(msg);
        ros::Duration(2).sleep();
        break;
    case 'L':
        pub_.publish(msg);
        ros::Duration(2).sleep();
        break;
    case 'D':
        pub.publish(msg);
        pub_.publish(msg);
        ros::Duration(2).sleep();
        break;
    default:
        pub.publish(msg);
        ros::Duration(2).sleep();
        break;
    }
}

void CGripperControl::Gripper_Close(char which){
    genCommand("c");
    switch (which){
    case 'R':
        pub.publish(msg);
        ros::Duration(2).sleep();
        break;
    case 'L':
        pub_.publish(msg);
        ros::Duration(2).sleep();
        break;
    case 'D':
        pub.publish(msg);
        pub_.publish(msg);
        ros::Duration(2).sleep();
        break;
    default:
        pub.publish(msg);
        ros::Duration(2).sleep();
        break;
    }
}

void CGripperControl::Gripper_anypose(char which, string pose){
    switch (which){
    case 'R':
        genCommand(pose);
        pub.publish(msg);
        ros::Duration(2).sleep();
        break;
    case 'L':
        genCommand(pose);
        pub_.publish(msg);
        ros::Duration(2).sleep();
        break;
    default:
        genCommand(pose);
        pub.publish(msg);
        ros::Duration(2).sleep();
        break;
    }
}
//pose范围0-255
void CGripperControl::Dual_Gripper_anypose(string pose1,string pose2){
    genCommand(pose1);
    pub.publish(msg);
    genCommand(pose2);
    pub_.publish(msg);
    ros::Duration(2).sleep();
}

void CGripperControl::SetClose(string input){
    strClose=input;
}

void CGripperControl::SetOpen(string input){
    strOpen=input;
}

void CGripperControl::NewClose(char which){
    Gripper_anypose(which,strClose);
}

void CGripperControl::NewOpen(char which){
    Gripper_anypose(which,strOpen);
}

void CGripperControl::Gripper_Reset(){
    ros::Duration(1).sleep();
    genCommand("r");
    pub.publish(msg);
    pub_.publish(msg);
    ros::Duration(0.5).sleep();
    genCommand("a");
    pub.publish(msg);
    pub_.publish(msg);
    ros::Duration(3.5).sleep();
}