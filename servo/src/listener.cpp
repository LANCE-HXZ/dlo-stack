#include "ros/ros.h"
#include <servo/JointQuantity.h>
 
//回调函数，简单来说就是收到了这个消息，就做出什么反应在这里面定义。这里是打印一行东西
void chatterCallback(const servo::JointQuantity::ConstPtr & msg)
{
    ROS_INFO("I heard: [%f]", msg->a1);
    ROS_INFO("I heard: [%f]", msg->a2);
    ROS_INFO("I heard: [%f]", msg->a3);
    ROS_INFO("I heard: [%f]", msg->a4);
    ROS_INFO("I heard: [%f]", msg->a5);
    ROS_INFO("I heard: [%f]", msg->a6);
    ROS_INFO("I heard: [%f]", msg->a7);    
}
 
int main(int argc, char **argv)
{
    ros::init(argc,argv,"servo_listener");
    ros::NodeHandle n;
    //订阅了“chatter”这个topic
    ros::Subscriber sub = n.subscribe("chatter",1000, chatterCallback);
    ros::spin();
    return 0;
}
