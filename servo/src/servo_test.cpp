#include "ros/ros.h"
#include <servo/JointQuantity.h>
 

//回调函数，简单来说就是收到了这个消息，就做出什么反应在这里面定义。这里是打印一行东西
 
int main(int argc, char **argv)
{
    ros::init(argc,argv,"servo_listener");
    ros::NodeHandle n;
    ros::Publisher chatter = n.advertise<servo::JointQuantity>("chatter",1000);
    ros::Rate loop_rate(10); //发布频率为10FPS
    //订阅了“chatter”这个topic
    servo::JointQuantity msg;
    msg.a1 = 0;
    msg.a2 = 0;
    msg.a3 = 0;
    msg.a4 = 0;
    msg.a5 = 0;
    msg.a6 = 0;
    msg.a7 = 1;
    // ros::spin();
    //发布消息
    chatter.publish(msg);
    loop_rate.sleep();//休眠一段时间以使得发布频率为 10Hz。
    return 0;
}