#include "ros/ros.h"
#include <servo/JointQuantity.h>
 
int main(int argc, char **argv)
{
    ros::init(argc,argv, "servo_talker");
    ros::NodeHandle n;
    //topic的名称为chatter,1000为缓冲区，缓冲区中的消息在大于 1000 个的时候就会开始丢弃先前发布的消息。 
    ros::Publisher chatter = n.advertise<servo::JointQuantity>("chatter",1000);
    ros::Rate loop_rate(10); //发布频率为10FPS
 
    while(n.ok())
    {
        servo::JointQuantity msg;
        //给消息里的变量赋值
        msg.a1 = 0;
        msg.a2 = 0;
        msg.a3 = 0;
        msg.a4 = 0;
        msg.a5 = 0;
        msg.a6 = 0;
        msg.a7 = 1;
 
        //发布消息
        chatter.publish(msg);
        ros::spinOnce(); //可用回调函数
        loop_rate.sleep();//休眠一段时间以使得发布频率为 10Hz。
    }
    return 0;
}
