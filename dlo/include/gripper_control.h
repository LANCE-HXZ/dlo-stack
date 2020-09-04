#ifndef GRIPPER_CONTROL
#define GRIPPER_CONTROL


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dlo/Robotiq2FGripper_robot_output.h"
#include <string>

/*使用单个夹爪，请先
"rosrun robotiq_2f_gripper_control X_Robotiq2FGripperRtuNode.py  /dev/ttyUSB(num)   "
X为"R"或"L",num为当前USB号

同时使用两只夹爪时，运行
"roslaunch robotiq_2f_gripper_control robotiq_2f_dual_gripper_control.launch "
默认右手为ttyUSB0，左手为ttyUSB1,若不符合，则运行
"roslaunch robotiq_2f_gripper_control robotiq_2f_dual_gripper_control.launch device0:="/dev/ttyUSB(num) "  device1:="/dev/ttyUSB(num) ""
*/

using namespace std;

class CGripperControl{
    private:
        dlo::Robotiq2FGripper_robot_output msg;
        ros::Publisher pub;
        ros::Publisher pub_;
        ros::NodeHandle nh_;
        string strClose = "255", strOpen = "0";
    public:
        CGripperControl();//ros::NodeHandle &nh);
        ~CGripperControl();

        void genCommand(string pose);
        void Gripper_Open(char which='R');
        void Gripper_Close(char which='R');
        void Gripper_anypose(char which='R', string pose="0");
        void Dual_Gripper_anypose(string pose1="0", string pose2="0");
        void Gripper_Reset();
        void SetClose(string input);
        void SetOpen(string input);
        void NewOpen(char which='R');
        void NewClose(char which='R');
};

#endif