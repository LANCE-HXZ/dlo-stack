#pragma once

/* public include */
#include <iostream>
#include <vector>
#include <string>
#include <time.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

/* private include */
#include "time_stamp.h"
#include "minas_client.h"
#include "ethercat_manager.h"
#include "minas_hw/minas_cmd.h"
#include "minas_hw/minas_state.h"
#include "minas_hw/SetHome.h"

//-- time stamp period in nanoseconds
#define TIME_STAMP_PERIOD 4e+6

//-- Unit command per millimeter
#define UNIT_COMMAND_MM 0x19999a

/* parameter of motors */
#define ROTA_RESOLU 8388607.0
#define ROTA_GEAR 124.0

#define UPPER_RESOLU 8388607.0
#define UPPER_GEAR 0.2 // unit r/mm

using namespace std;

typedef uint8_t MinasHandle;

/* type define */
typedef enum
{
    UPPER =0 ,
    ROTA ,
}CMD_STATE;


class MinasApp
{
public:
    MinasApp(const string &_ifname, ros::NodeHandle* nodehandle);
    MinasApp(const MinasApp &) = delete;
    MinasApp &operator=(const MinasApp &) = delete;
    ~MinasApp() {}

    void printMsg(MinasHandle _handle, uint32_t _cycle);
    void printMsgInput(MinasHandle _handle);
    void printMsgOutput(MinasHandle _handle);

    uint32 read_position(MinasHandle _handle);
    uint32 read_status(MinasHandle _handle);

    void run();

private:
    //-- handle for minas clients
    vector<MinasHandle> hMinas;
    

    //-- ROS declaration
    ros::NodeHandle nh_;
    ros::Publisher minas_pub_;
    ros::Subscriber minas_sub_;

    ros::ServiceServer set_home_pos_;   

    minas_hw::minas_state state_msg_;
    minas_hw::minas_cmd   cmd_msg_;

    float pos_limit[2];
    float neg_limit[2];

    //-- Time Stamp
    TimeStamp timeStamp;

    //-- network adapter
    string ifname;

    //-- EtherCAT manager
    ethercat::EtherCatManager manager;

    //-- vector for minas clients in application
    vector<minas_control::MinasClient *> vecClient;

    //-- vector for motors' initial position and target position
    vector<int32_t> vecInitialPos;

    //-- vector for minas input and output
    vector<minas_control::MinasInput> vecInput;
    vector<minas_control::MinasOutput> vecOutput;

    //-- motor arrive at specified position
    vector<bool> vecArriveFlag;

    /* function */
    void minasInit(void);
    void minasFree(void);

    void rosInit(void);

    void minasCtrl(MinasHandle _handle, double _cmd);

    void cmdCallback(const minas_hw::minas_cmd &cmd);
    bool handle_set_home(minas_hw::SetHome::Request &req,
                         minas_hw::SetHome::Response &res);
    void readInput(void);
    void pubMsgs(void);
};
