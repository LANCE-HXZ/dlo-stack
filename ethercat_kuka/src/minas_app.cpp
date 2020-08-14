#include "minas_app.h"

MinasApp::MinasApp(const string &_ifname, ros::NodeHandle* nodehandle) : ifname(_ifname),
                                            nh_(*nodehandle),
                                            manager(_ifname),
                                            timeStamp(TIME_STAMP_PERIOD)                                            
{
    /* Search and config all devices */
    for (uint8_t clientNo = 0; clientNo < manager.getNumClients(); clientNo++)
    {
        vecClient.push_back(new minas_control::MinasClient(manager, clientNo + 1));
        
        //-- change to MINAS driver ID
        hMinas.push_back(clientNo);

        //-- motor not arrive
        vecArriveFlag.push_back(FALSE);
    }
    minasInit();
    rosInit();

}

void MinasApp::minasInit(void)
{
    cout << "=========>> [Enter minasInit]" << endl;
    for (uint8_t clientNo = 0; clientNo < hMinas.size(); clientNo++)
    {
        // uint8_t clientNo = 1;
        minas_control::MinasClient *client = vecClient[clientNo];
        //-- clear error
        client->reset();
        if (clientNo == ROTA)
        {
            client->setTrqueForEmergencyStop(100);      // 100%
            client->setOverLoadLevel(500);               // 50%
            client->setOverSpeedLevel(1200);             // r/min
            client->setMotorWorkingRange(100);          // 0.1
            client->setInterpolationTimePeriod(4000);   // 4 msec
            //  -- set profile velocity
            client->setProfileVelocity(0x3200000);
            //-- set profile acceleration
            client->setProfileAcceleration(0x8000000);
            //-- set profile deceleration
            client->setProfileDeceleration(0x8000000);
        }
        else
        {
            client->setTrqueForEmergencyStop(100);      // 100%
            client->setOverLoadLevel(500);               // 50%
            client->setOverSpeedLevel(1200);             // r/min
            client->setMotorWorkingRange(0.1);          // 0.1
            client->setInterpolationTimePeriod(4000);   // 4 msec
            //  -- set profile velocity
            client->setProfileVelocity(0x3200000);
            //-- set profile acceleration
            client->setProfileAcceleration(0x8000000);
            //-- set profile deceleration
            client->setProfileDeceleration(0x8000000);
        }
        // -- set parameter from PANATERM test program
        
        // -- servo on
        client->servoOn();
        //-- get a initial position for motor action once
        minas_control::MinasInput input = client->readInputs();
        vecInput.push_back(input);

        printMsgInput(clientNo);

        vecInitialPos.push_back(vecInput[clientNo].position_actual_value);

        //-- set target position
        minas_control::MinasOutput output;
        memset(&output, 0x00, sizeof(minas_control::MinasOutput));
        vecOutput.push_back(output);

        //-- Position control
        vecOutput[clientNo].target_position = vecInitialPos[clientNo];
    }

    cout << "=========>> [Leave minasInit]" << endl;
}

void MinasApp::rosInit(void)
{
    minas_pub_ = nh_.advertise<minas_hw::minas_state>("minas_state", 10);
    minas_sub_ = nh_.subscribe("minas_cmd", 1, &MinasApp::cmdCallback,this);

    set_home_pos_ = nh_.advertiseService("minas_set_home", &MinasApp::handle_set_home,this);

    if (!nh_.getParam("minas_hw/rota_pos_limit", pos_limit[ROTA]))
        { ROS_ERROR("minas_hw/rota_pos_limit"); }
    if (!nh_.getParam("minas_hw/rota_neg_limit", neg_limit[ROTA]))
        { ROS_ERROR("No minas_hw/rota_neg_limit param"); }
    if (!nh_.getParam("minas_hw/upper_pos_limit", pos_limit[UPPER]))
        { ROS_ERROR("No minas_hw/upper_pos_limit param"); }
    if (!nh_.getParam("minas_hw/upper_neg_limit", neg_limit[UPPER]))
        { ROS_ERROR("No minas_hw/upper_neg_limit param"); }    
}

void MinasApp::cmdCallback(const minas_hw::minas_cmd &cmd)
{
    printf("I'm heard cmd is [%f], [%f]",cmd.rota,cmd.upper);

    //soft limit
    if (cmd.rota >= pos_limit[ROTA])
    {
        cmd_msg_.rota = pos_limit[ROTA];
        ROS_ERROR("Set cmd is out of pos range [%d]",ROTA);
    }
    else if (cmd.rota <= neg_limit[ROTA])
    {
        cmd_msg_.rota = neg_limit[ROTA];
        ROS_ERROR("Set cmd is out of neg range [%d]",ROTA);
    }
    else
    {
        cmd_msg_.rota = cmd.rota;
    }

    if (cmd.upper >= pos_limit[UPPER])
    {
        cmd_msg_.upper = -pos_limit[UPPER];
        ROS_ERROR("Set cmd is out of pos range [%d]",UPPER);
    }
    else if (cmd.upper <= neg_limit[UPPER])
    {
        cmd_msg_.upper = -neg_limit[UPPER];
        ROS_ERROR("Set cmd is out of neg range [%d]",UPPER);
    }
    else
    {
        cmd_msg_.upper = - cmd.upper;
    }

    minasCtrl(ROTA,cmd_msg_.rota);
    minasCtrl(UPPER,cmd_msg_.upper);
}

bool MinasApp::handle_set_home(minas_hw::SetHome::Request &req,
                         minas_hw::SetHome::Response &res)
{
    ROS_INFO("I'm in service");
    minas_control::MinasClient *client1 = vecClient[0];
    minas_control::MinasClient *client2 = vecClient[1];
    uint8_t num = req.client_num;
    int32_t homeset_current, pos_current;
    int16_t inhibit_current;
    // int32_t homeset_current = client->read_homeset();
    // ROS_INFO("homeset is %d",homeset_current);
    /*debug*/
    switch (num)
    {
        case 0:
            pos_current = client1->read_actual_pos();
            ROS_INFO("pos_current is %d",pos_current);
            client1->set_home_pos(-pos_current);
            break;
        case 1:
            pos_current = client2->read_actual_pos();
            ROS_INFO("pos_current is %d",pos_current);
            client2->set_home_pos(-pos_current);
            break;
        default:
            break;
    }
    res.success = true;
    return true;
}

void MinasApp::minasFree(void)
{
    cout << "=========>> [Enter minasFree]" << endl;

    for (vector<minas_control::MinasClient *>::iterator it = vecClient.begin(); it != vecClient.end(); ++it)
    {
        minas_control::MinasClient *client = (*it);
        
        minas_control::MinasInput input = client->readInputs();

        client->printPDSStatus(input);
        client->printPDSOperation(input);

        //-- servo off
        client->servoOff();
    }

    cout << "=========>> [Leave minasFree]" << endl;
}

void MinasApp::minasCtrl(MinasHandle _handle, double _cmd)
{
    cout << "=========>> [Enter minasCtrl]" << endl;
    minas_control::MinasClient *client = vecClient[_handle];
    
    if ((int)_handle == ROTA)
    {
        vecOutput[_handle].target_position = ((ROTA_RESOLU * ROTA_GEAR) /360.0) * _cmd;
    }
    else
    {
        vecOutput[_handle].target_position = ((UPPER_RESOLU * UPPER_GEAR)) * _cmd;
    }
    //-- Position control
   
    vecOutput[_handle].max_motor_speed = 1140;   // rad/min
    vecOutput[_handle].target_torque = 5000;     // 0% (unit 0.1%)
    vecOutput[_handle].max_torque = 5000;        // 50% (unit 0.1%)
    vecOutput[_handle].operation_mode = 0x01;   // position profile mode (pp)
    vecOutput[_handle].controlword = 0x003f;    // move to operation enabled + new set-point (bit4) + change set immediately (bit5)
    
    printf("device [%d]: output target position is [%d]:",_handle,vecOutput[_handle].target_position);
    //-- pp control model setup
    //-- see controlword(6040.h) p.107 & statusword(6041.h) p.113
    client->writeOutputs(vecOutput[_handle]);
    while (!(vecInput[_handle].statusword & 0x1000))
    {
        vecInput[_handle] = client->readInputs(); // bit12 (set-point acknowledge)
    }
    
    printMsgInput(_handle);

    vecOutput[_handle].controlword &= ~0x0010; // clear new set-point (bit4)
    client->writeOutputs(vecOutput[_handle]);


    printMsgInput(_handle);

    cout << "Target position has been changed to: "<< hex << vecOutput[_handle].target_position << endl;

    cout << "=========>> [Leave minasConfig]" << endl;

    cout << "=========>> [Leave minasCtrl]" << endl;
}

void MinasApp::readInput(void)
{
    for (uint8_t clientNo = 0; clientNo < hMinas.size(); clientNo++)
    {
        minas_control::MinasClient *client = vecClient[clientNo];
        vecInput[clientNo] = client->readInputs();
    }
}

void MinasApp::pubMsgs(void)
{
    uint32_t rota_state, upper_state;

    rota_state = read_status(ROTA);
    state_msg_.rota.pos = ((int)read_position(ROTA) * 360.0)  / (ROTA_RESOLU * ROTA_GEAR);
    state_msg_.rota.lower_limit  = (rota_state >> 0) & 0x01;
    state_msg_.rota.upper_limit  = (rota_state >> 1) & 0x01;
    state_msg_.rota.origin_point = (rota_state >> 2) & 0x01;

    upper_state = read_status(UPPER);
    state_msg_.upper.pos =  - ((int)read_position(UPPER)) / (UPPER_RESOLU * UPPER_GEAR);
    state_msg_.upper.lower_limit  = (upper_state >> 0) & 0x01;
    state_msg_.upper.upper_limit  = (upper_state >> 1) & 0x01;
    state_msg_.upper.origin_point = (upper_state >> 2) & 0x01;

    minas_pub_.publish(state_msg_);
}

void MinasApp::run(void)
{
    ros::Rate loop_rate_(1000);
    while (ros::ok())
    {
        readInput();
        pubMsgs();

        ros::spinOnce();
        loop_rate_.sleep();
    }
    minasFree();
}


/* public function */
uint32 MinasApp::read_position(MinasHandle _handle)
{
    return vecInput[_handle].position_actual_value;
}

uint32 MinasApp::read_status(MinasHandle _handle)
{
    return vecInput[_handle].digital_inputs;
}

void MinasApp::printMsg(MinasHandle _handle, uint32_t _cycle)
{
    printf("\n\n\n");
    printf("Period   %d\n", _cycle);
    timeStamp.printMsg();
    printMsgInput(_handle);
    printMsgOutput(_handle);
}

void MinasApp::printMsgInput(MinasHandle _handle)
{
    printf("Input:\n");
        printf("   603Fh %08x :Error code\n", vecInput[_handle].error_code);
        printf("   6041h %08x :Statusword\n", vecInput[_handle].statusword);
        printf("   6061h %08x :Modes of operation display\n", vecInput[_handle].operation_mode);
        printf("   6064h %08x :Position actual value\n", vecInput[_handle].position_actual_value);
        printf("   606Ch %08x :Velocity actual value\n", vecInput[_handle].velocity_actual_value);
        printf("   6077h %08x :Torque actual value\n", vecInput[_handle].torque_actual_value);
        printf("   60B9h %08x :Touch probe status\n", vecInput[_handle].touch_probe_status);
        printf("   60BAh %08x :Touch probe pos1 pos value\n", vecInput[_handle].touch_probe_posl_pos_value);
        printf("   60FDh %08x :Digital inputs\n", vecInput[_handle].digital_inputs);
}
    
void MinasApp::printMsgOutput(MinasHandle _handle)
{
    printf("Output:\n");
        printf("   6040h %08x :Controlword\n", vecOutput[_handle].controlword);
        printf("   6060h %08x :Mode of operation\n", vecOutput[_handle].operation_mode);
        printf("   6071h %08x :Target Torque\n", vecOutput[_handle].target_torque);
        printf("   6072h %08x :Max Torque\n", vecOutput[_handle].max_torque);
        printf("   607Ah %08x :Target Position\n", vecOutput[_handle].target_position);
        printf("   6080h %08x :Max motor speed\n", vecOutput[_handle].max_motor_speed);
        printf("   60B8h %08x :Touch Probe function\n", vecOutput[_handle].touch_probe_function);
        printf("   60FFh %08x :Target Velocity\n", vecOutput[_handle].target_velocity);
        printf("   60B0h %08x :Position Offset\n", vecOutput[_handle].position_offset);
}