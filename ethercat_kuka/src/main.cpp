#include <sstream>
#include <iostream>
#include <string>

#include "minas_app.h"

#define ETHERCAT_PORT "enp0s31f6" // for cdy debug

int main(int argc, char *argv[])
{        
    ros::init(argc, argv, "minas_hw");
    ros::NodeHandle nh;

    std::string ethercat_port;
    if (!nh.getParam("minas_hw/ethercat_port", ethercat_port))
    { 
        ROS_ERROR("No ethercat_port param"); 
        ethercat_port = ETHERCAT_PORT;
        // return -1;
    }
    printf("Port is %s", ethercat_port.c_str());
    

    MinasApp app(ethercat_port.c_str(), &nh);
    app.run();

    std::cout << "End program." << std::endl;
    return 0;
}

