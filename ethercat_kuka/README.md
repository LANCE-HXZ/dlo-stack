# A ROS driver for panasonic EtherCat protocol

## Install ros-soem for Open EtherCAT Library

`sudo apt-get install ros-melodic-soem`

or

`sudo apt-get install ros-kinetic-soem`

## Install ethercat_grant

`sudo apt-get install ros-melodic-ethercat-grant`

or

`sudo apt-get install ros-kinetic-ethercat-grant`

## catkin_make

## How to use

Topic list:

- /minas_cmd

> Send cmd to control minas driver

- /minas_status

> Echo this topic to recieve message of minas driver

Service list:

- /minas_set_home

> Call this service to set zero point.
> Parameter is :
> 0 -> UPPER
> 1 -> ROTA
