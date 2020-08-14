#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
roslib.load_manifest('robotiq_modbus_rtu')
import rospy
import robotiq_2f_gripper_control.baseRobotiq2FGripper
import robotiq_modbus_rtu.comModbusRtu
import os, sys
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

class GripperLoop(object):

    def __init__(self, deviceID, name):    
        #Gripper is a 2F with a TCP connection
        self.gripper = robotiq_2f_gripper_control.baseRobotiq2FGripper.robotiqbaseRobotiq2FGripper()
        self.gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()

        #We connect to the address received as an argument
        self.gripper.client.connectToDevice(deviceID)

        node_name = name + 'robotiq2FGripper'
        rospy.init_node(node_name)

        #The Gripper status is published on the topic named 'Robotiq2FGripperRobotInput'
        publisher_name = name + 'Robotiq2FGripperRobotInput'
        self.pub = rospy.Publisher(publisher_name, inputMsg.Robotiq2FGripper_robot_input)

        #The Gripper command is received from the topic named 'Robotiq2FGripperRobotOutput'
        subscriber_name = name + 'Robotiq2FGripperRobotOutput'
        rospy.Subscriber(subscriber_name, outputMsg.Robotiq2FGripper_robot_output, self.gripper.refreshCommand)
        

        #We loop
        while not rospy.is_shutdown():

            #Get and publish the Gripper status
            status = self.gripper.getStatus()
            self.pub.publish(status)     

            #Wait a little
            #rospy.sleep(0.05)

            #Send the most recent command
            self.gripper.sendCommand()

            #Wait a little
            #rospy.sleep(0.05)
            
if __name__ == '__main__':
    try:
        GripperLoop(sys.argv[1], sys.argv[2])
    except rospy.ROSInterruptException: pass
