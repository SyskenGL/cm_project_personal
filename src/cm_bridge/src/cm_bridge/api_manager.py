#!/usr/bin/env python3
import rospy
import serial
import actionlib
from std_msgs.msg import String
from collections import defaultdict
from cm_ros.wrapper import CMNode
from cm_msgs.msg import CallAPIAction, CallAPIResult


class CMAPIManager(CMNode):

    def __init__(self, name='cm_api_manager'):
        CMNode.__init__(self, name)

        # Set initial configurations
        self.config = defaultdict(lambda: None)
        self.__init_config()

        # Serial Stream
        self.__serial = None
        self.__subscribe()

        # ROS Topic used to publish serial output
        self.__pub_serial = rospy.Publisher('~serial', String, queue_size=10)

    def __init_config(self):
        # Serial port used to communicate with the robot
        self.config['serial_port'] = rospy.get_param('~serial_port')

        # Number of symbols transmitted in one second
        self.config['baud_rate'] = rospy.get_param('~baud_rate')

    def __subscribe(self):
        try:
            self.__serial = serial.Serial(
                self.config['serial_port'],
                self.config['baud_rate']
            )
        except serial.SerialException as err:
            rospy.logerr('[{name}] - {err}'.format(name=rospy.get_name(), err=err))
            raise err

    def __unsubscribe(self):
        if (self.__serial is not None) and self.__serial.isOpen():
            self.__serial.close()

    def __listen(self):
        # Listen on the serial port then publish on the topic
        while not rospy.is_shutdown():
            serial_out = self.__serial.readline()
            self.__pub_serial.publish(serial_out)

    def run(self):
        self.__listen()

