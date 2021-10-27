#!/usr/bin/env python3
import rospy
import serial
import actionlib
from cm_ros.wrapper import CMNode


class CMAPIManager(CMNode):

    def __init__(self, name='cm_api_manager'):
        CMNode.__init__(self, name)

        # Serial port used to communicate with the robot
        self.serial_port = rospy.get_param('~serial_port')

        # Serial Stream
        self.__serial = None
        self.subscribe()

    def subscribe(self):
        try:
            self.__serial = serial.Serial(self.serial_port)
        except serial.SerialException as err:
            rospy.logerr('[{name}] - {err}'.format(name=rospy.get_name(), err=err))
            raise err

    def unsubscribe(self):
        if (self.__serial is not None) and self.__serial.isOpen():
            self.__serial.close()

    def run(self):
        while True:
            pass
