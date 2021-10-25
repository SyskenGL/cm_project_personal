#!/usr/bin/env python3
import rospy
import serial
from cm_ros.wrapper import CMNode


class CMSerialManager(CMNode):

    def __init__(self, name='cm_serial_reader'):
        CMNode.__init__(self, name)

        # Serial port used to communicate with the robot
        self.serial_port = rospy.get_param('~serial_port')

        # Serial stream
        self.__serial = None

    def subscribe(self):
        try:
            self.__serial = serial.Serial(self.serial_port)
        except serial.SerialException as err:
            rospy.logerr('From {name} - {err}'.format(name=rospy.get_name(), err=err))

    def unsubscribe(self):
        if (self.__serial is not None) and self.__serial.isOpen():
            self.__serial.close()

    def run(self):
        while not rospy.is_shutdown():
            response = self.__serial.readline()
            print(response)