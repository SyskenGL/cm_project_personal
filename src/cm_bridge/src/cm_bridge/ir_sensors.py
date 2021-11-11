#!/usr/bin/env python3
import rospy
from cm_ros.wrapper import CMNode
from cm_msgs.msg import Event, IRSensorsInfo


class CMIRSensors(CMNode):

    def __init__(self, name='cm_ir_sensors'):
        CMNode.__init__(self, name)

        self.__ir_sensors_info = None

        self.__pub_ir_sensors = rospy.Publisher(
            '~info',
            IRSensorsInfo,
            queue_size=5,
            latch=True
        )
        self.__sub_event = None

    def on_event_published(self, event):
        new_ir_sensors_info = event.robot_info.ir_sensors_info
        if self.__ir_sensors_info != new_ir_sensors_info:
            self.__ir_sensors_info = new_ir_sensors_info
            self.__pub_ir_sensors.publish(self.__ir_sensors_info)

    def run(self):
        self.__sub_event = rospy.Subscriber(
            '/cm_bridge/cm_low_level/event',
            Event,
            self.on_event_published
        )
        rospy.spin()
