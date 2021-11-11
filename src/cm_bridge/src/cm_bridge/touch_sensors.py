#!/usr/bin/env python3
import rospy
from cm_ros.wrapper import CMNode
from cm_msgs.msg import Event, TouchSensorsInfo, SetTouchSensorsEnabledAction


class CMTouchSensors(CMNode):

    def __init__(self, name='cm_touch_sensors'):
        CMNode.__init__(self, name)

        self.__touch_sensors_info = None

        self.__pub_touch_sensors = rospy.Publisher(
            '~info',
            TouchSensorsInfo,
            queue_size=5,
            latch=True
        )
        self.__sub_event = None
        
        self.__touch_sensors_server(
            '~__touch_sensors_server',
            SetTouchSensorsEnabledAction,
            execute_cb=self.__on_serial_as_called,
            auto_start=False
        )

    def on_event_published(self, event):
        new_touch_sensors_info = event.robot_info.touch_sensors_info
        if self.__touch_sensors_info != new_touch_sensors_info:
            self.__touch_sensors_info = new_touch_sensors_info
            self.__pub_touch_sensors.publish(self.__touch_sensors_info)

    def run(self):
        self.__sub_event = rospy.Subscriber(
            '/cm_bridge/cm_low_level/event',
            Event,
            self.on_event_published
        )
        rospy.spin()
