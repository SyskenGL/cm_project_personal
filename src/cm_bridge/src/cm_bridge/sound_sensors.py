#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from cm_ros.wrapper import CMNode
from cm_msgs.msg import SoundSensorsInfo
from cm_bridge.low_level import unpack_sound_sensors_info


class CMSoundSensors(CMNode):

    def __init__(self, name='cm_sound_sensors'):
        CMNode.__init__(self, name)

        self.__sound_sensors_info = None

        self.__pub_sound_sensors = rospy.Publisher('~info', SoundSensorsInfo, queue_size=5, latch=True)
        self.__sub_events = None

    def on_event_published(self, event):
        new_sound_sensors_info = unpack_sound_sensors_info(event.data)
        if self.__sound_sensors_info is None or self.__sound_sensors_info != new_sound_sensors_info:
            self.__sound_sensors_info = new_sound_sensors_info
            self.__pub_sound_sensors.publish(self.__sound_sensors_info)

    def run(self):
        self.__sub_events = rospy.Subscriber(
            '/cm_bridge/cm_low_level/events', String, self.on_event_published
        )
        rospy.spin()