#!/usr/bin/env python3
import rospy
from cm_ros.wrapper import CMNode
from cm_msgs.msg import Event, SoundSensorsInfo


class CMSoundSensors(CMNode):

    def __init__(self, name='cm_sound_sensors'):
        CMNode.__init__(self, name)

        self.__sound_sensors_info = None

        self.__pub_sound_sensors = rospy.Publisher(
            '~info',
            SoundSensorsInfo,
            queue_size=5,
            latch=True
        )
        self.__sub_event = None

    def __on_event_published(self, event):
        new_sound_sensors_info = event.robot_info.sound_sensors_info
        if self.__sound_sensors_info != new_sound_sensors_info:
            self.__sound_sensors_info = new_sound_sensors_info
            self.__pub_sound_sensors.publish(self.__sound_sensors_info)

    def run(self):
        self.__sub_event = rospy.Subscriber(
            '/cm_bridge/cm_low_level/event',
            Event,
            self.__on_event_published
        )
        rospy.spin()
