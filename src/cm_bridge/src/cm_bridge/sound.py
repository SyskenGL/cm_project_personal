#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from cm_ros.wrapper import CMNode
from cm_msgs.msg import SoundInfo
from cm_bridge.low_level import unpack_sound_info


class CMSound(CMNode):

    def __init__(self, name='cm_sound'):
        CMNode.__init__(self, name)

        self.__sound_info = None

        self.__pub_sound = rospy.Publisher('~info', SoundInfo, queue_size=5, latch=True)
        self.__sub_events = None

    def on_event_published(self, event):
        new_sound_info = unpack_sound_info(event.data)
        if self.__sound_info is None or self.__sound_info != new_sound_info:
            self.__sound_info = new_sound_info
            self.__pub_sound.publish(self.__sound_info)

    def run(self):
        self.__sub_events = rospy.Subscriber(
            '/cm_bridge/cm_low_level/events', String, self.on_event_published
        )
        rospy.spin()