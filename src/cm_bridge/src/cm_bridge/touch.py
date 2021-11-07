#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from cm_ros.wrapper import CMNode
from cm_msgs.msg import TouchInfo
from cm_bridge.low_level import unpack_touch_info


class CMTouch(CMNode):

    def __init__(self, name='cm_touch'):
        CMNode.__init__(self, name)

        self.__touch_info = None

        self.__pub_touch = rospy.Publisher('~info', TouchInfo, queue_size=5, latch=True)
        self.__sub_events = None

    def on_event_published(self, event):
        new_touch_info = unpack_touch_info(event.data)
        if self.__touch_info is None or self.__touch_info != new_touch_info:
            self.__touch_info = new_touch_info
            self.__pub_touch.publish(self.__touch_info)

    def run(self):
        self.__sub_events = rospy.Subscriber(
            '/cm_bridge/cm_low_level/events', String, self.on_event_published
        )
        rospy.spin()