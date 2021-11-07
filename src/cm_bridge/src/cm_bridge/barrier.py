#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from cm_ros.wrapper import CMNode
from cm_msgs.msg import BarrierInfo
from cm_bridge.low_level import unpack_barrier_info


class CMBarrier(CMNode):

    def __init__(self, name='cm_barrier'):
        CMNode.__init__(self, name)

        self.__barrier_info = None

        self.__pub_barrier = rospy.Publisher('~info', BarrierInfo, queue_size=5, latch=True)
        self.__sub_events = None

    def on_event_published(self, event):
        new_barrier_info = unpack_barrier_info(event.data)
        if self.__barrier_info is None or self.__barrier_info != new_barrier_info:
            self.__barrier_info = new_barrier_info
            self.__pub_barrier.publish(self.__barrier_info)

    def run(self):
        self.__sub_events = rospy.Subscriber(
            '/cm_bridge/cm_low_level/events', String, self.on_event_published
        )
        rospy.spin()