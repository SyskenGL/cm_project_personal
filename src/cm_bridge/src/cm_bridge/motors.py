#!/usr/bin/env python3
import rospy
import actionlib
from cm_ros.wrapper import CMNode
from cm_msgs.msg import MotorsInfo, MoveAction, MoveResult, CallAPIAction, CallAPIGoal


class CMMotors(CMNode):

    def __init__(self, name='cm_motors'):
        CMNode.__init__(self, name)

        # Move command
        self.__move_command = '#M:'  \
            '{M0_angle},{M0_angle},' \
            '{M1_angle},{M1_angle},' \
            '{M2_angle},{M2_angle},' \
            '{M3_angle},{M3_angle},' \
            '{M4_angle},{M4_angle},' \
            '{M5_angle},{M5_angle}'  \
            '[{id}|\n'

        # ROS Topic used to publish motors info
        self.__pub_motors_info = rospy.Publisher(
            '~motors_info',
            MotorsInfo,
            queue_size=5,
            latch=True
        )

        # Initialize Action Server used to move motors
        self.__move_server = actionlib.SimpleActionServer(
            rospy.get_name(),
            MoveAction,
            execute_cb=self.__on_api_call_received,
            auto_start=False
        )

        # Initialize Action Client used to call API
        self.__call_api_client = actionlib.SimpleActionClient(
            'api_manager_node',
            CallAPIAction
        )
        self.__call_api_client.wait_for_server()

    def __get_motors_info(self):
        pass

    def __on_move_received(self):
        pass

    def run(self):
        pass