#!/usr/bin/env python3
import rospy
import actionlib
from cm_ros.wrapper import CMNode
from cm_ros.queued_action_server import QueuedActionServer
from func_timeout import func_set_timeout, FunctionTimedOut
from cm_bridge.low_level import pack_leds_request, pack_status_request
from cm_msgs.msg import WriteOnSerialAction, WriteOnSerialGoal
from cm_msgs.msg import Response, LEDsInfo, SetLEDsAction


class CMLEDs(CMNode):

    def __init__(self, name="cm_leds"):
        CMNode.__init__(self, name)

        self.__pub_leds_info = rospy.Publisher(
            "~info", LEDsInfo, queue_size=5, latch=True
        )

        self.__leds_server = QueuedActionServer(
            "~leds_server",
            SetLEDsAction,
            execute_cb=self.__on_leds_server_called,
            auto_start=False,
        )

        self.__serial_client = actionlib.SimpleActionClient(
            "/cm_bridge/cm_low_level/serial_server", WriteOnSerialAction
        )
        self.__serial_client.wait_for_server()

        self.__leds_info = None
        self.__get_initial_leds_info()

    def __get_initial_leds_info(self):
        self.__serial_client.send_goal(WriteOnSerialGoal(message=pack_status_request()))
        self.__serial_client.wait_for_result()
        assigned_id = self.__serial_client.get_result()

        if self.__serial_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            fatal = "Unable to get initial leds info."
            rospy.logfatal(fatal)
            rospy.signal_shutdown(fatal)
        else:
            try:
                response = self.__wait_for_response(assigned_id)
                new_leds_info = response.robot_info.leds_info
                self.__update_leds_info(new_leds_info)
            except FunctionTimedOut:
                fatal = "Unable to get initial leds info. Wait timed out."
                rospy.logfatal(fatal)
                rospy.signal_shutdown(fatal)

    def __update_leds_info(self, new_leds_info):
        if self.__leds_info != new_leds_info:
            self.__leds_info = new_leds_info
            self.__pub_leds_info.publish(self.__leds_info)

    def __on_leds_server_called(self, request):
        succeed = True

        if self.__is_request_ignorable(request):

            self.__serial_client.send_goal(
                WriteOnSerialGoal(
                    message=pack_leds_request(
                        request.flash, request.left_led, request.right_led
                    )
                )
            )
            self.__serial_client.wait_for_result()
            assigned_id = self.__serial_client.get_result()

            if self.__serial_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
                succeed = False
            else:
                try:
                    response = self.__wait_for_response(assigned_id)
                    new_leds_info = response.robot_info.leds_info
                    self.__update_leds_info(new_leds_info)
                except FunctionTimedOut:
                    rospy.logwarn("No response received. Wait timed out.")
                    succeed = False

        if succeed:
            self.__leds_server.set_succeeded()
        else:
            self.__leds_server.set_aborted()

    def __is_request_ignorable(self, request):
        return (
            request.flash == self.__leds_info.flash
            and request.left_led == self.__leds_info.left_led
            and request.right_led == self.__leds_info.right_led
        )

    @func_set_timeout(10)
    def __wait_for_response(self, assigned_id):
        while not rospy.is_shutdown():
            response = rospy.wait_for_message(
                "/cm_bridge/cm_low_level/response",
                Response,
            )
            if assigned_id == response.id:
                return response

    def run(self):
        self.__leds_server.start()
        rospy.spin()
