#!/usr/bin/env python3
import rospy
import actionlib
from cm_ros.wrapper import CMNode
from cm_ros.queued_action_server import QueuedActionServer
from func_timeout import func_set_timeout, FunctionTimedOut
from cm_bridge.low_level import pack_display_request, pack_status_request
from cm_msgs.msg import WriteOnSerialAction, WriteOnSerialGoal
from cm_msgs.msg import Response, DisplayInfo, SetDisplayAction


class CMDisplay(CMNode):

    def __init__(self, name="cm_display"):
        CMNode.__init__(self, name)

        self.__pub_display_info = rospy.Publisher(
            "~info", DisplayInfo, queue_size=5, latch=True
        )

        self.__display_server = QueuedActionServer(
            "~display_server",
            SetDisplayAction,
            execute_cb=self.__on_display_server_called,
            auto_start=False,
        )

        self.__serial_client = actionlib.SimpleActionClient(
            "/cm_bridge/cm_low_level/serial_server", WriteOnSerialAction
        )
        self.__serial_client.wait_for_server()

        self.__display_info = None
        self.__get_initial_display_info()

    def __get_initial_display_info(self):
        self.__serial_client.send_goal(WriteOnSerialGoal(message=pack_status_request()))
        self.__serial_client.wait_for_result()
        assigned_id = self.__serial_client.get_result()

        if self.__serial_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            fatal = "Unable to get initial display info."
            rospy.logfatal(fatal)
            rospy.signal_shutdown(fatal)
        else:
            try:
                response = self.__wait_for_response(assigned_id)
                new_display_info = response.robot_info.display_info
                self.__update_display_info(new_display_info)
            except FunctionTimedOut:
                fatal = "Unable to get initial display info. Wait timed out."
                rospy.logfatal(fatal)
                rospy.signal_shutdown(fatal)

    def __update_display_info(self, new_display_info):
        if self.__display_info != new_display_info:
            self.__display_info = new_display_info
            self.__pub_display_info.publish(self.__display_info)

    def __on_display_server_called(self, request):
        succeed = True

        if not self.__is_request_ignorable(request):

            self.__serial_client.send_goal(
                WriteOnSerialGoal(message=pack_display_request(request.face_code))
            )
            self.__serial_client.wait_for_result()
            assigned_id = self.__serial_client.get_result()

            if self.__serial_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
                succeed = False
            else:
                try:
                    response = self.__wait_for_response(assigned_id)
                    new_display_info = response.robot_info.display_info
                    self.__update_display_info(new_display_info)
                except FunctionTimedOut:
                    rospy.logwarn("No response received. Wait timed out.")
                    succeed = False

        if succeed:
            self.__display_server.set_succeeded()
        else:
            self.__display_server.set_aborted()

    def __is_request_ignorable(self, request):
        return request.face_code == self.__display_info.face_code

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
        self.__display_server.start()
        rospy.spin()
