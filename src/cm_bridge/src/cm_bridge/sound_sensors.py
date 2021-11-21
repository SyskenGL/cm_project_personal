#!/usr/bin/env python3
import rospy
import actionlib
from cm_ros.wrapper import CMNode
from cm_ros.queued_simple_action_server import QueuedSimpleActionServer
from func_timeout import func_set_timeout, FunctionTimedOut
from cm_bridge.low_level import pack_sound_sensors_request, pack_status_request
from cm_msgs.msg import WriteOnSerialAction, WriteOnSerialGoal
from cm_msgs.msg import Response, Event, SoundSensorsInfo, SetSoundSensorsAction


class CMSoundSensors(CMNode):

    def __init__(self, name="cm_sound_sensors"):
        CMNode.__init__(self, name)

        self.__pub_sound_sensors_info = rospy.Publisher(
            "~info", SoundSensorsInfo, queue_size=5, latch=True
        )
        self.__sub_event = None

        self.__sound_sensors_server = QueuedSimpleActionServer(
            "~server",
            SetSoundSensorsAction,
            execute_cb=self.__on_sound_sensors_server_called,
            auto_start=False,
        )

        self.__serial_client = actionlib.SimpleActionClient(
            "/cm_bridge/cm_low_level/server", WriteOnSerialAction
        )
        self.__serial_client.wait_for_server()

        self.__sound_sensors_info = None
        self.__get_initial_sound_sensors_info()

    def __get_initial_sound_sensors_info(self):
        self.__serial_client.send_goal(WriteOnSerialGoal(message=pack_status_request()))
        self.__serial_client.wait_for_result()
        assigned_id = self.__serial_client.get_result()

        if self.__serial_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            fatal = "Unable to get initial sound sensors info."
            rospy.logfatal(fatal)
            rospy.signal_shutdown(fatal)
        else:
            try:
                response = self.__wait_for_response(assigned_id)
                new_sound_sensors_info = response.robot_info.sound_sensors_info
                self.__update_sound_sensors_info(new_sound_sensors_info)
            except FunctionTimedOut:
                fatal = "Unable to get initial sound sensors info. Wait timed out."
                rospy.logfatal(fatal)
                rospy.signal_shutdown(fatal)

    def __update_sound_sensors_info(self, new_sound_sensors_info):
        if self.__sound_sensors_info != new_sound_sensors_info:
            self.__sound_sensors_info = new_sound_sensors_info
            self.__pub_sound_sensors_info.publish(self.__sound_sensors_info)

    def __on_event_published(self, event):
        self.__update_sound_sensors_info(event.robot_info.sound_sensors_info)

    def __on_sound_sensors_server_called(self, request):
        succeed = True

        if not self.__is_request_ignorable(request):

            self.__serial_client.send_goal(
                WriteOnSerialGoal(
                    message=pack_sound_sensors_request(
                        request.enabled, request.auto_follow
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
                    new_sound_sensors_info = response.robot_info.sound_sensors_info
                    self.__update_sound_sensors_info(new_sound_sensors_info)
                except FunctionTimedOut:
                    rospy.logwarn("No response received. Wait timed out.")
                    succeed = False

        if succeed:
            self.__sound_sensors_server.set_succeeded()
        else:
            self.__sound_sensors_server.set_aborted()

    def __is_request_ignorable(self, request):
        return (
            request.enabled == self.__sound_sensors_info.enabled
            and request.auto_follow == self.__sound_sensors_info.auto_follow
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
        self.__sub_event = rospy.Subscriber(
            "/cm_bridge/cm_low_level/event", Event, self.__on_event_published
        )
        self.__sound_sensors_server.start()
        rospy.spin()
