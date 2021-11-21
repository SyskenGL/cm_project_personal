#!/usr/bin/env python3
import rospy
import actionlib
from threading import Lock
from cm_ros.wrapper import CMNode
from cm_ros.queued_simple_action_server import QueuedSimpleActionServer
from func_timeout import func_set_timeout, FunctionTimedOut
from cm_bridge.low_level import pack_motors_request, pack_status_request
from cm_msgs.msg import WriteOnSerialAction, WriteOnSerialGoal
from cm_msgs.msg import Response, Event, MotorsInfo, SetMotorsAction


class CMMotors(CMNode):

    def __init__(self, name="cm_motors"):
        CMNode.__init__(self, name)

        self.__pub_motors_info = rospy.Publisher(
            "~info", MotorsInfo, queue_size=5, latch=True
        )
        self.__sub_event = None

        self.__motors_server = QueuedSimpleActionServer(
            "~server",
            SetMotorsAction,
            execute_cb=self.__on_motors_server_called,
            auto_start=False,
        )

        self.__serial_client = actionlib.SimpleActionClient(
            "/cm_bridge/cm_low_level/server", WriteOnSerialAction
        )
        self.__serial_client.wait_for_server()

        self.__lock = Lock()
        self.__violation = None
        self.__motors_info = None
        self.__get_initial_motors_info()

    def __get_initial_motors_info(self):
        self.__serial_client.send_goal(WriteOnSerialGoal(message=pack_status_request()))
        self.__serial_client.wait_for_result()
        assigned_id = self.__serial_client.get_result()

        if self.__serial_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            fatal = "Unable to get initial motors info."
            rospy.logfatal(fatal)
            rospy.signal_shutdown(fatal)
        else:
            try:
                response = self.__wait_for_response(assigned_id)
                new_motors_info = response.robot_info.motors_info
                self.__violation = response.robot_info.ir_sensors_info.violation
                self.__update_motors_info(new_motors_info)
            except FunctionTimedOut:
                fatal = "Unable to get initial motors info. Wait timed out."
                rospy.logfatal(fatal)
                rospy.signal_shutdown(fatal)

    def __update_motors_info(self, new_motors_info):
        if self.__motors_info != new_motors_info:
            self.__motors_info = new_motors_info
            self.__pub_motors_info.publish(self.__motors_info)

    def __on_event_published(self, event):
        with self.__lock:
            self.__update_motors_info(event.robot_info.motors_info)
            self.__violation = event.ir_sensors_info.violation

    def __on_motors_server_called(self, request):
        succeed = True

        if not self.__is_request_ignorable(request):

            self.__serial_client.send_goal(
                WriteOnSerialGoal(
                    message=pack_motors_request(
                        [
                            request.m0,
                            request.m1,
                            request.m2,
                            request.m3,
                            request.m4,
                            request.m5,
                        ]
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
                    with self.__lock:
                        self.__update_motors_info(response.robot_info.motors_info)
                except FunctionTimedOut:
                    rospy.logwarn("No response received. Wait timed out.")
                    succeed = False

        else:
            with self.__lock:
                succeed = not self.__violation

        if succeed:
            self.__motors_server.set_succeeded()
        else:
            self.__motors_server.set_aborted()

    def __is_request_ignorable(self, request):
        with self.__lock:
            return (
                request.m0.angle == self.__motors_info.m0.angle
                and request.m1.angle == self.__motors_info.m1.angle
                and request.m2.angle == self.__motors_info.m2.angle
                and request.m3.angle == self.__motors_info.m3.angle
                and request.m4.angle == self.__motors_info.m4.angle
                and request.m5.angle == self.__motors_info.m5.angle
                or self.__violation
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
        self.__motors_server.start()
        rospy.spin()