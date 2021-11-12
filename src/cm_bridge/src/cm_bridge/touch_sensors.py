#!/usr/bin/env python3
import rospy
import actionlib
from cm_ros.wrapper import CMNode
from func_timeout import func_set_timeout, FunctionTimedOut
from cm_bridge.low_level import pack_touch_sensors_request
from cm_msgs.msg import WriteOnSerialAction, WriteOnSerialGoal
from cm_msgs.msg import Response, Event, TouchSensorsInfo, SetTouchSensorsAction


class CMTouchSensors(CMNode):

    def __init__(self, name='cm_touch_sensors'):
        CMNode.__init__(self, name)

        self.__touch_sensors_info = None

        self.__pub_touch_sensors_info = rospy.Publisher(
            '~info',
            TouchSensorsInfo,
            queue_size=5,
            latch=True
        )
        self.__sub_event = None

        self.__touch_sensors_server = actionlib.SimpleActionServer(
            '~touch_sensors_server',
            SetTouchSensorsAction,
            execute_cb=self.__on_touch_sensors_server_called,
            auto_start=False
        )
        self.__serial_client = actionlib.SimpleActionClient(
            '/cm_bridge/cm_low_level/serial_server',
            WriteOnSerialAction
        )

    def __on_touch_sensors_server_called(self, request):
        succeed = True

        if not self.__is_request_ignorable(request):

            self.__serial_client.send_goal(WriteOnSerialGoal(
                message=pack_touch_sensors_request(request.enabled)
            ))
            self.__serial_client.wait_for_result()
            assigned_id = self.__serial_client.get_result()

            if self.__serial_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
                succeed = False
            else:
                try:
                    self.__wait_for_serial_response(assigned_id)
                except FunctionTimedOut:
                    rospy.logwarn('No response received. Wait timed out.')
                    succeed = False

        if succeed:
            self.__touch_sensors_server.set_succeeded()
        else:
            self.__touch_sensors_server.set_aborted()

    def __is_request_ignorable(self, request):
        if self.__touch_sensors_info is None:
            return False
        if request.enabled != self.__touch_sensors_info.enabled:
            return False
        return True

    @func_set_timeout(10)
    def __wait_for_serial_response(self, assigned_id):
        while not rospy.is_shutdown():
            response = rospy.wait_for_message(
                '/cm_bridge/cm_low_level/response',
                Response,
            )
            if assigned_id == response.id:
                self.__update_touch_sensors_info(
                    response.robot_info.touch_sensors_info
                )

    def __on_event_published(self, event):
        self.__update_touch_sensors_info(event.robot_info.touch_sensors_info)

    def __update_touch_sensors_info(self, new_touch_sensors_info):
        if self.__touch_sensors_info != new_touch_sensors_info:
            self.__touch_sensors_info = new_touch_sensors_info
            self.__pub_touch_sensors_info.publish(self.__touch_sensors_info)

    def run(self):
        self.__serial_client.wait_for_server()
        self.__wait_for_serial_response(0)
        self.__sub_event = rospy.Subscriber(
            '/cm_bridge/cm_low_level/event',
            Event,
            self.__on_event_published
        )
        self.__touch_sensors_server.start()
        rospy.spin()
