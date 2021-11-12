#!/usr/bin/env python3
import rospy
import actionlib
from cm_ros.wrapper import CMNode
from func_timeout import func_set_timeout, FunctionTimedOut
from cm_bridge.low_level import pack_ir_sensors_request
from cm_msgs.msg import WriteOnSerialAction, WriteOnSerialGoal
from cm_msgs.msg import Response, Event, IRSensorsInfo, SetIRSensorsAction


class CMIRSensors(CMNode):

    def __init__(self, name='cm_ir_sensors'):
        CMNode.__init__(self, name)

        self.__ir_sensors_info = None
        self.__update_ir_sensors_info(rospy.wait_for_message(
                '/cm_bridge/cm_low_level/response',
                Response,
            ).robot_info.ir_sensors_info
        )

        self.__pub_ir_sensors_info = rospy.Publisher(
            '~info',
            IRSensorsInfo,
            queue_size=5,
            latch=True
        )
        self.__sub_event = None

        self.__ir_sensors_server = actionlib.SimpleActionServer(
            '~ir_sensors_server',
            SetIRSensorsAction,
            execute_cb=self.__on_ir_sensors_server_called,
            auto_start=False
        )
        self.__serial_client = actionlib.SimpleActionClient(
            '/cm_bridge/cm_low_level/serial_server',
            WriteOnSerialAction
        )

        self.__serial_client.wait_for_server()

    def __on_ir_sensors_server_called(self, request):
        succeed = True

        if not self.__is_request_ignorable(request):

            self.__serial_client.send_goal(WriteOnSerialGoal(
                message=pack_ir_sensors_request(request.barrier)
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
            self.__ir_sensors_server.set_succeeded()
        else:
            self.__ir_sensors_server.set_aborted()

    def __is_request_ignorable(self, request):
        if self.__ir_sensors_info is None:
            return False
        if request.barrier != self.__ir_sensors_info.barrier:
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
                self.__update_ir_sensors_info(
                    response.robot_info.ir_sensors_info
                )

    def __on_event_published(self, event):
        self.__update_ir_sensors_info(event.robot_info.ir_sensors_info)

    def __update_ir_sensors_info(self, new_ir_sensors_info):
        if self.__ir_sensors_info != new_ir_sensors_info:
            self.__ir_sensors_info = new_ir_sensors_info
            self.__pub_ir_sensors_info.publish(self.__ir_sensors_info)

    def run(self):
        self.__sub_event = rospy.Subscriber(
            '/cm_bridge/cm_low_level/event',
            Event,
            self.__on_event_published
        )
        self.__ir_sensors_server.start()
        rospy.spin()
