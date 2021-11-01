#!/usr/bin/env python3
import rospy
import serial
import actionlib
from std_msgs.msg import String
from collections import defaultdict
from cm_ros.wrapper import CMNode
from cm_msgs.msg import CallAPIAction, CallAPIResult


class CMAPIManager(CMNode):

    def __init__(self, name='cm_api_manager'):
        CMNode.__init__(self, name)

        # Set initial configurations
        self.config = defaultdict(lambda: None)
        self.__init_config()

        # Serial Stream
        self.__serial = None
        self.__subscribe()

        # ROS Topic used to publish serial output
        self.__pub_serial = rospy.Publisher('~serial', String, queue_size=5)

        # Unique ID of call request
        self.__api_call_id = -1

        # Initialize Action Server used to call API
        self.__call_api_server = actionlib.SimpleActionServer(
            rospy.get_name(),
            CallAPIAction,
            execute_cb=self.__on_api_call_received,
            auto_start=False
        )

    def __init_config(self):
        # Serial port used to communicate with the robot
        self.config['serial_port'] = rospy.get_param('~serial_port')

        # Number of symbols transmitted in one second
        self.config['baud_rate'] = rospy.get_param('~baud_rate')

        # Maximum time that each call to read will wait.
        self.config['read_timeout'] = rospy.get_param('~read_timeout')

        # Maximum time that each call to write will wait.
        self.config['write_timeout'] = rospy.get_param('~write_timeout')

    def __subscribe(self):
        try:
            self.__serial = serial.Serial(
                self.config['serial_port'],
                self.config['baud_rate'],
                timeout=self.config['read_timeout'],
                write_timeout=self.config['write_timeout']
            )
        except serial.SerialException as err:
            rospy.logerr('Node {name} - {err}'.format(name=rospy.get_name(), err=err))
            raise err

    def __unsubscribe(self):
        if (self.__serial is not None) and self.__serial.isOpen():
            self.__serial.close()

    def __listen_serial(self):
        while not rospy.is_shutdown():
            out = self.__serial.readline()
            if out and not rospy.is_shutdown():
                self.__pub_serial.publish(out)

    def __get_api_call_id(self):
        self.__api_call_id = (self.__api_call_id + 1) % 65536
        return self.__api_call_id

    def __on_api_call_received(self, goal):
        succeed = True

        # Complete the request by adding the call api ID
        api_call_id = self.__get_api_call_id()
        goal.request += '[{id}|\n'.format(id=api_call_id)

        # Write on serial port
        try:
            self.__serial.write(bytes(goal.request, 'utf-8'))
        except serial.SerialTimeoutException as err:
            rospy.logwarn('Node {name} - {err}'.format(name=rospy.get_name(), err=err))
            succeed = False

        # Submit the result
        result = CallAPIResult(id=api_call_id)
        if succeed:
            self.__call_api_server.set_succeeded(result)
        else:
            self.__call_api_server.set_aborted(result)

    def run(self):
        self.__call_api_server.start()
        self.__listen_serial()
        self.__unsubscribe()