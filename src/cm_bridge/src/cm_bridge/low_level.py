#!/usr/bin/env python3
import re
import rospy
import serial
import actionlib
from std_msgs.msg import String
from collections import defaultdict
from cm_ros.wrapper import CMNode
from cm_msgs.msg import WriteOnSerialAction, WriteOnSerialResult


class CMLowLevel(CMNode):

    def __init__(self, name='cm_low_level'):
        CMNode.__init__(self, name)

        self.__config = defaultdict(lambda: None)
        self.__configure()

        self.__serial = None
        self.__open_serial()

        self.__pub_events = rospy.Publisher('~events', String, queue_size=5, latch=True)
        self.__pub_responses = rospy.Publisher('~responses', String, queue_size=5, latch=True)

        self.__write_on_serial_id = 0
        self.__serial_as = actionlib.SimpleActionServer(
            '~serial_as',
            WriteOnSerialAction,
            execute_cb=self.__on_serial_write_requested,
            auto_start=False
        )

    def __configure(self):
        self.__config['serial_port'] = rospy.get_param('~serial_port')
        self.__config['baud_rate'] = rospy.get_param('~baud_rate')
        self.__config['read_timeout'] = rospy.get_param('~read_timeout')
        self.__config['write_timeout'] = rospy.get_param('~write_timeout')

    def __open_serial(self):
        try:
            self.__serial = serial.Serial(
                self.__config['serial_port'],
                self.__config['baud_rate'],
                timeout=self.__config['read_timeout'],
                write_timeout=self.__config['write_timeout']
            )
        except serial.SerialException as err:
            rospy.logerr('Node {name} - {msg}'.format(name=rospy.get_name(), msg=err))
            raise err

    def __close_serial(self):
        if (self.__serial is not None) and self.__serial.isOpen():
            self.__serial.close()

    def __listen_serial(self):
        while not rospy.is_shutdown():
            if msg := str(self.__serial.readline(), 'utf-8'):
                if msg[0] == '#':
                    if re.search(r'\[\d+\|$', msg):
                        self.__pub_responses.publish(msg)
                    else:
                        self.__pub_events.publish(msg)
                else:
                    rospy.logdebug('Node {name} - serial: {msg}'.format(name=rospy.get_name(), msg=msg))
        self.__close_serial()

    def __on_serial_write_requested(self, request):
        succeed = True

        self.__write_on_serial_id = (self.__write_on_serial_id % 65535) + 1
        try:
            self.__serial.write(bytes(request.msg.format(id=self.__write_on_serial_id), 'utf-8'))
        except serial.SerialTimeoutException as err:
            rospy.logwarn('Node {name} - {warn}'.format(name=rospy.get_name(), warn=err))
            succeed = False

        result = WriteOnSerialResult(id=self.__write_on_serial_id)
        if succeed:
            self.__call_api_server.set_succeeded(result)
        else:
            self.__call_api_server.set_aborted(result)

    def run(self):
        self.__serial_as.start()
        self.__listen_serial()
