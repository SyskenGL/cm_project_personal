#!/usr/bin/env python3
import re
import rospy
import serial
import actionlib
from std_msgs.msg import String
from collections import defaultdict
from cm_ros.wrapper import CMNode
from cm_msgs.msg import WriteOnSerialAction, WriteOnSerialResult
from cm_msgs.msg import TouchInfo, EnableTouchGoal, EnableBarrierGoal, EnableSoundGoal


MSG_INFO = {
    'header': '#',
    'flags': {
        'status': 'R',
        'motors': 'M',
        'leds': 'L',
        'sound': 'S',
        'ir': 'O',
        'touch': 'T',
        'display': 'V'
    },
    'flags_sep': '|',
    'values_sep': ',',
    'footer': '[{id}|\n'
}
REQUEST_STRUCT = '{header}{flag}:{content}{footer}'


def compose_status_request():
    return REQUEST_STRUCT.format(
        header=MSG_INFO['header'],
        flag=MSG_INFO['flags']['status'],
        content='',
        footer=MSG_INFO['footer']
    )


def compose_touch_request(enable_touch):
    content = '{touch_enabled}'.format(touch_enabled=int(enable_touch.touch_enabled))
    return REQUEST_STRUCT.format(
        header=MSG_INFO['header'],
        flag=MSG_INFO['flags']['touch'],
        content=content,
        footer=MSG_INFO['footer']
    )


def compose_ir_request(enable_barrier):
    content = '{barrier_enabled}'.format(barrier_enabled=int(enable_barrier.barrier_enabled))
    return REQUEST_STRUCT.format(
        header=MSG_INFO['header'],
        flag=MSG_INFO['flags']['ir'],
        content=content,
        footer=MSG_INFO['footer']
    )


def compose_sound_request(enable_sound):
    content = '{sound_enabled}' + MSG_INFO['values_sep'] + '{auto_follow_enabled}'
    content = content.format(
        sound_enabled=int(enable_sound.sound_enabled),
        auto_follow_enabled=int(enable_sound.auto_follow_enabled)
    )
    return REQUEST_STRUCT.format(
        header=MSG_INFO['header'],
        flag=MSG_INFO['flags']['sound'],
        content=content,
        footer=MSG_INFO['footer']
    )


def unpack_touch_info(msg):
    touch_info = None
    regex = MSG_INFO['flags']['touch'] + '(.+?)\\' + MSG_INFO['flags_sep']
    regex_result = re.search(regex, msg)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(MSG_INFO['values_sep'])]
        touch_info = TouchInfo(
            touch_enabled=bool(values[0]),
            lt_sensor=bool(values[1]),
            up_sensor=bool(values[2]),
            rt_sensor=bool(values[3])
        )
    return touch_info


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
        self.__serial_server = actionlib.SimpleActionServer(
            '~serial_server',
            WriteOnSerialAction,
            execute_cb=self.__on_serial_write,
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
                if msg[0] == MSG_INFO['header']:
                    if re.search(r'\[\d+\|$', msg):
                        self.__pub_responses.publish(msg)
                    else:
                        self.__pub_events.publish(msg)
                else:
                    rospy.logdebug('Node {name} - serial: {msg}'.format(name=rospy.get_name(), msg=msg))
        self.__close_serial()

    def __on_serial_write(self, request):
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

    def __get_initial_status(self):
        try:
            self.__serial.write(
                bytes(compose_status_request().format(id=self.__write_on_serial_id), 'utf-8')
            )
        except serial.SerialTimeoutException as err:
            rospy.logerr('Node {name} - {err}'.format(name=rospy.get_name(), err=err))
            raise err

    def run(self):
        self.__get_initial_status()
        self.__serial_server.start()
        self.__listen_serial()