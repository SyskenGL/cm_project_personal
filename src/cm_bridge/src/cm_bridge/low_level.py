#!/usr/bin/env python3
import re
import rospy
import serial
import actionlib
from std_msgs.msg import String
from collections import defaultdict
from cm_ros.wrapper import CMNode
from cm_msgs.msg import WriteOnSerialAction, WriteOnSerialResult
from cm_msgs.msg import TouchInfo, DisplayInfo, BarrierInfo, SoundInfo, MotorsInfo, LEDsInfo


def compose_status_request():
    return '#R:[{id}|\n'


def unpack_motors_info(msg):
    motors_info = None
    regex_result = re.search('M:(.+?)\|', msg)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(',')]
        motors_info = MotorsInfo(
            m0_angle=values[0],
            m1_angle=values[1],
            m2_angle=values[2],
            m3_angle=values[3],
            m4_angle=values[4],
            m5_angle=values[5]
        )
    return motors_info


def unpack_leds_info(msg):
    leds_info = None
    regex_result = re.search('L:(.+?)\|', msg)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(',')]
        leds_info = LEDsInfo(
            front_led_enabled=bool(values[0]),
            front_led_intensity=values[1],
            left_led_enabled=bool(values[2]),
            left_led_red=values[3],
            left_led_green=values[4],
            left_led_blue=values[5],
            left_led_blink=bool(values[6]),
            right_led_enabled=bool(values[7]),
            right_led_red=values[8],
            right_led_green=values[9],
            right_led_blue=values[10],
            right_led_blink=bool(values[11])
        )
    return leds_info


def unpack_sound_info(msg):
    sound_info = None
    regex_result = re.search('S:(.+?)\|', msg)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(',')]
        sound_info = SoundInfo(
            enabled=bool(values[0]),
            auto_follow=bool(values[1]),
            front=bool(values[2]),
            rear=bool(values[3]),
            right=bool(values[4]),
            left=bool(values[5]),
            front_right=bool(values[6]),
            front_left=bool(values[7]),
            rear_right=bool(values[8]),
            rear_left=bool(values[9])
        )
    return sound_info


def unpack_barrier_info(msg):
    barrier_info = None
    regex_result = re.search('O:(.+?)\|', msg)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(',')]
        barrier_info = BarrierInfo(
            enabled=bool(values[0]),
            front_distance=values[1],
            rear_distance=values[2],
            right_distance=values[3],
            left_distance=values[4],
            violation=bool(values[5])
        )
    return barrier_info


def unpack_touch_info(msg):
    touch_info = None
    regex_result = re.search('T:(.+?)\|', msg)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(',')]
        touch_info = TouchInfo(
            enabled=bool(values[0]),
            left_sensor=bool(values[1]),
            up_sensor=bool(values[2]),
            right_sensor=bool(values[3])
        )
    return touch_info


def unpack_display_info(msg):
    display_info = None
    msg = re.sub('\[(.+?)\|', '|', msg)
    regex_result = re.search('V:(.+?)\|', msg)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(',')]
        display_info = DisplayInfo(face_code=values[0])
    return display_info


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
                if msg[0] == '#':
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