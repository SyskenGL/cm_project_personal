#!/usr/bin/env python3
import re
import rospy
import serial
import actionlib
from collections import defaultdict
from cm_ros.wrapper import CMNode
from cm_msgs.msg import Event, Response
from cm_msgs.msg import WriteOnSerialAction, WriteOnSerialResult
from cm_msgs.msg import TouchSensorsInfo, DisplayInfo, IRSensorsInfo
from cm_msgs.msg import SoundSensorsInfo, MotorsInfo, LEDsInfo, RobotInfo


def pack_status_request():
    return '#R:[{id}|\n'


def pack_touch_sensors_request(enabled):
    return '#T:{enabled}'.format(enabled=enabled) + '[{id}|\n'


def pack_ir_sensors_request(barrier_enabled):
    return '#O:{barrier_enabled}'.format(barrier_enabled=barrier_enabled) + '[{id}|\n'


def unpack_response_id(msg):
    response_id = None
    regex_result = re.search(r'\[\d+\|', msg)
    if regex_result:
        response_id = int(regex_result.group()[1:-1])
    return response_id


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


def unpack_sound_sensors_info(msg):
    sound_sensors_info = None
    regex_result = re.search('S:(.+?)\|', msg)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(',')]
        sound_sensors_info = SoundSensorsInfo(
            enabled=bool(values[0]),
            auto_follow_enabled=bool(values[1]),
            front=bool(values[2]),
            rear=bool(values[3]),
            right=bool(values[4]),
            left=bool(values[5]),
            front_right=bool(values[6]),
            front_left=bool(values[7]),
            rear_right=bool(values[8]),
            rear_left=bool(values[9])
        )
    return sound_sensors_info


def unpack_ir_sensors_info(msg):
    ir_sensors_info = None
    regex_result = re.search('O:(.+?)\|', msg)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(',')]
        ir_sensors_info = IRSensorsInfo(
            barrier_enabled=bool(values[0]),
            front_distance=values[1],
            rear_distance=values[2],
            right_distance=values[3],
            left_distance=values[4],
            violation_detected=bool(values[5])
        )
    return ir_sensors_info


def unpack_touch_sensors_info(msg):
    touch_sensors_info = None
    regex_result = re.search('T:(.+?)\|', msg)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(',')]
        touch_sensors_info = TouchSensorsInfo(
            enabled=bool(values[0]),
            left=bool(values[1]),
            up=bool(values[2]),
            right=bool(values[3])
        )
    return touch_sensors_info


def unpack_display_info(msg):
    display_info = None
    msg = re.sub('\[(.+?)\|', '|', msg)
    regex_result = re.search('V:(.+?)\|', msg)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(',')]
        display_info = DisplayInfo(displayed_face_code=values[0])
    return display_info


def unpack_robot_info(msg):
    return RobotInfo(
        motors_info=unpack_motors_info(msg),
        leds_info=unpack_leds_info(msg),
        sound_sensors_info=unpack_sound_sensors_info(msg),
        ir_sensors_info=unpack_ir_sensors_info(msg),
        touch_sensors_info=unpack_touch_sensors_info(msg),
        display_info=unpack_display_info(msg)
    )


class CMLowLevel(CMNode):

    def __init__(self, name='cm_low_level'):
        CMNode.__init__(self, name)

        self.__config = defaultdict(lambda: None)
        self.__configure()

        self.__serial = None
        self.__open_serial()

        self.__pub_event = rospy.Publisher(
            '~event',
            Event,
            queue_size=5,
            latch=True
        )
        self.__pub_response = rospy.Publisher(
            '~response',
            Response,
            queue_size=5,
            latch=True
        )

        self.__serial_write_id = 0
        self.__serial_server = actionlib.SimpleActionServer(
            '~serial_server',
            WriteOnSerialAction,
            execute_cb=self.__on_serial_server_called,
            auto_start=False
        )

    def __configure(self):
        self.__config['serial_port'] = rospy.get_param('~serial_port')
        self.__config['baud_rate'] = rospy.get_param('~baud_rate')
        self.__config['read_timeout'] = rospy.get_param('~read_timeout')
        self.__config['write_timeout'] = rospy.get_param('~write_timeout')

    def __open_serial(self):
        self.__serial = serial.Serial(
            self.__config['serial_port'],
            self.__config['baud_rate'],
            timeout=self.__config['read_timeout'],
            write_timeout=self.__config['write_timeout']
        )

    def __close_serial(self):
        if (self.__serial is not None) and self.__serial.isOpen():
            self.__serial.close()

    def __listen_serial(self):
        while not rospy.is_shutdown():
            if msg := str(self.__serial.readline(), 'utf-8'):
                if msg[0] == '#':
                    robot_info = unpack_robot_info(msg)
                    response_id = unpack_response_id(msg)
                    if response_id is not None:
                        response = Response(id=response_id, robot_info=robot_info)
                        self.__pub_response.publish(response)
                    else:
                        event = Event(robot_info=robot_info)
                        self.__pub_event.publish(event)
                else:
                    rospy.logdebug('Debug info from serial: %s', msg)
        self.__close_serial()

    def __on_serial_server_called(self, request):
        succeed = True

        self.__serial_write_id = (self.__serial_write_id % 65535) + 1
        try:
            request = bytes(request.msg.format(id=self.__serial_write_id), 'utf-8')
            self.__serial.write(request)
        except serial.SerialTimeoutException as err:
            rospy.logwarn(err)
            succeed = False

        result = WriteOnSerialResult(assigned_id=self.__serial_write_id)
        if succeed:
            self.__serial_server.set_succeeded(result)
        else:
            self.__serial_server.set_aborted(result)

    def run(self):
        self.__serial_server.start()
        self.__listen_serial()
