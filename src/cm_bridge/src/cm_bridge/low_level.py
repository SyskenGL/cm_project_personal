#!/usr/bin/env python3
import re
import rospy
import serial
from collections import defaultdict
from cm_ros.wrapper import CMNode
from cm_ros.queued_simple_action_server import QueuedSimpleActionServer
from cm_msgs.msg import LED, Flash, Motor
from cm_msgs.msg import Event, Response
from cm_msgs.msg import WriteOnSerialAction, WriteOnSerialResult
from cm_msgs.msg import TouchSensorsInfo, DisplayInfo, IRSensorsInfo
from cm_msgs.msg import SoundSensorsInfo, MotorsInfo, LEDsInfo, RobotInfo


def pack_status_request():
    return "#R:[{id}|\n"


def pack_motors_request(motors):
    return "#M:{m0_angle},{m0_speed},".format(
        m0_angle=motors[0].angle, m0_speed=motors[0].speed
    ) + "{m1_angle},{m1_speed},".format(
        m1_angle=motors[1].angle, m1_speed=motors[1].speed
    ) + "{m2_angle},{m2_speed},".format(
        m2_angle=motors[2].angle, m2_speed=motors[2].speed
    ) + "{m3_angle},{m3_speed},".format(
        m3_angle=motors[3].angle, m3_speed=motors[3].speed
    ) + "{m4_angle},{m4_speed},".format(
        m4_angle=motors[4].angle, m4_speed=motors[4].speed
    ) + "{m5_angle},{m5_speed}".format(
        m5_angle=motors[5].angle, m5_speed=motors[5].speed
    ) + "[{id}|\n"


def pack_leds_request(flash, left_led, right_led):
    return "#L:{on},{intensity},".format(
        on=int(flash.on), intensity=flash.intensity
    ) + "{on},{red},{green},{blue},{blink},".format(
        on=int(left_led.on),
        red=left_led.red, green=left_led.green, blue=left_led.blue,
        blink=int(left_led.blink)
    ) + "{on},{red},{green},{blue},{blink}".format(
        on=int(right_led.on),
        red=right_led.red, green=right_led.green, blue=right_led.blue,
        blink=int(right_led.blink)
    ) + "[{id}|\n"


def pack_sound_sensors_request(enabled, auto_follow):
    return "#S:{enabled},{auto_follow}".format(
        enabled=int(enabled), auto_follow=int(auto_follow)
    ) + "[{id}|\n"


def pack_ir_sensors_request(barrier):
    return "#O:{barrier}".format(barrier=int(barrier)) + "[{id}|\n"


def pack_touch_sensors_request(enabled):
    return "#T:{enabled}".format(enabled=int(enabled)) + "[{id}|\n"


def pack_display_request(face_code):
    return "#V:{face_code}".format(face_code=int(face_code)) + "[{id}|\n"


def unpack_response_id(message):
    response_id = None
    regex_result = re.search("\[\d\|", message)
    if regex_result:
        response_id = int(regex_result.group()[1:-1])
    return response_id


def unpack_motors_info(message):
    motors_info = None
    regex_result = re.search("M:(.*?)\|", message)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(",")]
        motors_info = MotorsInfo(
            m0=Motor(angle=values[0], speed=None),
            m1=Motor(angle=values[1], speed=None),
            m2=Motor(angle=values[2], speed=None),
            m3=Motor(angle=values[3], speed=None),
            m4=Motor(angle=values[4], speed=None),
            m5=Motor(angle=values[5], speed=None),
        )
    return motors_info


def unpack_leds_info(message):
    leds_info = None
    regex_result = re.search("L:(.*?)\|", message)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(",")]
        leds_info = LEDsInfo(
            flash=Flash(on=bool(values[0]), intensity=values[1]),
            left_led=LED(
                on=bool(values[2]),
                red=values[3],
                green=values[4],
                blue=values[5],
                blink=bool(values[6]),
            ),
            right_led=LED(
                on=bool(values[7]),
                red=values[8],
                green=values[9],
                blue=values[10],
                blink=bool(values[11]),
            ),
        )
    return leds_info


def unpack_sound_sensors_info(message):
    sound_sensors_info = None
    regex_result = re.search("S:(.*?)\|", message)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(",")]
        sound_sensors_info = SoundSensorsInfo(
            enabled=bool(values[0]),
            auto_follow=bool(values[1]),
            front=bool(values[2]),
            rear=bool(values[3]),
            right=bool(values[4]),
            left=bool(values[5]),
            front_right=bool(values[6]),
            front_left=bool(values[7]),
            rear_right=bool(values[8]),
            rear_left=bool(values[9]),
        )
    return sound_sensors_info


def unpack_ir_sensors_info(message):
    ir_sensors_info = None
    regex_result = re.search("O:(.*?)\|", message)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(",")]
        ir_sensors_info = IRSensorsInfo(
            barrier=bool(values[0]),
            front_distance=values[1],
            rear_distance=values[2],
            right_distance=values[3],
            left_distance=values[4],
            violation=bool(values[5]),
        )
    return ir_sensors_info


def unpack_touch_sensors_info(message):
    touch_sensors_info = None
    regex_result = re.search("T:(.*?)\|", message)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(",")]
        touch_sensors_info = TouchSensorsInfo(
            enabled=bool(values[0]),
            left=bool(values[1]),
            up=bool(values[2]),
            right=bool(values[3]),
        )
    return touch_sensors_info


def unpack_display_info(message):
    display_info = None
    message = re.sub("\[(.*?)\|", "|", message)
    regex_result = re.search("V:(.*?)\|", message)
    if regex_result:
        values = [int(value) for value in regex_result.group(1).split(",")]
        display_info = DisplayInfo(face_code=values[0])
    return display_info


def unpack_robot_info(message):
    return RobotInfo(
        motors_info=unpack_motors_info(message),
        leds_info=unpack_leds_info(message),
        sound_sensors_info=unpack_sound_sensors_info(message),
        ir_sensors_info=unpack_ir_sensors_info(message),
        touch_sensors_info=unpack_touch_sensors_info(message),
        display_info=unpack_display_info(message),
    )


class CMLowLevel(CMNode):

    def __init__(self, name="cm_low_level"):
        CMNode.__init__(self, name)

        self.__config = defaultdict(lambda: None)
        self.__configure()

        self.__serial = None
        self.__open_serial()

        self.__pub_event = rospy.Publisher("~event", Event, queue_size=5, latch=True)
        self.__pub_response = rospy.Publisher(
            "~response", Response, queue_size=5, latch=True
        )

        self.__serial_write_id = 0
        self.__serial_server = QueuedSimpleActionServer(
            "~server",
            WriteOnSerialAction,
            self.__on_serial_server_called,
            auto_start=False,
        )

    def __configure(self):
        self.__config["serial_port"] = rospy.get_param("~serial_port")
        self.__config["baud_rate"] = rospy.get_param("~baud_rate")
        self.__config["read_timeout"] = rospy.get_param("~read_timeout")
        self.__config["write_timeout"] = rospy.get_param("~write_timeout")

    def __open_serial(self):
        self.__serial = serial.Serial(
            self.__config["serial_port"],
            self.__config["baud_rate"],
            timeout=self.__config["read_timeout"],
            write_timeout=self.__config["write_timeout"],
        )
        if not self.__serial.isOpen():
            fatal = "Unable to open serial port."
            rospy.logfatal(fatal)
            rospy.signal_shutdown(fatal)

    def __close_serial(self):
        if (self.__serial is not None) and self.__serial.isOpen():
            self.__serial.close()

    def __listen_serial(self):
        while not rospy.is_shutdown():
            if message := str(self.__serial.readline(), "utf-8"):
                if message[0] == "#":
                    if response_id := unpack_response_id(message):
                        self.__pub_response.publish(
                            Response(
                                id=response_id,
                                robot_info=unpack_robot_info(message)
                            )
                        )
                    else:
                        self.__pub_event.publish(
                            Event(robot_info=unpack_robot_info(message))
                        )
                else:
                    rospy.logdebug("Debug info from serial: %s", message)
        self.__close_serial()

    def __on_serial_server_called(self, request):
        succeed = True

        self.__serial_write_id = ((self.__serial_write_id + 1) % 65535)
        request = bytes(request.message.format(id=self.__serial_write_id), "utf-8")
        try:
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
