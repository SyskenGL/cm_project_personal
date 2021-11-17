#!/usr/bin/env python3
import rospy
import pyaudio
from collections import defaultdict
from std_msgs.msg import Header
from cm_ros.wrapper import CMNode
from cm_msgs.msg import Audio, AudioInfo


class CMMicrophone(CMNode):

    def __init__(self, name="cm_microphone"):
        CMNode.__init__(self, name)

        self.__config = defaultdict(lambda: None)
        self.__configure()

        self.__audio_stream = None
        self.__open_microphone()

        self.__pub_audio = rospy.Publisher("~audio", Audio, queue_size=5)
        self.__pub_audio_info = rospy.Publisher(
            "~info", AudioInfo, queue_size=5, latch=True
        )

        self.__pub_audio_info.publish(
            AudioInfo(
                chunk=pow(2, self.__config["chunk_exp"]),
                rate=self.__config["rate"],
                channels=self.__config["channels"],
                format=self.__config["format"],
            )
        )

    def __configure(self):
        self.__config["chunk_exp"] = rospy.get_param("~chunk_exp")
        self.__config["rate"] = rospy.get_param("~rate")
        self.__config["channels"] = rospy.get_param("~channels")
        self.__config["format"] = rospy.get_param("~format")

    def __open_microphone(self):
        self.__audio_stream = pyaudio.PyAudio().open(
            format=self.__config["format"],
            channels=self.__config["channels"],
            rate=self.__config["rate"],
            frames_per_buffer=pow(2, self.__config["chunk_exp"]),
            input=True,
        )

    def __close_microphone(self):
        if self.__audio_stream is not None and self.__audio_stream.is_active():
            self.__audio_stream.close()

    def run(self):
        while not rospy.is_shutdown():
            audio_msg = Audio(
                header=Header(stamp=rospy.Time.now()),
                data=self.__audio_stream.read(pow(2, self.__config["chunk_exp"])),
            )
            self.__pub_audio.publish(audio_msg)
        self.__close_microphone()
