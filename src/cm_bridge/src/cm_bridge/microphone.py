#!/usr/bin/env python3
import rospy
import pyaudio
from collections import defaultdict
from std_msgs.msg import Header
from cm_ros.wrapper import CMNode
from cm_msgs.msg import Audio, AudioInfo


class CMMicrophone(CMNode):

    def __init__(self, name='cm_microphone'):
        CMNode.__init__(self, name)

        self.__audio_stream = None

        self.__pub_audio = rospy.Publisher('~audio', Audio, queue_size=5)
        self.pub_audio_info = rospy.Publisher('~audio_info', AudioInfo, queue_size=10, latch=True)

        # Set initial configurations
        self.config = defaultdict(lambda: None)
        self.init_config()

    def init_config(self):
        # Mandatory configurations to be set
        self.config['chunk_exp'] = rospy.get_param('~chunk_exp')
        self.config['rate'] = rospy.get_param('~rate')
        self.config['channels'] = rospy.get_param('~channels')
        self.config['format'] = rospy.get_param('~format')

        # Apply initial configurations
        self.subscribe()
        rospy.loginfo('{name} Initial configurations: {config}.'.format(
            name=rospy.get_name(),
            config=self.config
        ))

        # Publish Audio Info
        audio_info_msg = AudioInfo(
            chunk=pow(2, self.config['chunk_exp']),
            rate=self.config['rate'],
            channels=self.config['channels'],
            format=self.config['format']
        )
        self.pub_audio_info.publish(audio_info_msg)

    def subscribe(self):
        # Create an Audio Stream
        self.audio_stream = pyaudio.PyAudio().open(
            format=self.config['format'],
            channels=self.config['channels'],
            rate=self.config['rate'],
            frames_per_buffer=pow(2, self.config['chunk_exp']),
            input=True
        )

    def unsubscribe(self):
        if self.audio_stream is not None and self.audio_stream.is_active():
            self.audio_stream.close()

    def run(self):
        while not rospy.is_shutdown():
            # Create the Audio msg and public the Audio msg
            audio_msg = Audio(
                header=Header(stamp=rospy.Time.now()),
                data=self.audio_stream.read(pow(2, self.config['chunk_exp']))
            )
            self.pub_audio.publish(audio_msg)
        self.unsubscribe()