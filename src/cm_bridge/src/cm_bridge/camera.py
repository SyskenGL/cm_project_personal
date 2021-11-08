#!/usr/bin/env python3
import cv2
import rospy
import pyfakewebcam
from collections import defaultdict
from sensor_msgs.msg import Image
from cm_ros.wrapper import CMNode
from cv_bridge import CvBridge, CvBridgeError


class CMCamera(CMNode):

    def __init__(self, name='cm_camera'):
        CMNode.__init__(self, name)

        self.__config = defaultdict(lambda: None)
        self.__configure()

        self.__cv_bridge = CvBridge()

        self.__cap = None
        self.__vcam = None
        self.__open_camera()

        self.__pub_image = rospy.Publisher('~image', Image, queue_size=5)

    def __configure(self):
        self.__config['width'] = rospy.get_param('~width')
        self.__config['height'] = rospy.get_param('~height')
        self.__config['desired_fps'] = rospy.get_param('~desired_fps')
        self.__config['publish_rate'] = rospy.get_param('~publish_rate')
        self.__config['vcam'] = rospy.get_param('~vcam')
        if self.__config['desired_fps'] < self.__config['publish_rate']:
            self.__config['publish_rate'] = self.__config['desired_fps']
            rospy.logwarn('Node {name} - publish_rate forced to desired_fps'.format(name=rospy.get_name()))

    def __open_camera(self):
        self.__cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.__cap.isOpened():
            rospy.logerr('Node {name} - Unable to open camera by index.'.format(name=rospy.get_name()))
        self.__cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.__cap.set(cv2.CAP_PROP_FPS, self.__config['desired_fps'])
        self.__cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.__config['width'])
        self.__cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.__config['height'])
        self.__vcam = pyfakewebcam.FakeWebcam(
            self.__config['vcam'],
            self.__config['width'],
            self.__config['height']
        )

    def __close_camera(self):
        if self.__cap.isOpened():
            self.__cap.release()

    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.__cap.read()
            if not ret:
                rospy.logwarn('Node {name} - Can\'t receive any frame.'.format(name=rospy.get_name()))
                break
            try:
                self.__vcam.schedule_frame(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                image = self.__cv_bridge.cv2_to_imgmsg(frame, encoding='passthrough')
                image.header.stamp = rospy.Time.now()
                self.__pub_image.publish(image)
            except CvBridgeError as err:
                rospy.logwarn('Node {name} - {warn}'.format(name=rospy.get_name(), warn=err))
            except Exception as err:
                rospy.logwarn('Node {name} - {warn}'.format(name=rospy.get_name(), warn=err))
        self.__close_camera()
