#!/usr/bin/env python3
import cv2
import rospy
import pyfakewebcam
from collections import defaultdict
from sensor_msgs.msg import Image
from cm_ros.wrapper import CMNode
from cv_bridge import CvBridge, CvBridgeError


class CMCamera(CMNode):

    def __init__(self, name="cm_camera"):
        CMNode.__init__(self, name)

        self.__config = defaultdict(lambda: None)
        self.__configure()

        self.__cv_bridge = CvBridge()

        self.__cap = None
        self.__vcam = None
        self.__open_camera()

        self.__pub_image = rospy.Publisher("~image", Image, queue_size=5)

    def __configure(self):
        self.__config["width"] = rospy.get_param("~width")
        self.__config["height"] = rospy.get_param("~height")
        self.__config["desired_fps"] = rospy.get_param("~desired_fps")
        self.__config["publish_rate"] = rospy.get_param("~publish_rate")
        self.__config["vcam"] = rospy.get_param("~vcam")

        if self.__config["desired_fps"] < self.__config["publish_rate"]:
            self.__config["publish_rate"] = self.__config["desired_fps"]
            rospy.logwarn(
                "Parameter publish_rate forced to desired_fps. "
                "Parameter publish_rate should be less than desired_fps."
            )

    def __open_camera(self):
        self.__cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.__cap.isOpened():
            fatal = "Unable to open camera by index."
            rospy.logfatal(fatal)
            rospy.signal_shutdown(fatal)

        self.__cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        self.__cap.set(cv2.CAP_PROP_FPS, self.__config["desired_fps"])
        self.__cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.__config["width"])
        self.__cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.__config["height"])

        rospy.logdebug("Actual desired_fps: %s", self.__cap.get(cv2.CAP_PROP_FPS))
        rospy.logdebug("Actual width: %s", self.__cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        rospy.logdebug("Actual height: %s", self.__cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.__vcam = pyfakewebcam.FakeWebcam(
            self.__config["vcam"], 
            int(self.__cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self.__cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        )

    def __close_camera(self):
        if self.__cap.isOpened():
            self.__cap.release()

    def run(self):
        rate = rospy.Rate(self.__config["publish_rate"])
        while not rospy.is_shutdown():
            ret, frame = self.__cap.read()
            if not ret:
                rospy.logwarn("Can't receive any frame.")
            else:
                try:
                    self.__vcam.schedule_frame(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                    image = self.__cv_bridge.cv2_to_imgmsg(frame, encoding="passthrough")
                    image.header.stamp = rospy.Time.now()
                    self.__pub_image.publish(image)
                except (CvBridgeError, Exception) as err:
                    rospy.logwarn(err)
            rate.sleep()
        self.__close_camera()
