#!/usr/bin/env python3
import rospy


class CMNode:

    """
    A ROS Node wrapper
    """

    def __init__(self, name="cm_node"):

        """
        Parameters
        ------------------------------
        :param: name : str
            The name of the ROS node
        """

        rospy.init_node(name)
        rospy.on_shutdown(self.__on_ros_shutdown)
        rospy.loginfo("Node {name} initialized.".format(name=rospy.get_name()))

    def __on_ros_shutdown(self):

        """
        Callback function called whenever rospy.spin() stops
        """

        rospy.loginfo("Node {name} is stopping.".format(name=rospy.get_name()))
        self.on_ros_shutdown()
        rospy.loginfo("Node {name} stopped.".format(name=rospy.get_name()))

    def on_ros_shutdown(self):

        """
        User defined callback function called whenever rospy.spin() stops
        """

        pass

    def run(self):

        """
        Virtual method that corresponds to the code of the Node
        """

        raise NotImplementedError

    def start(self):

        """
        Starts the node by executing the run method
        """

        rospy.loginfo("Node {name} started.".format(name=rospy.get_name()))
        self.run()
