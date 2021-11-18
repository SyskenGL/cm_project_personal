#!/usr/bin/env python3
import rospy
import queue
import threading
import actionlib
from actionlib.server_goal_handle import ServerGoalHandle


class QueuedSimpleActionServer:

    def __init__(self, name, action, execute_cb, auto_start=False):
        self.__goals_queue = queue.Queue()
        self.__current_goal = ServerGoalHandle()
        self.__goal_lock = threading.Lock()

        self.__execute_cb = execute_cb
        self.__loop_thread = threading.Thread(target=self.__loop)
        self.__loop_thread.start()

        self.__action_server = actionlib.ActionServer(
            name, action, self.__internal_goal_cb, auto_start=auto_start
        )

    def __loop(self):
        loop_duration = rospy.Duration.from_sec(0.1)
        while not rospy.is_shutdown():
            try:
                new_goal = self.__goals_queue.get(
                    block=True, timeout=loop_duration.to_sec()
                )
                with self.__goal_lock:
                    self.__current_goal = new_goal
                    self.__current_goal.set_accepted()
                self.__execute_cb(new_goal.get_goal())
            except queue.Empty:
                pass

    def __internal_goal_cb(self, goal):
        self.__goals_queue.put(goal)

    def get_default_result(self):
        return self.__action_server.ActionResultType()

    def publish_feedback(self, feedback):
        with self.__goal_lock:
            self.__current_goal.publish_feedback(feedback)

    def set_succeeded(self, result=None, text=""):
        with self.__goal_lock:
            if not result:
                result = self.get_default_result()
            self.__current_goal.set_succeeded(result, text)

    def set_aborted(self, result=None, text=""):
        with self.__goal_lock:
            if not result:
                result = self.get_default_result()
            self.__current_goal.set_aborted(result, text)

    def start(self):
        self.__action_server.start()