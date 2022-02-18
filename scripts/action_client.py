import sys
from copy import copy
from typing import List

import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from klampt.model import trajectory
from klampt.io import ros


class ActionClient(object):
    def __init__(self, ns: str, joint_names: List[str]) -> None:
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._joint_names = joint_names
        self._goal_time_tolerance = rospy.Time(0.1)
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, positions, time):
        self._waypoints.append(positions)
        self._times.append(time)

    def start(self):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = ros.to_JointTrajectory(
            trajectory.Trajectory(milestones=self._waypoints, times=self._times),
            link_joint_names=self._joint_names,
        )
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.goal_time_tolerance = self._goal_time_tolerance
        self._client.send_goal(goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._waypoints = []
        self._times = []
