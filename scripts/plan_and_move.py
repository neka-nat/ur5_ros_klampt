#!/usr/bin/env python
from typing import List, Optional
import os
import time
import numpy as np
import rospy
from rospkg import RosPack
packages = RosPack()
from action_client import ActionClient

import klampt
from klampt.plan.cspace import MotionPlan
from klampt.plan.robotcspace import RobotCSpace
from klampt.model.collide import WorldCollider
from klampt.model import ik
from klampt import vis


class Planner:
    def __init__(self, world_file: str = os.path.join(packages.get_path("ur5_ros_klampt"), "model/world.xml")) -> None:
        self.world = klampt.WorldModel()
        res = self.world.readFile(world_file)
        self.robot = self.world.robot(0)
        if not res:
            raise RuntimeError("Unable to load world")
        self.collider = WorldCollider(self.world)
        self.sim = klampt.Simulator(self.world)
        self.cspace = RobotCSpace(self.robot, self.collider)
        self.cspace.eps = 1e-2
        self.cspace.setup()
        MotionPlan.setOptions(type="rrt", perturbationRadius=0.25, bidirectional=True, shortcut=True)

    def get_ee_link_pos(self, angles: List[float]) -> List[float]:
        self.robot.setConfig(angles)
        link = self.robot.link("ee_link")
        print(self.robot.link("base_link").getWorldPosition([0, 0, 0]))
        print(self.robot.link("wrist_1_link").getWorldPosition([0, 0, 0]))
        return link.getWorldPosition([0, 0, 0])

    def solve_ik(self, target_pos: List[float]) -> Optional[List[float]]:
        link = self.robot.link("ee_link")
        goal = ik.objective(link, local=[0.0, 0.0, 0.0], world=target_pos)
        if ik.solve(goal):
            return self.robot.getConfig()
        else:
            return None

    def plan(self, goal: List[float], start: Optional[List[float]] = None) -> Optional[List[List[float]]]:
        planner = MotionPlan(self.cspace)
        if start is None:
            start = self.sim.controller(0).getCommandedConfig()
        planner.setEndpoints(start, goal)
        planner.planMore(10)
        path = planner.getPath()
        planner.space.close()
        planner.close()
        return path

    def visualize(self, path: List[List]) -> None:
        vis.add("world", self.world)
        vis.animate(("world", self.robot.getName()), path)
        def callback():
            time.sleep(0.1)

        vis.loop(setup=vis.show, callback=callback)


if __name__ == "__main__":
    rospy.init_node("plan_and_move", anonymous=True)
    link_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    active_joint_slice = slice(2, 8)
    initial_pose = np.deg2rad([0.0, 0.0, 0.0, -60.0, 120.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time_cnt = 0
    action_client = ActionClient("/arm_controller/", link_names)
    # Move to initial pose
    action_client.add_point(initial_pose[active_joint_slice], 2.0 * time_cnt)
    action_client.start()
    action_client.wait()
    time_cnt += 1

    planner = Planner()
    cur_pos = planner.get_ee_link_pos(initial_pose)
    target_pos = cur_pos
    # Move to +x
    target_pos[0] += 0.3
    target = planner.solve_ik(target_pos)
    path1 = planner.plan(target, initial_pose)
    for p in path1:
        action_client.add_point(p[active_joint_slice], 2.0 * time_cnt)
        time_cnt += 1

    # Move to +z
    target_pos[2] += 0.2
    prev_target = target
    target = planner.solve_ik(target_pos)
    path2 = planner.plan(target, prev_target)
    for p in path2:
        action_client.add_point(p[active_joint_slice], 2.0 * time_cnt)
        time_cnt += 1
    action_client.start()
    planner.visualize(path1 + path2)
