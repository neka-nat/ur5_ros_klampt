#!/usr/bin/env python
from typing import List, Optional
import os
import time
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
        MotionPlan.setOptions(type="rrt", perturbationRadius=0.25, bidirectional=True)

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
    action_client = ActionClient("/arm_controller/", link_names)
    planner = Planner()
    wps = [planner.solve_ik([0.5, 0.0, 0.3]), planner.solve_ik([0.5, 0.0, 0.7])]
    path = planner.plan(wps[0])
    print(path)
    for i, p in enumerate(path):
        action_client.add_point(p[1:7], i * 3.0)
    action_client.start()
    planner.visualize(path)
