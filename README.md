# ur5_ros_klampt

## Generate URDF from xacro

```sh
xacro --inorder universal_robot/ur_description/urdf/ur5_robot.urdf.xacro use_simulation:=false -o ur5.urdf
```

## Dependencies

* [ur_gazebo](https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_gazebo)

## Run

First, launch ur5 gazebo simulation.

```sh
roslaunch ur_gazebo ur5.launch
```

Run klampt planner.

```sh
rosrun ur5_ros_klampt plan_and_move.py
```
