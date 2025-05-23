# armada_ros2

This repository contains the descriptions and (soon) moveit configurations for the robot embodiments found at the NERVE Center in the robot ARMada. The armada_bringup package contains the launch files used to bring up the (soon) real and Gazebo simulatied robots. MuJoCo, PyBullet and Isaac Sim versions are planned but currently only Gazebo simulations have been implemented and tested.

## Requirements

This project requires the following dependencies to be cloned into the same workspace:

- [panda_ros2](https://github.com/flynnbm/panda_ros2)
- [tmr_ros2](https://github.com/flynnbm/tmr_ros2)

It is a good idea to keep all of these reposiories in the same workspace since they are all robot driver/description/bringup related.

## ROS 2 Version

This project was built and tested using **ROS 2 Jazzy**.

## Basic Usage

To run the example simulation using Gazebo and MoveIt:

```bash
ros2 launch armada_bringup gazebo_move_group.launch.py
```

You can see all available launch arguments by adding the `--show-args` flag:

```bash
ros2 launch armada_bringup gazebo_move_group.launch.py --show-args
```

### Available Options

- **No arguments**  
  Defaults to using the **Panda robot** with a simple block pedestal.

- **Techman robot**  
  Launch the Techman TM5-700 robot using:

  ```bash
  ros2 launch armada_bringup gazebo_move_group.launch.py robot_make:=tmr robot_model:=tm5-700
  ```

This will launch the TM5-700 on the same block pedestal.

## Robot Descriptions

Robot description files (URDF/Xacro) are located in:

```
armada_description/<robot_name>/xacro/
```

Each robot has its own subdirectory containing its Xacro description files. You can customize or reference these files for simulation, visualization, or further development. Other simulators should make used of the URDF/xacro format similar to Gazebo but I am currently unfamiliar with exactly what modifications may need to be made to these files in order to make them fully usable.
