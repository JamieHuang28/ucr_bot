# ucr_pkgs
UCR(Underwater Cleaning Robot) is a project of HOME(Human-machine Ocean Mechanic Engineering) team, College of Mechanical Engineering, Zhejiang University. UCR packages are developed on ROS. This package is a collection of packages used on UCR such as simulation package and runing-online package.

# Prerequisit
* ROS
* epos_hardware
* turtlebot

# Getting Started
## 1. Enable the robot
The robot could be enabled either by running online(option A) or by simulating(option B). First switch to branch "steer_bot_hardware_from_master"

```$git checkout steer_bot_hardware_from_master```


### Option A: Running Online
Launch the EPOS driver. EPOS is servo motor of UCR.

```$roslaunch ucr_epos_hardware epos.launch```

Launch the basic motion controller. This controller can drive the wheels of UCR given speed and steer angle.

```$roslaunch ucr_control control.launch```

### Option B: Simulation
Launch the world in Gazebo. It will open gazebo, load the world, and start simulating the UCR hardware.

```$roslaunch ucr_gazebo ucr_empty_world.launch```

Launch the basic motion controller.

```$roslaunch ucr_control control_gazebo.launch```

## 2. Teleop
When robot is enabled, connect a xbox360 joystick to PC and launch the teleoperator

```$roslaunch ucr_teleop xbox360_teleop_from_turtlebot.launch```

Note that "*from_turtlebot" means that the source code is in turtlebot.

# Package List
* ucr_epos_hardware

Hardware driver for EPOS. Only used when running online.

* ucr_control

Integrate speed controller and steer angle controller.

* ucr_description

Describe the physical structure.

* ucr_gazebo

Based on Gazebo, a simulation tool. It provides simulation scenarios(or called "world"), also implements simulation of robot sensors and actuators.

* ucr_rviz

Based on rviz, a visualization tool.

* ucr_teleop

Based on teleoperation package of turtlebot(a popular ROS project).

* ucrbot

The raw file exported from SolidWorks.
