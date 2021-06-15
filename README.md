# SofAR assignment - Human Robot Collaboration

> **Authors**: Marco Gabriele Fedozzi, Georgii Kurshakov, Laura Triglia

This package contains part of the materials necessary to run the assignment for the *Software Architectures for Robotics* course of the Robotics Engineering/(J)EMARO MSc at [University of Genoa](https://courses.unige.it/10635).

A list of the other necessary packages are presented at the end of this document.

# Compiling and Running

After installing all the packages in a ROS workspace (*the project has been developed and tested in Noetic*) run
```bash
.../ros_ws$ catkin_make
```
**TWICE**, since one of the other legacy packages seems to have a dependancy issue and will throw a series of warning the first time. Ignore them, recompile, and if warnings or error still persist there might be something off with your configuration, be sure all packages are correctly installed and the ROS workspace sourced.

## Baxter - Unity Simulation

First of all follow the steps presented at the **SofAR-Human-Robot-Collaboration** repository (link at the end) to set up the Unity Environment. Then pass to your ROS system for the following steps.
A few components need to be run, so as many separate shells should be open at the same time:

1. **Unity - ROS connection**
```bash
.../ros_ws$ roslaunch human_baxter_collaboration human_baxter_collaboration.launch
```
Wait for the green text to appear, then start the Unity simulation ('Play' button).
This will initiate the connection between the simulation and the ROS environment.

2. **Finite State Machine**
```bash
.../ros_ws$ roslaunch sofar_hbc_01 sofar_hbc_01.launch
```
Ignore the warnings, here as well ae simply due to legacy code. Once the systems starts running you can pass to the third (optional) phase.

3. **Collision Detection** (optional)
```bash
.../ros_ws$ rosrun sofar_hbc_01 collision_detection
```
This will start the node responsible of tracking collisions (better said, distances) between Baxter and the human, as well as between the two Baxter arms, in real time.
This node is not necessary, and has quite a few limitations:
- the real Baxter has a low level controller that makes collisions between arms not possible. Here the detection is mainly used by the simulation, and to avoid such forced movements in the real robot;
- no motion tracking is performed in the real scenario, hence the human tf's are only those simulated by the somewhat-limited human;
- _collision detection_ does not mean _collision avoidance_: dynamic obstacles are not avoided and the plan is not modified at runtime. When a possible collision (_distance lower then threshold_) is detected, the ongoing trajectory is stopped (by sending a message to the simulation) and the FSM is informed of type (*severity*) of the collision.  
a. **"LOW"** for human-robot collisions: being the human very mobile and relatively fast compared to the robot, the FSM simply stops and re-plans for the previous state  
b. **"HIGH"**, for robot-robot collisions: the robot is slower, but it could repeat the same collision-prone movements if a simple replanning was asked. That's why in this case the FSM goes into the *ERROR STATE*, bringing the arms to their initial joint states, waiting for a random (thus different for each arm) amount of time before trying to re-plan from scratch.

## Real world test

The implementation of the system in a real world test does follow almost entirely the same steps already presented, since the limited system we're gonna use does rely on the simulation for the entire sensing part (minus robot proprioception).
Unity simulation should be launched as in the previous case. Remember to correctly export the ROS master IP and port that will be present in the local network.
The steps are thus as follows:

1. **Unity - ROS connection**
```bash
.../ros_ws$ roslaunch human_baxter_collaboration human_baxter_collaboration.launch
```

2. **Robot Controller**
```bash
.../ros_ws$ roslaunch sofar_hbc_01 joint_trajectory_client.launch
```
Forward the trajectories to the actual robot.

3. **Finite State Machine**
```bash
.../ros_ws$ roslaunch sofar_hbc_01 lab_sofar_hbc_01.launch
```
Launches a node which forwards the gripper commands to the robot, together with all the other nodes described.

4. **Collision Detection** (optional)
```bash
.../ros_ws$ rosrun sofar_hbc_01 collision_detection
```
Might be counterproductive in the real scenario, use with care.

# Documentation

See Doxygen page at: [HumanRobotCollaboration Documentation](https://hypothe.github.io/sofar_hbc_01/)

# Other resources

| name          | link                                       | description                                  |
| ---- 				    	| ---- 									                             | -----				                                    |
| moveit_robots | https://github.com/hypothe/moveit_robots   | baxter config files for moveit (modified)    |
| baxter        | https://github.com/RethinkRobotics/baxter  | baxter description and interfaces (install all of the presented packages) |
| SofAR-Human-Robot-Collaboration | https://github.com/hypothe/SofAR-Human-Robot-Collaboration  | Unity connection (modified) |

