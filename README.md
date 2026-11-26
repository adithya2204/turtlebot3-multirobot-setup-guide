# TurtleBot3 Multi-Robot Deployment Guide (ROS Noetic, Raspberry Pi)

A step-by-step guide to running TurtleBot3 Burger robots with ROS Noetic on a Raspberry Pi. It goes from a single robot (network setup, bringup, teleop, mapping and navigation) up to **five robots navigating on one shared map and moving in triangle, square and pentagon formations**. Everything here was used on real robots.

| Repository | Role |
|---|---|
| **turtlebot3-multirobot-setup-guide** (this repo) | Step-by-step setup: Raspberry Pi, networking, SLAM, multi-robot bringup |
| [turtlebot3_multirobot_navigation](https://github.com/adithya2204/turtlebot3_multirobot_navigation) | Per-robot localisation and navigation on a shared map |
| [multi_robot_formation](https://github.com/adithya2204/multi_robot_formation) | Triangle, square and pentagon formation controllers |

## Contents

1. [System details](#1-system-details)
2. [Network and environment setup](#2-network-and-environment-setup)
3. [Single robot: bringup, teleop, RViz](#3-single-robot-bringup-teleop-rviz)
4. [Mapping (SLAM)](#4-mapping-slam)
5. [Single-robot navigation](#5-single-robot-navigation)
6. [Multi-robot navigation (5 robots)](#6-multi-robot-navigation-5-robots)
7. [Formation control](#7-formation-control)
8. [Common problems](#8-common-problems)
9. [Useful commands](#9-useful-commands)

---

## 1. System details

| Item | Value |
|---|---|
| Robot | TurtleBot3 Burger |
| Single Board Computer (SBC) | Raspberry Pi |
| SBC operating system | Ubuntu 20.04 LTS Server |
| ROS | Noetic Ninjemys (on the SBC and the remote PC) |
| LiDAR | LDS-01 |

**Before you start:** flash the SBC and install ROS Noetic and the TurtleBot3 packages on the SBC and the PC, following the [ROBOTIS TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) (choose *Noetic*). See also the [ROS Noetic wiki](https://wiki.ros.org/noetic).

---

## 2. Network and environment setup

The remote PC runs the ROS master. Every robot and the PC must be on the same network, and every machine must point `ROS_MASTER_URI` at the PC.

| Variable | On the remote PC | On each robot (SBC) |
|---|---|---|
| `ROS_MASTER_URI` | `http://<PC IP>:11311` | `http://<PC IP>:11311` |
| `ROS_HOSTNAME` | `<PC IP>` | `<that SBC's IP>` |

Find a machine's IP with `hostname -I`.

### 2.1 Connect the Raspberry Pi to Wi-Fi

```bash
sudo nano /etc/netplan/50-cloud-init.yaml   # set the SSID and password
sudo netplan apply
```

### 2.2 SBC `~/.bashrc`

Add these lines at the end of `~/.bashrc` on each robot:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export LDS_MODEL=LDS-01
export TURTLEBOT3_MODEL=burger
export ROS_MASTER_URI=http://<PC IP>:11311
export ROS_HOSTNAME=<SBC IP>
```

### 2.3 Remote PC `~/.bashrc`

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export LDS_MODEL=LDS-01
export TURTLEBOT3_MODEL=burger
export ROS_MASTER_URI=http://<PC IP>:11311
export ROS_HOSTNAME=<PC IP>
```

Run `source ~/.bashrc` (or open a new terminal) after editing.

---

## 3. Single robot: bringup, teleop, RViz

| Where | Command |
|---|---|
| PC | `roscore` |
| PC → SBC | `ssh ubuntu@<SBC IP>` |
| SBC | `roslaunch turtlebot3_bringup turtlebot3_robot.launch` |
| PC | `roslaunch turtlebot3_bringup turtlebot3_remote.launch` |
| PC | `rosrun rviz rviz -d $(rospack find turtlebot3_description)/rviz/burger.rviz` |
| PC | `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch` |

---

## 4. Mapping (SLAM)

With the robot brought up as in section 3, start Simultaneous Localisation and Mapping (SLAM) on the PC:

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

Drive around with keyboard teleop until the map is complete, then save it:

```bash
rosrun map_server map_saver -f ~/map
```

This writes `~/map.pgm` and `~/map.yaml`. **For the multi-robot setup**, save the map as `worldmap` inside the navigation package instead (after cloning it in section 6.1):

```bash
rosrun map_server map_saver -f ~/catkin_ws/src/turtlebot3_multirobot_navigation/turtlebot3_multirobot_navigation/maps/worldmap
```

---

## 5. Single-robot navigation

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

In RViz, set the robot's pose with *2D Pose Estimate*, then send goals with *2D Nav Goal*.

---

## 6. Multi-robot navigation (5 robots)

**Prerequisites:** every robot works on its own (section 3), and a map of the space has been saved as `worldmap.pgm` / `worldmap.yaml` (section 4).

### 6.1 Build the packages (on the remote PC)

```bash
cd ~/catkin_ws/src
git clone https://github.com/adithya2204/multi_robot_formation.git
git clone https://github.com/adithya2204/turtlebot3_multirobot_navigation.git
cd ~/catkin_ws && catkin_make
source ~/.bashrc
```

Put the map in `turtlebot3_multirobot_navigation/turtlebot3_multirobot_navigation/maps/` if you have not already.

### 6.2 Launch

**PC:** start the master.

```bash
roscore
```

**Each robot:** bring it up in its own namespace (`tb3_0` to `tb3_4`). For example, for `tb3_1`:

```bash
ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_bringup turtlebot3_robot.launch \
    multi_robot_name:="tb3_1" set_lidar_frame_id:="tb3_1/base_scan"
```

Repeat on the other robots, replacing `tb3_1` with `tb3_0`, `tb3_2`, `tb3_3` and `tb3_4`.

**PC:** start localisation and navigation for all five robots, plus RViz.

```bash
roslaunch turtlebot3_multirobot_navigation multirobot.launch
```

### 6.3 In RViz

The toolbar has one tool per robot, plus three formation tools:

| Tool | Publishes to |
|---|---|
| *2D Pose Estimate* 1–5 | `/tb3_0/initialpose` … `/tb3_4/initialpose` |
| *2D Nav Goal* 1–5 | `/tb3_0/move_base_simple/goal` … `/tb3_4/move_base_simple/goal` |
| *2D Nav Goal* 6 | `/triangle_center_goal` |
| *2D Nav Goal* 7 | `/square_center_goal` |
| *2D Nav Goal* 8 | `/pentagon_center_goal` |

1. Set each robot's starting pose with *2D Pose Estimate* 1–5.
2. Send individual goals with *2D Nav Goal* 1–5.

![Positions of the 2D Pose Estimate and 2D Nav Goal tools in RViz](https://github.com/user-attachments/assets/4c6a3b4d-2864-4b9c-8230-fea2d45d795f)

---

## 7. Formation control

Keep `multirobot.launch` running (section 6.2) and start one formation node in a new terminal. Then click the formation's centre with its goal tool. Details of each formation are in the [multi_robot_formation README](https://github.com/adithya2204/multi_robot_formation).

| Formation | Robots | Node | RViz tool (topic) |
|---|---|---|---|
| Triangle | `tb3_0`–`tb3_2` | `rosrun multi_robot_formation triangle_formation.py` | *2D Nav Goal* 6 (`/triangle_center_goal`) |
| Square | `tb3_0`–`tb3_3` | `rosrun multi_robot_formation square_formation.py` | *2D Nav Goal* 7 (`/square_center_goal`) |
| Pentagon | `tb3_0`–`tb3_4` | `rosrun multi_robot_formation pentagon_formation.py` | *2D Nav Goal* 8 (`/pentagon_center_goal`) |

---

## 8. Common problems

| Symptom | Check |
|---|---|
| PC can't see robot topics, or robots can't reach the master | `ROS_MASTER_URI` and `ROS_HOSTNAME` on every machine (section 2); all machines on the same network |
| TF "extrapolation into the future/past" warnings | System clocks of the PC and the SBCs are out of sync; synchronise them (for example with `chrony` or `ntpdate`) |
| A robot does not move in multi-robot mode | It was brought up with the right `ROS_NAMESPACE` and `multi_robot_name`, and its pose was set in RViz |
| `rosrun` says a formation script is not executable | Run `chmod +x ~/catkin_ws/src/multi_robot_formation/scripts/*.py` (older clones only) |

---

## 9. Useful commands

| Command | Description |
|---|---|
| `catkin_make` | Build the catkin workspace |
| `hostname -I` | Print this machine's IP addresses |
| `ip a` / `ifconfig` | Show network interfaces and addresses |
| `iwgetid` | Show the current Wi-Fi SSID |
| `iw dev` | Show wireless device information |
| `rostopic list` | List active topics (useful to confirm each robot's namespace) |

---

**Author:** Adithya Pothula. Written during a summer internship in the Systems and Control Lab, Department of Electrical Engineering, IIT Madras (July 2025).
