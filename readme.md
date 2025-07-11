# TurtleBot3 Burger Setup with ROS Noetic on Raspberry Pi (Ubuntu 20.04 LTS)

This guide outlines the steps and configurations needed to set up and run the TurtleBot3 Burger using ROS Noetic on a Raspberry Pi Single Board Computer (SBC) with Ubuntu 20.04 LTS Server.

## System Details

- **SBC:** Raspberry Pi
- **OS:** Ubuntu 20.04 LTS (Server)
- **ROS Version:** Noetic Ninjemys
- **TurtleBot3 Model:** burger
- **LDS Model:** LDS-01

## Basic Linux \& ROS Commands

| Command | Description |
| :-- | :-- |
| `catkin_make` | Builds your ROS workspace from source |
| `ifconfig` | Displays network interface configuration |
| `iwgetid` | Shows the current Wi-Fi network SSID |
| `ip a` | Displays IP address and network interface details |
| `iw dev` | Displays wireless device information |
| `hostname -I` | Prints all assigned IP addresses of the host |

## SBC Wi-Fi Networking Setup

To connect the Raspberry Pi to a Wi-Fi network:

**Wi-Fi Details:**

- **SSID:** CONTROL LAB 203C
- **Password:** DCL@IITM

**Steps:**

1. Edit the Netplan configuration:

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

2. Apply the changes:

```bash
sudo netplan apply
```


## SBC `.bashrc` Setup

Edit the `~/.bashrc` file to automatically source ROS and set environment variables:

```bash
sudo nano ~/.bashrc
```

Add these lines at the end:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export LDS_MODEL=LDS-01
export ROS_MASTER_URI=http://192.168.10.101:11311
export ROS_HOSTNAME=192.168.10.108
export TURTLEBOT3_MODEL=burger
```


## TurtleBot3 Bringup (on SBC)

Start the TurtleBot3 robot:

```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```


## Remote PC Setup

**Connecting to Raspberry Pi:**

```bash
ssh ubuntu@192.168.10.108
```

**Start ROS Core (on PC if acting as master):**

```bash
roscore
```


## RViz Visualization (on Remote PC)

**Launch remote visualization:**

```bash
roslaunch turtlebot3_bringup turtlebot3_remote.launch
```

**Open RViz with TurtleBot3 config:**

```bash
rosrun rviz rviz -d $(rospack find turtlebot3_description)/rviz/burger.rviz
```


## Keyboard Teleoperation

Control the robot using keyboard (on PC):

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```


## Remote PC `.bashrc` Setup

Edit the `~/.bashrc` file to prepare the PC to communicate with the SBC:

```bash
sudo nano ~/.bashrc
```

Add these lines:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.10.101:11311
export ROS_HOSTNAME=192.168.10.101
export LDS_MODEL=LDS-01
export TURTLEBOT3_MODEL=burger
```


## Final Notes

- Ensure both SBC and PC are on the **same Wi-Fi network**.
- The **ROS master** is typically run on the PC; SBC is the robot host.
- **IP addresses** must be correctly set in `.bashrc` on both devices.

