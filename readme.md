# TurtleBot3 Burger Setup with ROS Noetic on Raspberry Pi (Ubuntu 20.04 LTS)

This guide outlines the steps and configurations needed to set up and run the TurtleBot3 Burger using ROS Noetic on a Raspberry Pi Single Board Computer (SBC) with Ubuntu 20.04 LTS Server and how to setup multi robot formation

<br>

## System Details

- **SBC:** Raspberry Pi
- **OS:** Ubuntu 20.04 LTS (Server)
- **ROS Version:** Noetic Ninjemys
- **TurtleBot3 Model:** burger
- **LDS Model:** LDS-01

<br>

## Links for reference
-**for turtlebot:** https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

-**for ROS noetic:** https://wiki.ros.org/noetic

<br>

## Basic Linux \& ROS Commands

| Command | Description |
| :-- | :-- |
| `catkin_make` | Builds your ROS workspace from source |
| `ifconfig` | Displays network interface configuration |
| `iwgetid` | Shows the current Wi-Fi network SSID |
| `ip a` | Displays IP address and network interface details |
| `iw dev` | Displays wireless device information |
| `hostname -I` | Prints all assigned IP addresses of the host |

<br>

## TurtleBot3 Bringup (on SBC) ##


**To connect the Raspberry Pi to a Wi-Fi network:**

**Steps:**

1. Edit the Netplan configuration:

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

change the current SSID and password if connecting to a new network


2. Apply the changes:

```bash
sudo netplan apply
```


**SBC .bashrc Setup**

Edit the `~/.bashrc` file to source ROS and set environment variables automatically:

```bash
sudo nano ~/.bashrc
```

**Add these lines at the end:**

for <SBC IP>: `hostname -I` on SBC
for <host IP>: `hostname -I` on remote PC

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export LDS_MODEL=LDS-01
export ROS_MASTER_URI=http://<host IP>:11311
export ROS_HOSTNAME=<SBC IP>
export TURTLEBOT3_MODEL=burger
```

**Start the TurtleBot3 robot:**

```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
<br>

## Remote PC Setup ##

**Connecting to Raspberry Pi:**

```bash
ssh ubuntu@<SBC IP>
```

**Remote PC `.bashrc` Setup**

Edit the `~/.bashrc` file to prepare the PC to communicate with the SBC:

```bash
sudo nano ~/.bashrc
```

Add these lines:
for <host IP>: `hostname -I` on remote PC

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://<host IP>:11311
export ROS_HOSTNAME=<host IP>
export LDS_MODEL=LDS-01
export TURTLEBOT3_MODEL=burger
```

**Start ROS Core (on PC if acting as master):**

```bash
roscore
```

**RViz Visualization (on Remote PC)**

**Launch remote visualization:**

```bash
roslaunch turtlebot3_bringup turtlebot3_remote.launch
```

**Open RViz with TurtleBot3 config:**

```bash
rosrun rviz rviz -d $(rospack find turtlebot3_description)/rviz/burger.rviz
```

**Keyboard Teleoperation**

Control the robot using keyboard (on PC):

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

