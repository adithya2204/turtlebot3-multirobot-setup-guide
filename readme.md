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
<br>

## SLAM Mapping ##

**Start SLAM mapping:**

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

Navigate the robot around your environment to build the map using keyboard teleoperation command.

**Saving the Map**
Save the generated map:

```bash
rosrun map_server map_saver -f ~/map
```
Replace ~/map with your desired save location and filename.
<br>

## Autonomous Navigation ##

**Launch autonomous navigation with your saved map:**

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```
Replace $HOME/map.yaml with your desired save location and filename.

<br>
<br>

# Multi-Robot Control (for 5 Robots) #

**Prerequisites:**

-All the above steps are executed and the robot is working

-SLAM mapping is done and is saved as both .pgm and .yaml as worldmap

**Preparation**

**Clone required repositories in your ~/catkin_ws/src/ directory**

```bash
cd ~/catkin_ws/src/
git clone https://github.com/adithya2204/multi_robot_formation.git
git clone https://github.com/adithya2204/turtlebot3_multirobot_navigation.git
cd ..
catkin_make
source ~/.bashrc
```


**Save the .pgm and .yaml map files to:**
`turtlebot3_multirobot_navigation/turtlebot3_multirobot_navigation/maps`

<br>

## Launching the System ##

**On Remote PC:**

Start ROS Master:

```bash
roscore
```

**SBC Bringup for Each Robot**

In separate terminals for each TurtleBot:

Example For tb3_1:

```bash
ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_1" set_lidar_frame_id:="tb3_1/base_scan"
```

Repeat for other robots, replacing tb3_1 with tb3_0, tb3_2, tb3_3, and tb3_4.

<br>

## Multi-Robot Navigation ##

**On the remote PC:**

```bash
roslaunch turtlebot3_multirobot_navigation multirobot.launch
```

In RViz:

Perform initial pose estimation for each robot.

Assign the first five 2D Nav Goals to tb3_0 to tb3_4 respectively for individual control.

<br>
<br>

# Formation Control #
<br>

## Triangle Formation (3 Bots: tb3_0, tb3_1, tb3_2): ##

Run Multi Robot launch file

```bash
roslaunch turtlebot3_multirobot_navigation multirobot.launch
```

Run the formation script (in a new terminal):

```bash
rosrun multi_robot_formation triangle_formation.py
```

In RViz:

Assign a 2D Nav Goal after the last robot (for the 6th goal) to control the formation's movement.[ publishes to `/triangle_center/goal` ]

<br>

## Square Formation (4 Bots: tb3_0, tb3_1, tb3_2, tb3_3) ##

Run Multi Robot launch file

```bash
roslaunch turtlebot3_multirobot_navigation multirobot.launch
```

Run the formation script:

```bash
rosrun multi_robot_formation square_formation.py
```

In RViz:

Assign a 2D Nav Goal after the triangle center goal (for the 7th goal) to control the square formation.[ publishes to `/square_center/goal` ]

<br>

## Pentagon Formation (4 Bots: tb3_0, tb3_1, tb3_2, tb3_3, tb3_4) ##

Run Multi Robot launch file

```bash
roslaunch turtlebot3_multirobot_navigation multirobot.launch
```

Run the formation script:

```bash
rosrun multi_robot_formation pentagon_formation.py
```

In RViz:

Assign a 2D Nav Goal after the square center goal (for the 8th goal) to control the pentagon formation.[ publishes to `/pentagon_center/goal` ]


## Positions of 2D pose estimates and 2D nav goals ##

![pointer](https://github.com/user-attachments/assets/4c6a3b4d-2864-4b9c-8230-fea2d45d795f)










