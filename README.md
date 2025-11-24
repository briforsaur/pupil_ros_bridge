# Pupil ROS Bridge

The Pupil Core eye tracking headset from Pupil Labs is managed by the Pupil Capture
software, which allows access to real-time data over a ZeroMQ (ZMQ) Inter-Process
Communication (IPC) backbone. Messages on the IPC backbone are encoded by MessagePack.
In order to access this data from ROS, a 'bridging' code is needed to interact with
ZMQ, extract the MessagePack-encoded messages, and forward them to nodes in the ROS
network.

This ROS package contains Python code and ROS message definitions to enable the transfer
of data from the Pupil Capture IPC backbone to be published as topics in ROS Noetic.

## How to use

### Installation
Install ROS Noetic following the [instructions on the ROS wiki](https://wiki.ros.org/noetic/Installation). The following instructions assume you are using Ubuntu or Debian, and may
need to be adjusted if you are using Windows 10 or a different Linux distribution.

Source ROS on the command line.
```
$ source /opt/ros/noetic/setup.bash
```

Create a catkin workspace with a folder for source code (if you already set up a 
workspace, skip to the next step). For example:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make
```

Clone this package's source code into the `~/catkin_ws/src` folder. The resulting folder
structure should be:
```
catkin_ws
├── build
├── devel
├── src
    └── pupil_ros_bridge
        ├── msg
        ├── scripts
        └── src
```

Now build the package by calling `catkin_make` from the `catkin_ws` folder.

After the build completes, source the bash script in the `catkin_ws/devel` folder to
add the package to your path:
```
source ~/catkin_ws/devel/setup.bash
```

### Running the Bridge Node
Plug in the Pupil Core headset and start the Pupil Capture software. Under the "Network
API" plugin, identify the network interface port and IP address. By default, the URI for
local connections is `tcp://127.0.0.1:50020` (aka `tcp://localhost:50020`). If you are
running ROS and Pupil Capture on the same computer, you can use the "Connect locally"
URI. If you are running ROS on a separate computer, write down the "Connect remotely"
URI which depends on what IP address your computer is assigned on your local network.
Note: the IP address may not be static, it can change each time your computer connects
to the network.

Start the core ROS service in a terminal window (remember to source ROS first):
```
$ roscore
```

In a new terminal window, source ROS and then source the catkin workspace. The bridge
node has a command line interface for setting up the connection to the Pupil headset and
specifying the desired topics to be published. To see how to use the command line
interface, input the following in the terminal:
```
rosrun pupil_ros_bridge pupil_ipc_bridge.py -h
```

The default IP address is `127.0.0.1` (localhost) and the default port is `50020`. If
you are connecting locally and Pupil Capture is using the default network settings, you
can run the node by simply listing the desired topics (only `pupil` and `gaze` are
valid topics).

Run the node with the desired settings using `rosrun`. To test that the node is 
receiving and broadcasting data properly, run one of the example subscriber nodes
(`gaze_subscriber.py` or `pupil_subscriber.py`). The subscriber node should print some
sample data to the terminal. Note: data will only be available if Pupil Capture is able
to detect a wearer's pupils, therefore it must be worn by someone before the example
subscribers will print anything to the terminal.

### Examples
To connect to Pupil Capture using default network settings on the same computer,
subscribing to both the pupil and gaze topics, run:
```
rosrun pupil_ros_bridge pupil_ipc_bridge.py pupil gaze
```

To connect to Pupil Capture using the default network port on a different computer with 
a "Connect remotely" URI in Pupil Capture of `tcp://192.168.0.130:50020`, subscribing
only to the pupil topic, run:
```
rosrun pupil_ros_bridge pupil_ipc_bridge.py --ipc_ip 192.168.0.130 pupil
```

To connect to Pupil Capture using a non-default port (for example, port 8000), 
subscribing only to the gaze topic, run:
```
rosrun pupil_ros_bridge pupil_ipc_bridge.py --ipc_port 8000 gaze
```