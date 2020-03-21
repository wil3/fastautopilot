
#Installation

1. Install Gazebo 8.1.1 following instructions
   [here](http://gazebosim.org/tutorials?tut=install_ubuntu)

2. Clone our fork of PX4 repo
https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html

```
wget
https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_nuttx.sh
```
run  ubuntu_sim.sh

3. Clone the GZMOCAP library
```
git clone https://github.com/wil3/gzmocap.git
```

4. Clone our fork or pygazebo
```
git clone https://github.com/wil3/pygazebo.git
```
 Make sure to switch to the correct branch 
```
git checkout f_msgs
```
Install sudo pip install .

4. Clone pymavlink
There is a bug in how pymavlink is installed with dronekit, it uses the wrong
version causes a "Exception in message handler for HEARTBEAT" message

Install from source or install separately so version is 2.2.4 > 
4. Clone this library

4. Need to properly configure PX4 to use mocap data,

* ATT_EXT_HDG_M 2
* CBRK_GPSFAIL = 240024

4. enable logging by adding the file to, 
PX4/build_posix_sitl_lpe/tmp/rootfs/fs/microsd/etc/logging/logger_topics.txt

# Run PX4 in SITL
```
make posix_sitl_lpe gazebo
```

To run in headless you can set the variable
```
HEADLESS=1 make posix_sitl_lpe gazebo
```

# Changes to PX4 firmware

Data collected from MAVLink stream rates must match 

src/modules/mavlink/mavlink_main.cpp


If we record many flights we can determine the distributions for the inputs 
so we can (*hopfully*) make a faster estimate of the best path


# GA notes

When mutating what distribution should be use? Should we stay close to the
original value? This may make flight smoother

# Logging
The PX4 logger can be customize to determine which topics are logged.
You must create a file
build_posix_sitl_lpe/tmp/rootfs/fs/microsd/etc/logging/logger_topics.txt and
specify the topics and rates 

see here for more details, https://dev.px4.io/en/log/logging.html


# Simulator

to run faster than real time, modify the real_time_update_rate in the world
file. Setting to 0 will make it go as fast as possible
http://gazebosim.org/tutorials?tut=modifying_world

default 0.002 step 500


After 128 iterations we got a segment fault. Try running without gzclient

It appears Gazebo is always publishing certain topics whether or not you
subscribe. Is there a way to not have it always publish


## Message passing using google protobufs
WARNING the compilied messages in pygazebo will not work with Gazebo 8.1.1, must
recompile.
```
protoc --proto_path=gazebo/gazebo/msgs
--python_out=/pygazebo/pygazebo/msg
msgs/*proto
```
