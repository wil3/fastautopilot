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
