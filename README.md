# Phone IMU Publisher

## Overview

This ROS Package was created in order to transform IMU data collected from phone to a format that's compatible with the rest of data.

This package reads data from file and publishes it on `/phone/imu` topic.

## How to start

* Run `catkin_make` in your workspace so that package is built.
* To launch this node run:

```bash
source devel/setup.bash
roscore
```

And in new terminal window

```bash
source devel/setup.bash
roslaunch phone_publisher phone_publisher.launch
```

When node is launched, data is read from a file and is published to `/phone/imu` topic.
At the same time a bagfile is being recorded that contains data from this topic. Bagfile is saved in data folder in package directory.

## Timestamp

Problem with phone IMU is that we could not read the data directly through ROS. To do that we read the data from file, publish it to a topic and then record a bag. We need to somehow synchronize the timestamp with other IMUs. To do that user should provide starting timestamps in form of parameters `secs` and `nsecs` equal to the first timestamp in bag with other IMUs.

## Launch parameters

* **dataFilePath** - path to file that contains IMU data collected from the phone. *Default* - "{packageDirectory}/data/quaternion.txt"
* **secs** - Seconds of starting timestamp. *Default* - 0
* **nsecs** - Nanoseconds of starting timestamp. *Default* - 0
* **bagFile** - name of the bagfile that will be created. *Default* - "phone_imu"
