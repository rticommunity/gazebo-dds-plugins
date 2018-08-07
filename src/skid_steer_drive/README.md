# Skid steer drive plugin
It allows us to manage a skid steer drive robot and obtain its information. It contains some publishers and one subscriber:
* Publications 
    * [nav_msgs/msg/Odometry] - Send the information of the odometry sensor
    * [sensor_msgs/msg/JointState] - Send the state of the joints of the robot
* Subscriptions
    * [geometry_msgs/msg/Twist] - Recieve the next velocity and rotation velocity

## How to run
You will need two terminals to run this plugin. 

**First terminal**

In case we have not added the path where libraries are located to the envionment variable GAZEBO_PLUGIN_PATH,
we have to add it via the following command:

```
$ export GAZEBO_PLUGIN_PATH=$HOME/dds-gazebo-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```
Once the environment variable is set, we can execute Gazebo with its specific world.

```
$ gazebo dds-gazebo-plugins/resources/worlds/SkidSteerDrive.world --verbose
```
**Second terminal**

The plugin contains an example publisher to send information to the robot. You need to run this publisher to move it. You will use the same publisher that you use with the differential drive
```
$ dds-gazebo-plugins/build/src/diff_drive/diffdrivepublisher -d <domain id> -t <topic name> -s "(<linear velocity in axis x> <angular velocity in axis z>)"
```

You can check the help of the publisher with the flag `-h` for more information.
