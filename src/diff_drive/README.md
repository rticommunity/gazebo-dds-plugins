# Differential drive plugin
It allows us to manage a differential drive robot and obtain its information. 
It contains some publishers and one subscriber:
* Publications 
    * [nav_msgs/msg/Odometry] - Send the information of the odometry sensor
    * [sensor_msgs/msg/JointState] - Send the state of the joints of the robot
* Subscriptions
    * [geometry_msgs/msg/Twist] - Recieve the next velocity and rotation velocity

## How to run
You will need two terminals to run this plugin. 

**First terminal**

In case we have not added the path where libraries are located to the 
envionment variable GAZEBO_PLUGIN_PATH, we have to add it via the following 
command:

```
$ export GAZEBO_PLUGIN_PATH=$HOME/dds-gazebo-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```
Once the environment variable is set, we can execute Gazebo with its specific world.

```
$ gazebo dds-gazebo-plugins/resources/worlds/DifferentialDrive.world --verbose
```
**Second terminal**

The plugin contains an example publisher to send information to the robot. 
You need to run this publisher to move it.
```
$ dds-gazebo-plugins/build/src/diff_drive/diffdrivepublisher -d <domain id> -t <topic name> -s "(<linear velocity in axis x> <angular velocity in axis z>)"
```

You can check the help of the publisher with the flag `-h` for more information.

## Using plugin with custom worlds

You need to add to your custom world inside the differential drive model that 
you want to manage with the following sdf information:
```
<plugin name='differential_drive_controller'
              filename='diff_drive/libDdsDiffDrivePlugin.so'>
    <update_rate>2</update_rate>
    <left_joint>left_wheel_hinge</left_joint>
    <right_joint>right_wheel_hinge</right_joint>
    <dds_domain_id>0</dds_domain_id>
    <wheel_separation>0.39</wheel_separation>
    <wheel_diameter>0.15</wheel_diameter>
    <wheel_acceleration>0</wheel_acceleration>
    <wheel_torque>5</wheel_torque>
    <topic_name_twist>cmd_vel</topic_name_twist>
    <odometry_source>encoder</odometry_source>
    <topic_name_odometry>odom</topic_name_odometry>
    <topic_name_joint>joint</topic_name_joint>
    <odometryFrame>odom</odometryFrame>
    <legacy_mode>true</legacy_mode>
</plugin>
```

In addition, you can add the tag `dds_qos_profile_file` and `dds_qos_profile` 
inside the plugin tag to use a specific QoS and not the default QoS
