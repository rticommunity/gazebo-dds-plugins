# Skid steer drive plugin
It allows us to manage a skid steer drive robot and obtain its information. 
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
$ gazebo dds-gazebo-plugins/resources/worlds/SkidSteerDrive.world --verbose
```
**Second terminal**

The plugin contains an example publisher to send information to the robot. 
You need to run this publisher to move it. You will use the same publisher 
that you use with the differential drive
```
$ dds-gazebo-plugins/build/src/diff_drive/diffdrivepublisher -d <domain id> -t <topic name> -s "(<linear velocity in axis x> <angular velocity in axis z>)"
```

You can check the help of the publisher with the flag `-h` for more information.

## Using plugin with custom worlds

You need to add to your custom world inside the skid steer drive model that 
you want to manage with the following sdf information:
```
<plugin name="SkidSteerDrivePlugin" filename="skid_steer_drive/libDdsSkidSteerDrivePlugin.so">
    <dds_domain_id>0</dds_domain_id>
    <topic_name_twist>cmd_vel</topic_name_twist>
    <topic_name_odometry>odom</topic_name_odometry>
    <topic_name_joint>joint</topic_name_joint>
    <right_front_joint>right_front</right_front_joint>
    <right_rear_joint>right_rear</right_rear_joint>
    <left_front_joint>left_front</left_front_joint>
    <left_rear_joint>left_rear</left_rear_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.15</wheel_diameter>
    <wheel_torque>5.0</wheel_torque>
    <update_rate>2</update_rate>
    <covariance_x>0.0001</covariance_x>
    <covariance_y>0.0001</covariance_y>
    <covariance_yaw>0.01</covariance_yaw>
</plugin>
```

In addition, you can add the tag `dds_qos_profile_file` and `dds_qos_profile` 
inside the plugin tag to use a specific QoS and not the default QoS
