# Skid Steer Drive Plugin

It allows us to manage a skid steer drive robot and obtain its information.
The Skid Steer Drive plugin publishes and subscribes to the following Topics:

* Publications
  * **odom** [`nav_msgs::msg::Odometry`] -- Publishes information provided by
    the odometry sensor.
  * **joint** [`sensor_msgs::msg::JointState`] -- Publishes the state of the
    joints of the robot.
* Subscriptions
  * **cmd_vel** [`geometry_msgs::msg::Twist`] -- Subscribes to the next
    velocity and rotation velocity.

## How To Run the Skid Steer Drive Plugin

To run the plugin, you will need to use two terminals: one terminal to launch
Gazebo along with plugin, and another terminal to run a Skid Steer Drive
publisher.

### First Terminal

To launch the plugin you will need to make sure Gazebo knows where it is
located. You can either set the `GAZEBO_PLUGIN_PATH` environment variable to
the plugin directory:

```bash
export GAZEBO_PLUGIN_PATH=/path/to/gazebo-dds-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```

Or alternatively, add the full path as part of the plugin definition in the
`.world` file:

```xml
<plugin name="SkidSteerDrivePlugin"
        filename="/path/to/skid_steer_drive/libDdsSkidSteerDrivePlugin.so">
    <!-- ... -->
</plugin>
```

Once you have set the environment accordingly, execute Gazebo and pass the
demonstration world as a parameter:

```bash
gazebo ./resources/worlds/SkidSteerDrive.world --verbose
```

### Second Terminal

This plugin uses the publisher application included in the Differential Drive
plugin to to send requests to the robot; that is, to send instructions in order
to move it.

You can run the publisher application as follows:

```bash
gazebo-dds-plugins/build/src/diff_drive/diffdrivepublisher \
    -d <domain id> \
    -t <topic name> \
    -s "(<linear velocity in axis x> <angular velocity in axis z>)"
```

For more information on how to run the application, run `diffdrivepublisher -h`.

## Using Skid Steer Drive with Custom Worlds

To run the Skid Steer Drive Plugin with a custom world, add the following sdf
information within the `world` tag:

```xml
<plugin name="SkidSteerDrivePlugin"
        filename="skid_steer_drive/libDdsSkidSteerDrivePlugin.so">
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

In addition, you may specify a `dds_qos_profile_file` and `dds_qos_profile`
within the plugin tag to use a specific QoS profile instead of the default one.

```xml
<plugin name="SkidSteerDrivePlugin"
        filename="skid_steer_drive/libDdsSkidSteerDrivePlugin.so">
    <!-- ... -->
</plugin>
