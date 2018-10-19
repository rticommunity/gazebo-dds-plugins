# Differential Drive Plugin

This plugin allows us to manage a differential drive robot and obtain its
information. The Differential Drive plugin publishes and subscribes to the
following Topics:

* Publications
  * **odom** [`nav_msgs::msg::Odometry`] -- Publishes information provided by
    the odometry sensor.
  * **joint** [`sensor_msgs::msg::JointState`] -- Publishes the state of the
    joints of the robot Subscriptions.
* Subscriptions
  * **cmd_vel** [`geometry_msgs::msg::Twist`] -- Subscribes to the next
    velocity and rotation velocity.

## How To Run the Differential Drive Plugin

To run the plugin, you will need to use two terminals: one terminal to launch
Gazebo along with plugin, and another terminal to run a Differential Drive
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
<plugin name='differential_drive_controller'
        filename='/path/to/diff_drive/libDdsDiffDrivePlugin.so'>
    <!-- ... -->
</plugin>
```

Once you have set the environment accordingly, execute Gazebo and pass the
demonstration world as a parameter:

```bash
gazebo ./resources/worlds/DifferentialDrive.world --verbose
```

### Second Terminal

The plugin contains an example publisher application to send requests to the
robot; that is, to send instructions in order to move it.

You can run the publisher application as follows:

```bash
gazebo-dds-plugins/build/src/diff_drive/diffDrivePublisher \
    -d <domain id> \
    -t <topic name> \
    -s "linear_velocity: <axis x> angular_velocity: <axis z>"
```

For more information on how to run the application, run `diffDrivePublisher -h`.

## Using Differential Drive Plugin with Custom Worlds

To run the Differential Drive Plugin with a custom world, add the following sdf
information within the `world` tag:

```xml
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

In addition, you may specify a `dds_qos_profile_file` and `dds_qos_profile`
within the plugin tag to use a specific QoS profile instead of the default one.

```xml
<plugin name='differential_drive_controller'
        filename='diff_drive/libDdsDiffDrivePlugin.so'>
    <!-- ... -->
    <dds_qos_profile_file>resources/xml/ExampleQosProfiles.xml</dds_qos_profile_file>
    <dds_qos_profile>ExampleLibrary::TransientLocalProfile</dds_qos_profile>
    <!-- ... -->
</plugin>
```
