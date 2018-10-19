# IMU Scan Plugin

This plugin provides access to the produced by Inertial Measurement Units (IMU).
The IMU Scan Plugin publishes the following Topics:

* Publications
  * [sensor_msgs/msg/Imu] - Sends information on the state of the IMU sensor.

## How To Run the IMU Scan Plugin

To launch the plugin you will need to make sure Gazebo knows where it is
located. You can either set the `GAZEBO_PLUGIN_PATH` environment variable to
the plugin directory:

```bash
export GAZEBO_PLUGIN_PATH=/path/to/gazebo-dds-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```

Or alternatively, add the full path as part of the plugin definition in the
`.world` file:

```xml
<plugin name="imu_plugin" filename="/path/to/imu_scan/libDdsImuPlugin.so">
    <!-- ... -->
</plugin>
```

Once you have set the environment accordingly, execute Gazebo and pass the
demonstration world as a parameter:

```bash
gazebo ./resources/worlds/ImuSensor.world --verbose
```

You can subscribe to the information published by the plugin using `rtiddsspy`.

## Using IMU Scan Plugin With Custom Worlds

To run the IMU Scan Plugin with a custom world, add the following sdf
information within the `world` tag:

```xml
<plugin name="imu_plugin" filename="imu_scan/libDdsImuPlugin.so">
    <dds_domain_id>0</dds_domain_id>
    <topic_name>imu_scan</topic_name>
    <gaussian_noise>0.2</gaussian_noise>
    <rpy_offset>0 0 0</rpy_offset>
</plugin>
```

In addition, you may specify a `dds_qos_profile_file` and `dds_qos_profile`
within the plugin tag to use a specific QoS profile instead of the default one.

```xml
<plugin name='imu_plugin' filename='imu_scan/libDdsImuPlugin.so'>
    <!-- ... -->
    <dds_qos_profile_file>resources/xml/ExampleQosProfiles.xml</dds_qos_profile_file>
    <dds_qos_profile>ExampleLibrary::TransientLocalProfile</dds_qos_profile>
</plugin>
```
