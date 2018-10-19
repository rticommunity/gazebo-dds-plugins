# Camera Plugin

This plugin provides access to the information of produced by a camera. The
Camera Plugin publishes the following Topics:

* Publications
  * [sensor_msgs/msg/CameraInfo] - Send the information of the camera.
  * [sensor_msgs/msg/Image] - Sends raw image captured by the camera.

## How To Run the Camera Plugin

To launch the plugin you will need to make sure Gazebo knows where it is
located. You can either set the `GAZEBO_PLUGIN_PATH` environment variable to
the plugin directory:

```bash
export GAZEBO_PLUGIN_PATH=/path/to/gazebo-dds-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```

Or alternatively, add the full path as part of the plugin definition in the
`.world` file:

```xml
<plugin name="camera_controller" filename="/path/to/camera/libDdsCameraPlugin.so">
    <!-- ... -->
</plugin>
```

Once you have set the environment accordingly, execute Gazebo and pass the
demonstration world as a parameter:

```bash
gazebo ./resources/worlds/Camera.world --verbose
```

You can subscribe to the information published by the plugin using `rtiddsspy`.

## Using Camera Plugin With Custom Worlds

To run the Bumper Scan Plugin with a custom world, add the following sdf
information within the `world` tag:

```xml
<plugin name="camera_controller" filename="camera/libDdsCameraPlugin.so">
    <update_rate>0.0</update_rate>
    <frame_name>camera_link</frame_name>
    <dds_domain_id>0</dds_domain_id>
    <topic_name_image>image_raw</topic_name_image>
    <topic_name_camera_info>camera_info</topic_name_camera_info>
    <hack_baseline>0.07</hack_baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
    <cx_prime>0.0</cx_prime>
    <cx>0.0</cx>
    <cy>0.0</cy>
    <focal_length>0</focal_length>
    <border_crop>true</border_crop>
</plugin>
```

In addition, you may specify a `dds_qos_profile_file` and `dds_qos_profile`
within the plugin tag to use a specific QoS profile instead of the default one.

```xml
<plugin name='camera_controller' filename='camera/libDdsCameraPlugin.so'>
    <!-- ... -->
    <dds_qos_profile_file>resources/xml/ExampleQosProfiles.xml</dds_qos_profile_file>
    <dds_qos_profile>ExampleLibrary::TransientLocalProfile</dds_qos_profile>
    <!-- ... -->
</plugin>
```
