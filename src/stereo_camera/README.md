# Stereo camera plugin
It allows us to manage two synchronized cameras and obtain its information. 
It contains the following publishers:
* Publications 
    * [sensor_msgs/msg/CameraInfo] - Send the information of the camera 
    * [sensor_msgs/msg/Image] - Send raw image captured by the camera

## How to run
You will only need one terminal to run this plugin. 

In case we have not added the path where libraries are located to the 
envionment variable GAZEBO_PLUGIN_PATH, we have to add it via the following 
command:

```
$ export GAZEBO_PLUGIN_PATH=$HOME/dds-gazebo-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```
Once the environment variable is set, we can execute Gazebo with its specific world.

```
$ gazebo dds-gazebo-plugins/resources/worlds/Camera.world --verbose
```
## Using plugin with custom worlds

You need to add to your custom world inside the stereo camera sensor that 
you want to manage with the following sdf information:
```
 <plugin name="stereo_camera_controller" filename="stereo_camera/libDdsStereoCameraPlugin.so">
    <update_rate>0.0</update_rate>
    <dds_domain_id>0</dds_domain_id>
    <frame_name>camera_link</frame_name>
    <topic_name_image>image_raw</topic_name_image>
    <topic_name_camera_info>camera_info</topic_name_camera_info>
    <frameName>left_camera_optical_frame</frameName>
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

In addition, you can add the tag `dds_qos_profile_file` and `dds_qos_profile` 
inside the plugin tag to use a specific QoS and not the default QoS
