# Camera plugin
Implements a camera. It contains the following publishers:
* Publications 
    * [sensor_msgs/msg/CameraInfo] - Send the information of the camera 
    * [sensor_msgs/msg/Image] - Send raw image captured by the camera

## How to run
You will only need one terminal to run this plugin. You have to run Gazebo with the specific world.

```
$ gazebo $HOME/dds-gazebo-plugins/resources/worlds/Camera.world --verbose
```