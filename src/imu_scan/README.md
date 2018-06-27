# Imu scan plugin
Implements an imu ( inertial measurement unit ) sensor. It contains the following publisher.
* Publications 
    * [sensor_msgs/msg/Imu] - Send the information of the imu sensor

## How to run
You will only need one terminal to run this plugin. You have to run Gazebo with the specific world.

```
$ gazebo $HOME/dds-gazebo-plugins/resources/worlds/ImuSensor.world --verbose
```