# Laser scan plugin
Implements a laser sensor. It contains the following publisher.
* Publications 
    * [sensor_msgs/msg/LaserScanMsg] - Send the information of the laser sensor

## How to run
You will only need one terminal to run this plugin. You have to run Gazebo with the specific world.

```
$ gazebo $HOME/dds-gazebo-plugins/resources/worlds/LaserSensor.world --verbose
```