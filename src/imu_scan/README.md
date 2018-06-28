# Imu scan plugin
It allows us to obtain the information of an inertial measurement unit (imu) sensor. It contains the following publisher:
* Publications 
    * [sensor_msgs/msg/Imu] - Send the information of the imu sensor

## How to run
You will only need one terminal to run this plugin. 

In case we have not added the path where libraries are located to the envionment variable GAZEBO_PLUGIN_PATH,
we have to add it via the following command:

```
$ export GAZEBO_PLUGIN_PATH=$HOME/dds-gazebo-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```
Once the environment variable is set, we can execute Gazebo with its specific world.

```
$ gazebo $HOME/dds-gazebo-plugins/resources/worlds/ImuSensor.world --verbose
```