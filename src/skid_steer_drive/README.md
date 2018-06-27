# Skid steer drive plugin
Its allows us control a skid drive robot. It contains some publishers and one subscriber:
* Publications 
    * [nav_msgs/msg/Odometry] - Send the information of the odometry sensor
    * [sensor_msgs/msg/JointState] - Send the state of the joints of the robot
* Subscriptions
    * [geometry_msgs/msg/Twist] - Recieve the next velocity and rotation velocity

## How to run
You will need two terminals to run this plugin. 

**First terminal**

You have to run Gazebo with the specific world.

```
$ gazebo $HOME/dds-gazebo-plugins/resources/worlds/DifferentialDrive.world --verbose
```
**Second terminal**

The plugin contains an example publisher to send information to the robot. You need to run this publisher to move it. You will use the same publisher that you use with the differential drive
```
$HOME/dds-gazebo-plugins/build/src/diff_drive/diffdrivepublisher <domain id> <topic name> <linear velocity axis x> <angular velocity axis z>
```