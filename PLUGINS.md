# Description of the plugins

## Initial notes
In order to use the plugins you need to export the folder where libraries are located. For that you need to do:

```
$ export GAZEBO_PLUGIN_PATH=$HOME/dds-gazebo-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```

## Plugins

### Bumper scan
Implements a gazebo contact sensor. Its information is:
* Publications 
    * [gazebo_msgs/msg/ContactsState] - Send the state of the contact sensor

#### How to run
You will only need one terminal to run this plugin. You have to run gazebo with the specific world.

```
$ gazebo resources/worlds/BumperSensor.world --verbose
```
<hr/> 

### Camera
Implements a gazebo camera. Its information is:
* Publications 
    * [sensor_msgs/msg/CameraInfo] - Send the information of the camera 
    * [sensor_msgs/msg/Image] - Send raw image captured by the camera

#### How to run
You will only need one terminal to run this plugin. You have to run gazebo with the specific world.

```
$ gazebo resources/worlds/Camera.world --verbose
```

<hr/> 

### Differential drive
Let us control a differential drive robot. Its information is:
* Publications 
    * [nav_msgs/msg/Odometry] - Send the information of the odometry sensor
    * [sensor_msgs/msg/JointState] - Send the state of the joints of the robot
* Subscriptions
    * [geometry_msgs/msg/Twist] - Recieve the next velocity and rotation velocity

##### How to run
You will need two terminals to run this plugin. 

**First terminal**

You have to run gazebo with the specific world.

```
$ gazebo resources/worlds/DifferentialDrive.world --verbose
```
**Second terminal**

The plugin contains an example publisher to send information to the robot. You need to run this publisher to move it.
```
$HOME/dds-gazebo-plugins/build/src/diff_drive/diffdrivepublisher <domain id> <topic name> <linear velocity axis x> <angular velocity axis z>
```
<hr/> 

### Elevator
Let us control an elevator. Its information is:
* Subscriptions
    * [std_msgs/msg/Int32] - Recieve the next floor of the elevator

**Example world:** resource/worlds/Elevator.world

##### How to run
You will need two terminals to run this plugin. 

**First terminal**

You need to export the library folder of Gazebo before run gazebo with the specific world.

```
$ export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-7/plugins:$GAZEBO_PLUGIN_PATH
$ gazebo resources/worlds/Elevator.world --verbose
```
**Second terminal**

The plugin contains an example publisher to send information to the elevator. You need to run this publisher to change its current floor.
```
$HOME/dds-gazebo-plugins/build/src/elevator/elevatorpublisher <domain id> <topic name> <next floor>
```

### Imu scan
A few details about that sensor

**Example world:** resource/worlds/ImuSensor.world

#### How to run
To run this plugin you will need 2 terminals.

### Laser scan
A few details about that sensor

**Example world:** resource/worlds/LaserSensor.world

#### How to run
To run this plugin you will need 2 terminals.

### Skid steer drive
Let us control a skid  drive robot. Its information is:
* Publications 
    * [nav_msgs/msg/Odometry] - Send the information of the odometry sensor
    * [sensor_msgs/msg/JointState] - Send the state of the joints of the robot
* Subscriptions
    * [geometry_msgs/msg/Twist] - Recieve the next velocity and rotation velocity

##### How to run
You will need two terminals to run this plugin. 

**First terminal**

You have to run gazebo with the specific world.

```
$ gazebo resources/worlds/DifferentialDrive.world --verbose
```
**Second terminal**

The plugin contains an example publisher to send information to the robot. You need to run this publisher to move it. You will use the same publisher that you use with the differential drive
```
$HOME/dds-gazebo-plugins/build/src/diff_drive/diffdrivepublisher <domain id> <topic name> <linear velocity axis x> <angular velocity axis z>
```

### Stereo camera
Implements a gazebo stereo camera. Its information is:
* Publications 
    * [sensor_msgs/msg/CameraInfo] - Send the information of the camera 
    * [sensor_msgs/msg/Image] - Send raw image captured by the camera

#### How to run
You will only need one terminal to run this plugin. You have to run gazebo with the specific world.

```
$ gazebo resources/worlds/Camera.world --verbose
```

