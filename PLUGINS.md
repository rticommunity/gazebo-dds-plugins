# Description of the plugins

## Initial notes
In order to use the plugins we need to export the folder where libraries are located. For that we need to do:

```
$ export GAZEBO_PLUGIN_PATH=$HOME/dds-gazebo-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```

## Plugins

### Bumper scan
Implements a gazebo contact sensor. Its information is:
* Publications 
    * [gazebo_msgs/msg/ContactsState] - Send the state of the contact sensor

#### How to run
We will only need one terminal to run this plugin. We have to run gazebo with the specific world.

```
$ gazebo resources/worlds/BumperSensor.world --verbose
```

### Camera
A few details about that sensor

**Example world:** resource/worlds/Camera.world

#### How to run
To run this plugin we will need 2 terminals.

### Differential drive
Let us control a differential drive robot. Its information is:
* Publications 
    * [nav_msgs/msg/Odometry] - Send the information of the odometry sensor
    * [sensor_msgs/msg/JointState] - Send the state of the joints of the robot
* Subscriptions
    * [geometry_msgs/msg/Twist] - Recieve the next velocity and rotation velocity

**Example world:** resource/worlds/DifferentialDrive.world

#### How to run
You will need two terminals to run this plugin. 

##### First terminal
You have to run gazebo with the specific world.

```
$ gazebo resources/worlds/DifferentialDrive.world --verbose
```
##### Second terminal
The plugin contains an example publisher to send information to the robot. You need to run this publisher to move it.
```
$HOME/dds-gazebo-plugins/build/src/diff_drive/diffdrivepublisher <domain id> <topic name> <linear velocity axis x> <angular velocity axis z>
```

### Elevator
A few details about that sensor

**Example world:** resource/worlds/Elevator.world

#### How to run
To run this plugin we will need 2 terminals.

### Imu scan
A few details about that sensor

**Example world:** resource/worlds/ImuSensor.world

#### How to run
To run this plugin we will need 2 terminals.

### Laser scan
A few details about that sensor

**Example world:** resource/worlds/LaserSensor.world

#### How to run
To run this plugin we will need 2 terminals.

### Skid steer drive
A few details about that sensor

**Example world:** resource/worlds/SkidSteerDrive.world

#### How to run
To run this plugin we will need 2 terminals.

### Stereo camera
A few details about that sensor

**Example world:** resource/worlds/StereoCamera.world

#### How to run
To run this plugin we will need 2 terminals.
