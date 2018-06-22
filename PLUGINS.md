# Description of the plugins

## Initial notes
In order to use the plugins you need to export the folder where libraries are located. For that you need to do:

```
$ export GAZEBO_PLUGIN_PATH=$HOME/dds-gazebo-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```

## Plugins

### Bumper scan
Implements a contact sensor. It contains the following publisher.
* Publications 
    * [gazebo_msgs/msg/ContactsState] - Send the state of the contact sensor

#### How to run
You will only need one terminal to run this plugin. You have to run Gazebo with the specific world.

```
$ gazebo $HOME/dds-gazebo-plugins/resources/worlds/BumperSensor.world --verbose
```
<hr/> 

### Camera
Implements a camera. It contains the following publishers:
* Publications 
    * [sensor_msgs/msg/CameraInfo] - Send the information of the camera 
    * [sensor_msgs/msg/Image] - Send raw image captured by the camera

#### How to run
You will only need one terminal to run this plugin. You have to run Gazebo with the specific world.

```
$ gazebo $HOME/dds-gazebo-plugins/resources/worlds/Camera.world --verbose
```

<hr/> 

### Differential drive
Its allows us control a differential drive robot. It contains some publishers and one subscriber:
* Publications 
    * [nav_msgs/msg/Odometry] - Send the information of the odometry sensor
    * [sensor_msgs/msg/JointState] - Send the state of the joints of the robot
* Subscriptions
    * [geometry_msgs/msg/Twist] - Recieve the next velocity and rotation velocity

#### How to run
You will need two terminals to run this plugin. 

**First terminal**

You have to run Gazebo with the specific world.

```
$ gazebo $HOME/dds-gazebo-plugins/resources/worlds/DifferentialDrive.world --verbose
```
**Second terminal**

The plugin contains an example publisher to send information to the robot. You need to run this publisher to move it.
```
$HOME/dds-gazebo-plugins/build/src/diff_drive/diffdrivepublisher <domain id> <topic name> <linear velocity in axis x> <angular velocity in axis z>
```
<hr/> 

### Elevator
Its allows us control an elevator. It contains the following subscriber.
* Subscriptions
    * [std_msgs/msg/Int32] - Recieve the next floor of the elevator

#### How to run
You will need two terminals to run this plugin. 

**First terminal**

You need to export the library folder of Gazebo before run Gazebo with the specific world.

```
$ export GAZEBO_PLUGIN_PATH=/usr/lib/<Gazebo folder> /plugins:$GAZEBO_PLUGIN_PATH
$ gazebo $HOME/dds-gazebo-plugins/resources/worlds/Elevator.world --verbose
```
**Second terminal**

The plugin contains an example publisher to send information to the elevator. You need to run this publisher to change its current floor.
```
$HOME/dds-gazebo-plugins/build/src/elevator/elevatorpublisher <domain id> <topic name> <next floor>
```
<hr/> 

### Imu scan
Implements an imu ( inertial measurement unit ) sensor. It contains the following publisher.
* Publications 
    * [sensor_msgs/msg/Imu] - Send the information of the imu sensor

#### How to run
You will only need one terminal to run this plugin. You have to run Gazebo with the specific world.

```
$ gazebo $HOME/dds-gazebo-plugins/resources/worlds/ImuSensor.world --verbose
```

<hr/> 

### Laser scan
Implements a laser sensor. It contains the following publisher.
* Publications 
    * [sensor_msgs/msg/LaserScanMsg] - Send the information of the laser sensor

#### How to run
You will only need one terminal to run this plugin. You have to run Gazebo with the specific world.

```
$ gazebo $HOME/dds-gazebo-plugins/resources/worlds/LaserSensor.world --verbose
```

<hr/> 

### Skid steer drive
Its allows us control a skid drive robot. It contains some publishers and one subscriber:
* Publications 
    * [nav_msgs/msg/Odometry] - Send the information of the odometry sensor
    * [sensor_msgs/msg/JointState] - Send the state of the joints of the robot
* Subscriptions
    * [geometry_msgs/msg/Twist] - Recieve the next velocity and rotation velocity

#### How to run
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
<hr/> 

### Stereo camera
Implements a stereo camera. It contains the following publishers.
* Publications 
    * [sensor_msgs/msg/CameraInfo] - Send the information of the camera 
    * [sensor_msgs/msg/Image] - Send raw image captured by the camera

#### How to run
You will only need one terminal to run this plugin. You have to run Gazebo with the specific world.

```
$ gazebo $HOME/dds-gazebo-plugins/resources/worlds/Camera.world --verbose
```

