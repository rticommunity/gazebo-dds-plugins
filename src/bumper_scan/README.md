# Bumper scan plugin
Implements a contact sensor. It contains the following publisher.
* Publications 
    * [gazebo_msgs/msg/ContactsState] - Send the state of the contact sensor

## How to run
You will only need one terminal to run this plugin. You have to run Gazebo with the specific world.

```
$ gazebo $HOME/dds-gazebo-plugins/resources/worlds/BumperSensor.world --verbose
```