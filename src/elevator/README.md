# Elevator plugin
Its allows us control an elevator. It contains the following subscriber.
* Subscriptions
    * [std_msgs/msg/Int32] - Recieve the next floor of the elevator

## How to run
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