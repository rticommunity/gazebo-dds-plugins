# Elevator plugin
It allows us to manage an elevator. It contains the following subscriber:
* Subscriptions
    * [std_msgs/msg/Int32] - Recieve the next floor of the elevator

## How to run
You will need two terminals to run this plugin. 

**First terminal**

To run this plugin we have to add the Gazebo libraries folder to the environment variable GAZEBO_PLUGIN_PATH. 

In case we have not added the path where libraries are located to the environment variable GAZEBO_PLUGIN_PATH, we have to add it via the following commands:

```
$ export GAZEBO_PLUGIN_PATH=/usr/lib/<gazebo folder>/plugins:$GAZEBO_PLUGIN_PATH
$ export GAZEBO_PLUGIN_PATH=$HOME/dds-gazebo-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```
Once the environment variable is set, we can execute Gazebo with its specific world.

```
$ gazebo dds-gazebo-plugins/resources/worlds/Elevator.world --verbose
```
**Second terminal**

The plugin contains an example publisher to send information to the elevator. You need to run this publisher to change its current floor.
```
$ dds-gazebo-plugins/build/src/elevator/elevatorpublisher <domain id> <topic name> <next floor>
```