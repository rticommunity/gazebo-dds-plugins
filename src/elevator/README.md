# Elevator plugin
It allows us to manage an elevator. It contains the following subscriber:
* Subscriptions
    * [std_msgs/msg/Int32] - Recieve the next floor of the elevator

## How to run
You will need two terminals to run this plugin. 

**First terminal**

To run this plugin we have to add the Gazebo libraries folder to the 
environment variable GAZEBO_PLUGIN_PATH. 

In case we have not added the path where libraries are located to the 
environment variable GAZEBO_PLUGIN_PATH, we have to add it via the 
following commands:

```
$ export GAZEBO_PLUGIN_PATH=/usr/lib/<gazebo folder>/plugins:$GAZEBO_PLUGIN_PATH
$ export GAZEBO_PLUGIN_PATH=$HOME/dds-gazebo-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```
Once the environment variable is set, we can execute Gazebo with its specific world.

```
$ gazebo dds-gazebo-plugins/resources/worlds/Elevator.world --verbose
```
**Second terminal**

The plugin contains an example publisher to send information to the elevator. 
You need to run this publisher to change its current floor. 
```
$ dds-gazebo-plugins/build/src/elevator/elevatorpublisher -d <domain id> -t <topic name> -s <next floor>
```

You can check the help of the publisher with the flag `-h` for more information.

## Using plugin with custom worlds

You need to add to your custom world inside the elevator model that you want 
to manage with the following sdf information:
```
<plugin filename="elevator/libDdsElevatorPlugin.so" name="elevator_plugin">
    <lift_joint>elevator::lift</lift_joint>
    <door_joint>elevator::door</door_joint>
    <floor_height>3.075</floor_height>
    <door_wait_time>10</door_wait_time>
    <topic>elevator</topic>
    <topic_name>elevator</topic_name>
    <dds_domain_id>0</dds_domain_id>
</plugin>
```

In addition, you can add the tag `dds_qos_profile_file` and `dds_qos_profile` 
inside the plugin tag to use a specific QoS and not the default QoS
