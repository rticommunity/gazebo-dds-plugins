# Elevator Plugin

It allows us to manage an elevator. It contains the following subscriber:

* Subscriptions
  * **elevator** [`std_msgs::msg::Int32`] -- Subscribes to an integer value that
    indicates the floor the elevator should go next.

## How To Run the Elevator Plugin

To run the API plugin, you will need to use two terminals: one terminal to
launch Gazebo along with the plugin, and another terminal to run the application
that controls the elevator.

### First Terminal

To launch the plugin you will need to make sure Gazebo knows where it is
located. You can either specify the full path to the library in the world
definition or set `GAZEBO_PLUGIN_PATH` environment variable as follows:

```bash
export GAZEBO_PLUGIN_PATH=/path/to/gazebo-dds-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```

Or alternatively, add the full path as part of the plugin definition in the
`.world` file:

```xml
<plugin name="elevator_plugin" filename="/path/to/libDdsElevatorPlugin.so">
    <!-- ... -->
</plugin>
```

Once you have set the environment accordingly, execute Gazebo and pass the
demonstration world as a parameter:

```bash
gazebo ./resources/worlds/Elevator.world --verbose
```

### Second Terminal

The example includes a publisher application you can move the elevator to a
different floor:

You can run the publisher application as follows:

```bash
gazebo-dds-plugins/build/src/elevator/elevatorPublisher \
    -d <domain id> \
    -t <topic name> \
    -s "floor: <next floor>"
```

For more information on how to run the application, run `elevatorPublisher -h`.

## Using API Plugin with Custom Worlds

To run the Elevator Plugin with a custom world, add the following sdf
information within the `world` tag:

```xml
<plugin name="elevator_plugin" filename="elevator/libDdsElevatorPlugin.so">
    <lift_joint>elevator::lift</lift_joint>
    <door_joint>elevator::door</door_joint>
    <floor_height>3.075</floor_height>
    <door_wait_time>10</door_wait_time>
    <topic>elevator</topic>
    <topic_name>elevator</topic_name>
    <dds_domain_id>0</dds_domain_id>
</plugin>
```

In addition, you may specify a `dds_qos_profile_file` and `dds_qos_profile`
within the plugin tag to use a specific QoS profile instead of the default one.

```xml
<plugin name='elevator_plugin' filename='elevator/libDdsElevatorPlugin.so'>
    <!-- ... -->
    <dds_qos_profile_file>resources/xml/ExampleQosProfiles.xml</dds_qos_profile_file>
    <dds_qos_profile>ExampleLibrary::TransientLocalProfile</dds_qos_profile>
    <!-- ... -->
</plugin>
```
