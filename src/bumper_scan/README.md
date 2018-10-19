# Bumper Scan Plugin

This plugin provides access to the information produced by a contact sensor.
The Bumper Scan plugin publishes the following Topics:

* Publications
  * **bumper_scan** [`gazebo_msgs::msg::ContactsState`] -- Publishes the state
    of the contact sensor.

## How To Run the Bumper Scan Plugin

To launch the plugin you will need to make sure Gazebo knows where it is
located. You can either set the `GAZEBO_PLUGIN_PATH` environment variable to
the plugin directory:

```bash
export GAZEBO_PLUGIN_PATH=/path/to/gazebo-dds-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```

Or alternatively, add the full path as part of the plugin definition in the
`.world` file:

```xml
<plugin name='box_bumper_controller'
        filename='/path/to/libDdsBumperScanPlugin.so'>
    <!-- ... -->
</plugin>
```

Once you have set the environment accordingly, execute Gazebo and pass the
demonstration world as a parameter:

```bash
gazebo ./resources/worlds/BumperSensor.world --verbose
```

You can subscribe to the information published by the plugin using `rtiddsspy`.

## Using Bumper Scan Plugin With Custom Worlds

To run the Bumper Scan Plugin with a custom world, add the following sdf
information within the `world` tag:

```xml
<plugin name="box_bumper_controller"
        filename="bumper_scan/libDdsBumperScanPlugin.so">
    <topic_name>bumper_topic</topic_name>
    <dds_domain_id>0</dds_domain_id>
</plugin>
```

In addition, you may specify a `dds_qos_profile_file` and `dds_qos_profile`
within the plugin tag to use a specific QoS profile instead of the default one.

```xml
<plugin name='box_bumper_controller'
        filename='bumper_scan/libDdsBumperScanPlugin.so'>
    <!-- ... -->
    <dds_qos_profile_file>resources/xml/ExampleQosProfiles.xml</dds_qos_profile_file>
    <dds_qos_profile>ExampleLibrary::TransientLocalProfile</dds_qos_profile>
    <!-- ... -->
</plugin>
```
