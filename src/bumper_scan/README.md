# Bumper scan plugin
It allows us to obtain the information of a contact sensor. It contains the 
following publisher:
* Publications 
    * [gazebo_msgs/msg/ContactsState] - Send the state of the contact sensor

## How to run
You will only need one terminal to run this plugin. 

In case we have not added the path where libraries are located to the 
envionment variable GAZEBO_PLUGIN_PATH, we have to add it via the 
following command:

```
$ export GAZEBO_PLUGIN_PATH=$HOME/dds-gazebo-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```
Once the environment variable is set, we can execute Gazebo with its specific 
world.

```
$ gazebo dds-gazebo-plugins/resources/worlds/BumperSensor.world --verbose
```

## Using plugin with custom worlds

You need to add to your custom world inside the bumper sensor that you want to 
manage with the following sdf information:
```
<plugin name="box_bumper_controller" filename="bumper_scan/libDdsBumperScanPlugin.so">
    <topic_name>bumper_topic</topic_name>
    <dds_domain_id>0</dds_domain_id>
</plugin>
```

In addition, you can add the tag `dds_qos_profile_file` and `dds_qos_profile` 
inside the plugin tag to use a specific QoS and not the default QoS
