# API Plugin

This plugin provides a number of remote procedures that allow DDS applications
to manage the simulation environment remotely.

The list of remote procedures includes:

* `delete_model` -- Deletes a model.
* `delete_light` -- Deletes a light.
* `get_light_properties` -- Provides the properties of a light.
* `get_world_properties` -- Provides the properties of a world.
* `get_joint_properties` -- Provides the properties of a joint.
* `get_link_properties` -- Provides the properties of a link.
* `get_link_state` -- Provides the current state of a link.
* `get_model_properties` -- Provides the properties of a model.
* `get_model_state` - Provides the current state of a model.
* `set_light_properties` - Updates the properties of a light.
* `set_link_properties` - Updates the properties of a link.
* `set_joint_properties` - Updates the properties of a joint.
* `set_model_state` - Updates the current state of a model.
* `set_link_state` - Update the current state of a link.
* `reset_simulation` - Resets the simulation to its initial state.
* `reset_world` - Resets the world to its initial state.
* `pause_physics` - Pauses the physics of the simulation.
* `unpause_physics` - Unpauses the physics of the simulation.

## How To Run the API Plugin

To run the API plugin, you will need to use two terminals: one terminal to
launch Gazebo along with the plugin, and another terminal to send service
requests.

### First Terminal

To launch the plugin you will need to make sure Gazebo knows where it is
located. You can either set the `GAZEBO_PLUGIN_PATH` environment variable to
the plugin directory:

```bash
export GAZEBO_PLUGIN_PATH=/path/to/gazebo-dds-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```

Or alternatively, add the full path as part of the plugin definition in the
`.world` file:

```xml
<plugin name='DdsApiPlugin' filename='/path/to/libDdsApiPlugin.so'>
    <!-- ... -->
</plugin>
```

Once you have set the environment accordingly, execute Gazebo and pass the
demonstration world as a parameter:

```bash
gazebo ./resources/worlds/Demonstration.world --verbose
```

### Second Terminal

The example includes a publisher application you can use to send request to the
services.

You can run the API publisher as follows:

```bash
gazebo-dds-plugins/build/src/api_plugin/apiPublisher \
    -d <domain id> \
    -t <topic name> \
    -s "<variable1: <value1> <variable2>: <value2>)"
```

For more information on how to run the application, run `apiPublisher -h`.

Below, we show a few examples on how to use the publisher application.

#### Example #1

In this first example, we call the `set_light_properties` service with the new
information of a light named `sun` using Domain ID 0:

```bash
./build/src/api_plugin/apiPublisher \
    -d 0 \
    -s set_light_properties \
    -i "light_name: sun diffuse: 100 200 100 100 attenuation_constant: 5  attenuation_linear: 0.5 attenuation_quadratic: 0.5"
```

#### Example 2

In this second example, we call `set_link_properties` service and provide new
information on the link named `chassis` using Domain ID 0:

```bash
./build/src/api_plugin/apiPublisher \
    -d 0 \
    -s set_link_properties \
    -i "link_name:chassis com_position:0 0 0 gravity_mode:true  mass:4.5 ixx:0.5 ixy:0.6 ixz:0.7 iyy:0.8 iyz:0.2 izz:0.3"
```

## Using API Plugin with Custom Worlds

To run the API Plugin with a custom world, add the following sdf
information within the `world` tag:

```xml
<plugin name='DdsApiPlugin' filename='api_plugin/libDdsApiPlugin.so'>
    <dds_domain_id>0</dds_domain_id>
</plugin>
```

In addition, you may specify a `dds_qos_profile_file` and `dds_qos_profile`
within the plugin tag to use a specific QoS profile instead of the default one.

```xml
<plugin name='DdsApiPlugin' filename='api_plugin/libDdsApiPlugin.so'>
    <!-- ... -->
    <dds_qos_profile_file>resources/xml/ExampleQosProfiles.xml</dds_qos_profile_file>
    <dds_qos_profile>ExampleLibrary::TransientLocalProfile</dds_qos_profile>
    <!-- ... -->
</plugin>
```
