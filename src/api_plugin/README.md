# Api plugin
It contains a bunch of services that allows us to manage the environment of the simulation. The services are:
*   `delete_model` - Delete a model for its name
*   `delete_light` - Delete a light for its name
*   `get_light_properties` - Obtain the properties of a specified light
*   `get_world_properties` - Obtain the properties of the world
*   `get_joint_properties` - Obtain the properties of a specified joint
*   `get_link_properties` - Obtain the properties of a specified link
*   `get_link_state` - Obtain the current state of a specified link
*   `get_model_properties` - Obtain the properties of a specified model
*   `get_model_state` - Obtain the current state of a specified model
*   `set_light_properties` - Update the properties of a specified light
*   `set_link_properties` - Update the properties of a specified link
*   `set_joint_properties` - Update the properties of a specified joint
*   `set_model_state` - Update the current state of a specified model
*   `set_link_state` - Update the current state of a specified link
*   `reset_simulation` - Reset the simulation to its initial state
*   `reset_world` - Reset the world to its initial state
*   `pause_physics` - Pause the physics of the simulation
*   `unpause_physics` - Unpause the physics of the simulation

## How to run
You will need two terminals to run this plugin. 

**First terminal**

We will use a complex world to use that plugin. For that reason, you need to 
add a few path to the environment variable GAZEBO_PLUGIN_PATH. We have to add 
it via the following commands:

```
$ export GAZEBO_PLUGIN_PATH=/usr/lib/<gazebo folder>/plugins:$GAZEBO_PLUGIN_PATH
$ export GAZEBO_PLUGIN_PATH=$HOME/dds-gazebo-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```

Once the environment variable is set, we can execute Gazebo with its specific 
world.

```
$ gazebo dds-gazebo-plugins/resources/worlds/Demonstration.world --verbose
```

**Second terminal**

The plugin contains an example publisher to send request to the services.
```
$ dds-gazebo-plugins/build/src/api_plugin/apipublisher -d <domain id> -t 
<topic name> -s "<variable1: <value1> <variable2>: <value2>)"
```

You can check the help of the publisher with the flag `-h` for more information

In addition, we can see a few example of how to use the publisher here:

In this first example, we call the service `set_light_properties` with the new 
information of the light named sun and using the domain id 0

```
./build/src/api_plugin/apipublisher -d 0 -s set_light_properties -i 
"light_name: sun diffuse: 100 200 100 100 attenuation_constant: 5 
attenuation_linear: 0.5  attenuation_quadratic: 0.5"
```

In this second example, we call the service `set_link_properties` with the new 
information of the link named chassis and using the domain id 0

```
./build/src/api_plugin/apipublisher -d 0 -s set_link_properties -i 
"link_name:chassis com_position:0 0 0 gravity_mode:true  mass:4.5 ixx:0.5 
ixy:0.6 ixz:0.7 iyy:0.8 iyz:0.2 izz:0.3"
```

## Using plugin with custom worlds

You need to add to your custom world inside the tag `world` the following 
sdf information:
```
<plugin name='DdsApiPlugin' filename='api_plugin/libDdsApiPlugin.so'>
    <dds_domain_id>0</dds_domain_id>
</plugin>
```

In addition, you can add the tag `dds_qos_profile_file` and `dds_qos_profile` 
inside the plugin tag to use a specific QoS and not the default QoS
