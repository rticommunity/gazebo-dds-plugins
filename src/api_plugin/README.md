# Api plugin
It contains a bunch of services that allows us to manage the environment of the simulation. You can see the services that the plugin contains here:
*   delete_model
*   delete_light
*   get_light_properties
*   get_world_properties
*   get_joint_properties
*   get_link_properties
*   get_link_state
*   get_model_properties
*   get_model_state
*   set_light_properties
*   set_link_properties
*   reset_simulation
*   reset_world
*   pause_physics
*   unpause_physics

## How to run
You will need two terminals to run this plugin. 

**First terminal**

In case we have not added the path where libraries are located to the envionment variable GAZEBO_PLUGIN_PATH,
we have to add it via the following command:

```
$ export GAZEBO_PLUGIN_PATH=$HOME/dds-gazebo-plugins/build/src/:$GAZEBO_PLUGIN_PATH
```
Once the environment variable is set, we can execute Gazebo with its specific world.

```
$ gazebo dds-gazebo-plugins/resources/worlds/DifferentialDrive.world --verbose
```
**Second terminal**

The plugin contains an example publisher to send request to the services.
```
$ dds-gazebo-plugins/build/src/api_plugin/apipublisher -d <domain id> -t <topic name> -s "<variable1: <value1> <variable2>: <value2>)"
```

You can check the help of the publisher with the flag `-h` for more information

In addition, we can see a few example of how to use the publisher here:

In this first example, we call the service `set_light_properties` with the new information of the light named sun and using the domain id 226

```
./build/src/api_plugin/apipublisher -d 226 -s set_light_properties -i "light_name: sun diffuse: 100 200 100 100 attenuation_constant: 5  attenuation_linear: 0.5  attenuation_quadratic: 0.5"
```

In this second example, we call the service `set_link_properties` with the new information of the link named chassis and using the domain id 226

```
./build/src/api_plugin/apipublisher -d 226 -s set_link_properties -i "link_name:chassis com_position:0 0 0 gravity_mode:true  mass:4.5 ixx:0.5 ixy:0.6 ixz:0.7 iyy:0.8 iyz:0.2 izz:0.3"
```