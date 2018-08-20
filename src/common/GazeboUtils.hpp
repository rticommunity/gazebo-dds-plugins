/*
 * Copyright 2018 Real-Time Innovations, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GAZEBO_UTILS_HPP
#define GAZEBO_UTILS_HPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include "Properties.h"

namespace gazebo { namespace dds { namespace utils {

/**
 * @brief Obtain a specific parameter from the world
 *
 * @param sdf object that contains the information of the world
 * @param tag_variable variable that will contain the value of the parameter
 * @param element_name name of the parameter from the world
 * @param element_default default value for the parameter
 */
template <typename T>
void get_world_parameter(
        sdf::ElementPtr sdf,
        T &tag_variable,
        const char *element_name,
        const T &element_default)
{
    if (sdf->HasElement(element_name)) {
        tag_variable = sdf->Get<T>(element_name);
    } else {
        tag_variable = element_default;
        gzwarn << "Missing <" << element_name
               << ">, set to default: " << tag_variable << std::endl;
    }
}

/**
 * @brief Obtain the current time of the simulation
 *
 * @param world pointer that will be used to obtain the time
 * @return the current time of the simulation
 */
common::Time get_sim_time(physics::WorldPtr world)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return world->SimTime();
#else
    return world->GetSimTime();
#endif
}

/**
 * @brief Obtain the name of the world
 *
 * @param world world that will return its name 
 * @return the name of the world
 */
std::string
        get_world_name(physics::WorldPtr world)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return world->Name();
#else
    return world->GetName();
#endif
}

/**
 * @brief Obtain model by name
 *
 * @param world pointer that will be used to obtain the model
 * @param model_name name of the model
 * @return the specified model
 */
gazebo::physics::ModelPtr
        get_model(physics::WorldPtr world, std::string model_name)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return world->ModelByName(model_name);
#else
    return world->GetModel(model_name);
#endif
}

/**
 * @brief Obtain light by name
 *
 * @param world pointer that will be used to obtain the light
 * @param light_name name of the light
 * @return the specified light
 */
gazebo::physics::LightPtr
        get_light(physics::WorldPtr world, std::string light_name)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return world->LightByName(light_name);
#else
    return world->Light(light_name);
#endif
}

/**
 * @brief Obtain entity by name
 *
 * @param world pointer that will be used to obtain the entity
 * @param entity_name name of the entity
 * @return the specified entity
 */
gazebo::physics::EntityPtr
        get_entity(physics::WorldPtr world, std::string entity_name)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return world->EntityByName(entity_name);
#else
    return world->GetEntity(entity_name);
#endif
}

/**
 * @brief Obtain joint by name
 *
 * @param world pointer that will be used to obtain the joint
 * @param joint_name name of the joint
 * @return the specified joint
 */
gazebo::physics::JointPtr
        get_joint(physics::WorldPtr world, std::string joint_name)
{
    gazebo::physics::JointPtr joint;
#if GAZEBO_MAJOR_VERSION >= 8
    for (unsigned int i = 0; i < world->ModelCount() && !joint; i++)
        joint = world->ModelByIndex(i)->GetJoint(joint_name);
#else
    for (unsigned int i = 0; i < world->GetModelCount() && !joint; i++)
        joint = world->GetModel(i)->GetJoint(joint_name);
#endif

    return joint;
}

}  // namespace utils
}  // namespace dds
}  // namespace gazebo

#endif  // GAZEBO_UTILS_HPP
