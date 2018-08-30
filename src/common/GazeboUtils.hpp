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
        T & tag_variable,
        const char * element_name,
        const T & element_default)
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
 * @param world world pointer that will be used to obtain the time.
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

}  // namespace utils
}  // namespace dds
}  // namespace gazebo

#endif  // GAZEBO_UTILS_HPP
