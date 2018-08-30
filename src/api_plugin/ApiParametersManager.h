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

#ifndef API_PARAMETERS_MANAGER_H
#define API_PARAMETERS_MANAGER_H

#include <iostream>
#include <map>
#include <string>
#include <unordered_map>

#include "common/ParametersManager.hpp"

namespace gazebo { namespace dds { namespace utils {

class ApiParametersManager : public ParametersManager {
public:
    /**
     * @brief Constructor
     */
    ApiParametersManager(int argc, char *argv[]);

    /**
     * @brief Destructor
     */
    ~ApiParametersManager();

    /**
     * @brief Validate the information of a basic sample. This sample contains 
     * element_name and frame_name(optional).
     */
    void validate_basic_sample(
            std::string service_name,
            std::string entity_name,
            std::string frame_name = "");

    /**
     * @brief Validate the request information of the set_light_properties
     * service
     */
    void validate_set_light_properties_sample();

};

}  // namespace utils
}  // namespace dds
}  // namespace gazebo

#endif  // API_PARAMETERS_MANAGER_H
