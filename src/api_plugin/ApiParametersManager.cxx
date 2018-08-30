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

#include "ApiParametersManager.h"

namespace gazebo { namespace dds { namespace utils {

ApiParametersManager::ApiParametersManager(int argc, char *argv[]) :
        ParametersManager(argc,argv)
{
}

ApiParametersManager::~ApiParametersManager()
{
}

void ApiParametersManager::validate_basic_sample(
        std::string service_name,
        std::string entity_name,
        std::string frame_name)
{
    std::string exception;
    std::string arguments_expected(entity_name + ": <" + entity_name + "> ");

    // Check if frame_name is requested
    bool frame_flag = false;
    if (!frame_name.empty()) {
        frame_flag = true;
    }

    // Check request information
    if (sample_information_[entity_name].empty()
        || (frame_flag && sample_information_[frame_name].empty())) {
        exception
                = "\nERROR: Missing  arguments to call service: \nMissing "
                  "arguments:";

        if (sample_information_[entity_name].empty())
            exception += " " + entity_name + ",";

        if (frame_flag && sample_information_[frame_name].empty()) {
            exception += " " + frame_name + ",";
            arguments_expected += frame_name + ": <" + frame_name + ">";
        }

        exception += "\n\nExcepted: "
                        "apipublisher -d <domain_id> -s " + service_name + 
                        " -i " 
                        "\"" + arguments_expected + "\"";

        throw std::runtime_error(exception);
    }
}

void ApiParametersManager::validate_set_light_properties_sample()
{
    std::string exception;

    // Check request information
    if (sample_information_["light_name"].empty() ||
        sample_information_["diffuse"].empty() || 
        sample_information_["attenuation_constant"].empty() || 
        sample_information_["attenuation_linear"].empty() ||
        sample_information_["attenuation_quadratic"].empty()) {

        exception
                = "\nERROR: Missing  arguments to call service: \nMissing "
                    "arguments:";
        
        if (sample_information_["light_name"].empty())
            exception+= " light_name,";
        
        if (sample_information_["diffuse"].empty())
            exception+= " diffuse,";

        if (sample_information_["attenuation_constant"].empty())
            exception+= " attenuation_constant,";

        if (sample_information_["attenuation_linear"].empty())
            exception+= " attenuation_linear,";

        if (sample_information_["attenuation_quadratic"].empty())
            exception+= " attenuation_quadratic";

        exception += "\n\nExcepted: apipublisher -d <domain_id> -s "
            "set_light_properties -i \"light_name: <light_name> diffuse: "
            "<diffuse> attenuation_constant: <attenuation_constant> "
            "attenuation_linear: <attenuation_linear> "
            "attenuation_quadratic: <attenuation_quadratic>\"";
        throw std::runtime_error(exception);
    }
    else if (sample_information_["diffuse"].size() != 4){
        exception = "\nERROR: Missing values to call service: \nMissing "
            "values : diffuse (need 4 values) ";

        exception += "\n\nExcepted: apipublisher -d <domain_id> -s "
            "set_light_properties -i \"light_name: <light_name> diffuse: "
            "<diffuse: 4> attenuation_constant: <attenuation_constant> "
            "attenuation_linear: <attenuation_linear> "
            "attenuation_quadratic: <attenuation_quadratic>\"";
        throw std::runtime_error(exception);
    }
}

}  // namespace utils
}  // namespace dds
}  // namespace gazebo