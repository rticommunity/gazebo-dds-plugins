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

    // Check missing arguments
    std::string missing_arguments
            = check_missing_arguments({ "light_name",
                                        "diffuse",
                                        "attenuation_constant",
                                        "attenuation_linear",
                                        "attenuation_quadratic" });

    // Check request information
    if (!missing_arguments.empty()) {
        exception
                = "\nERROR: Missing  arguments to call service: \nMissing "
                  "arguments:"
                + missing_arguments;
    } else {
        // Check multivalue arguments
        std::string multivalue_arguments
                = check_multivalue_arguments({ "diffuse" }, 4);

        if (!multivalue_arguments.empty()) {
            exception
                    = "\nERROR: Missing values to call service: \nMissing "
                      "values:"
                    + multivalue_arguments;
        }
    }

    if (!exception.empty()) {
        exception += "\n\nExcepted: apipublisher -d <domain_id> -s "
            "set_light_properties -i \"light_name: <light_name> diffuse: "
            "<r> <g> <b> <a> attenuation_constant: <attenuation_constant> "
            "attenuation_linear: <attenuation_linear> "
            "attenuation_quadratic: <attenuation_quadratic>\"";
        throw std::runtime_error(exception);
    }
}

void ApiParametersManager::validate_set_link_properties_sample()
{
    std::string exception;

    // Check missing arguments
    std::string missing_arguments = check_missing_arguments({
            "link_name",
            "com_position",
            "gravity_mode",
            "mass",
            "ixx",
            "ixy",
            "ixz",
            "iyy",
            "iyz",
            "izz"
    });

    // Check request information
    if (!missing_arguments.empty()) {
        exception
                = "\nERROR: Missing  arguments to call service: \nMissing "
                  "arguments:";

        exception += missing_arguments;
    } else {
        // Check multivalue arguments
        std::string multivalue_arguments
                = check_multivalue_arguments({ "com_position" }, 3);

        if (!multivalue_arguments.empty()) {
            exception
                    = "\nERROR: Missing values to call service: \nMissing "
                      "values:"
                    + multivalue_arguments;
        }
    }

    if (!exception.empty()) {
        exception += "\n\nExcepted: apipublisher -d <domain_id> -s "
            "set_link_properties -i \"link_name: <link_name> com_position: "
            "<x> <y> <z> gravity_mode: <gravity_mode> "
            "mass: <mass> ixx: <ixx> ixy: <ixy> ixz: <ixz> iyy: <iyy> "
            "iyz: <iyz> izz: <izz>\"";
        throw std::runtime_error(exception);
    }
}

void ApiParametersManager::validate_set_joint_properties_sample()
{
    std::string exception;

    // Check missing arguments
    std::string missing_arguments = check_missing_arguments({ "joint_name",
                                                              "damping",
                                                              "hiStop",
                                                              "loStop",
                                                              "erp",
                                                              "cfm",
                                                              "stop_erp",
                                                              "stop_cfm",
                                                              "fudge_factor",
                                                              "fmax",
                                                              "vel" });

    // Check request information
    if (!missing_arguments.empty()) {
        exception
                = "\nERROR: Missing  arguments to call service: \nMissing "
                  "arguments:"
                + missing_arguments;
    }

    if (!exception.empty()) {
        exception += "\n\nExcepted: apipublisher -d <domain_id> -s "
            "set_joint_properties -i \"joint_name: <joint_name> damping: "
            "<value_axi1> <value_axi2> hiStop: <value_axi1> <value_axi2> "
            "loStop: <value_axi1> <value_axi2> erp: <value_axi1> <value_axi2> "
            "cfm: <value_axi1> <value_axi2> stop_erp: <value_axi1> <value_axi2>"
            " stop_cfm: <value_axi1> <value_axi2> "
            "fudge_factor: <value_axi1> <value_axi2> fmax: <value_axi1> "
            "<value_axi2> vel: <value_axi1> <value_axi2>\""
            "\n\nNote: The second value of every variable is only needed in "
            "Universal joints. Universal joints require two axis.";
        throw std::runtime_error(exception);
    }
}

void ApiParametersManager::validate_set_model_state_sample()
{
    std::string exception;

    // Check missing arguments
    std::string missing_arguments
            = check_missing_arguments({ "model_name",
                                        "pose_position",
                                        "pose_orientation",
                                        "twist_linear",
                                        "twist_angular",
                                        "reference_frame" });

    // Check request information
    if (!missing_arguments.empty()) {
        exception
                = "\nERROR: Missing  arguments to call service: \nMissing "
                  "arguments:"
                + missing_arguments;
    } else {
        // Check multivalue arguments
        std::string multivalue_arguments = check_multivalue_arguments(
                { "pose_position",
                  "pose_orientation",
                  "twist_linear",
                  "twist_angular" },
                3);

        if (!multivalue_arguments.empty()) {
            exception
                    = "\nERROR: Missing values to call service: \nMissing "
                      "values:"
                    + multivalue_arguments;
        }
    }

    if (!exception.empty()) {
        exception += "\n\nExcepted: apipublisher -d <domain_id> -s "
            "set_model_state -i \"model_name: <model_name> pose_position: "
            "<x> <y> <z> pose_orientation: <x> <y> <z> "
            "twist_linear: <x> <y> <z> twist_angular: <x> <y> <z> "
            " reference_frame:<reference_frame>\"";
        throw std::runtime_error(exception);
    }
}

void ApiParametersManager::validate_set_link_state_sample()
{
    std::string exception;

    // Check missing arguments
    std::string missing_arguments
            = check_missing_arguments({ "link_name",
                                        "pose_position",
                                        "pose_orientation",
                                        "twist_linear",
                                        "twist_angular",
                                        "reference_frame" });

    // Check request information
    if (!missing_arguments.empty()) {
        exception
                = "\nERROR: Missing  arguments to call service: \nMissing "
                  "arguments:"
                + missing_arguments;
    } else {
        // Check multivalue arguments
        std::string multivalue_arguments = check_multivalue_arguments(
                { "pose_position",
                  "pose_orientation",
                  "twist_linear",
                  "twist_angular" },
                3);

        if (!multivalue_arguments.empty()) {
            exception
                    = "\nERROR: Missing values to call service: \nMissing "
                      "values:"
                    + multivalue_arguments;
        }
    }

    if (!exception.empty()) {
        exception += "\n\nExcepted: apipublisher -d <domain_id> -s "
            "set_link_state -i \"link_name: <link_name> pose_position: "
            "<x> <y> <z> pose_orientation: <x> <y> <z> "
            "twist_linear: <x> <y> <z> twist_angular: <x> <y> <z> "
            " reference_frame:<reference_frame>\"";
        throw std::runtime_error(exception);
    }
}

std::string ApiParametersManager::check_missing_arguments(
        const std::vector<std::string> & variable_list)
{
    std::string missing_arguments = "";

    for (unsigned int i = 0; i < variable_list.size(); i++) {
        if (sample_information_.find(variable_list[i])
            == sample_information_.end()) {
            missing_arguments += " " + variable_list[i] + ",";
        }
    }

    // Remove last cell ","
    if (!missing_arguments.empty())
        missing_arguments.pop_back();

    return missing_arguments;
}

std::string ApiParametersManager::check_multivalue_arguments(
        const std::vector<std::string> &variable_list,
        const std::vector<int> &number_value_list)
{
    std::string multivalue_arguments = "";

    for (unsigned int i = 0; i < variable_list.size(); i++) {
        if (sample_information_[variable_list[i]].size() != number_value_list[i]) {
            multivalue_arguments = " " + variable_list[i] + " (needs "
                    + std::to_string(number_value_list[i]) + " values),";
        }
    }

    // Remove last cell ","
    if (!multivalue_arguments.empty())
        multivalue_arguments.pop_back();

    return multivalue_arguments;
}

std::string ApiParametersManager::check_multivalue_arguments(
        const std::vector<std::string> &variable_list,
        int number_value)
{
    std::string multivalue_arguments = "";

    for (unsigned int i = 0; i < variable_list.size(); i++) {
        if (sample_information_[variable_list[i]].size() != number_value) {
            multivalue_arguments = " " + variable_list[i] + " (needs "
                    + std::to_string(number_value) + " values),";
        }
    }

    // Remove last cell ","
    if (!multivalue_arguments.empty())
        multivalue_arguments.pop_back();

    return multivalue_arguments;
}

}  // namespace utils
}  // namespace dds
}  // namespace gazebo