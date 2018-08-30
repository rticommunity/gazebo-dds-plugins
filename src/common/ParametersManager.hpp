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

#ifndef PARAMETERS_MANAGER_HPP
#define PARAMETERS_MANAGER_HPP

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <sstream>
#include <iterator>
#include <unordered_map>

namespace gazebo { namespace dds { namespace utils {

class ParametersManager {
public:
    /**
     * @brief Constructor
     */
    ParametersManager(int argc, char *argv[])
    {
        for (int i = 1; i < argc; i++) {
            if (argv[i][0] == '-' && i + 1 < argc && argv[i + 1][0] != '-') {
                argument_map_.insert(std::pair<std::string, std::string>(
                        argv[i], argv[i + 1]));
            } else if (argv[i][0] == '-') {
                argument_map_.insert(
                        std::pair<std::string, std::string>(argv[i], ""));
            }
        }
    }

    /**
     * @brief Destructor
     */
    ~ParametersManager()
    {
    }

    /**
     * @brief Get value of a specific flag
     *
     * @param key_value key that will be looked for on the map.
     * @return string that contains the value of the key
     */
    std::string get_flag_value(std::string key_value)
    {
        if (argument_map_.find(key_value) == argument_map_.end()) {
            throw std::runtime_error(
                    std::string("ParametersManager: Error in parameter '")
                    + key_value + "'");
        } else if (argument_map_[key_value] == "" && key_value != "-h") {
            throw std::runtime_error(
                    std::string(
                            "ParametersManager: Missing value for argument '")
                    + key_value + "'");
        }

        return argument_map_[key_value];
    }

    /**
     * @brief Process the information of the sample
     *
     * @param key_value key that will be looked for on the map.
     */
    void process_sample_information(std::string key_value)
    {
        std::string values = get_flag_value(key_value);

        process_values(values);

        std::istringstream buffer(values);
        std::istream_iterator<std::string> it_begin(buffer), it_end;

        std::vector<std::string> value_list(it_begin, it_end);

        std::string current_variable;
        for (int i = 0; i < value_list.size(); i++) {
            if (value_list[i][value_list[i].size() - 1] == ':') {
                current_variable
                        = value_list[i].substr(0, value_list[i].size() - 1);
            } else {
                sample_information_[current_variable].push_back(value_list[i]);
            }
        }
    }

    /**
     * @brief Get the sample information. It will process the information If it
     * was not processed
     *
     * @return map that contains all the values of the key
     */
    std::unordered_map<std::string, std::vector<std::string>>
            & get_sample_information(std::string key_value = "")
    {
        if(!key_value.empty() && sample_information_.empty())
            process_sample_information(key_value);

        return sample_information_;
    }

    /**
     * @brief Check if the key exist
     *
     * @param key_value key that will be checked.
     * @return if the key exist or not
     */
    bool has_flag(std::string key_value)
    {
        bool result = false;

        if (argument_map_.find(key_value) != argument_map_.end()) {
            result = true;
        }

        return result;
    }

    /**
     * @brief Validate the information of a basic sample. This sample contains 
     * element_name and frame_name(optional).
     */
    void validate_basic_sample(
            std::string service_name,
            std::string entity_name,
            std::string frame_name = "")
    {
        std::string exception;
        std::string arguments_expected(
                entity_name + ": <" + entity_name + "> ");

        // Check if frame_name is requested
        bool frame_flag = false;
        if(!frame_name.empty()){
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

            if (frame_flag && sample_information_[frame_name].empty()){
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

    /**
     * @brief Validate the request information of the set_light_properties
     * service
     */
    void validate_set_light_properties_sample()
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
 
private:
    /**
     * @brief Process the raw data to format them correctly
     *
     * @param data the raw data that will be processed.
     */
    void process_values(std::string &data)
    {
        std::size_t index = data.find(":");
        while (index < std::string::npos) {
            if (data[index + 1] != ' ') {
                data.insert(index + 1, " ");
            }

            index = data.find(":", index + 1);
        }
    }

protected:
    std::map<std::string, std::string> argument_map_;
    std::unordered_map<std::string, std::vector<std::string>>
            sample_information_;
};

}  // namespace utils
}  // namespace dds
}  // namespace gazebo

#endif  // PARAMETERS_MANAGER_HPP
