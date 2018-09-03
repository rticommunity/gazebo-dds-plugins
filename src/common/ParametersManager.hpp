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
     * @brief Validate that the sample information has all the variable that
     * it needs
     *
     * @param arguments list of variable that it will be checked
     * @param error message of error 
     * @param expected message of expected 
     */
    void validate_sample(
            std::vector<std::string> arguments,
            std::string error,
            std::string expected)
    {
        std::string exception;

        // Check missing arguments
        std::string missing_arguments
                = check_missing_arguments( arguments );

        // Check request information
        if (!missing_arguments.empty()) {
            exception
                    = error
                    + missing_arguments;
        } 

        if (!exception.empty()) {
            exception += expected;
            throw std::runtime_error(exception);
        }
    }

    /**
     * @brief Validate that the sample information has all the variable that
     * it needs. In addition, it checks the number of value of the multivalue
     * arguments
     *
     * @param arguments list of variable that it will be checked
     * @param error message of missing arguments
     * @param multi_value_arguments list of variable that have multivalue
     * @param multivalue error message of multivalues arguments with wrong number
     * of values
     * @param number_values correct number of value for multivalue arguments
     * @param expected message of expected
     */
    void validate_complex_sample( std::vector<std::string> arguments,
            std::string missing_error,
            std::vector<std::string> multi_value_arguments,
            int number_values,
            std::string multivalue_error,
            std::string expected)
    {
        std::string exception;

        // Check missing arguments
        std::string missing_arguments = check_missing_arguments(arguments);

        // Check request information
        if (!missing_arguments.empty()) {
            exception = missing_error + missing_arguments;
        } else {
            // Check multivalue arguments
            std::string multivalue_arguments = check_multivalue_arguments(
                    multi_value_arguments, number_values);

            if (!multivalue_arguments.empty()) {
                exception = multivalue_error + multivalue_arguments;
            }
        }

        if (!exception.empty()) {
            exception += expected;
            throw std::runtime_error(exception);
        }
    }

    /**
     * @brief Validate that the request information has all the variable that
     * it needs
     *
     * @param variable list list of variable that it will be checked
     * @return the missing arguments
     */
    std::string check_missing_arguments(
            const std::vector<std::string> &variable_list)
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

    /**
     * @brief Validate that the multi-value variables of the sample request
     *  has the correct number of elements
     *
     * @param variable_list list of variable that it will be checked
     * @param number_value_list list of number of elements that it will be use to check it
     * @return the multivalue arguments with a wrong number of elements
     */
    std::string check_multivalue_arguments(
            const std::vector<std::string> &variable_list,
            const std::vector<int> &number_value_list)
    {
        std::string multivalue_arguments = "";

        for (unsigned int i = 0; i < variable_list.size(); i++) {
            if (sample_information_[variable_list[i]].size()
                != number_value_list[i]) {
                multivalue_arguments = " " + variable_list[i] + " (needs "
                        + std::to_string(number_value_list[i]) + " values),";
            }
        }

        // Remove last cell ","
        if (!multivalue_arguments.empty())
            multivalue_arguments.pop_back();

        return multivalue_arguments;
    }

    /**
     * @brief Validate that the multi-value variables of the sample request
     *  has the correct number of elements
     *
     * @param variable_list list of variable that it will be checked
     * @param number_value number of elements that it will be use to check it
     * @return the multivalue arguments with a wrong number of elements
     */
    std::string check_multivalue_arguments(
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
