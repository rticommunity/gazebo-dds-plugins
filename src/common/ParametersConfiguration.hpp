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

#ifndef PARAMETERS_CONFIGURATION_HPP
#define PARAMETERS_CONFIGURATION_HPP

#include <iostream>
#include <string>
#include <vector>

namespace gazebo { namespace dds { namespace utils {

class ParametersConfiguration {
public:
    /**
     * @brief Constructor
     */
    ParametersConfiguration()
    {
    }

    /**
     * @brief Destructor
     */
    ~ParametersConfiguration()
    {
    }

    /**
     * @brief Constructor
     * 
     * @param arguments list of variable that it will be checked
     * @param missing_error message of missing error
     * @param expected message of expected
     */
    ParametersConfiguration(
            const std::vector<std::string> &arguments,
            const std::string &missing_error,
            const std::string &expected)
    {
        arguments_ = arguments;
        missing_error_ = missing_error;
        expected_ = expected;
    }

    /**
     * @brief Constructor
     * 
     * @param arguments list of variable that it will be checked
     * @param error message of missing arguments
     * @param multi_value_arguments list of variable that have multivalue
     * @param multivalue error message of multivalues arguments with wrong
     * number of values
     * @param number_values correct number of value for multivalue arguments
     * @param expected message of expected
     */
    ParametersConfiguration(
            const std::vector<std::string> &arguments,
            const std::string &missing_error,
            const std::vector<std::string> &multivalue_arguments,
            int number_values,
            const std::string &multivalue_error,
            const std::string &expected)
    {
        arguments_ = arguments;
        missing_error_ = missing_error;
        multivalue_arguments_ = multivalue_arguments;
        number_values_ = number_values;
        multivalue_error_ = multivalue_error;
        expected_ = expected;
    }

    /**
     * @brief Update arguments of the configuration
     *
     * @param arguments the new arguments
     */
    void arguments(const std::vector<std::string> &arguments)
    {
        arguments_ = arguments;
    }

    /**
     * @brief Obtain the arguments of the configuration
     *
     * @return the arguments
     */
    std::vector<std::string> &arguments()
    {
        return arguments_;
    }

    /**
     * @brief Update the missing error of the configuration
     *
     * @param missing_error the new missing_error
     */
    void missing_error(const std::string &missing_error)
    {
        missing_error_ = missing_error;
    }

    /**
     * @brief Obtain the missing error message of the configuration
     *
     * @return the missing error
     */
    std::string &missing_error()
    {
        return missing_error_;
    }

    /**
     * @brief Update the multivalue arguments of the configuration
     *
     * @param multivalue_arguments the new multivalue_arguments
     */
    void multivalue_arguments(
            const std::vector<std::string> &multivalue_arguments)
    {
        multivalue_arguments_ = multivalue_arguments;
    }

    /**
     * @brief Obtain the multivalue arguments of the configuration
     *
     * @return the multivalue arguments
     */
    std::vector<std::string> &multivalue_arguments()
    {
        return multivalue_arguments_;
    }

    /**
     * @brief Update the number of values to the multivalue arguments
     *
     * @param number_values the new number_values
     */
    void number_values(int number_values)
    {
        number_values_ = number_values;
    }

    /**
     * @brief Obtain the number of values of the configuration
     *
     * @return the number of values
     */
    int &number_values()
    {
        return number_values_;
    }

    /**
     * @brief Update the multivalue error of the configuration
     *
     * @param multivalue_error the new multivalue_error
     */
    void multivalue_error(const std::string &multivalue_error)
    {
        multivalue_error_ = multivalue_error;
    }

    /**
     * @brief Obtain the multivalue error message of the configuration
     *
     * @return the multivalue error
     */
    std::string &multivalue_error()
    {
        return multivalue_error_;
    }

    /**
     * @brief Update the expected message of the configuration
     *
     * @param expected the new expected message
     */
    void expected(const std::string &expected)
    {
        expected_ = expected;
    }


    /**
     * @brief Obtain the expected message of the configuration
     *
     * @return the expected message
     */
    std::string &expected()
    {
        return expected_;
    }

protected:
    std::vector<std::string> arguments_;
    std::string missing_error_;
    std::vector<std::string> multivalue_arguments_;
    int number_values_;
    std::string multivalue_error_;
    std::string expected_;
};

}  // namespace utils
}  // namespace dds
}  // namespace gazebo

#endif  // PARAMETERS_CONFIGURATION_HPP
