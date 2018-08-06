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

#ifndef COMMAND_LINE_PARSER_HPP
#define COMMAND_LINE_PARSER_HPP

#include <iostream>
#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <string>

namespace gazebo { namespace dds { namespace utils {

class CommandLineParser {
public:
    /**
     * @brief Constructor
     */
    CommandLineParser(int argc, char *argv[])
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
    ~CommandLineParser()
    {
    }

    std::string get_value(std::string key_value)
    {
        if (argument_map_.find(key_value) != argument_map_.end())
            return argument_map_[key_value];

        return "";
    }

    std::vector<std::string> get_values(std::string key_value)
    {
        std::string values = get_value(key_value);

        // Process values
        std::size_t found;
        found = values.find("(");
        if (found != std::string::npos)
            values.erase(found, 1);

        found = values.find(")");
        if (found != std::string::npos)
            values.erase(found, 1);

        std::istringstream buffer(values);
        std::istream_iterator<std::string> it_begin(buffer), it_end;

        return std::vector<std::string>(it_begin, it_end);
    }

    bool has_flag(std::string key_value)
    {
        bool result = false;

        if (argument_map_.find(key_value) != argument_map_.end()) {
            result = true;
        }

        return result;
    }

private:
    std::map<std::string, std::string> argument_map_;
};

}  // namespace utils
}  // namespace dds
}  // namespace gazebo

#endif  // COMMAND_LINE_PARSER_HPP
