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

#include <iostream>
#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <string>

class CommandLineParser {
public:
    /**
     * @brief Constructor
     */
    CommandLineParser(int argc, char *argv[])
    {
        for (int i = 1; i+1 < argc; i = i + 2) {
            argument_map_.insert(
                    std::pair<std::string, std::string>(argv[i], argv[i + 1]));
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
        std::istringstream buffer(get_value("-s"));
        std::istream_iterator<std::string> beg(buffer), end;

        return std::vector<std::string>(beg, end);
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
