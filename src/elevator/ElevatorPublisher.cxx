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

#include <dds/core/ddscore.hpp>
#include <dds/domain/find.hpp>
#include <dds/pub/ddspub.hpp>

#include "std_msgs/msg/Int32.hpp"

void publisher_main(int domain_id, std::string topic_name, int floor)
{
    // Create a DomainParticipant with default Qos
    dds::domain::DomainParticipant participant(dds::domain::find(domain_id));
    if (participant == dds::core::null) {
        participant = dds::domain::DomainParticipant(domain_id);
    }

    // Create a Topic -- and automatically register the type
    dds::topic::Topic<std_msgs::msg::Int32> topic(participant, topic_name);

    // Create a DataWriter with default Qos (Publisher created in-line)
    dds::pub::DataWriter<std_msgs::msg::Int32> writer(
            dds::pub::Publisher(participant), topic);

    std_msgs::msg::Int32 sample;
    sample.data(floor);

    rti::util::sleep(dds::core::Duration::from_millisecs(200));

    // Write sample
    std::cout << "Sending data..." << std::endl;
    writer.write(sample);
}

int main(int argc, char *argv[])
{
    if (argc < 4) {
        std::cerr << "Missing arguments." << std::endl
                  << "Template: elevatorpublisher <domain id> <topic name> "
                     "<floor>"
                  << std::endl;
        return -1;
    }

    try {
        publisher_main(atoi(argv[1]), std::string(argv[2]), atoi(argv[3]));
    } catch (const std::exception &ex) {
        // This will catch DDS exceptions
        std::cerr << "Exception in publisher_main(): " << ex.what()
                  << std::endl;
        return -1;
    }

    return 0;
}