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

#include <cstdlib>

#include <dds/core/ddscore.hpp>
#include <dds/domain/find.hpp>
#include <dds/pub/ddspub.hpp>

#include "common/CommandLineParser.hpp"
#include "common/DdsUtils.hpp"
#include "geometry_msgs/msg/Twist.hpp"

void publisher_main(
        int domain_id,
        std::string topic_name,
        double linear_x,
        double angular_z)
{
    // Find a DomainParticipant
    dds::domain::DomainParticipant participant(dds::domain::find(domain_id));
    if (participant == dds::core::null) {
        participant = dds::domain::DomainParticipant(domain_id);
    }

    // Create a Topic -- and automatically register the type
    dds::topic::Topic<geometry_msgs::msg::Twist> topic(participant, topic_name);

    // Create a DataWriter with default Qos (Publisher created in-line)
    dds::pub::DataWriter<geometry_msgs::msg::Twist> writer(
            dds::pub::Publisher(participant), topic);

    geometry_msgs::msg::Twist sample;
    sample.linear().x(linear_x);
    sample.angular().z(angular_z);

    // Wait until it has a publication matched
    gazebo::dds::utils::wait_for_publication_matched(
            writer, dds::core::Duration(4));

    // Write sample
    std::cout << "Sending data..." << std::endl;
    writer.write(sample);

    writer.wait_for_acknowledgments(dds::core::Duration(4));
}

int main(int argc, char *argv[])
{
    int ret_code = 0;
    CommandLineParser cmd_parser(argc, argv);

    if (cmd_parser.has_flag("-h") ) {
        std::cout<< "Usage: diffdrivepublisher [options]" <<std::endl
                 << "Generic options:" <<std::endl
                 << "-help                           - Prints this page and exits" <<std::endl
                 << "-d                              - Sets the domainId (default 226)" <<std::endl
                 << "-t                              - Sets the topic name" <<std::endl
                 << "-s                              - Sets information of the sample" <<std::endl;
        return 0;
    }

    // Handle signals (e.g., CTRL+C)
    gazebo::dds::utils::setup_signal_handler();

    try {
        std::vector<std::string> sample_information
                = cmd_parser.get_values("-s");

        publisher_main(
                atoi(cmd_parser.get_value("-d").c_str()),
                std::string(cmd_parser.get_value("-t")),
                atof(sample_information[0].c_str()),
                atof(sample_information[1].c_str()));
    } catch (const std::exception &ex) {
        // This will catch DDS exceptions
        std::cerr << "Exception in publisher_main(): " << ex.what()
                  << std::endl;
        ret_code = -1;
    }

    dds::domain::DomainParticipant::finalize_participant_factory();

    return ret_code;
}
