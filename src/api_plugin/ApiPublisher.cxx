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
#include <rti/request/rtirequest.hpp>

#include "common/CommandLineParser.hpp"
#include "common/DdsUtils.hpp"

#include "gazebo_msgs/srv/Default_Response.hpp"
#include "gazebo_msgs/srv/DeleteModel_Request.hpp"

void publisher_main(
        int domain_id,
        std::string service_name)
{
    // Find a DomainParticipant
    dds::domain::DomainParticipant participant(dds::domain::find(domain_id));
    if (participant == dds::core::null) {
        participant = dds::domain::DomainParticipant(domain_id);
    }

    rti::request::RequesterParams requester_params(participant);
    auto qos_provider = dds::core::QosProvider::Default();
    requester_params.service_name(service_name);

    rti::request::Requester<
            gazebo_msgs::srv::DeleteModel_Request,
            gazebo_msgs::srv::Default_Response>
            requester(requester_params);

    while (rti::request::matched_replier_count(requester) == 0) {
            rti::util::sleep(dds::core::Duration::from_millisecs(100));
        }

    // Send the request
    auto request_id =
            requester.send_request(gazebo_msgs::srv::DeleteModel_Request("object_1"));

    // Receive replies
    const auto MAX_WAIT = dds::core::Duration::from_secs(5);

    bool in_progress = true;
    while(in_progress) {
        auto replies = requester.receive_replies(MAX_WAIT);

        /* When receive_replies times out,
            * it returns an empty reply collection
            */
        if (replies.length() == 0) {
            throw std::runtime_error("Timed out waiting for replies");
            return;
        }

        for (const auto& reply : replies) {
            if (reply.info().valid()) {
                std::cout << reply.data().status_message() << std::endl;
            }
        }
    }
    // Write sample
    std::cout << "Sending data..." << std::endl;
}

int main(int argc, char *argv[])
{
    int ret_code = 0;

    gazebo::dds::utils::CommandLineParser cmd_parser(argc, argv);

    if (cmd_parser.has_flag("-h")) {
        std::cout << "Usage: apipublisher [options]" << std::endl
                  << "Generic options:" << std::endl
                  << "\t-h                      - Prints this page and exits"
                  << std::endl
                  << "\t-d <domain id>          - Sets the domainId (default 0)"
                  << std::endl
                  << "\t-t <topic name>         - Sets the topic name"
                  << std::endl
                  << "\t-s <sample information> - Sets information of the "
                     "sample (default 0, 0)"
                  << std::endl;
        return 0;
    }

    // Handle signals (e.g., CTRL+C)
    gazebo::dds::utils::setup_signal_handler();

    try {
        // Check arguments
        int domain_id = 0;
        if (cmd_parser.has_flag("-d")) {
            domain_id = atoi(cmd_parser.get_value("-d").c_str());
        }

        publisher_main(
                domain_id,
                std::string(cmd_parser.get_value("-t")));

    } catch (const std::exception &ex) {
        // This will catch DDS and CommandLineParser exceptions
        std::cerr << ex.what() << std::endl;
        ret_code = -1;
    }

    dds::domain::DomainParticipant::finalize_participant_factory();

    return ret_code;
}