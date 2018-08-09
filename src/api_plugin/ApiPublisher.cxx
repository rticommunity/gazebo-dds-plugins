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

#include <unordered_map>

#include <dds/core/ddscore.hpp>
#include <dds/domain/find.hpp>
#include <dds/pub/ddspub.hpp>
#include <rti/request/rtirequest.hpp>

#include "common/CommandLineParser.hpp"
#include "common/DdsUtils.hpp"

#include "gazebo_msgs/srv/Default_Response.hpp"
#include "gazebo_msgs/srv/DeleteModel_Request.hpp"
#include "gazebo_msgs/srv/DeleteLight_Request.hpp"

void delete_model(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        std::string model_name)
{
    rti::request::Requester<
            gazebo_msgs::srv::DeleteModel_Request,
            gazebo_msgs::srv::Default_Response>
            requester(::dds::core::null);

    gazebo::dds::utils::create_requester<gazebo_msgs::srv::DeleteModel_Request,
            gazebo_msgs::srv::Default_Response>
            (requester, participant, service_name);

    gazebo_msgs::srv::DeleteModel_Request request(model_name);

    gazebo_msgs::srv::Default_Response reply = gazebo::dds::utils::call_service<
            gazebo_msgs::srv::DeleteModel_Request,
            gazebo_msgs::srv::Default_Response>(requester, request);

    std::cout << reply.status_message() << std::endl;
}

void delete_light(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        std::string light_name)
{
    rti::request::Requester<
            gazebo_msgs::srv::DeleteLight_Request,
            gazebo_msgs::srv::Default_Response>
            requester(::dds::core::null);

    gazebo::dds::utils::create_requester<gazebo_msgs::srv::DeleteLight_Request,
            gazebo_msgs::srv::Default_Response>
            (requester, participant, service_name);

    gazebo_msgs::srv::DeleteLight_Request request(light_name);

    gazebo_msgs::srv::Default_Response reply = gazebo::dds::utils::call_service<
            gazebo_msgs::srv::DeleteLight_Request,
            gazebo_msgs::srv::Default_Response>(requester, request);

    std::cout << reply.status_message() << std::endl;
}

int main(int argc, char *argv[])
{
    int ret_code = 0;

    // Set of the unordered_map of functions
    std::unordered_map<
            std::string,
            std::function<void(
                    const dds::domain::DomainParticipant &,
                    const std::string &,
                    const std::string &)>>
            service_map;

    service_map["delete_model"] = std::bind(
                    &delete_model,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3);

    gazebo::dds::utils::CommandLineParser cmd_parser(argc, argv);

    if (cmd_parser.has_flag("-h")) {
        std::cout << "Usage: apipublisher [options]" << std::endl
                  << "Generic options:" << std::endl
                  << "\t-h                      - Prints this page and exits"
                  << std::endl
                  << "\t-d <domain id>          - Sets the domainId (default 0)"
                  << std::endl
                  << "\t-s <service name>         - Sets the service name"
                  << std::endl
                  << "\t-i <sample information> - Sets information of the "
                     "sample"
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

        std::string service_name = std::string(cmd_parser.get_value("-s"));

        // Find a DomainParticipant
        dds::domain::DomainParticipant participant(
                dds::domain::find(domain_id));
        if (participant == dds::core::null) {
            participant = dds::domain::DomainParticipant(domain_id);
        }

        service_map[service_name](participant,
                service_name,
                std::string(cmd_parser.get_value("-i")));

    } catch (const std::exception &ex) {
        // This will catch DDS and CommandLineParser exceptions
        std::cerr << ex.what() << std::endl;
        ret_code = -1;
    }

    dds::domain::DomainParticipant::finalize_participant_factory();

    return ret_code;
}