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

#include "common/DdsUtils.hpp"
#include "ApiParametersManager.h"

#include "gazebo_msgs/srv/Default_Response.hpp"
#include "gazebo_msgs/srv/DeleteLight_Request.hpp"
#include "gazebo_msgs/srv/DeleteModel_Request.hpp"
#include "gazebo_msgs/srv/GetJointProperties_Request.hpp"
#include "gazebo_msgs/srv/GetJointProperties_Response.hpp"
#include "gazebo_msgs/srv/GetLightProperties_Request.hpp"
#include "gazebo_msgs/srv/GetLightProperties_Response.hpp"
#include "gazebo_msgs/srv/GetLinkProperties_Request.hpp"
#include "gazebo_msgs/srv/GetLinkProperties_Response.hpp"
#include "gazebo_msgs/srv/GetLinkState_Request.hpp"
#include "gazebo_msgs/srv/GetLinkState_Response.hpp"
#include "gazebo_msgs/srv/GetModelProperties_Request.hpp"
#include "gazebo_msgs/srv/GetModelProperties_Response.hpp"
#include "gazebo_msgs/srv/GetModelState_Request.hpp"
#include "gazebo_msgs/srv/GetModelState_Response.hpp"
#include "gazebo_msgs/srv/GetWorldProperties_Response.hpp"
#include "gazebo_msgs/srv/SetJointProperties_Request.hpp"
#include "gazebo_msgs/srv/SetLightProperties_Request.hpp"
#include "gazebo_msgs/srv/SetLinkProperties_Request.hpp"
#include "gazebo_msgs/srv/SetLinkState_Request.hpp"
#include "gazebo_msgs/srv/SetModelState_Request.hpp"
#include "std_msgs/msg/Empty.hpp"

template <typename T, typename T2>
T2 send_request(
        const dds::domain::DomainParticipant &participant,
        const std::string &service_name,
        T &request)
{
    rti::request::Requester<T, T2> requester(::dds::core::null);

    // Create requester
    gazebo::dds::utils::create_requester<T, T2>(
            requester,
            participant,
            service_name,
            ::dds::core::QosProvider::Default());

    // Call the service
    T2 reply = gazebo::dds::utils::call_service<T, T2>(requester, request);

    return reply;
}

void delete_model(
        const dds::domain::DomainParticipant &participant,
        const std::string &service_name,
        gazebo::dds::utils::ApiParametersManager parameters_manager)
{
    // Check request information
    parameters_manager.validate_basic_sample(service_name, "model_name");

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::DeleteModel_Request request(
            sample_information["model_name"][0]);

    gazebo_msgs::srv::Default_Response reply = send_request<
            gazebo_msgs::srv::DeleteModel_Request,
            gazebo_msgs::srv::Default_Response>(
            participant, service_name, request);

    std::cout << reply.status_message() << std::endl;
}

void delete_light(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ApiParametersManager parameters_manager)
{
    // Check request information
    parameters_manager.validate_basic_sample(service_name, "light_name");

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::DeleteLight_Request request(
            sample_information["light_name"][0]);

    gazebo_msgs::srv::Default_Response reply = send_request<
            gazebo_msgs::srv::DeleteLight_Request,
            gazebo_msgs::srv::Default_Response>(
            participant, service_name, request);

    std::cout << reply.status_message() << std::endl;
}

void get_light_properties(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ApiParametersManager parameters_manager)
{
    // Check request information
    parameters_manager.validate_basic_sample(service_name, "light_name");

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::GetLightProperties_Request request(
            sample_information["light_name"][0]);

    gazebo_msgs::srv::GetLightProperties_Response reply = send_request<
            gazebo_msgs::srv::GetLightProperties_Request,
            gazebo_msgs::srv::GetLightProperties_Response>(
            participant, service_name, request);

    if (reply.success()) {
        std::cout << reply << std::endl;
    } else {
        std::cout << reply.status_message() << std::endl;
    }
}

void get_world_properties(
        const dds::domain::DomainParticipant &participant,
        std::string service_name)
{
    std_msgs::msg::Empty request;

    gazebo_msgs::srv::GetWorldProperties_Response reply = send_request<
            std_msgs::msg::Empty,
            gazebo_msgs::srv::GetWorldProperties_Response>(
            participant, service_name, request);

    std::cout << reply << std::endl;
}

void get_joint_properties(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ApiParametersManager parameters_manager)
{
    // Check request information
    parameters_manager.validate_basic_sample(service_name, "joint_name");

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::GetJointProperties_Request request(
            sample_information["joint_name"][0]);

    gazebo_msgs::srv::GetJointProperties_Response reply = send_request<
            gazebo_msgs::srv::GetJointProperties_Request,
            gazebo_msgs::srv::GetJointProperties_Response>(
            participant, service_name, request);

    if (reply.success()) {
        std::cout << reply << std::endl;
    } else {
        std::cout << reply.status_message() << std::endl;
    }
}

void get_link_properties(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ApiParametersManager parameters_manager)
{
    // Check request information
    parameters_manager.validate_basic_sample(service_name, "link_name");

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::GetLinkProperties_Request request(
            sample_information["link_name"][0]);

    gazebo_msgs::srv::GetLinkProperties_Response reply = send_request<
            gazebo_msgs::srv::GetLinkProperties_Request,
            gazebo_msgs::srv::GetLinkProperties_Response>(
            participant, service_name, request);

    if (reply.success()) {
        std::cout << reply << std::endl;
    } else {
        std::cout << reply.status_message() << std::endl;
    }
}

void get_link_state(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ApiParametersManager parameters_manager)
{
    // Check request information
    parameters_manager.validate_basic_sample(
            service_name, "link_name", "reference_frame");

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::GetLinkState_Request request(
            sample_information["link_name"][0],
            sample_information["reference_frame"][0]);

    gazebo_msgs::srv::GetLinkState_Response reply = send_request<
            gazebo_msgs::srv::GetLinkState_Request,
            gazebo_msgs::srv::GetLinkState_Response>(
            participant, service_name, request);

    if (reply.success()) {
        std::cout << reply << std::endl;
    } else {
        std::cout << reply.status_message() << std::endl;
    }
}

void get_model_properties(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ApiParametersManager parameters_manager)
{
    // Check request information
    parameters_manager.validate_basic_sample(
            service_name, "model_name");

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::GetModelProperties_Request request(
            sample_information["model_name"][0]);

    gazebo_msgs::srv::GetModelProperties_Response reply = send_request<
            gazebo_msgs::srv::GetModelProperties_Request,
            gazebo_msgs::srv::GetModelProperties_Response>(
            participant, service_name, request);
    
    if (reply.success()) {
        std::cout << reply << std::endl;
    } else {
        std::cout << reply.status_message() << std::endl;
    }
}

void get_model_state(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ApiParametersManager parameters_manager)
{
    // Check request information
    parameters_manager.validate_basic_sample(
            service_name, "model_name", "relative_entity_name");

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::GetModelState_Request request(
            sample_information["model_name"][0],
            sample_information["relative_entity_name"][0]);

    gazebo_msgs::srv::GetModelState_Response reply = send_request<
            gazebo_msgs::srv::GetModelState_Request,
            gazebo_msgs::srv::GetModelState_Response>(
            participant, service_name, request);

    if (reply.success()) {
        std::cout << reply << std::endl;
    } else {
        std::cout << reply.status_message() << std::endl;
    }
}

void set_light_properties(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ApiParametersManager parameters_manager)
{
    // Check request information
    parameters_manager.validate_set_light_properties_sample();

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    // Fill the sample
    gazebo_msgs::srv::SetLightProperties_Request request;
    request.light_name(sample_information["light_name"][0]);

    request.diffuse().r(stof(sample_information["diffuse"][0]));
    request.diffuse().g(stof(sample_information["diffuse"][1]));
    request.diffuse().b(stof(sample_information["diffuse"][2]));
    request.diffuse().a(stof(sample_information["diffuse"][3]));

    request.attenuation_constant(
            stof(sample_information["attenuation_constant"][0]));
    request.attenuation_linear(
            stof(sample_information["attenuation_linear"][0]));
    request.attenuation_quadratic(
            stof(sample_information["attenuation_quadratic"][0]));

    gazebo_msgs::srv::Default_Response reply = send_request<
            gazebo_msgs::srv::SetLightProperties_Request,
            gazebo_msgs::srv::Default_Response>(
            participant, service_name, request);

    std::cout << reply << std::endl;
}

void set_link_properties(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ApiParametersManager parameters_manager)
{
    // Check request information
    parameters_manager.validate_set_link_properties_sample();

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    // Fill the sample
    gazebo_msgs::srv::SetLinkProperties_Request request;
    request.link_name(sample_information["link_name"][0]);

    request.com().position().x(stof(sample_information["com_position"][0]));
    request.com().position().y(stof(sample_information["com_position"][1]));
    request.com().position().z(stof(sample_information["com_position"][2]));

    request.gravity_mode(
            (strcasecmp("true", sample_information["gravity_mode"][0].c_str())
             == 0));

    request.mass(stof(sample_information["mass"][0]));
    request.ixx(stof(sample_information["ixx"][0]));
    request.ixy(stof(sample_information["ixy"][0]));
    request.ixz(stof(sample_information["ixz"][0]));
    request.iyy(stof(sample_information["iyy"][0]));
    request.iyz(stof(sample_information["iyz"][0]));
    request.izz(stof(sample_information["izz"][0]));

    gazebo_msgs::srv::Default_Response reply = send_request<
            gazebo_msgs::srv::SetLinkProperties_Request,
            gazebo_msgs::srv::Default_Response>(
            participant, service_name, request);

    std::cout << reply << std::endl;
}

void set_joint_properties(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ApiParametersManager parameters_manager)
{
    // Check request information
    parameters_manager.validate_set_joint_properties_sample();

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    // Fill the sample
    gazebo_msgs::srv::SetJointProperties_Request request;
    request.joint_name(sample_information["joint_name"][0]);

    int num_axis = sample_information["damping"].size();

    request.ode_joint_config().damping().resize(num_axis);
    request.ode_joint_config().hiStop().resize(num_axis);
    request.ode_joint_config().loStop().resize(num_axis);
    request.ode_joint_config().erp().resize(num_axis);
    request.ode_joint_config().cfm().resize(num_axis);
    request.ode_joint_config().stop_erp().resize(num_axis);
    request.ode_joint_config().stop_cfm().resize(num_axis);
    request.ode_joint_config().fudge_factor().resize(num_axis);
    request.ode_joint_config().fmax().resize(num_axis);
    request.ode_joint_config().vel().resize(num_axis);

    for (int i = 0; i < num_axis; i++) {
        request.ode_joint_config().damping()[i]
                = stof(sample_information["damping"][i]);
        request.ode_joint_config().hiStop()[i]
                = stof(sample_information["hiStop"][i]);
        request.ode_joint_config().loStop()[i]
                = stof(sample_information["loStop"][i]);
        request.ode_joint_config().erp()[i]
                = stof(sample_information["erp"][i]);
        request.ode_joint_config().cfm()[i]
                = stof(sample_information["cfm"][i]);
        request.ode_joint_config().stop_erp()[i]
                = stof(sample_information["stop_erp"][i]);
        request.ode_joint_config().stop_cfm()[i]
                = stof(sample_information["stop_cfm"][i]);
        request.ode_joint_config().fudge_factor()[i]
                = stof(sample_information["fudge_factor"][i]);
        request.ode_joint_config().fmax()[i]
                = stof(sample_information["fmax"][i]);
        request.ode_joint_config().vel()[i]
                = stof(sample_information["vel"][i]);
    }

    gazebo_msgs::srv::Default_Response reply = send_request<
            gazebo_msgs::srv::SetJointProperties_Request,
            gazebo_msgs::srv::Default_Response>(
            participant, service_name, request);

    std::cout << reply << std::endl;
}

void set_model_state(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ApiParametersManager parameters_manager)
{
    // Check request information
    parameters_manager.validate_set_model_state_sample();

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    // Fill the sample
    gazebo_msgs::srv::SetModelState_Request request;
    request.model_state().model_name(sample_information["model_name"][0]);

    request.model_state().pose().position().x(
            stof(sample_information["pose_position"][0]));
    request.model_state().pose().position().y(
            stof(sample_information["pose_position"][1]));
    request.model_state().pose().position().z(
            stof(sample_information["pose_position"][2]));

    request.model_state().pose().orientation().x(
            stof(sample_information["pose_orientation"][0]));
    request.model_state().pose().orientation().y(
            stof(sample_information["pose_orientation"][1]));
    request.model_state().pose().orientation().z(
            stof(sample_information["pose_orientation"][2]));

    request.model_state().twist().linear().x(
            stof(sample_information["twist_linear"][0]));
    request.model_state().twist().linear().y(
            stof(sample_information["twist_linear"][1]));
    request.model_state().twist().linear().z(
            stof(sample_information["twist_linear"][2]));

    request.model_state().twist().angular().x(
            stof(sample_information["twist_angular"][0]));
    request.model_state().twist().angular().y(
            stof(sample_information["twist_angular"][1]));
    request.model_state().twist().angular().z(
            stof(sample_information["twist_angular"][2]));

    request.model_state().reference_frame(
            sample_information["reference_frame"][0]);

    gazebo_msgs::srv::Default_Response reply = send_request<
            gazebo_msgs::srv::SetModelState_Request,
            gazebo_msgs::srv::Default_Response>(
            participant, service_name, request);

    std::cout << reply << std::endl;
}

void set_link_state(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ApiParametersManager parameters_manager)
{
    // Check request information
    parameters_manager.validate_set_link_state_sample();

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    // Fill the sample
    gazebo_msgs::srv::SetLinkState_Request request;
    request.link_state().link_name(sample_information["link_name"][0]);

    request.link_state().pose().position().x(
            stof(sample_information["pose_position"][0]));
    request.link_state().pose().position().y(
            stof(sample_information["pose_position"][1]));
    request.link_state().pose().position().z(
            stof(sample_information["pose_position"][2]));

    request.link_state().pose().orientation().x(
            stof(sample_information["pose_orientation"][0]));
    request.link_state().pose().orientation().y(
            stof(sample_information["pose_orientation"][1]));
    request.link_state().pose().orientation().z(
            stof(sample_information["pose_orientation"][2]));

    request.link_state().twist().linear().x(
            stof(sample_information["twist_linear"][0]));
    request.link_state().twist().linear().y(
            stof(sample_information["twist_linear"][1]));
    request.link_state().twist().linear().z(
            stof(sample_information["twist_linear"][2]));

    request.link_state().twist().angular().x(
            stof(sample_information["twist_angular"][0]));
    request.link_state().twist().angular().y(
            stof(sample_information["twist_angular"][1]));
    request.link_state().twist().angular().z(
            stof(sample_information["twist_angular"][2]));

    request.link_state().reference_frame(
            sample_information["reference_frame"][0]);

    gazebo_msgs::srv::Default_Response reply = send_request<
            gazebo_msgs::srv::SetLinkState_Request,
            gazebo_msgs::srv::Default_Response>(
            participant, service_name, request);

    std::cout << reply << std::endl;
}

void send_empty_request(
        const dds::domain::DomainParticipant &participant,
        std::string service_name)
{
    std_msgs::msg::Empty request;

    gazebo_msgs::srv::Default_Response reply = send_request<
            std_msgs::msg::Empty,
            gazebo_msgs::srv::Default_Response>(
            participant, service_name, request);

    std::cout << reply << std::endl;
}

int main(int argc, char *argv[])
{
    int ret_code = 0;

    // Set the unordered_map of functions
    std::unordered_map<
            std::string,
            std::function<void(
                    const dds::domain::DomainParticipant &,
                    const std::string &,
                    const gazebo::dds::utils::ApiParametersManager &)>>
            service_map;

    std::unordered_map<
            std::string,
            std::function<void(
                    const dds::domain::DomainParticipant &,
                    const std::string &)>>
            empty_service_map;

    // Initialize the map of functions
    service_map["delete_model"] = std::bind(
            &delete_model,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

    service_map["delete_light"] = std::bind(
            &delete_light,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

    service_map["get_light_properties"] = std::bind(
            &get_light_properties,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

    service_map["get_joint_properties"] = std::bind(
            &get_joint_properties,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

    service_map["get_link_properties"] = std::bind(
            &get_link_properties,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

    service_map["get_link_state"] = std::bind(
            &get_link_state,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

    service_map["get_model_properties"] = std::bind(
            &get_model_properties,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

    service_map["get_model_state"] = std::bind(
            &get_model_state,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

    service_map["set_light_properties"] = std::bind(
            &set_light_properties,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

    service_map["set_link_properties"] = std::bind(
            &set_link_properties,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

    service_map["set_joint_properties"] = std::bind(
            &set_joint_properties,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

    service_map["set_model_state"] = std::bind(
            &set_model_state,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

    service_map["set_link_state"] = std::bind(
            &set_link_state,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

    empty_service_map["get_world_properties"] = std::bind(
            &get_world_properties,
            std::placeholders::_1,
            std::placeholders::_2);

    empty_service_map["reset_simulation"] = std::bind(
            &send_empty_request, std::placeholders::_1, std::placeholders::_2);

    empty_service_map["reset_world"] = std::bind(
            &send_empty_request, std::placeholders::_1, std::placeholders::_2);

    empty_service_map["pause_physics"] = std::bind(
            &send_empty_request, std::placeholders::_1, std::placeholders::_2);

    empty_service_map["unpause_physics"] = std::bind(
            &send_empty_request, std::placeholders::_1, std::placeholders::_2);

    gazebo::dds::utils::ApiParametersManager parameters_manager(argc, argv);

    if (parameters_manager.has_flag("-h")) {
        std::cout << "Usage: apipublisher [options]" << std::endl
                  << "Generic options:" << std::endl
                  << "\t-h                      - Prints this page and exits"
                  << std::endl
                  << "\t-d <domain id>          - Sets the domainId (default 0)"
                  << std::endl
                  << "\t-s <service name>       - Sets the service name"
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
        if (parameters_manager.has_flag("-d")) {
            domain_id = atoi(parameters_manager.get_flag_value("-d").c_str());
        }

        std::string service_name
                = std::string(parameters_manager.get_flag_value("-s"));

        // Find a DomainParticipant
        dds::domain::DomainParticipant participant(
                dds::domain::find(domain_id));
        if (participant == dds::core::null) {
            participant = dds::domain::DomainParticipant(domain_id);
        }

        // Call the service
        if (parameters_manager.has_flag("-i")) {
            parameters_manager.process_sample_information("-i");
            service_map[service_name](
                    participant, service_name, parameters_manager);
        } else {
            empty_service_map[service_name](participant, service_name);
        }

    } catch (const std::exception &ex) {
        // This will catch DDS and CommandLineParser exceptions
        std::cerr << ex.what() << std::endl;
        ret_code = -1;
    }

    dds::domain::DomainParticipant::finalize_participant_factory();

    return ret_code;
}