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

#include "common/DdsUtils.hpp"
#include "common/ParametersManager.hpp"
#include "common/ParametersConfiguration.hpp"

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
        gazebo::dds::utils::ParametersManager parameters_manager)
{
    // Initialize parameters configuration
    gazebo::dds::utils::ParametersConfiguration parameters;
    parameters.arguments({ "model_name" });
    parameters.missing_error(
            "\nERROR: Missing  arguments to call service: \nMissing "
            "arguments:");
    parameters.expected(
            "\n\nExcepted: apipublisher -d <domain_id> -s "
            "delete_model -i \"model_name: <model_name>\"");

    // Check request information
    parameters_manager.validate_sample(parameters);

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::DeleteModel_Request request(
            sample_information["model_name"][0]);

    // Send request
    gazebo_msgs::srv::Default_Response reply = send_request<
            gazebo_msgs::srv::DeleteModel_Request,
            gazebo_msgs::srv::Default_Response>(
            participant, service_name, request);

    std::cout << reply.status_message() << std::endl;
}

void delete_light(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ParametersManager parameters_manager)
{
    // Initialize parameters configuration
    gazebo::dds::utils::ParametersConfiguration parameters;
    parameters.arguments({ "light_name" });
    parameters.missing_error(
            "\nERROR: Missing  arguments to call service: \nMissing "
            "arguments:");
    parameters.expected(
            "\n\nExcepted: apipublisher -d <domain_id> -s "
            "delete_light -i \"light_name: <light_name>\"");

    // Check request information
    parameters_manager.validate_sample(parameters);

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::DeleteLight_Request request(
            sample_information["light_name"][0]);

    // Send request
    gazebo_msgs::srv::Default_Response reply = send_request<
            gazebo_msgs::srv::DeleteLight_Request,
            gazebo_msgs::srv::Default_Response>(
            participant, service_name, request);

    std::cout << reply.status_message() << std::endl;
}

void get_light_properties(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ParametersManager parameters_manager)
{
    // Initialize parameters configuration
    gazebo::dds::utils::ParametersConfiguration parameters;
    parameters.arguments({ "light_name" });
    parameters.missing_error(
            "\nERROR: Missing  arguments to call service: \nMissing "
            "arguments:");
    parameters.expected(
            "\n\nExcepted: apipublisher -d <domain_id> -s "
            "get_light_properties -i \"light_name: <light_name>\"");

    // Check request information
    parameters_manager.validate_sample(parameters);

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::GetLightProperties_Request request(
            sample_information["light_name"][0]);

    // Send request
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

    // Send request
    gazebo_msgs::srv::GetWorldProperties_Response reply = send_request<
            std_msgs::msg::Empty,
            gazebo_msgs::srv::GetWorldProperties_Response>(
            participant, service_name, request);

    std::cout << reply << std::endl;
}

void get_joint_properties(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ParametersManager parameters_manager)
{
    // Initialize parameters configuration
    gazebo::dds::utils::ParametersConfiguration parameters;
    parameters.arguments({ "joint_name" });
    parameters.missing_error(
            "\nERROR: Missing  arguments to call service: \nMissing "
            "arguments:");
    parameters.expected(
            "\n\nExcepted: apipublisher -d <domain_id> -s "
            "get_joint_properties -i \"joint_name: <joint_name>\"");

    // Check request information
    parameters_manager.validate_sample(parameters);

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::GetJointProperties_Request request(
            sample_information["joint_name"][0]);

    // Send request
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
        gazebo::dds::utils::ParametersManager parameters_manager)
{
    // Initialize parameters configuration
    gazebo::dds::utils::ParametersConfiguration parameters;
    parameters.arguments({ "link_name" });
    parameters.missing_error(
            "\nERROR: Missing  arguments to call service: \nMissing "
            "arguments:");
    parameters.expected(
            "\n\nExcepted: apipublisher -d <domain_id> -s "
            "get_link_properties -i \"link_name: <link_name>\"");

    // Check request information
    parameters_manager.validate_sample(parameters);

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::GetLinkProperties_Request request(
            sample_information["link_name"][0]);

    // Send request
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
        gazebo::dds::utils::ParametersManager parameters_manager)
{
    // Initialize parameters configuration
    gazebo::dds::utils::ParametersConfiguration parameters;
    parameters.arguments({ "link_name", "reference_frame" });
    parameters.missing_error(
            "\nERROR: Missing  arguments to call service: \nMissing "
            "arguments:");
    parameters.expected(
            "\n\nExcepted: apipublisher -d <domain_id> -s "
            "get_link_state -i \"link_name: <link_name> "
            "reference_frame: <reference_frame>\"");

    // Check request information
    parameters_manager.validate_sample(parameters);

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::GetLinkState_Request request(
            sample_information["link_name"][0],
            sample_information["reference_frame"][0]);

    // Send request
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
        gazebo::dds::utils::ParametersManager parameters_manager)
{
    // Initialize parameters configuration
    gazebo::dds::utils::ParametersConfiguration parameters;
    parameters.arguments({ "model_name" });
    parameters.missing_error(
            "\nERROR: Missing  arguments to call service: \nMissing "
            "arguments:");
    parameters.expected(
            "\n\nExcepted: apipublisher -d <domain_id> -s "
            "get_model_properties -i \"model_name: <model_name>\"");

    // Check request information
    parameters_manager.validate_sample(parameters);

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::GetModelProperties_Request request(
            sample_information["model_name"][0]);

    // Send request
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
        gazebo::dds::utils::ParametersManager parameters_manager)
{
    // Initialize parameters configuration
    gazebo::dds::utils::ParametersConfiguration parameters;
    parameters.arguments({ "model_name", "relative_entity_name" });
    parameters.missing_error(
            "\nERROR: Missing  arguments to call service: \nMissing "
            "arguments:");
    parameters.expected(
            "\n\nExcepted: apipublisher -d <domain_id> -s "
            "get_model_state -i \"model_name: <model_name> "
            "relative_entity_name: <relative_entity_name>\"");

    // Check request information
    parameters_manager.validate_sample(parameters);

    std::unordered_map<std::string, std::vector<std::string>> sample_information
            = parameters_manager.get_sample_information();

    gazebo_msgs::srv::GetModelState_Request request(
            sample_information["model_name"][0],
            sample_information["relative_entity_name"][0]);

    // Send request
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
        gazebo::dds::utils::ParametersManager parameters_manager)
{
    // Initialize parameters configuration
    gazebo::dds::utils::ParametersConfiguration parameters;
    parameters.arguments({ "light_name",
                           "diffuse",
                           "attenuation_constant",
                           "attenuation_linear",
                           "attenuation_quadratic" });
    parameters.missing_error(
            "\nERROR: Missing  arguments to call service: \nMissing "
            "arguments:");
    parameters.multivalue_arguments({ "diffuse" });
    parameters.number_values(4);
    parameters.multivalue_error(
            "\nERROR: Missing values to call service: \nMissing values:");
    parameters.expected(
            "\n\nExcepted: apipublisher -d <domain_id> -s "
            "set_light_properties -i \"light_name: <light_name> diffuse: "
            "<r> <g> <b> <a> attenuation_constant: <attenuation_constant> "
            "attenuation_linear: <attenuation_linear> "
            "attenuation_quadratic: <attenuation_quadratic>\"");

    // Check request information
    parameters_manager.validate_complex_sample(parameters);

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

    // Send request
    gazebo_msgs::srv::Default_Response reply = send_request<
            gazebo_msgs::srv::SetLightProperties_Request,
            gazebo_msgs::srv::Default_Response>(
            participant, service_name, request);

    std::cout << reply << std::endl;
}

void set_link_properties(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ParametersManager parameters_manager)
{
    // Initialize parameters configuration
    gazebo::dds::utils::ParametersConfiguration parameters;
    parameters.arguments({ "link_name",
                           "com_position",
                           "gravity_mode",
                           "mass",
                           "ixx",
                           "ixy",
                           "ixz",
                           "iyy",
                           "iyz",
                           "izz" });
    parameters.missing_error(
            "\nERROR: Missing  arguments to call service: \nMissing "
            "arguments:");
    parameters.multivalue_arguments({ "com_position" });
    parameters.number_values(3);
    parameters.multivalue_error(
            "\nERROR: Missing values to call service: \nMissing values:");
    parameters.expected(
            "\n\nExcepted: apipublisher -d <domain_id> -s "
            "set_link_properties -i \"link_name: <link_name> com_position: "
            "<x> <y> <z> gravity_mode: <gravity_mode> "
            "mass: <mass> ixx: <ixx> ixy: <ixy> ixz: <ixz> iyy: <iyy> "
            "iyz: <iyz> izz: <izz>\"");

    // Check request information
    parameters_manager.validate_complex_sample(parameters);
   
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

    // Send request
    gazebo_msgs::srv::Default_Response reply = send_request<
            gazebo_msgs::srv::SetLinkProperties_Request,
            gazebo_msgs::srv::Default_Response>(
            participant, service_name, request);

    std::cout << reply << std::endl;
}

void set_joint_properties(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ParametersManager parameters_manager)
{
    // Initialize parameters configuration
    gazebo::dds::utils::ParametersConfiguration parameters;
    parameters.arguments({ "joint_name",
                           "damping",
                           "hiStop",
                           "loStop",
                           "erp",
                           "cfm",
                           "stop_erp",
                           "stop_cfm",
                           "fudge_factor",
                           "fmax",
                           "vel" });
    parameters.missing_error(
            "\nERROR: Missing  arguments to call service: \nMissing "
            "arguments:");
    parameters.expected(
            "\n\nExcepted: apipublisher -d <domain_id> -s "
            "set_joint_properties -i \"joint_name: <joint_name> damping: "
            "<value_axis1> <value_axis2> hiStop: <value_axis1> <value_axis2> "
            "loStop: <value_axis1> <value_axis2> erp: <value_axis1> "
            "<value_axis2> "
            "cfm: <value_axis1> <value_axis2> stop_erp: <value_axis1> "
            "<value_axis2> stop_cfm: <value_axis1> <value_axis2> "
            "fudge_factor: <value_axis1> <value_axis2> fmax: <value_axis1> "
            "<value_axis2> vel: <value_axis1> <value_axis2>\""
            "\n\nNote: The second value of every variable is only needed in "
            "Universal joints. Universal joints require two axis.");

    // Check request information
    parameters_manager.validate_sample(parameters);

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

    // Send request
    gazebo_msgs::srv::Default_Response reply = send_request<
            gazebo_msgs::srv::SetJointProperties_Request,
            gazebo_msgs::srv::Default_Response>(
            participant, service_name, request);

    std::cout << reply << std::endl;
}

void set_model_state(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ParametersManager parameters_manager)
{
    // Initialize parameters configuration
    gazebo::dds::utils::ParametersConfiguration parameters;
    parameters.arguments({ "model_name",
                           "pose_position",
                           "pose_orientation",
                           "twist_linear",
                           "twist_angular",
                           "reference_frame" });
    parameters.missing_error(
            "\nERROR: Missing  arguments to call service: \nMissing "
            "arguments:");
    parameters.multivalue_arguments({ "pose_position",
                                      "pose_orientation",
                                      "twist_linear",
                                      "twist_angular" });
    parameters.number_values(3);
    parameters.multivalue_error(
            "\nERROR: Missing values to call service: \nMissing values:");
    parameters.expected(
            "\n\nExcepted: apipublisher -d <domain_id> -s "
            "set_model_state -i \"model_name: <model_name> pose_position: "
            "<x> <y> <z> pose_orientation: <x> <y> <z> "
            "twist_linear: <x> <y> <z> twist_angular: <x> <y> <z> "
            " reference_frame:<reference_frame>\"");

    // Check request information
    parameters_manager.validate_complex_sample(parameters);

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

    // Send request
    gazebo_msgs::srv::Default_Response reply = send_request<
            gazebo_msgs::srv::SetModelState_Request,
            gazebo_msgs::srv::Default_Response>(
            participant, service_name, request);

    std::cout << reply << std::endl;
}

void set_link_state(
        const dds::domain::DomainParticipant &participant,
        std::string service_name,
        gazebo::dds::utils::ParametersManager parameters_manager)
{
    // Initialize parameters configuration
    gazebo::dds::utils::ParametersConfiguration parameters;
    parameters.arguments({ "link_name",
                           "pose_position",
                           "pose_orientation",
                           "twist_linear",
                           "twist_angular",
                           "reference_frame" });
    parameters.missing_error(
            "\nERROR: Missing  arguments to call service: \nMissing "
            "arguments:");
    parameters.multivalue_arguments({ "pose_position",
                                      "pose_orientation",
                                      "twist_linear",
                                      "twist_angular" });
    parameters.number_values(3);
    parameters.multivalue_error(
            "\nERROR: Missing values to call service: \nMissing values:");
    parameters.expected(
            "\n\nExcepted: apipublisher -d <domain_id> -s "
            "set_link_state -i \"link_name: <link_name> pose_position: "
            "<x> <y> <z> pose_orientation: <x> <y> <z> "
            "twist_linear: <x> <y> <z> twist_angular: <x> <y> <z> "
            " reference_frame:<reference_frame>\"");

    // Check request information
    parameters_manager.validate_complex_sample(parameters);

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

    // Send request
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

    // Send request
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
                    const gazebo::dds::utils::ParametersManager &)>>
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

    gazebo::dds::utils::ParametersManager parameters_manager(argc, argv);

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