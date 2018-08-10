/*
 * Copyright 2018 Real-Time Innovations, Inc.
 * Copyright 2012-2014 Open Source Robotics Foundation
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

#include "common/Properties.h"
#include "common/GazeboUtils.hpp"
#include "common/DdsUtils.hpp"
#include "ApiPlugin.h"

namespace gazebo { namespace dds {

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ApiPlugin)

ApiPlugin::ApiPlugin()
        : participant_(::dds::core::null),
          delete_model_replier_(::dds::core::null),
          delete_light_replier_(::dds::core::null),
          get_light_properties_replier_(::dds::core::null),
          get_world_properties_replier_(::dds::core::null),
          delete_model_listener_(std::bind(
                  &ApiPlugin::delete_model,
                  this,
                  std::placeholders::_1)),
          delete_light_listener_(std::bind(
                  &ApiPlugin::delete_light,
                  this,
                  std::placeholders::_1)),
          get_light_properties_listener_(std::bind(
                  &ApiPlugin::get_light_properties,
                  this,
                  std::placeholders::_1)),
          get_world_properties_listener_(std::bind(
                  &ApiPlugin::get_world_properties,
                  this,
                  std::placeholders::_1))
{
}

ApiPlugin::~ApiPlugin()
{
}

void ApiPlugin::Load(physics::WorldPtr parent, sdf::ElementPtr sdf)
{
    world_ = parent;

    // Obtain the qos profile information from loaded world
    std::string qos_profile_file;
    utils::get_world_parameter<std::string>(
            sdf, qos_profile_file, QOS_PROFILE_FILE_PROPERTY_NAME.c_str(), "");

    std::string qos_profile;
    utils::get_world_parameter<std::string>(
            sdf, qos_profile, QOS_PROFILE_PROPERTY_NAME.c_str(), "");

    ::dds::core::QosProvider qos_provider(::dds::core::null);
    if (qos_profile_file != "" || qos_profile != "") {
        qos_provider = ::dds::core::QosProvider(qos_profile_file, qos_profile);
    } else {
        qos_provider = ::dds::core::QosProvider::Default();
    }

    // Obtain the domain id from loaded world
    int domain_id;
    utils::get_world_parameter<int>(
            sdf, domain_id, DOMAIN_ID_PROPERTY_NAME.c_str(), 0);

    utils::find_domain_participant(
            domain_id, participant_, qos_provider, qos_profile);

    // Create services
    utils::create_replier<
            gazebo_msgs::srv::DeleteModel_Request,
            gazebo_msgs::srv::Default_Response>(
            delete_model_replier_, participant_, "delete_model");

    delete_model_replier_.listener(&delete_model_listener_);

    utils::create_replier<
            gazebo_msgs::srv::DeleteLight_Request,
            gazebo_msgs::srv::Default_Response>(
            delete_light_replier_, participant_, "delete_light");

    delete_light_replier_.listener(&delete_light_listener_);

    utils::create_replier<
            gazebo_msgs::srv::GetLightProperties_Request,
            gazebo_msgs::srv::GetLightProperties_Response>(
            get_light_properties_replier_,
            participant_,
            "get_light_properties");

    get_light_properties_replier_.listener(&get_light_properties_listener_);

    utils::create_replier<
            gazebo_msgs::srv::GetWorldProperties_Request,
            gazebo_msgs::srv::GetWorldProperties_Response>(
            get_world_properties_replier_,
            participant_,
            "get_world_properties");

    get_world_properties_replier_.listener(&get_world_properties_listener_);

    gzmsg << std::endl;
    gzmsg << "Starting Api plugin" << std::endl;
    gzmsg << "* Services:" << std::endl;
    gzmsg << "  - delete_model "
          << " [gazebo_msgs/srv/DeleteModel_Request]/[gazebo_msgs/srv/"
             "Default_Response]"
          << std::endl;
    gzmsg << "  - delete_light "
          << " [gazebo_msgs/srv/DeleteLight_Request]/[gazebo_msgs/srv/"
             "Default_Response]"
          << std::endl;
    gzmsg << "  - get_light_properties "
          << " [gazebo_msgs/srv/GetLightProperties_Request]/[gazebo_msgs/srv/"
             "GetLightProperties_Response]"
          << std::endl;
    gzmsg << "  - get_world_properties "
          << " [gazebo_msgs/srv/GetWorldProperties_Request]/[gazebo_msgs/srv/"
             "GetWorldProperties_Response]"
          << std::endl;
}

gazebo_msgs::srv::Default_Response
        ApiPlugin::delete_model(gazebo_msgs::srv::DeleteModel_Request request)
{
#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::ModelPtr model = world_->ModelByName(request.model_name());
#else
    gazebo::physics::ModelPtr model = world_->GetModel(request.model_name());
#endif

    gazebo_msgs::srv::Default_Response reply;
    if (model) {
        world_->RemoveModel(model);

        reply.success(true);
        reply.status_message("DeleteModel: successfully deleted model");
    }

    return reply;
}

gazebo_msgs::srv::Default_Response
        ApiPlugin::delete_light(gazebo_msgs::srv::DeleteLight_Request request)
{
#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::LightPtr light = world_->LightByName(request.light_name());
#else
    gazebo::physics::LightPtr light = world_->Light(request.light_name());
#endif

    gazebo_msgs::srv::Default_Response reply;
    if (light) {
        reply.success(true);
        reply.status_message("DeleteLight: successfully deleted light");
    } else {
        reply.success(false);
        reply.status_message(
                "DeleteLight: Requested light '"
                + request.light_name().to_std_string() + "' not found!");
    }

    return reply;
}

gazebo_msgs::srv::GetLightProperties_Response ApiPlugin::get_light_properties(
        gazebo_msgs::srv::GetLightProperties_Request request)
{
#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::LightPtr light = world_->LightByName(request.light_name());
#else
    gazebo::physics::LightPtr light = world_->Light(request.light_name());
#endif

    gazebo_msgs::srv::GetLightProperties_Response reply;

    if (light == NULL) {
        reply.success(false);
        reply.status_message(
                "GetLightProperties: Requested light '"
                + request.light_name().to_std_string() + "' not found!");
    } else {
        gazebo::msgs::Light light_sample;
        light->FillMsg(light_sample);

        reply.diffuse().r(light_sample.diffuse().r());
        reply.diffuse().g(light_sample.diffuse().g());
        reply.diffuse().b(light_sample.diffuse().b());
        reply.diffuse().a(light_sample.diffuse().a());

        reply.attenuation_constant(light_sample.attenuation_constant());
        reply.attenuation_linear(light_sample.attenuation_linear());
        reply.attenuation_quadratic(light_sample.attenuation_quadratic());

        reply.success(true);
        reply.status_message("GetLightProperties: got properties");
    }

    return reply;
}

gazebo_msgs::srv::GetWorldProperties_Response ApiPlugin::get_world_properties(
        gazebo_msgs::srv::GetWorldProperties_Request request)
{
    gazebo_msgs::srv::GetWorldProperties_Response reply;

#if GAZEBO_MAJOR_VERSION >= 8
    reply.sim_time(world_->SimTime().Double());
    reply.model_names().resize(world_->ModelCount());
    for (unsigned int i = 0; i < world_->ModelCount(); i++)
        reply.model_names()[i] = world_->ModelByIndex(i)->GetName();
#else
    reply.sim_time(world_->GetSimTime().Double());
    reply.model_names().resize(world_->GetModelCount());
    for (unsigned int i = 0; i < world_->GetModelCount(); i++)
        reply.model_names()[i] = world_->GetModel(i)->GetName();
#endif

    reply.rendering_enabled(true);
    reply.success(true);
    reply.status_message("GetWorldProperties: got properties");

    return reply;
}

}  // namespace dds
}  // namespace gazebo
