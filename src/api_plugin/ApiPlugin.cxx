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
          get_joint_properties_replier_(::dds::core::null),
          get_link_properties_replier_(::dds::core::null),
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
                  std::placeholders::_1)),
          get_joint_properties_listener_(std::bind(
                  &ApiPlugin::get_joint_properties,
                  this,
                  std::placeholders::_1)),
          get_link_properties_listener_(std::bind(
                  &ApiPlugin::get_link_properties,
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

    utils::create_replier<
            gazebo_msgs::srv::GetJointProperties_Request,
            gazebo_msgs::srv::GetJointProperties_Response>(
            get_joint_properties_replier_,
            participant_,
            "get_joint_properties");

    get_joint_properties_replier_.listener(&get_joint_properties_listener_);

    utils::create_replier<
            gazebo_msgs::srv::GetLinkProperties_Request,
            gazebo_msgs::srv::GetLinkProperties_Response>(
            get_link_properties_replier_,
            participant_,
            "get_link_properties");

    get_link_properties_replier_.listener(&get_link_properties_listener_);

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

gazebo_msgs::srv::GetJointProperties_Response ApiPlugin::get_joint_properties(
        gazebo_msgs::srv::GetJointProperties_Request request)
{
    gazebo_msgs::srv::GetJointProperties_Response reply;
    gazebo::physics::JointPtr joint;
#if GAZEBO_MAJOR_VERSION >= 8
    for (unsigned int i = 0; i < world_->ModelCount() && !joint; i ++)
    {
        joint = world_->ModelByIndex(i)->GetJoint(request.joint_name());
#else
    for (unsigned int i = 0; i < world_->GetModelCount() && !joint; i ++)
    {
        joint = world_->GetModel(i)->GetJoint(request.joint_name());
#endif
    }

    if (!joint)
    {
        reply.success(false);
        reply.status_message("GetJointProperties: joint not found");
    }
    else
    {
        reply.damping().resize(1);
        reply.damping()[0] = joint->GetDamping(0);

        reply.position().clear();
        reply.position().resize(1);
        reply.rate().resize(1);
        
#if GAZEBO_MAJOR_VERSION >= 8
        reply.position()[0] = joint->Position(0);
#else
        reply.position()[0] = joint->GetAngle(0).Radian();
#endif

        reply.rate()[0] = joint->GetVelocity(0);

        reply.success(true);
        reply.status_message("GetJointProperties: got properties");
    }
    return reply;
}

gazebo_msgs::srv::GetLinkProperties_Response ApiPlugin::get_link_properties(
            gazebo_msgs::srv::GetLinkProperties_Request request)
{
    gazebo_msgs::srv::GetLinkProperties_Response reply;
#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::Link * link = dynamic_cast<gazebo::physics::Link *>(world_->EntityByName(request.link_name()).get());
#else
    gazebo::physics::Link * link = dynamic_cast<gazebo::physics::Link *>(world_->GetEntity(request.link_name()).get());
#endif

    if (!link)
    {
        reply.success(false);
        reply.status_message("GetLinkProperties: link not found, did you forget to scope the link by model name?");
    }
    else
    {
        reply.gravity_mode(link->GetGravityMode());

        gazebo::physics::InertialPtr inertia = link->GetInertial();

#if GAZEBO_MAJOR_VERSION >= 8
        reply.mass(link->GetInertial()->Mass());

        reply.ixx(inertia->IXX());
        reply.iyy(inertia->IYY());
        reply.izz(inertia->IZZ());
        reply.ixy(inertia->IXY());
        reply.ixz(inertia->IXZ());
        reply.iyz(inertia->IYZ());

        ignition::math::Vector3d com = link->GetInertial()->CoG();
#else
        reply.mass(link->GetInertial()->GetMass());

        reply.ixx(inertia->GetIXX());
        reply.iyy(inertia->GetIYY());
        reply.izz(inertia->GetIZZ());
        reply.ixy(inertia->GetIXY());
        reply.ixz(inertia->GetIXZ());
        reply.iyz(inertia->GetIYZ());

        ignition::math::Vector3d com = link->GetInertial()->GetCoG().Ign();
#endif
        reply.com().position().x(com.X());
        reply.com().position().y(com.Y());
        reply.com().position().z(com.Z());
        reply.com().orientation().x(0);
        reply.com().orientation().y(0);
        reply.com().orientation().z(0);
        reply.com().orientation().w(1);

        reply.success(true);
        reply.status_message("GetLinkProperties: got properties");
    }

    return reply;
}

}  // namespace dds
}  // namespace gazebo
