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
          get_model_properties_replier_(::dds::core::null),
          reset_simulation_replier_(::dds::core::null),
          reset_world_replier_(::dds::core::null),
          pause_physics_replier_(::dds::core::null),
          unpause_physics_replier_(::dds::core::null),
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
                  std::placeholders::_1)),
          get_model_properties_listener_(std::bind(
                  &ApiPlugin::get_model_properties,
                  this,
                  std::placeholders::_1)),
          reset_simulation_listener_(std::bind(
                  &ApiPlugin::reset_simulation,
                  this,
                  std::placeholders::_1)),
          reset_world_listener_(std::bind(
                  &ApiPlugin::reset_world,
                  this,
                  std::placeholders::_1)),
          pause_physics_listener_(std::bind(
                  &ApiPlugin::pause_physics,
                  this,
                  std::placeholders::_1)),
          unpause_physics_listener_(std::bind(
                  &ApiPlugin::unpause_physics,
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

#if GAZEBO_MAJOR_VERSION >= 8
    std::string world_name = world_->Name();
#else
    std::string world_name = world_->GetName();
#endif

    gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gazebo_node_->Init(world_name);
    gazebo_pub_ = gazebo_node_->Advertise<gazebo::msgs::Request>("~/request");

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
            get_link_properties_replier_, participant_, "get_link_properties");

    get_link_properties_replier_.listener(&get_link_properties_listener_);

    utils::create_replier<
            gazebo_msgs::srv::GetModelProperties_Request,
            gazebo_msgs::srv::GetModelProperties_Response>(
            get_model_properties_replier_,
            participant_,
            "get_model_properties");

    get_model_properties_replier_.listener(&get_model_properties_listener_);

    utils::create_replier<
            std_msgs::msg::Empty,
            gazebo_msgs::srv::Default_Response>(
            reset_simulation_replier_, participant_, "reset_simulation");

    reset_simulation_replier_.listener(&reset_simulation_listener_);

    utils::create_replier<
            std_msgs::msg::Empty,
            gazebo_msgs::srv::Default_Response>(
            reset_world_replier_, participant_, "reset_world");

    reset_world_replier_.listener(&reset_world_listener_);

    utils::create_replier<
            std_msgs::msg::Empty,
            gazebo_msgs::srv::Default_Response>(
            pause_physics_replier_, participant_, "pause_physics");

    pause_physics_replier_.listener(&pause_physics_listener_);

    utils::create_replier<
            std_msgs::msg::Empty,
            gazebo_msgs::srv::Default_Response>(
            unpause_physics_replier_, participant_, "unpause_physics");

    unpause_physics_replier_.listener(&unpause_physics_listener_);

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
    gazebo::physics::ModelPtr model
            = utils::get_model(world_, request.model_name());

    gazebo_msgs::srv::Default_Response reply;
    if (model) {
        gazebo::msgs::Request *msg = gazebo::msgs::CreateRequest(
                "entity_delete", request.model_name());
        gazebo_pub_->Publish(*msg, true);

        reply.success(true);
        reply.status_message("DeleteModel: successfully deleted model");
    }

    return reply;
}

gazebo_msgs::srv::Default_Response
        ApiPlugin::delete_light(gazebo_msgs::srv::DeleteLight_Request request)
{
    gazebo::physics::LightPtr light
            = utils::get_light(world_, request.light_name());

    gazebo_msgs::srv::Default_Response reply;
    if (light) {
        gazebo::msgs::Request *msg = gazebo::msgs::CreateRequest(
                "entity_delete", request.light_name());
        gazebo_pub_->Publish(*msg, true);

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
    gazebo::physics::LightPtr light
            = utils::get_light(world_, request.light_name());

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
    gazebo::physics::JointPtr joint
            = utils::get_joint(world_, request.joint_name());

    if (!joint) {
        reply.success(false);
        reply.status_message("GetJointProperties: joint not found");
    } else {
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
    gazebo::physics::Link *link = dynamic_cast<gazebo::physics::Link *>(
            utils::get_entity(world_, request.link_name()).get());

    if (!link) {
        reply.success(false);
        reply.status_message(
                "GetLinkProperties: link not found, did you forget to scope "
                "the link by model name?");
    } else {
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

gazebo_msgs::srv::GetModelProperties_Response ApiPlugin::get_model_properties(
        gazebo_msgs::srv::GetModelProperties_Request request)
{
    gazebo_msgs::srv::GetModelProperties_Response reply;

    gazebo::physics::ModelPtr model
            = utils::get_model(world_, request.model_name());

    if (!model) {
        reply.success(false);
        reply.status_message("GetModelProperties: model does not exist");
    } else {
        // get model parent name
        gazebo::physics::Model *parent_model
                = dynamic_cast<gazebo::physics::Model *>(
                        model->GetParent().get());
        if (parent_model)
            reply.parent_model_name(parent_model->GetName());

        // get list of child bodies, geoms and children model names
        reply.child_model_names().resize(model->GetChildCount());
        reply.body_names().resize(model->GetChildCount());
        reply.geom_names().resize(model->GetChildCount());
        for (unsigned int i = 0; i < model->GetChildCount(); i++) {
            gazebo::physics::Link *body = dynamic_cast<gazebo::physics::Link *>(
                    model->GetChild(i).get());
            if (body) {
                reply.body_names()[i] = body->GetName();
                // get list of geoms
                for (unsigned int j = 0; j < body->GetChildCount(); j++) {
                    gazebo::physics::Collision *geom
                            = dynamic_cast<gazebo::physics::Collision *>(
                                    body->GetChild(i).get());
                    if (geom)
                        reply.geom_names()[i] = geom->GetName();
                }
            }

            gazebo::physics::Model *child_model
                    = dynamic_cast<gazebo::physics::Model *>(
                            model->GetChild(i).get());
            if (child_model) {
                reply.child_model_names()[i] = child_model->GetName();
            }
        }

        // get list of joints
        gazebo::physics::Joint_V joints = model->GetJoints();
        reply.joint_names().resize(joints.size());
        for (unsigned int i = 0; i < joints.size(); i++)
            reply.joint_names()[i] = joints[i]->GetName();

        // is model static
        reply.is_static(model->IsStatic());

        reply.success(true);
        reply.status_message("GetModelProperties: got properties");
    }

    return reply;
}

gazebo_msgs::srv::Default_Response
        ApiPlugin::reset_simulation(std_msgs::msg::Empty request)
{
    world_->Reset();  

    gazebo_msgs::srv::Default_Response reply;

    reply.success(true);
    reply.status_message("Reset simulation: successfully");

    return reply;
}

gazebo_msgs::srv::Default_Response
        ApiPlugin::reset_world(std_msgs::msg::Empty request)
{
    world_->ResetEntities(gazebo::physics::Base::MODEL);
    gazebo_msgs::srv::Default_Response reply;

    reply.success(true);
    reply.status_message("Reset world: successfully");

    return reply;
}

gazebo_msgs::srv::Default_Response
        ApiPlugin::pause_physics(std_msgs::msg::Empty request)
{
    world_->SetPaused(true);
    gazebo_msgs::srv::Default_Response reply;

    reply.success(true);
    reply.status_message("Pause physics: successfully");

    return reply;
}

gazebo_msgs::srv::Default_Response
        ApiPlugin::unpause_physics(std_msgs::msg::Empty request)
{
    world_->SetPaused(false);

    gazebo_msgs::srv::Default_Response reply;

    reply.success(true);
    reply.status_message("Unpause physics: successfully");

    return reply;
}

}  // namespace dds
}  // namespace gazebo
