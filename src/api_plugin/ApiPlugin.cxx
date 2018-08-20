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

    std::string world_name = utils::get_world_name(world_);
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

    // Initialize samples
    joint_properties_reply_.damping().resize(1);
    joint_properties_reply_.position().resize(1);
    joint_properties_reply_.rate().resize(1);

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

    if (model) {
        gazebo::msgs::Request *msg = gazebo::msgs::CreateRequest(
                "entity_delete", request.model_name());
        gazebo_pub_->Publish(*msg, true);

        default_reply_.success(true);
        default_reply_.status_message(
                "DeleteModel: successfully deleted model");
    } else {
        default_reply_.success(false);
        default_reply_.status_message(
                "DeleteModel: Requested model '"
                + request.model_name().to_std_string() + "' not found!");
    }

    return default_reply_;
}

gazebo_msgs::srv::Default_Response
        ApiPlugin::delete_light(gazebo_msgs::srv::DeleteLight_Request request)
{
    gazebo::physics::LightPtr light
            = utils::get_light(world_, request.light_name());

    if (light) {
        gazebo::msgs::Request *msg = gazebo::msgs::CreateRequest(
                "entity_delete", request.light_name());
        gazebo_pub_->Publish(*msg, true);

        default_reply_.success(true);
        default_reply_.status_message(
                "DeleteLight: successfully deleted light");
    } else {
        default_reply_.success(false);
        default_reply_.status_message(
                "DeleteLight: Requested light '"
                + request.light_name().to_std_string() + "' not found!");
    }

    return default_reply_;
}

gazebo_msgs::srv::GetLightProperties_Response ApiPlugin::get_light_properties(
        gazebo_msgs::srv::GetLightProperties_Request request)
{
    gazebo::physics::LightPtr light
            = utils::get_light(world_, request.light_name());

    if (light == NULL) {
        light_properties_reply_.success(false);
        light_properties_reply_.status_message(
                "GetLightProperties: Requested light '"
                + request.light_name().to_std_string() + "' not found!");
    } else {
        gazebo::msgs::Light light_sample;
        light->FillMsg(light_sample);

        light_properties_reply_.diffuse().r(light_sample.diffuse().r());
        light_properties_reply_.diffuse().g(light_sample.diffuse().g());
        light_properties_reply_.diffuse().b(light_sample.diffuse().b());
        light_properties_reply_.diffuse().a(light_sample.diffuse().a());

        light_properties_reply_.attenuation_constant(
                light_sample.attenuation_constant());
        light_properties_reply_.attenuation_linear(
                light_sample.attenuation_linear());
        light_properties_reply_.attenuation_quadratic(
                light_sample.attenuation_quadratic());

        light_properties_reply_.success(true);
        light_properties_reply_.status_message(
                "GetLightProperties: got properties");
    }

    return light_properties_reply_;
}

gazebo_msgs::srv::GetWorldProperties_Response ApiPlugin::get_world_properties(
        gazebo_msgs::srv::GetWorldProperties_Request request)
{
    get_world_models();

    world_properties_reply_.rendering_enabled(true);
    world_properties_reply_.success(true);
    world_properties_reply_.status_message(
            "GetWorldProperties: got properties");

    return world_properties_reply_;
}

gazebo_msgs::srv::GetJointProperties_Response ApiPlugin::get_joint_properties(
        gazebo_msgs::srv::GetJointProperties_Request request)
{
    gazebo::physics::JointPtr joint
            = utils::get_joint(world_, request.joint_name());

    if (!joint) {
        joint_properties_reply_.success(false);
        joint_properties_reply_.status_message(
                "GetJointProperties: joint not found");
    } else {
        joint_properties_reply_.damping()[0] = joint->GetDamping(0);
        joint_properties_reply_.rate()[0] = joint->GetVelocity(0);
        get_joint_position(joint);

        joint_properties_reply_.success(true);
        joint_properties_reply_.status_message(
                "GetJointProperties: got properties");
    }
    return joint_properties_reply_;
}

gazebo_msgs::srv::GetLinkProperties_Response ApiPlugin::get_link_properties(
        gazebo_msgs::srv::GetLinkProperties_Request request)
{
    gazebo::physics::Link *link = dynamic_cast<gazebo::physics::Link *>(
            utils::get_entity(world_, request.link_name()).get());

    if (!link) {
        link_properties_reply_.success(false);
        link_properties_reply_.status_message(
                "GetLinkProperties: link not found, did you forget to scope "
                "the link by model name?");
    } else {
        link_properties_reply_.gravity_mode(link->GetGravityMode());

        gazebo::physics::InertialPtr inertia = link->GetInertial();

        ignition::math::Vector3d center_of_gravity
                = get_link_inertial(link, inertia);

        link_properties_reply_.com().position().x(center_of_gravity.X());
        link_properties_reply_.com().position().y(center_of_gravity.Y());
        link_properties_reply_.com().position().z(center_of_gravity.Z());
        link_properties_reply_.com().orientation().x(0);
        link_properties_reply_.com().orientation().y(0);
        link_properties_reply_.com().orientation().z(0);
        link_properties_reply_.com().orientation().w(1);

        link_properties_reply_.success(true);
        link_properties_reply_.status_message(
                "GetLinkProperties: got properties");
    }

    return link_properties_reply_;
}

gazebo_msgs::srv::GetModelProperties_Response ApiPlugin::get_model_properties(
        gazebo_msgs::srv::GetModelProperties_Request request)
{
    gazebo::physics::ModelPtr model
            = utils::get_model(world_, request.model_name());

    if (!model) {
        model_properties_reply_.success(false);
        model_properties_reply_.status_message(
                "GetModelProperties: model does not exist");
    } else {
        // get model parent name
        gazebo::physics::Model *parent_model
                = dynamic_cast<gazebo::physics::Model *>(
                        model->GetParent().get());
        if (parent_model)
            model_properties_reply_.parent_model_name(parent_model->GetName());

        // get list of child bodies, geoms and children model names
        model_properties_reply_.child_model_names().resize(
                model->GetChildCount());
        model_properties_reply_.body_names().resize(model->GetChildCount());
        model_properties_reply_.geom_names().resize(model->GetChildCount());

        model_properties_reply_.child_model_names().clear();
        model_properties_reply_.body_names().clear();
        model_properties_reply_.geom_names().clear();

        for (unsigned int i = 0; i < model->GetChildCount(); i++) {
            gazebo::physics::Link *body = dynamic_cast<gazebo::physics::Link *>(
                    model->GetChild(i).get());
            if (body) {
                model_properties_reply_.body_names()[i] = body->GetName();
                // get list of geoms
                for (unsigned int j = 0; j < body->GetChildCount(); j++) {
                    gazebo::physics::Collision *geom
                            = dynamic_cast<gazebo::physics::Collision *>(
                                    body->GetChild(i).get());
                    if (geom)
                        model_properties_reply_.geom_names()[i]
                                = geom->GetName();
                }
            }

            gazebo::physics::Model *child_model
                    = dynamic_cast<gazebo::physics::Model *>(
                            model->GetChild(i).get());
            if (child_model) {
                model_properties_reply_.child_model_names()[i]
                        = child_model->GetName();
            }
        }

        // get list of joints
        gazebo::physics::Joint_V joints = model->GetJoints();
        model_properties_reply_.joint_names().resize(joints.size());
        model_properties_reply_.joint_names().clear();

        for (unsigned int i = 0; i < joints.size(); i++)
            model_properties_reply_.joint_names()[i] = joints[i]->GetName();

        // is model static
        model_properties_reply_.is_static(model->IsStatic());

        model_properties_reply_.success(true);
        model_properties_reply_.status_message(
                "GetModelProperties: got properties");
    }

    return model_properties_reply_;
}

gazebo_msgs::srv::Default_Response
        ApiPlugin::reset_simulation(std_msgs::msg::Empty request)
{
    world_->Reset();

    default_reply_.success(true);
    default_reply_.status_message("Reset simulation: successfully");

    return default_reply_;
}

gazebo_msgs::srv::Default_Response
        ApiPlugin::reset_world(std_msgs::msg::Empty request)
{
    world_->ResetEntities(gazebo::physics::Base::MODEL);

    default_reply_.success(true);
    default_reply_.status_message("Reset world: successfully");

    return default_reply_;
}

gazebo_msgs::srv::Default_Response
        ApiPlugin::pause_physics(std_msgs::msg::Empty request)
{
    world_->SetPaused(true);

    default_reply_.success(true);
    default_reply_.status_message("Pause physics: successfully");

    return default_reply_;
}

gazebo_msgs::srv::Default_Response
        ApiPlugin::unpause_physics(std_msgs::msg::Empty request)
{
    world_->SetPaused(false);

    default_reply_.success(true);
    default_reply_.status_message("Unpause physics: successfully");

    return default_reply_;
}

void ApiPlugin::get_world_models()
{
    world_properties_reply_.model_names().clear();
#if GAZEBO_MAJOR_VERSION >= 8
    world_properties_reply_.sim_time(world_->SimTime().Double());
    world_properties_reply_.model_names().resize(world_->ModelCount());
    for (unsigned int i = 0; i < world_->ModelCount(); i++)
        world_properties_reply_.model_names()[i]
                = world_->ModelByIndex(i)->GetName();
#else
    world_properties_reply_.sim_time(world_->GetSimTime().Double());
    world_properties_reply_.model_names().resize(world_->GetModelCount());
    for (unsigned int i = 0; i < world_->GetModelCount(); i++)
        world_properties_reply_.model_names()[i]
                = world_->GetModel(i)->GetName();
#endif
}

void ApiPlugin::get_joint_position(gazebo::physics::JointPtr joint)
{
#if GAZEBO_MAJOR_VERSION >= 8
    joint_properties_reply_.position()[0] = joint->Position(0);
#else
    joint_properties_reply_.position()[0] = joint->GetAngle(0).Radian();
#endif
}

ignition::math::Vector3d ApiPlugin::get_link_inertial(
        gazebo::physics::Link *link,
        gazebo::physics::InertialPtr inertia)
{
#if GAZEBO_MAJOR_VERSION >= 8
    link_properties_reply_.mass(link->GetInertial()->Mass());

    link_properties_reply_.ixx(inertia->IXX());
    link_properties_reply_.iyy(inertia->IYY());
    link_properties_reply_.izz(inertia->IZZ());
    link_properties_reply_.ixy(inertia->IXY());
    link_properties_reply_.ixz(inertia->IXZ());
    link_properties_reply_.iyz(inertia->IYZ());

    return link->GetInertial()->CoG();
#else
    link_properties_reply_.mass(link->GetInertial()->GetMass());

    link_properties_reply_.ixx(inertia->GetIXX());
    link_properties_reply_.iyy(inertia->GetIYY());
    link_properties_reply_.izz(inertia->GetIZZ());
    link_properties_reply_.ixy(inertia->GetIXY());
    link_properties_reply_.ixz(inertia->GetIXZ());
    link_properties_reply_.iyz(inertia->GetIYZ());

    return link->GetInertial()->GetCoG().Ign();
#endif
}

}  // namespace dds
}  // namespace gazebo
