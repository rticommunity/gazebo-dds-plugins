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
          get_link_state_replier_(::dds::core::null),
          get_model_properties_replier_(::dds::core::null),
          get_model_state_replier_(::dds::core::null),
          set_light_properties_replier_(::dds::core::null),
          set_link_properties_replier_(::dds::core::null),
          set_model_state_replier_(::dds::core::null),
          set_link_state_replier_(::dds::core::null),
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
          get_link_state_listener_(std::bind(
                  &ApiPlugin::get_link_state,
                  this,
                  std::placeholders::_1)),
          get_model_properties_listener_(std::bind(
                  &ApiPlugin::get_model_properties,
                  this,
                  std::placeholders::_1)),
          get_model_state_listener_(std::bind(
                  &ApiPlugin::get_model_state,
                  this,
                  std::placeholders::_1)),
          set_light_properties_listener_(std::bind(
                  &ApiPlugin::set_light_properties,
                  this,
                  std::placeholders::_1)),
          set_link_properties_listener_(std::bind(
                  &ApiPlugin::set_link_properties,
                  this,
                  std::placeholders::_1)),
          set_model_state_listener_(std::bind(
                  &ApiPlugin::set_model_state,
                  this,
                  std::placeholders::_1)),
          set_link_state_listener_(std::bind(
                  &ApiPlugin::set_link_state,
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

    // Create the gazebo transport node
    std::string world_name = utils::get_world_name(world_);
    gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gazebo_node_->Init(world_name);
    gazebo_pub_ = gazebo_node_->Advertise<gazebo::msgs::Request>("~/request");
    light_modify_pub_
            = gazebo_node_->Advertise<gazebo::msgs::Light>("~/light/modify");

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
            delete_model_replier_, participant_, "delete_model", qos_provider);

    delete_model_replier_.listener(&delete_model_listener_);

    utils::create_replier<
            gazebo_msgs::srv::DeleteLight_Request,
            gazebo_msgs::srv::Default_Response>(
            delete_light_replier_, participant_, "delete_light", qos_provider);

    delete_light_replier_.listener(&delete_light_listener_);

    utils::create_replier<
            gazebo_msgs::srv::GetLightProperties_Request,
            gazebo_msgs::srv::GetLightProperties_Response>(
            get_light_properties_replier_,
            participant_,
            "get_light_properties",
            qos_provider);

    get_light_properties_replier_.listener(&get_light_properties_listener_);

    utils::create_replier<
            std_msgs::msg::Empty,
            gazebo_msgs::srv::GetWorldProperties_Response>(
            get_world_properties_replier_,
            participant_,
            "get_world_properties",
            qos_provider);

    get_world_properties_replier_.listener(&get_world_properties_listener_);

    utils::create_replier<
            gazebo_msgs::srv::GetJointProperties_Request,
            gazebo_msgs::srv::GetJointProperties_Response>(
            get_joint_properties_replier_,
            participant_,
            "get_joint_properties",
            qos_provider);

    get_joint_properties_replier_.listener(&get_joint_properties_listener_);

    utils::create_replier<
            gazebo_msgs::srv::GetLinkProperties_Request,
            gazebo_msgs::srv::GetLinkProperties_Response>(
            get_link_properties_replier_,
            participant_,
            "get_link_properties",
            qos_provider);

    get_link_properties_replier_.listener(&get_link_properties_listener_);

    utils::create_replier<
            gazebo_msgs::srv::GetLinkState_Request,
            gazebo_msgs::srv::GetLinkState_Response>(
            get_link_state_replier_,
            participant_,
            "get_link_state",
            qos_provider);

    get_link_state_replier_.listener(&get_link_state_listener_);

    utils::create_replier<
            gazebo_msgs::srv::GetModelProperties_Request,
            gazebo_msgs::srv::GetModelProperties_Response>(
            get_model_properties_replier_,
            participant_,
            "get_model_properties",
            qos_provider);

    get_model_properties_replier_.listener(&get_model_properties_listener_);

    utils::create_replier<
            gazebo_msgs::srv::GetModelState_Request,
            gazebo_msgs::srv::GetModelState_Response>(
            get_model_state_replier_,
            participant_,
            "get_model_state",
            qos_provider);

    get_model_state_replier_.listener(&get_model_state_listener_);

    utils::create_replier<
            gazebo_msgs::srv::SetLightProperties_Request,
            gazebo_msgs::srv::Default_Response>(
            set_light_properties_replier_,
            participant_,
            "set_light_properties",
            qos_provider);

    set_light_properties_replier_.listener(&set_light_properties_listener_);

    utils::create_replier<
            gazebo_msgs::srv::SetLinkProperties_Request,
            gazebo_msgs::srv::Default_Response>(
            set_link_properties_replier_,
            participant_,
            "set_link_properties",
            qos_provider);

    set_link_properties_replier_.listener(&set_link_properties_listener_);

    utils::create_replier<
            gazebo_msgs::srv::SetModelState_Request,
            gazebo_msgs::srv::Default_Response>(
            set_model_state_replier_,
            participant_,
            "set_model_state",
            qos_provider);

    set_model_state_replier_.listener(&set_model_state_listener_);

    utils::create_replier<
            gazebo_msgs::srv::SetLinkState_Request,
            gazebo_msgs::srv::Default_Response>(
            set_link_state_replier_,
            participant_,
            "set_link_state",
            qos_provider);

    set_link_state_replier_.listener(&set_link_state_listener_);

    utils::create_replier<
            std_msgs::msg::Empty,
            gazebo_msgs::srv::Default_Response>(
            reset_simulation_replier_,
            participant_,
            "reset_simulation",
            qos_provider);

    reset_simulation_replier_.listener(&reset_simulation_listener_);

    utils::create_replier<
            std_msgs::msg::Empty,
            gazebo_msgs::srv::Default_Response>(
            reset_world_replier_, participant_, "reset_world", qos_provider);

    reset_world_replier_.listener(&reset_world_listener_);

    utils::create_replier<
            std_msgs::msg::Empty,
            gazebo_msgs::srv::Default_Response>(
            pause_physics_replier_,
            participant_,
            "pause_physics",
            qos_provider);

    pause_physics_replier_.listener(&pause_physics_listener_);

    utils::create_replier<
            std_msgs::msg::Empty,
            gazebo_msgs::srv::Default_Response>(
            unpause_physics_replier_,
            participant_,
            "unpause_physics",
            qos_provider);

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
          << " [std_msgs::msg::Empty]/[gazebo_msgs/srv/"
             "GetWorldProperties_Response]"
          << std::endl;
}

gazebo_msgs::srv::Default_Response
        ApiPlugin::delete_model(gazebo_msgs::srv::DeleteModel_Request request)
{
    // Obtain the model
    gazebo::physics::ModelPtr model
            = utils::get_model(world_, request.model_name());

    if (model) {
        // Send the message to delete the entity
        gazebo::msgs::Request *msg = gazebo::msgs::CreateRequest(
                "entity_delete", request.model_name());
        gazebo_pub_->Publish(*msg, true);

        default_reply_.success(true);
        default_reply_.status_message(
                "DeleteModel: successfully deleted model");
    } else {
        default_reply_.success(false);
        default_reply_.status_message("DeleteModel: model not found");
    }

    return default_reply_;
}

gazebo_msgs::srv::Default_Response
        ApiPlugin::delete_light(gazebo_msgs::srv::DeleteLight_Request request)
{
    // Obtain the light
    gazebo::physics::LightPtr light
            = utils::get_light(world_, request.light_name());

    if (light) {
        // Send the message to delete the entity
        gazebo::msgs::Request *msg = gazebo::msgs::CreateRequest(
                "entity_delete", request.light_name());
        gazebo_pub_->Publish(*msg, true);

        default_reply_.success(true);
        default_reply_.status_message(
                "DeleteLight: successfully deleted light");
    } else {
        default_reply_.success(false);
        default_reply_.status_message("DeleteLight: light not found");
    }

    return default_reply_;
}

gazebo_msgs::srv::GetLightProperties_Response ApiPlugin::get_light_properties(
        gazebo_msgs::srv::GetLightProperties_Request request)
{
    // Obtain the light
    gazebo::physics::LightPtr light
            = utils::get_light(world_, request.light_name());

    if (light == NULL) {
        light_properties_reply_.success(false);
        light_properties_reply_.status_message(
                "GetLightProperties: light not found");
    } else {
        // Fill the sample
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

gazebo_msgs::srv::GetWorldProperties_Response
        ApiPlugin::get_world_properties(std_msgs::msg::Empty request)
{
    // Obtain all the models of the world
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
    // Obtain the joint
    gazebo::physics::JointPtr joint
            = utils::get_joint(world_, request.joint_name());

    if (!joint) {
        joint_properties_reply_.success(false);
        joint_properties_reply_.status_message(
                "GetJointProperties: joint not found");
    } else {
        // Fill the sample
        get_joint_type(joint);

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
    // Obtain the link
    gazebo::physics::Link *link = dynamic_cast<gazebo::physics::Link *>(
            utils::get_entity(world_, request.link_name()).get());

    if (!link) {
        link_properties_reply_.success(false);
        link_properties_reply_.status_message(
                "GetLinkProperties: link not found");
    } else {
        // Fill the sample
        link_properties_reply_.gravity_mode(link->GetGravityMode());

        ignition::math::Vector3d center_of_gravity = get_link_inertial(link);

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

gazebo_msgs::srv::GetLinkState_Response ApiPlugin::get_link_state(
        gazebo_msgs::srv::GetLinkState_Request request)
{
    // Obtain the link and frame
    gazebo::physics::EntityPtr link
            = utils::get_entity(world_, request.link_name());
    gazebo::physics::EntityPtr frame
            = utils::get_entity(world_, request.reference_frame());

    if (!link) {
        link_state_reply_.success(false);
        link_state_reply_.status_message("GetLinkState: link not found");
        return link_state_reply_;
    }

    // Obtain the current state of the link
    get_entity_state(link, entity_pose_, entity_vpos_, entity_veul_);

    if (frame) {
        // Obtain the current state of the frame
        get_entity_state(frame, frame_pose_, frame_vpos_, frame_veul_);

        entity_pose_ = entity_pose_ - frame_pose_;
        entity_vpos_ = frame_pose_.Rot().RotateVectorReverse(
                entity_vpos_ - frame_vpos_);
        entity_veul_ = frame_pose_.Rot().RotateVectorReverse(
                entity_veul_ - frame_veul_);
    } else if (
            request.reference_frame() != ""
            && request.reference_frame() != "world"
            && request.reference_frame() != "map") {
        link_state_reply_.success(false);
        link_state_reply_.status_message(
                "GetLinkState: reference frame not found");
        return link_state_reply_;
    }

    // Fill the sample
    link_state_reply_.link_state().link_name(request.link_name());
    link_state_reply_.link_state().pose().position().x(entity_pose_.Pos().X());
    link_state_reply_.link_state().pose().position().y(entity_pose_.Pos().Y());
    link_state_reply_.link_state().pose().position().z(entity_pose_.Pos().Z());
    link_state_reply_.link_state().pose().orientation().x(
            entity_pose_.Rot().X());
    link_state_reply_.link_state().pose().orientation().y(
            entity_pose_.Rot().Y());
    link_state_reply_.link_state().pose().orientation().z(
            entity_pose_.Rot().Z());
    link_state_reply_.link_state().pose().orientation().w(
            entity_pose_.Rot().W());
    link_state_reply_.link_state().twist().linear().x(entity_vpos_.X());
    link_state_reply_.link_state().twist().linear().y(entity_vpos_.Y());
    link_state_reply_.link_state().twist().linear().z(entity_vpos_.Z());
    link_state_reply_.link_state().twist().angular().x(entity_veul_.X());
    link_state_reply_.link_state().twist().angular().y(entity_veul_.Y());
    link_state_reply_.link_state().twist().angular().z(entity_veul_.Z());
    link_state_reply_.link_state().reference_frame(request.reference_frame());

    link_state_reply_.success(true);
    link_state_reply_.status_message("GetLinkState: got state");
    return link_state_reply_;
}

gazebo_msgs::srv::GetModelProperties_Response ApiPlugin::get_model_properties(
        gazebo_msgs::srv::GetModelProperties_Request request)
{
    // Obtain the model
    gazebo::physics::ModelPtr model
            = utils::get_model(world_, request.model_name());

    if (!model) {
        model_properties_reply_.success(false);
        model_properties_reply_.status_message(
                "GetModelProperties: model not found");
    } else {
        // Get model parent name
        gazebo::physics::Model *parent_model
                = dynamic_cast<gazebo::physics::Model *>(
                        model->GetParent().get());
        if (parent_model)
            model_properties_reply_.parent_model_name(parent_model->GetName());

        // Get list of child bodies, geoms and children model names
        model_properties_reply_.child_model_names().clear();
        model_properties_reply_.body_names().clear();
        model_properties_reply_.geom_names().clear();

        model_properties_reply_.child_model_names().resize(
                model->GetChildCount());
        model_properties_reply_.body_names().resize(model->GetChildCount());
        model_properties_reply_.geom_names().resize(model->GetChildCount());

        for (unsigned int i = 0; i < model->GetChildCount(); i++) {
            gazebo::physics::Link *body = dynamic_cast<gazebo::physics::Link *>(
                    model->GetChild(i).get());
            if (body) {
                model_properties_reply_.body_names()[i] = body->GetName();
                // Get list of geoms
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

        // Get list of joints
        gazebo::physics::Joint_V joints = model->GetJoints();
        model_properties_reply_.joint_names().clear();
        model_properties_reply_.joint_names().resize(joints.size());

        for (unsigned int i = 0; i < joints.size(); i++)
            model_properties_reply_.joint_names()[i] = joints[i]->GetName();

        // Set static mode
        model_properties_reply_.is_static(model->IsStatic());

        model_properties_reply_.success(true);
        model_properties_reply_.status_message(
                "GetModelProperties: got properties");
    }

    return model_properties_reply_;
}

gazebo_msgs::srv::GetModelState_Response ApiPlugin::get_model_state(
        gazebo_msgs::srv::GetModelState_Request request)
{
    // Obtain the model and frame
    gazebo::physics::EntityPtr model
            = utils::get_entity(world_, request.model_name());
    gazebo::physics::EntityPtr frame
            = utils::get_entity(world_, request.relative_entity_name());

    if (!model) {
        model_state_reply_.success(false);
        model_state_reply_.status_message("GetModelState: model not found");
        return model_state_reply_;
    } else {
        // Set header of the sample
        current_time_ = utils::get_sim_time(world_);

        model_state_reply_.header().stamp().sec(current_time_.sec);
        model_state_reply_.header().stamp().nanosec(current_time_.nsec);
        model_state_reply_.header().frame_id() = request.relative_entity_name();

        // Obtain the current state of the model
        get_entity_state(model, entity_pose_, entity_vpos_, entity_veul_);

        ignition::math::Vector3d entity_pos = entity_pose_.Pos();
        ignition::math::Quaterniond entity_rot = entity_pose_.Rot();

        if (frame) {
            // Obtain the current state of the frame
            get_entity_state(frame, frame_pose_, frame_vpos_, frame_veul_);

            ignition::math::Pose3d entity_rel_pose = entity_pose_ - frame_pose_;
            entity_pos = entity_rel_pose.Pos();
            entity_rot = entity_rel_pose.Rot();

            entity_vpos_ = frame_pose_.Rot().RotateVectorReverse(
                    entity_vpos_ - frame_vpos_);
            entity_veul_ = frame_pose_.Rot().RotateVectorReverse(
                    entity_veul_ - frame_veul_);
        } else if (
                request.relative_entity_name() != ""
                && request.relative_entity_name() != "world"
                && request.relative_entity_name() != "map"
                && request.relative_entity_name() != "/map") {
            model_state_reply_.success(false);
            model_state_reply_.status_message(
                    "GetModelState: reference relative_entity_name not found");
            return model_state_reply_;
        }

        // Fill the sample
        model_state_reply_.pose().position().x(entity_pos.X());
        model_state_reply_.pose().position().y(entity_pos.Y());
        model_state_reply_.pose().position().z(entity_pos.Z());
        model_state_reply_.pose().orientation().w(entity_rot.W());
        model_state_reply_.pose().orientation().x(entity_rot.X());
        model_state_reply_.pose().orientation().y(entity_rot.Y());
        model_state_reply_.pose().orientation().z(entity_rot.Z());

        model_state_reply_.twist().linear().x(entity_vpos_.X());
        model_state_reply_.twist().linear().y(entity_vpos_.Y());
        model_state_reply_.twist().linear().z(entity_vpos_.Z());
        model_state_reply_.twist().angular().x(entity_veul_.X());
        model_state_reply_.twist().angular().y(entity_veul_.Y());
        model_state_reply_.twist().angular().z(entity_veul_.Z());

        model_state_reply_.success(true);
        model_state_reply_.status_message("GetModelState: got properties");
    }

    return model_state_reply_;
}

gazebo_msgs::srv::Default_Response ApiPlugin::set_light_properties(
        gazebo_msgs::srv::SetLightProperties_Request request)
{
    // Obtain the light
    gazebo::physics::LightPtr phy_light
            = utils::get_light(world_, request.light_name());

    if (phy_light == NULL) {
        default_reply_.success(false);
        default_reply_.status_message("setLightProperties: light not found");
    } else {
        // Fill the sample
        phy_light->FillMsg(light_sample);

        light_sample.mutable_diffuse()->set_r(request.diffuse().r());
        light_sample.mutable_diffuse()->set_g(request.diffuse().g());
        light_sample.mutable_diffuse()->set_b(request.diffuse().b());
        light_sample.mutable_diffuse()->set_a(request.diffuse().a());

        light_sample.set_attenuation_constant(request.attenuation_constant());
        light_sample.set_attenuation_linear(request.attenuation_linear());
        light_sample.set_attenuation_quadratic(request.attenuation_quadratic());

        light_modify_pub_->Publish(light_sample, true);

        default_reply_.success(true);
        default_reply_.status_message("setLightProperties: properties set");
    }

    return default_reply_;
}

gazebo_msgs::srv::Default_Response ApiPlugin::set_link_properties(
        gazebo_msgs::srv::SetLinkProperties_Request request)
{
    // Obtain the link
    gazebo::physics::Link *link = dynamic_cast<gazebo::physics::Link *>(
            utils::get_entity(world_, request.link_name()).get());

    if (!link) {
        default_reply_.success(false);
        default_reply_.status_message("SetLinkProperties: link not found");

    } else {
        gazebo::physics::InertialPtr mass = link->GetInertial();

        // Update the properties of the link
        mass->SetCoG(ignition::math::Vector3d(
                request.com().position().x(),
                request.com().position().y(),
                request.com().position().z()));
        mass->SetInertiaMatrix(
                request.ixx(),
                request.iyy(),
                request.izz(),
                request.ixy(),
                request.ixz(),
                request.iyz());
        mass->SetMass(request.mass());
        link->SetGravityMode(request.gravity_mode());

        default_reply_.success(true);
        default_reply_.status_message("SetLinkProperties: properties set");
    }

    return default_reply_;
}

gazebo_msgs::srv::Default_Response ApiPlugin::set_joint_properties(
        gazebo_msgs::srv::SetJointProperties_Request request)
{

    gazebo::physics::JointPtr joint = utils::get_joint(world_, request.joint_name());

    if (!joint) {
        default_reply_.success(false);
        default_reply_.status_message("SetJointProperties: joint not found");
    } else {
        for (unsigned int i = 0;
             i < request.ode_joint_config().damping().size();
             i++) {
            joint->SetDamping(i, request.ode_joint_config().damping()[i]);
            joint->SetParam(
                    "hi_stop", i, request.ode_joint_config().hiStop()[i]);
            joint->SetParam(
                    "lo_stop", i, request.ode_joint_config().loStop()[i]);
            joint->SetParam("erp", i, request.ode_joint_config().erp()[i]);
            joint->SetParam("cfm", i, request.ode_joint_config().cfm()[i]);
            joint->SetParam(
                    "stop_erp", i, request.ode_joint_config().stop_erp()[i]);
            joint->SetParam(
                    "stop_cfm", i, request.ode_joint_config().stop_cfm()[i]);
            joint->SetParam(
                    "fudge_factor",
                    i,
                    request.ode_joint_config().fudge_factor()[i]);
            joint->SetParam("fmax", i, request.ode_joint_config().fmax()[i]);
            joint->SetParam("vel", i, request.ode_joint_config().vel()[i]);
        }

        default_reply_.success(true);
        default_reply_.status_message("SetJointProperties: properties set");
    }

    return default_reply_;
}

gazebo_msgs::srv::Default_Response ApiPlugin::set_model_state(
        gazebo_msgs::srv::SetModelState_Request request)
{
    // Manage the information of the request
    ignition::math::Vector3d target_pos(
            request.model_state().pose().position().x(),
            request.model_state().pose().position().y(),
            request.model_state().pose().position().z());
    ignition::math::Quaterniond target_rot(
            request.model_state().pose().orientation().w(),
            request.model_state().pose().orientation().x(),
            request.model_state().pose().orientation().y(),
            request.model_state().pose().orientation().z());
    target_rot.Normalize();  // eliminates invalid rotation (0, 0, 0, 0)
    ignition::math::Pose3d target_pose(target_pos, target_rot);
    ignition::math::Vector3d target_pos_dot(
            request.model_state().twist().linear().x(),
            request.model_state().twist().linear().y(),
            request.model_state().twist().linear().z());
    ignition::math::Vector3d target_rot_dot(
            request.model_state().twist().angular().x(),
            request.model_state().twist().angular().y(),
            request.model_state().twist().angular().z());

    // Obtain model
    gazebo::physics::ModelPtr model
            = utils::get_model(world_, request.model_state().model_name());

    if (!model) {
        default_reply_.success(false);
        default_reply_.status_message("SetModelState: model not found");
    } else {
        // Obtain frame
        gazebo::physics::EntityPtr relative_entity = utils::get_entity(
                world_, request.model_state().reference_frame());

        if (relative_entity) {
            // Obtain the current state of the frame
            get_entity_pose(relative_entity, frame_pose_);

            target_pose = target_pose + frame_pose_;
            target_pos_dot = frame_pose_.Rot().RotateVector(target_pos_dot);
            target_rot_dot = frame_pose_.Rot().RotateVector(target_rot_dot);
        } else if (
                request.model_state().reference_frame() != ""
                && request.model_state().reference_frame() != "world"
                && request.model_state().reference_frame() != "map") {
            default_reply_.success(false);
            default_reply_.status_message(
                    "SetModelState: specified reference frame entity does not "
                    "exist");
        }

        // Set model pose
        bool is_paused = world_->IsPaused();
        world_->SetPaused(true);
        model->SetWorldPose(target_pose);
        world_->SetPaused(is_paused);

        // Set model velocity
        model->SetLinearVel(target_pos_dot);
        model->SetAngularVel(target_rot_dot);

        default_reply_.success(true);
        default_reply_.status_message("SetModelState: set model state done");
    }
    return default_reply_;
}

gazebo_msgs::srv::Default_Response ApiPlugin::set_link_state(
        gazebo_msgs::srv::SetLinkState_Request request)
{
    // Obtain link and frame
    gazebo::physics::Link *body = dynamic_cast<gazebo::physics::Link *>(
            utils::get_entity(world_, request.link_state().link_name()).get());
    gazebo::physics::EntityPtr frame
            = utils::get_entity(world_, request.link_state().reference_frame());

    if (!body) {
        default_reply_.success(false);
        default_reply_.status_message("SetLinkState: link not found");
    }

    // Manage the information of the request
    ignition::math::Vector3d target_pos(
            request.link_state().pose().position().x(),
            request.link_state().pose().position().y(),
            request.link_state().pose().position().z());
    ignition::math::Quaterniond target_rot(
            request.link_state().pose().orientation().w(),
            request.link_state().pose().orientation().x(),
            request.link_state().pose().orientation().y(),
            request.link_state().pose().orientation().z());
    ignition::math::Pose3d target_pose(target_pos, target_rot);
    ignition::math::Vector3d target_linear_vel(
            request.link_state().twist().linear().x(),
            request.link_state().twist().linear().y(),
            request.link_state().twist().linear().z());
    ignition::math::Vector3d target_angular_vel(
            request.link_state().twist().angular().x(),
            request.link_state().twist().angular().y(),
            request.link_state().twist().angular().z());

    if (frame) {
        // Obtain the current state of the frame
        get_entity_state(frame, frame_pose_, frame_vpos_, frame_veul_);

        ignition::math::Vector3d frame_pos = frame_pose_.Pos();
        ignition::math::Quaterniond frame_rot = frame_pose_.Rot();

        target_pose = target_pose + frame_pose_;

        target_linear_vel -= frame_vpos_;
        target_angular_vel -= frame_veul_;
    } else if (
            request.link_state().reference_frame() != ""
            && request.link_state().reference_frame() != "world"
            && request.link_state().reference_frame() != "map") {
        default_reply_.success(false);
        default_reply_.status_message("SetLinkState: failed");
    }

    // Set link pose
    bool is_paused = world_->IsPaused();
    world_->SetPaused(true);
    body->SetWorldPose(target_pose);
    world_->SetPaused(is_paused);

    // Set link velocity 
    body->SetLinearVel(target_linear_vel);
    body->SetAngularVel(target_angular_vel);

    default_reply_.success(true);
    default_reply_.status_message("SetLinkState: success");

    return default_reply_;
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

ignition::math::Vector3d
        ApiPlugin::get_link_inertial(gazebo::physics::Link *link)
{
    gazebo::physics::InertialPtr inertia = link->GetInertial();

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

void ApiPlugin::get_entity_state(
        gazebo::physics::EntityPtr &entity,
        ignition::math::Pose3d &entity_pose,
        ignition::math::Vector3d &entity_linear_vel,
        ignition::math::Vector3d &entity_angular_vel)
{
#if GAZEBO_MAJOR_VERSION >= 8
    entity_pose = entity->WorldPose();
    entity_linear_vel = entity->WorldLinearVel();
    entity_angular_vel = entity->WorldAngularVel();
#else
    entity_pose = entity->GetWorldPose().Ign();
    entity_linear_vel = entity->GetWorldLinearVel().Ign();
    entity_angular_vel = entity->GetWorldAngularVel().Ign();
#endif
}

void ApiPlugin::get_entity_pose(
        gazebo::physics::EntityPtr &entity,
        ignition::math::Pose3d &entity_pose)
{
#if GAZEBO_MAJOR_VERSION >= 8
    entity_pose = entity->WorldPose();
#else
    entity_pose = entity->GetWorldPose().Ign();
#endif
}

void ApiPlugin::get_joint_type(gazebo::physics::JointPtr joint)
{
    if (joint->GetMsgType() == msgs::Joint::REVOLUTE) {
        joint_properties_reply_.type(gazebo_msgs::srv::Type::REVOLUTE);
    } else if (joint->GetMsgType() == msgs::Joint::PRISMATIC) {
        joint_properties_reply_.type(gazebo_msgs::srv::Type::PRISMATIC);
    } else if (joint->GetMsgType() == msgs::Joint::UNIVERSAL) {
        joint_properties_reply_.type(gazebo_msgs::srv::Type::UNIVERSAL);
    } else if (joint->GetMsgType() == msgs::Joint::BALL) {
        joint_properties_reply_.type(gazebo_msgs::srv::Type::BALL);
    } else if (joint->GetMsgType() == msgs::Joint::FIXED) {
        joint_properties_reply_.type(gazebo_msgs::srv::Type::FIXED);
    }
}

}  // namespace dds
}  // namespace gazebo
