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

#ifndef API_PLUGIN_H
#define API_PLUGIN_H

#include <gazebo/gazebo.hh>

#include <dds/core/ddscore.hpp>
#include <dds/dds.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <rti/request/rtirequest.hpp>

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
#include "gazebo_msgs/srv/GetPhysicsProperties_Response.hpp"
#include "gazebo_msgs/srv/GetWorldProperties_Response.hpp"
#include "gazebo_msgs/srv/SetLightProperties_Request.hpp"
#include "std_msgs/msg/Empty.hpp"

#include "common/ReplierListener.hpp"

namespace gazebo { namespace dds {

class ApiPlugin : public WorldPlugin {
public:
    /**
     * @brief Constructor
     */
    ApiPlugin();

    /**
     * @brief Destructor
     */
    ~ApiPlugin();

    /**
     * @brief Load the plugin inside Gazebo's system
     *
     * @param parent object of Gazebo's sensor
     * @param sdf object of Gazebo's world
     */
    void Load(physics::WorldPtr parent, sdf::ElementPtr sdf) override;

    gazebo_msgs::srv::Default_Response
            delete_model(gazebo_msgs::srv::DeleteModel_Request);

    gazebo_msgs::srv::Default_Response
            delete_light(gazebo_msgs::srv::DeleteLight_Request request);

    gazebo_msgs::srv::GetLightProperties_Response get_light_properties(
            gazebo_msgs::srv::GetLightProperties_Request request);

    gazebo_msgs::srv::GetWorldProperties_Response get_world_properties(
            std_msgs::msg::Empty request);

    gazebo_msgs::srv::GetJointProperties_Response get_joint_properties(
            gazebo_msgs::srv::GetJointProperties_Request request);

    gazebo_msgs::srv::GetLinkProperties_Response get_link_properties(
            gazebo_msgs::srv::GetLinkProperties_Request request);

    gazebo_msgs::srv::GetLinkState_Response get_link_state(
            gazebo_msgs::srv::GetLinkState_Request request);

    gazebo_msgs::srv::GetModelProperties_Response get_model_properties(
            gazebo_msgs::srv::GetModelProperties_Request request);

    gazebo_msgs::srv::GetModelState_Response get_model_state(
            gazebo_msgs::srv::GetModelState_Request request);

    gazebo_msgs::srv::Default_Response set_light_properties(
            gazebo_msgs::srv::SetLightProperties_Request request);

    gazebo_msgs::srv::Default_Response
            reset_simulation(std_msgs::msg::Empty request);

    gazebo_msgs::srv::Default_Response
            reset_world(std_msgs::msg::Empty request);

    gazebo_msgs::srv::Default_Response
            pause_physics(std_msgs::msg::Empty request);
    
    gazebo_msgs::srv::Default_Response
            unpause_physics(std_msgs::msg::Empty request);

private:
    
    void get_world_models();

    void get_joint_position(gazebo::physics::JointPtr joint);

    ignition::math::Vector3d get_link_inertial(
            gazebo::physics::Link *link,
            gazebo::physics::InertialPtr inertia);

    void get_entity_state(
            gazebo::physics::EntityPtr &entity,
            ignition::math::Pose3d &entity_pose,
            ignition::math::Vector3d &entity_linear_vel,
            ignition::math::Vector3d &entity_angular_vel);

private:
    ::dds::domain::DomainParticipant participant_;

    rti::request::Replier<
            gazebo_msgs::srv::DeleteModel_Request,
            gazebo_msgs::srv::Default_Response>
            delete_model_replier_;
    ReplierListener<
            gazebo_msgs::srv::DeleteModel_Request,
            gazebo_msgs::srv::Default_Response>
            delete_model_listener_;

    rti::request::Replier<
            gazebo_msgs::srv::DeleteLight_Request,
            gazebo_msgs::srv::Default_Response>
            delete_light_replier_;
    ReplierListener<
            gazebo_msgs::srv::DeleteLight_Request,
            gazebo_msgs::srv::Default_Response>
            delete_light_listener_;

    rti::request::Replier<
            gazebo_msgs::srv::GetLightProperties_Request,
            gazebo_msgs::srv::GetLightProperties_Response>
            get_light_properties_replier_;
    ReplierListener<
            gazebo_msgs::srv::GetLightProperties_Request,
            gazebo_msgs::srv::GetLightProperties_Response>
            get_light_properties_listener_;

    rti::request::Replier<
            std_msgs::msg::Empty,
            gazebo_msgs::srv::GetWorldProperties_Response>
            get_world_properties_replier_;
    ReplierListener<
            std_msgs::msg::Empty,
            gazebo_msgs::srv::GetWorldProperties_Response>
            get_world_properties_listener_;

    rti::request::Replier<
            gazebo_msgs::srv::GetJointProperties_Request,
            gazebo_msgs::srv::GetJointProperties_Response>
            get_joint_properties_replier_;
    ReplierListener<
            gazebo_msgs::srv::GetJointProperties_Request,
            gazebo_msgs::srv::GetJointProperties_Response>
            get_joint_properties_listener_;

    rti::request::Replier<
            gazebo_msgs::srv::GetLinkProperties_Request,
            gazebo_msgs::srv::GetLinkProperties_Response>
            get_link_properties_replier_;
    ReplierListener<
            gazebo_msgs::srv::GetLinkProperties_Request,
            gazebo_msgs::srv::GetLinkProperties_Response>
            get_link_properties_listener_;

    rti::request::Replier<
            gazebo_msgs::srv::GetLinkState_Request,
            gazebo_msgs::srv::GetLinkState_Response>
            get_link_state_replier_;
    ReplierListener<
            gazebo_msgs::srv::GetLinkState_Request,
            gazebo_msgs::srv::GetLinkState_Response>
            get_link_state_listener_;

    rti::request::Replier<
            gazebo_msgs::srv::GetModelProperties_Request,
            gazebo_msgs::srv::GetModelProperties_Response>
            get_model_properties_replier_;
    ReplierListener<
            gazebo_msgs::srv::GetModelProperties_Request,
            gazebo_msgs::srv::GetModelProperties_Response>
            get_model_properties_listener_;

    rti::request::Replier<
            gazebo_msgs::srv::GetModelState_Request,
            gazebo_msgs::srv::GetModelState_Response>
            get_model_state_replier_;
    ReplierListener<
            gazebo_msgs::srv::GetModelState_Request,
            gazebo_msgs::srv::GetModelState_Response>
            get_model_state_listener_;

    rti::request::Replier<
            gazebo_msgs::srv::SetLightProperties_Request,
            gazebo_msgs::srv::Default_Response>
            set_light_properties_replier_;
    ReplierListener<
            gazebo_msgs::srv::SetLightProperties_Request,
            gazebo_msgs::srv::Default_Response>
            set_light_properties_listener_;

    rti::request::
            Replier<std_msgs::msg::Empty, gazebo_msgs::srv::Default_Response>
                    reset_simulation_replier_;
    ReplierListener<std_msgs::msg::Empty, gazebo_msgs::srv::Default_Response>
            reset_simulation_listener_;

    rti::request::
            Replier<std_msgs::msg::Empty, gazebo_msgs::srv::Default_Response>
                    reset_world_replier_;
    ReplierListener<std_msgs::msg::Empty, gazebo_msgs::srv::Default_Response>
            reset_world_listener_;

    rti::request::
            Replier<std_msgs::msg::Empty, gazebo_msgs::srv::Default_Response>
                    pause_physics_replier_;
    ReplierListener<std_msgs::msg::Empty, gazebo_msgs::srv::Default_Response>
            pause_physics_listener_;

    rti::request::
            Replier<std_msgs::msg::Empty, gazebo_msgs::srv::Default_Response>
                    unpause_physics_replier_;
    ReplierListener<std_msgs::msg::Empty, gazebo_msgs::srv::Default_Response>
            unpause_physics_listener_;

    gazebo_msgs::srv::Default_Response default_reply_;
    gazebo_msgs::srv::GetLightProperties_Response light_properties_reply_;
    gazebo_msgs::srv::GetWorldProperties_Response world_properties_reply_;
    gazebo_msgs::srv::GetJointProperties_Response joint_properties_reply_;
    gazebo_msgs::srv::GetLinkProperties_Response link_properties_reply_;
    gazebo_msgs::srv::GetModelProperties_Response model_properties_reply_;
    gazebo_msgs::srv::GetModelState_Response model_state_reply_;
    gazebo_msgs::srv::GetLinkState_Response link_state_reply_;

    ignition::math::Pose3d entity_pose_;
    ignition::math::Pose3d frame_pose_;
    ignition::math::Vector3d entity_vpos_;
    ignition::math::Vector3d entity_veul_;
    ignition::math::Vector3d frame_vpos_;
    ignition::math::Vector3d frame_veul_;

    gazebo::transport::PublisherPtr gazebo_pub_;
    gazebo::transport::PublisherPtr light_modify_pub_;
    gazebo::transport::NodePtr gazebo_node_;
    common::Time current_time_;
    physics::WorldPtr world_;
};

}  // namespace dds
}  // namespace gazebo

#endif  // API_PLUGIN_H
