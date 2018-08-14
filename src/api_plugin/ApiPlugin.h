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
#include "gazebo_msgs/srv/GetWorldProperties_Request.hpp"
#include "gazebo_msgs/srv/GetWorldProperties_Response.hpp"
#include "gazebo_msgs/srv/GetPhysicsProperties_Response.hpp"
#include "gazebo_msgs/srv/GetModelProperties_Request.hpp"
#include "gazebo_msgs/srv/GetModelProperties_Response.hpp"

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
            gazebo_msgs::srv::GetWorldProperties_Request request);

    gazebo_msgs::srv::GetJointProperties_Response get_joint_properties(
            gazebo_msgs::srv::GetJointProperties_Request request);

    gazebo_msgs::srv::GetLinkProperties_Response get_link_properties(
            gazebo_msgs::srv::GetLinkProperties_Request request);

    gazebo_msgs::srv::GetModelProperties_Response get_model_properties(
            gazebo_msgs::srv::GetModelProperties_Request request);

private:
    ::dds::domain::DomainParticipant participant_;

    rti::request::Replier<
            gazebo_msgs::srv::DeleteModel_Request,
            gazebo_msgs::srv::Default_Response>
            delete_model_replier_;

    ::dds::sub::LoanedSamples<gazebo_msgs::srv::DeleteModel_Request>
            delete_model_requests_;
    ReplierListener<
            gazebo_msgs::srv::DeleteModel_Request,
            gazebo_msgs::srv::Default_Response>
            delete_model_listener_;

    rti::request::Replier<
            gazebo_msgs::srv::DeleteLight_Request,
            gazebo_msgs::srv::Default_Response>
            delete_light_replier_;

    ::dds::sub::LoanedSamples<gazebo_msgs::srv::DeleteLight_Request>
            delete_light_requests_;
    ReplierListener<
            gazebo_msgs::srv::DeleteLight_Request,
            gazebo_msgs::srv::Default_Response>
            delete_light_listener_;

    rti::request::Replier<
            gazebo_msgs::srv::GetLightProperties_Request,
            gazebo_msgs::srv::GetLightProperties_Response>
            get_light_properties_replier_;

    ::dds::sub::LoanedSamples<gazebo_msgs::srv::GetLightProperties_Request>
            get_light_properties_requests_;
    ReplierListener<
            gazebo_msgs::srv::GetLightProperties_Request,
            gazebo_msgs::srv::GetLightProperties_Response>
            get_light_properties_listener_;

    rti::request::Replier<
            gazebo_msgs::srv::GetWorldProperties_Request,
            gazebo_msgs::srv::GetWorldProperties_Response>
            get_world_properties_replier_;

    ::dds::sub::LoanedSamples<gazebo_msgs::srv::GetWorldProperties_Request>
            get_world_properties_requests_;
    ReplierListener<
            gazebo_msgs::srv::GetWorldProperties_Request,
            gazebo_msgs::srv::GetWorldProperties_Response>
            get_world_properties_listener_;

    rti::request::Replier<
            gazebo_msgs::srv::GetJointProperties_Request,
            gazebo_msgs::srv::GetJointProperties_Response>
            get_joint_properties_replier_;

    ::dds::sub::LoanedSamples<gazebo_msgs::srv::GetJointProperties_Request>
            get_joint_properties_requests_;
    ReplierListener<
            gazebo_msgs::srv::GetJointProperties_Request,
            gazebo_msgs::srv::GetJointProperties_Response>
            get_joint_properties_listener_;

    rti::request::Replier<
            gazebo_msgs::srv::GetLinkProperties_Request,
            gazebo_msgs::srv::GetLinkProperties_Response>
            get_link_properties_replier_;

    ::dds::sub::LoanedSamples<gazebo_msgs::srv::GetLinkProperties_Request>
            get_link_properties_requests_;
    ReplierListener<
            gazebo_msgs::srv::GetLinkProperties_Request,
            gazebo_msgs::srv::GetLinkProperties_Response>
            get_link_properties_listener_;

    rti::request::Replier<
            gazebo_msgs::srv::GetModelProperties_Request,
            gazebo_msgs::srv::GetModelProperties_Response>
            get_model_properties_replier_;

    ::dds::sub::LoanedSamples<gazebo_msgs::srv::GetModelProperties_Request>
            get_model_properties_requests_;
    ReplierListener<
            gazebo_msgs::srv::GetModelProperties_Request,
            gazebo_msgs::srv::GetModelProperties_Response>
            get_model_properties_listener_;


    physics::WorldPtr world_;
};

}  // namespace dds
}  // namespace gazebo

#endif  // API_PLUGIN_H
