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

#include "geometry_msgs/msg/Wrench.hpp"
#include "gazebo_msgs/srv/Default_Response.hpp"
#include "gazebo_msgs/srv/DeleteModel_Request.hpp"

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

    gazebo_msgs::srv::Default_Response delete_model(gazebo_msgs::srv::DeleteModel_Request);

private:
    ::dds::domain::DomainParticipant participant_;

    rti::request::Replier<
            gazebo_msgs::srv::DeleteModel_Request,
            gazebo_msgs::srv::Default_Response>
            replier_;

    ::dds::sub::LoanedSamples<gazebo_msgs::srv::DeleteModel_Request> requests_;
    ReplierListener<
            gazebo_msgs::srv::DeleteModel_Request,
            gazebo_msgs::srv::Default_Response>
            listener_;

    physics::WorldPtr parent_;
};

}  // namespace dds
}  // namespace gazebo

#endif  // API_PLUGIN_H
