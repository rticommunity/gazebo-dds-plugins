/*
 * Copyright 2018 Real-Time Innovations, Inc.
 * Copyright 2012 Open Source Robotics Foundation 
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
 *
*/

#ifndef BUMPER_SCAN_H
#define BUMPER_SCAN_H

#include <gazebo/gazebo.hh>
#include <gazebo/plugins/ContactPlugin.hh>

#include <dds/core/ddscore.hpp>
#include <dds/dds.hpp>
#include <dds/pub/ddspub.hpp>

#include "gazebo_msgs/msg/ContactState.hpp"
#include "gazebo_msgs/msg/ContactsState.hpp"

namespace gazebo { namespace dds {

class BumperScan : public ContactPlugin {
public:
    /**
     * @brief Constructor
     */
    BumperScan();

    /**
     * @brief Destructor
     */
    virtual ~BumperScan();

    /**
     * @brief Load the plugin inside Gazebo's system
     *
     * @param parent object of Gazebo's sensor
     * @param sdf object of Gazebo's world
     */
    void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override;

private:
    /**
     * @brief Publish the current information of the contact sensor 
     */
    void on_scan();

private:
    ::dds::domain::DomainParticipant participant_;
    ::dds::topic::Topic<gazebo_msgs::msg::ContactsState> topic_;
    ::dds::pub::DataWriter<gazebo_msgs::msg::ContactsState> writer_;
    sensors::ContactSensor* sensor_;
    gazebo_msgs::msg::ContactsState contacts_sample_;
    gazebo_msgs::msg::ContactState contact_state_msg_;
    gazebo::msgs::Contact contact_;
    geometry_msgs::msg::Wrench total_wrench_msg_;
    geometry_msgs::msg::Wrench wrench_msg_;
    geometry_msgs::msg::Vector3 contact_position_msg_;
    geometry_msgs::msg::Vector3 contact_normal_msg_;
    event::ConnectionPtr sensor_connection_;
    common::Time current_time_;
    ignition::math::Quaterniond frame_rot_;
    ignition::math::Vector3d contact_force_;
    ignition::math::Vector3d contact_torque_;
    ignition::math::Vector3d contact_position_;
    ignition::math::Vector3d contact_normal_;
};

}  // namespace dds
}  // namespace gazebo

#endif  // BUMPER_SCAN_H
