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

#include <gazebo/sensors/ContactSensor.hh>

#include "common/GazeboDdsUtils.cxx"
#include "common/Properties.h"
#include "BumperScan.h"

namespace gazebo { namespace dds {

GZ_REGISTER_SENSOR_PLUGIN(BumperScan);

BumperScan::BumperScan()
        : ContactPlugin(),
          participant_(::dds::core::null),
          topic_(::dds::core::null),
          writer_(::dds::core::null)
{
}

BumperScan::~BumperScan()
{
}

void BumperScan::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
    sensor_ = dynamic_cast<gazebo::sensors::ContactSensor *>(parent.get());

    // Obtain the domain id from loaded world
    int domain_id;
    utils::get_world_parameter<int>(
            sdf, domain_id, DOMAIN_ID_PROPERTY_NAME.c_str(), 0);

    participant_ = ::dds::domain::find(domain_id);
    if (participant_ == ::dds::core::null) {
        participant_ = ::dds::domain::DomainParticipant(domain_id);
    }

    // Obtain topic name from loaded world
    std::string topic_name;
    utils::get_world_parameter<std::string>(
            sdf, topic_name, TOPIC_NAME_PROPERTY_NAME.c_str(), "bumper_scan");

    topic_ = ::dds::topic::Topic<gazebo_msgs::msg::ContactsState>(
            participant_, topic_name);

    writer_ = ::dds::pub::DataWriter<gazebo_msgs::msg::ContactsState>(
            ::dds::pub::Publisher(participant_), topic_);

    sensor_connection_
            = sensor_->ConnectUpdated(std::bind(&BumperScan::on_scan, this));

    gzmsg << "Starting Bumper plugin" << std::endl;
    gzmsg << "* Publications:" << std::endl;
    gzmsg << "  - " << topic_name << " [gazebo_msgs/msg/ContactsState]" 
          << std::endl;
}

void BumperScan::on_scan()
{
    current_time_ = sensor_->LastUpdateTime();
    contacts_sample_.header().stamp().sec(current_time_.sec);
    contacts_sample_.header().stamp().nanosec(current_time_.nsec);

    frame_rot_ = ignition::math::Quaterniond(1, 0, 0, 0);

    // Obtain contacts
    unsigned int contacts_size = sensor_->Contacts().contact_size();
    unsigned int contact_group_size;
    contacts_sample_.states().resize(contacts_size);

    for (unsigned int i = 0; i < contacts_size; ++i) {
        contact_ = sensor_->Contacts().contact(i);

        contact_state_msg_.collision1_name(contact_.collision1());
        contact_state_msg_.collision2_name(contact_.collision2());

        std::ostringstream stream;
        stream << "Info:  i:(" << i << "/" << contacts_size
               << ")     my geom:" << contact_state_msg_.collision1_name()
               << "   other geom:" << contact_state_msg_.collision2_name()
               << "         time:" << contact_.time().sec() << ", "
               << contact_.time().nsec() << std::endl;
        contact_state_msg_.info(stream.str());

        total_wrench_msg_.force().x(0);
        total_wrench_msg_.force().y(0);
        total_wrench_msg_.force().z(0);
        total_wrench_msg_.torque().x(0);
        total_wrench_msg_.torque().y(0);
        total_wrench_msg_.torque().z(0);

        // Obtain contact group
        contact_group_size = contact_.position_size();
        contact_state_msg_.wrenches().resize(contact_group_size);
        contact_state_msg_.contact_positions().resize(contact_group_size);
        contact_state_msg_.contact_normals().resize(contact_group_size);
        contact_state_msg_.depths().resize(contact_group_size);

        for (unsigned int j = 0; j < contact_group_size; ++j) {
            // Calculate force, torque
            contact_force_
                    = frame_rot_.RotateVectorReverse(ignition::math::Vector3d(
                            contact_.wrench(j).body_1_wrench().force().x(),
                            contact_.wrench(j).body_1_wrench().force().y(),
                            contact_.wrench(j).body_1_wrench().force().z()));
            contact_torque_
                    = frame_rot_.RotateVectorReverse(ignition::math::Vector3d(
                            contact_.wrench(j).body_1_wrench().torque().x(),
                            contact_.wrench(j).body_1_wrench().torque().y(),
                            contact_.wrench(j).body_1_wrench().torque().z()));

            // Set wrenches
            wrench_msg_.force().x(contact_force_.X());
            wrench_msg_.force().y(contact_force_.Y());
            wrench_msg_.force().z(contact_force_.Z());
            wrench_msg_.torque().x(contact_torque_.X());
            wrench_msg_.torque().y(contact_torque_.Y());
            wrench_msg_.torque().z(contact_torque_.Z());
            contact_state_msg_.wrenches()[j] = wrench_msg_;

            total_wrench_msg_.force().x(
                    total_wrench_msg_.force().x() + wrench_msg_.force().x());
            total_wrench_msg_.force().y(
                    total_wrench_msg_.force().y() + wrench_msg_.force().y());
            total_wrench_msg_.force().z(
                    total_wrench_msg_.force().z() + wrench_msg_.force().z());
            total_wrench_msg_.torque().x(
                    total_wrench_msg_.torque().x() + wrench_msg_.torque().x());
            total_wrench_msg_.torque().y(
                    total_wrench_msg_.torque().y() + wrench_msg_.torque().y());
            total_wrench_msg_.torque().z(
                    total_wrench_msg_.torque().z() + wrench_msg_.torque().z());

            // Set contact positions
            contact_position_
                    = frame_rot_.RotateVectorReverse(ignition::math::Vector3d(
                            contact_.position(j).x(),
                            contact_.position(j).y(),
                            contact_.position(j).z()));

            contact_position_msg_.x(contact_position_.X());
            contact_position_msg_.y(contact_position_.Y());
            contact_position_msg_.z(contact_position_.Z());
            contact_state_msg_.contact_positions()[j] = contact_position_msg_;

            // Set contact normal
            contact_normal_
                    = frame_rot_.RotateVectorReverse(ignition::math::Vector3d(
                            contact_.normal(j).x(),
                            contact_.normal(j).y(),
                            contact_.normal(j).z()));

            contact_normal_msg_.x(contact_normal_.X());
            contact_normal_msg_.y(contact_normal_.Y());
            contact_normal_msg_.z(contact_normal_.Z());
            contact_state_msg_.contact_normals()[j] = contact_normal_msg_;

            // Set contact depth
            contact_state_msg_.depths()[j] = contact_.depth(j);
        }

        contact_state_msg_.total_wrench() = total_wrench_msg_;
        contacts_sample_.states()[i] = contact_state_msg_;
    }

    writer_.write(contacts_sample_);
}

}  // namespace dds
}  // namespace gazebo
