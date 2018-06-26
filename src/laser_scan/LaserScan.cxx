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
 */

#include "common/Properties.h"
#include "common/GazeboDdsUtils.cxx"
#include "LaserScan.h"

namespace gazebo { namespace dds {

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(LaserScan)

LaserScan::LaserScan()
        : participant_(::dds::core::null),
          topic_(::dds::core::null),
          writer_(::dds::core::null)
{
}

LaserScan::~LaserScan()
{
}

void LaserScan::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
    // load plugin
    RayPlugin::Load(parent, sdf);

    // Store the pointer to the sensor
    sensor_ = parent;

    gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gazebo_node_->Init(parent->WorldName());

    // Obtain the domain id from loaded world
    int domain_id;
    utils::get_world_parameter<int>(
            sdf, domain_id, DOMAIN_ID_PROPERTY_NAME.c_str(), 0);

    participant_ = ::dds::domain::find(domain_id);
    if (participant_ == ::dds::core::null) {
        participant_ = ::dds::domain::DomainParticipant(domain_id);
    }

    // Obtain the topic name from loaded world
    std::string topic_name;
    utils::get_world_parameter<std::string>(
            sdf, topic_name, TOPIC_NAME_PROPERTY_NAME.c_str(), "laserScan");

    topic_ = ::dds::topic::find<
            ::dds::topic::Topic<sensor_msgs::msg::LaserScanMsg>>(
            participant_, topic_name);
    if (topic_ == ::dds::core::null) {
        topic_ = ::dds::topic::Topic<sensor_msgs::msg::LaserScanMsg>(
                participant_, topic_name);
    }

    // Change the maximum size of the sequences
    rti::core::policy::Property::Entry value(
            { "dds.data_writer.history.memory_manager.fast_pool.pool_"
              "buffer_max_size",
              "4096" });

    rti::core::policy::Property property;
    property.set(value);
    data_writer_qos_ << property;
    writer_ = ::dds::pub::DataWriter<sensor_msgs::msg::LaserScanMsg>(
            ::dds::pub::Publisher(participant_), topic_, data_writer_qos_);

    this->laser_scan_sub_ = this->gazebo_node_->Subscribe(
            this->sensor_->Topic(), &LaserScan::on_scan, this);

    gzmsg << "Starting laser plugin"<< std::endl;
    gzmsg << "* Publications:" << std::endl;
    gzmsg << "  - " << topic_name << " [sensor_msgs/msg/LaserScanMsg]" 
          << std::endl;
}

void LaserScan::on_scan(ConstLaserScanStampedPtr &msg)
{
    sample_.laser_id(sensor_->Id());
    sample_.header().stamp().sec(msg->time().sec());
    sample_.header().stamp().nanosec(msg->time().nsec());
    sample_.header().frame_id(sensor_->ParentName());

    sample_.world_pose().position().x(msg->scan().world_pose().position().x());
    sample_.world_pose().position().y(msg->scan().world_pose().position().y());
    sample_.world_pose().position().z(msg->scan().world_pose().position().z());

    sample_.world_pose().orientation().x(
            msg->scan().world_pose().orientation().x());
    sample_.world_pose().orientation().y(
            msg->scan().world_pose().orientation().y());
    sample_.world_pose().orientation().z(
            msg->scan().world_pose().orientation().z());
    sample_.world_pose().orientation().w(
            msg->scan().world_pose().orientation().w());

    sample_.angle_min(msg->scan().angle_min());
    sample_.angle_max(msg->scan().angle_max());
    sample_.angle_step(msg->scan().angle_step());
    sample_.range_min(msg->scan().range_min());
    sample_.range_max(msg->scan().range_max());
    sample_.count(msg->scan().count());
    sample_.vertical_angle_min(msg->scan().vertical_angle_min());
    sample_.vertical_angle_max(msg->scan().vertical_angle_max());
    sample_.vertical_angle_step(msg->scan().vertical_angle_step());
    sample_.vertical_count(msg->scan().vertical_count());

    sample_.ranges().resize(msg->scan().ranges_size());
    std::copy(
            msg->scan().ranges().begin(),
            msg->scan().ranges().end(),
            sample_.ranges().begin());

    sample_.intensities().resize(msg->scan().intensities_size());
    std::copy(
            msg->scan().intensities().begin(),
            msg->scan().intensities().end(),
            sample_.intensities().begin());

    writer_.write(sample_);
}

}  // namespace dds
}  // namespace gazebo
