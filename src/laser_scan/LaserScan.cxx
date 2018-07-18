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
#include "common/GazeboUtils.hpp"
#include "common/DdsUtils.hpp"
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

    // Obtain the qos profile information from loaded world
    std::string qos_profile_file;
    utils::get_world_parameter<std::string>(
            sdf, qos_profile_file, QOS_PROFILE_FILE_PROPERTY_NAME.c_str(), "");
    
    std::string qos_profile;
    utils::get_world_parameter<std::string>(
            sdf, qos_profile, QOS_PROFILE_PROPERTY_NAME.c_str(), "");

    ::dds::core::QosProvider qos_provider(::dds::core::null);
    if(qos_profile_file != "" || qos_profile !=""){
        qos_provider
            = ::dds::core::QosProvider(qos_profile_file, qos_profile);
    }
    else{
        qos_provider
            = ::dds::core::QosProvider::Default();
    }

    // Obtain the domain id from loaded world
    int domain_id;
    utils::get_world_parameter<int>(
            sdf, domain_id, DOMAIN_ID_PROPERTY_NAME.c_str(), 0);

    utils::find_domain_participant(
            domain_id, participant_, qos_provider, qos_profile);

    // Obtain the topic name from loaded world
    std::string topic_name;
    utils::get_world_parameter<std::string>(
            sdf, topic_name, TOPIC_NAME_PROPERTY_NAME.c_str(), "laserScan");

    utils::find_topic<sensor_msgs::msg::LaserScan>(
            participant_, topic_, topic_name);

    ::dds::pub::qos::DataWriterQos data_writer_qos
            = qos_provider.datawriter_qos();
    ::dds::pub::qos::PublisherQos publisher_qos = qos_provider.publisher_qos();

    // Change the maximum size of the sequences
    utils::set_unbounded_sequence_allocated_size(data_writer_qos, 4096);

    utils::create_datawriter<sensor_msgs::msg::LaserScan>(
            writer_, participant_, topic_, data_writer_qos, publisher_qos);

    this->laser_scan_sub_ = this->gazebo_node_->Subscribe(
            this->sensor_->Topic(), &LaserScan::on_scan, this);

    gzmsg << std::endl;
    gzmsg << "Starting laser plugin"<< std::endl;
    gzmsg << "* Publications:" << std::endl;
    gzmsg << "  - " << topic_name << " [sensor_msgs/msg/LaserScan]" 
          << std::endl;
}

void LaserScan::on_scan(ConstLaserScanStampedPtr &msg)
{
    sample_.header().stamp().sec(msg->time().sec());
    sample_.header().stamp().nanosec(msg->time().nsec());
    sample_.header().frame_id(sensor_->ParentName());

    sample_.angle_min(msg->scan().angle_min());
    sample_.angle_max(msg->scan().angle_max());
    sample_.angle_increment(msg->scan().angle_step());
    sample_.time_increment(0);
    sample_.scan_time(0);  
    sample_.range_min(msg->scan().range_min());
    sample_.range_max(msg->scan().range_max());

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
