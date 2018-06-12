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

#include <gazebo/gazebo.hh>
#include <gazebo/plugins/RayPlugin.hh>

#include <dds/core/ddscore.hpp>
#include <dds/domain/find.hpp>
#include <dds/pub/ddspub.hpp>

#include "sensor_msgs/msg/LaserScanMsg.hpp"

namespace gazebo { namespace dds {

class LaserScan : public RayPlugin {
public:
    /**
     * @brief Constructor
     */
    LaserScan();

    /**
     * @brief Destructor
     */
    ~LaserScan();

    /**
     * @brief Load the plugin inside Gazebo's system
     *
     * @param parent object of Gazebo's sensor
     * @param sdf object of Gazebo's world
     */
    void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override;

    /**
     * @brief Read the current sensor's information
     *
     * @param msg current information of the sensor
     */
    void on_scan(ConstLaserScanStampedPtr &msg);

private:
    sensors::SensorPtr sensor_;
    ::dds::domain::DomainParticipant participant_;
    ::dds::topic::Topic<sensor_msgs::msg::LaserScanMsg> topic_;
    ::dds::pub::qos::DataWriterQos data_writer_qos_;
    ::dds::pub::DataWriter<sensor_msgs::msg::LaserScanMsg> writer_;
    gazebo::transport::NodePtr gazebo_node_;
    gazebo::transport::SubscriberPtr laser_scan_sub_;
    sensor_msgs::msg::LaserScanMsg sample_;
};

}  // namespace dds
}  // namespace gazebo
