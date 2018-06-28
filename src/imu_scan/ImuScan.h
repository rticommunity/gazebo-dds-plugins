/* 
 * Copyright 2018 Real-Time Innovations, Inc.
 * Copyright [2015] [Alessandro Settimi]
 * 
 * email: ale.settimi@gmail.com
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>

#include <dds/core/ddscore.hpp>
#include <dds/pub/ddspub.hpp>

#include "sensor_msgs/msg/Imu.hpp"

namespace gazebo { namespace dds {

class ImuScan : public SensorPlugin {
public:
    /**
     * @brief Constructor
     */
    ImuScan();

    /**
     * @brief Destructor
     */
    ~ImuScan();

    /**
     * @brief Load the plugin inside Gazebo's system
     *
     * @param parent object of Gazebo's sensor
     * @param sdf object of Gazebo's world
     */
    void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override;

private:
    /**
     * @brief Read the current sensor's information
     *
     * @param msg current information of the sensor
     */
    void on_scan(ConstIMUPtr & msg);
    
    /**
     * @brief Gaussian noise generator
     *
     * @param mu offset value
     * @param sigma scaling value
     */
    double guassian_kernel(double mu, double sigma);

private:
    sensors::ImuSensor* sensor_;
    transport::NodePtr gazebo_node_;
    transport::SubscriberPtr imu_scan_sub_;
    ignition::math::Pose3d offset_;
    ignition::math::Quaterniond orientation_;
    ::dds::domain::DomainParticipant participant_;
    ::dds::topic::Topic<sensor_msgs::msg::Imu> topic_;
    ::dds::pub::DataWriter<sensor_msgs::msg::Imu> writer_;
    sensor_msgs::msg::Imu sample_;
    double gaussian_noise_;
    unsigned int seed_;
};

}  // namespace dds
}  // namespace gazebo
