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


#include <iostream>
#include <regex>

#include <gazebo/sensors/ImuSensor.hh>

#include <rti/domain/find.hpp>
#include <dds/dds.hpp>

#include "common/GazeboDdsUtils.cxx"
#include "common/Properties.h"
#include "ImuScan.h"

namespace gazebo { namespace dds {

GZ_REGISTER_SENSOR_PLUGIN(ImuScan)

ImuScan::ImuScan()
        : SensorPlugin(),
          participant_(::dds::core::null),
          topic_(::dds::core::null),
          writer_(::dds::core::null),
          seed_(0)
{
}

ImuScan::~ImuScan()
{
}

void ImuScan::Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
    // Store the pointer to the sensor
    sensor_ = dynamic_cast<gazebo::sensors::ImuSensor *>(parent.get());

    gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gazebo_node_->Init(parent->WorldName());

    // Obtain Imu's information from loaded world
    utils::get_world_parameter<double>(
            sdf, gaussian_noise_, "gaussian_noise", 0.0);

    if (sdf->HasElement("rpy_offset")) {
        offset_.Rot() = ignition::math::Quaterniond(
                sdf->Get<ignition::math::Vector3d>("rpy_offset"));
    } else {
        offset_.Rot() = ignition::math::Quaterniond::Identity;
        gzwarn << "Missing <rpy_offset>, set to default: "
               << offset_.Rot().Roll() << ' ' << offset_.Rot().Pitch() << ' '
               << offset_.Rot().Yaw() << std::endl;
    }

    // Obtain the qos profile information from loaded world
    std::string qos_profile_file;
    utils::get_world_parameter<std::string>(
            sdf, qos_profile_file, QOS_PROFILE_FILE_PROPERTY_NAME.c_str(), "");

    std::string qos_profile;
    utils::get_world_parameter<std::string>(
            sdf, qos_profile, QOS_PROFILE_PROPERTY_NAME.c_str(), "");

    ::dds::core::QosProvider qos_provider
            = ::dds::core::QosProvider(qos_profile_file);

    // Obtain the domain id from loaded world
    int domain_id;
    utils::get_world_parameter<int>(
            sdf, domain_id, DOMAIN_ID_PROPERTY_NAME.c_str(), 0);

    utils::create_participant(domain_id, participant_, qos_provider, qos_profile);

    // Obtain the topic name from loaded world
    std::string topic_name;
    utils::get_world_parameter<std::string>(
            sdf, topic_name, TOPIC_NAME_PROPERTY_NAME.c_str(), "ImuScan");

    topic_ = ::dds::topic::find<::dds::topic::Topic<sensor_msgs::msg::Imu>>(
            participant_, topic_name);
    if (topic_ == ::dds::core::null) {
        topic_ = ::dds::topic::Topic<sensor_msgs::msg::Imu>(
                participant_, topic_name);
    }
    
    writer_ = ::dds::pub::DataWriter<sensor_msgs::msg::Imu>(
            ::dds::pub::Publisher(
                    participant_, qos_provider.publisher_qos(qos_profile)),
            topic_,
            qos_provider.datawriter_qos(qos_profile));

    // Generate gazebo topic name
    std::string gazebo_topic_name = "/gazebo/" + sensor_->ScopedName() + "/imu";
    gazebo_topic_name
            = std::regex_replace(gazebo_topic_name, std::regex("::"), "/");

    this->imu_scan_sub_ = this->gazebo_node_->Subscribe(
            gazebo_topic_name, &ImuScan::on_scan, this);

    // Init constant information
    sample_.header().frame_id(sensor_->ParentName());

    double covariance = gaussian_noise_ * gaussian_noise_;
    sample_.orientation_covariance()[0] = covariance;
    sample_.orientation_covariance()[4] = covariance;
    sample_.orientation_covariance()[8] = covariance;
    sample_.angular_velocity_covariance()[0] = covariance;
    sample_.angular_velocity_covariance()[4] = covariance;
    sample_.angular_velocity_covariance()[8] = covariance;
    sample_.linear_acceleration_covariance()[0] = covariance;
    sample_.linear_acceleration_covariance()[4] = covariance;
    sample_.linear_acceleration_covariance()[8] = covariance;

    gzmsg << "Starting imu plugin" << std::endl;
    gzmsg << "* Publications:" << std::endl;
    gzmsg << "  - " << topic_name << " [sensor_msgs/msg/Imu]" << std::endl;
}

void ImuScan::on_scan(ConstIMUPtr &msg)
{
    // Applying offsets to the orientation measurement
    orientation_ = offset_.Rot() * sensor_->Orientation();

    sample_.header().stamp().sec(msg->stamp().sec());
    sample_.header().stamp().nanosec(msg->stamp().nsec());

    sample_.orientation().x(
            orientation_.X() + guassian_kernel(0, gaussian_noise_));
    sample_.orientation().y(
            orientation_.Y() + guassian_kernel(0, gaussian_noise_));
    sample_.orientation().z(
            orientation_.Z() + guassian_kernel(0, gaussian_noise_));
    sample_.orientation().w(
            orientation_.W() + guassian_kernel(0, gaussian_noise_));

    sample_.linear_acceleration().x(
            msg->linear_acceleration().x()
            + guassian_kernel(0, gaussian_noise_));
    sample_.linear_acceleration().y(
            msg->linear_acceleration().y()
            + guassian_kernel(0, gaussian_noise_));
    sample_.linear_acceleration().z(
            msg->linear_acceleration().z()
            + guassian_kernel(0, gaussian_noise_));

    sample_.angular_velocity().x(
            msg->angular_velocity().x() + guassian_kernel(0, gaussian_noise_));
    sample_.angular_velocity().y(
            msg->angular_velocity().y() + guassian_kernel(0, gaussian_noise_));
    sample_.angular_velocity().z(
            msg->angular_velocity().z() + guassian_kernel(0, gaussian_noise_));

    writer_.write(sample_);
}

double ImuScan::guassian_kernel(double mu, double sigma)
{
    // generation of two normalized uniform random variables
    double first_uniform_rand = static_cast<double>(rand_r(&seed_))
            / static_cast<double>(RAND_MAX);
    double second_uniform_rand = static_cast<double>(rand_r(&seed_))
            / static_cast<double>(RAND_MAX);

    // using Box-Muller transform to obtain a variable with a standard normal
    // distribution
    double standard_normal = sqrt(-2.0 * ::log(first_uniform_rand)) * 
            cos(2.0 * M_PI * second_uniform_rand);

    // scaling
    standard_normal = sigma * standard_normal + mu;
    return standard_normal;
}

}  // namespace dds
}  // namespace gazebo
