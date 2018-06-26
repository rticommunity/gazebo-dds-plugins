/*
 * Copyright 2018 Real-Time Innovations, Inc.
 * Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "common/GazeboDdsUtils.cxx"
#include "common/Properties.h"
#include "SkidSteerDrive.h"

namespace gazebo { namespace dds {

enum WHEEL_POSITION_ENUM {
    RIGHT_FRONT = 0,
    LEFT_FRONT = 1,
    RIGHT_REAR = 2,
    LEFT_REAR = 3,
};

GZ_REGISTER_MODEL_PLUGIN(SkidSteerDrive)

SkidSteerDrive::SkidSteerDrive()
        : participant_(::dds::core::null),
          topic_odometry_(::dds::core::null),
          topic_joint_state_(::dds::core::null),
          topic_twist_(::dds::core::null),
          writer_odometry_(::dds::core::null),
          writer_joint_state_(::dds::core::null),
          reader_(::dds::core::null)
{
}

SkidSteerDrive::~SkidSteerDrive()
{
}

void SkidSteerDrive::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    parent_ = parent;

    wheel_speed_[RIGHT_FRONT] = 0;
    wheel_speed_[LEFT_FRONT] = 0;
    wheel_speed_[RIGHT_REAR] = 0;
    wheel_speed_[LEFT_REAR] = 0;

    // Obtain information of the plugin from loaded world
    utils::get_world_parameter<double>(
            sdf, wheel_separation_, "wheel_separation", 0.34);
    utils::get_world_parameter<double>(
            sdf, wheel_diameter_, "wheel_diameter", 0.15);
    utils::get_world_parameter<double>(sdf, wheel_torque_, "wheel_torque", 5.0);
    utils::get_world_parameter<double>(
            sdf, covariance_x_, "covariance_x", 0.0001);
    utils::get_world_parameter<double>(
            sdf, covariance_y_, "covariance_y", 0.0001);
    utils::get_world_parameter<double>(
            sdf, covariance_yaw_, "covariance_yaw", 0.01);
    utils::get_world_parameter<double>(
            sdf, update_period_, "update_rate", 100.0);

    if (update_period_ > 0.0) {
        update_period_ = 1.0 / update_period_;
    } else {
        update_period_ = 0.0;
    }

    // Obtain time of the simulation
    last_update_ = utils::get_sim_time(parent_->GetWorld());

    // Obtain joints from loaded world
    joints_[LEFT_FRONT]
            = utils::get_joint(sdf, parent_, "left_front_joint", "left_front");
    joints_[RIGHT_FRONT] = utils::get_joint(
            sdf, parent_, "right_front_joint", "right_front");
    joints_[LEFT_REAR]
            = utils::get_joint(sdf, parent_, "left_rear_joint", "left_rear");
    joints_[RIGHT_REAR]
            = utils::get_joint(sdf, parent_, "right_rear_joint", "right_rear");

    joints_[LEFT_FRONT]->SetParam("fmax", 0, wheel_torque_);
    joints_[RIGHT_FRONT]->SetParam("fmax", 0, wheel_torque_);
    joints_[LEFT_REAR]->SetParam("fmax", 0, wheel_torque_);
    joints_[RIGHT_REAR]->SetParam("fmax", 0, wheel_torque_);

    // Obtain the domain id from loaded world
    int domain_id;
    utils::get_world_parameter<int>(
            sdf, domain_id, DOMAIN_ID_PROPERTY_NAME.c_str(), 0);

    participant_ = ::dds::domain::find(domain_id);
    if (participant_ == ::dds::core::null) {
        participant_ = ::dds::domain::DomainParticipant(domain_id);
    }

    // Obtain odometry topic name from loaded world
    std::string topic_name_odometry;
    utils::get_world_parameter<std::string>(
            sdf, topic_name_odometry, "topic_name_odometry", "OdometryState");

    topic_odometry_
            = ::dds::topic::find<::dds::topic::Topic<nav_msgs::msg::Odometry>>(
                    participant_, topic_name_odometry);
    if (topic_odometry_ == ::dds::core::null) {
        topic_odometry_ = ::dds::topic::Topic<nav_msgs::msg::Odometry>(
                participant_, topic_name_odometry);
    }

    writer_odometry_ = ::dds::pub::DataWriter<nav_msgs::msg::Odometry>(
            ::dds::pub::Publisher(participant_), topic_odometry_);

    // Obtain jointState topic name from loaded world
    std::string topic_name_joint;
    utils::get_world_parameter<std::string>(
            sdf, topic_name_joint, "topic_name_joint", "JointState");

    topic_joint_state_ = ::dds::topic::find<
            ::dds::topic::Topic<sensor_msgs::msg::JointState>>(
            participant_, topic_name_joint);
    if (topic_joint_state_ == ::dds::core::null) {
        topic_joint_state_ = ::dds::topic::Topic<sensor_msgs::msg::JointState>(
                participant_, topic_name_joint);
    }

    writer_joint_state_ = ::dds::pub::DataWriter<sensor_msgs::msg::JointState>(
            ::dds::pub::Publisher(participant_), topic_joint_state_);

    // Obtain twist topic name from loaded world
    std::string topic_name_twist;
    utils::get_world_parameter<std::string>(
            sdf, topic_name_twist, "topic_name_twist", "cmd_vel");

    topic_twist_ = ::dds::topic::find<
            ::dds::topic::Topic<geometry_msgs::msg::Twist>>(
            participant_, topic_name_twist);
    if (topic_twist_ == ::dds::core::null) {
        topic_twist_ = ::dds::topic::Topic<geometry_msgs::msg::Twist>(
                participant_, topic_name_twist);
    }

    rti::core::policy::Property::Entry value(
            { "dds.data_reader.history.depth", "1" });

    rti::core::policy::Property property;
    property.set(value);
    data_reader_qos_ << property;
    reader_ = ::dds::sub::DataReader<geometry_msgs::msg::Twist>(
            ::dds::sub::Subscriber(participant_),
            topic_twist_,
            data_reader_qos_);

    // Init samples
    joint_state_sample_.name().resize(WHEEL_NUMBER);
    joint_state_sample_.position().resize(WHEEL_NUMBER);
    joint_state_sample_.header().frame_id(parent_->GetName() + "/joint");

    odometry_sample_.header().frame_id(parent_->GetName() + "/odometry");
    odometry_sample_.child_frame_id(parent_->GetName() + "/chassis");

    // listen to the world update event
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&SkidSteerDrive::update_model, this));

    gzmsg << "Starting skid steer drive plugin" << std::endl;
    gzmsg << "* Publications:" << std::endl;
    gzmsg << "  - " << topic_name_odometry << " [nav_msgs/msg/Odometry]"
          << std::endl;
    gzmsg << "  - " << topic_name_joint << " [sensor_msgs/msg/JointState]"
          << std::endl;
    gzmsg << "* Subscriptions:" << std::endl;
    gzmsg << "  - " << topic_name_twist << " [geometry_msgs/msg/Twist]"
          << std::endl;
}

void SkidSteerDrive::Reset()
{
    last_update_ = utils::get_sim_time(parent_->GetWorld());

    for (unsigned int i = 0; i < WHEEL_NUMBER; i++) {
        joints_[i]->SetParam("fmax", 0, wheel_torque_);
    }
}

void SkidSteerDrive::update_model()
{
    for (unsigned int i = 0; i < WHEEL_NUMBER; i++) {
        if (fabs(wheel_torque_ - joints_[i]->GetParam("fmax", 0)) > 1e-6) {
            joints_[i]->SetParam("fmax", 0, wheel_torque_);
        }
    }

    current_time_ = utils::get_sim_time(parent_->GetWorld());
    double diff_time = (current_time_ - last_update_).Double();

    if (diff_time > update_period_) {
        publish_odometry();
        publish_joint_state();

        twist_samples_ = reader_.read();

        if (twist_samples_.length() > 0 && twist_samples_[0].info().valid()) {
            get_wheel_velocities(twist_samples_[0].data());
        }

        // Update joints velocity
        joints_[LEFT_FRONT]->SetParam(
                "vel", 0, wheel_speed_[LEFT_FRONT] / (wheel_diameter_ / 2.0));
        joints_[RIGHT_FRONT]->SetParam(
                "vel", 0, wheel_speed_[RIGHT_FRONT] / (wheel_diameter_ / 2.0));
        joints_[LEFT_REAR]->SetParam(
                "vel", 0, wheel_speed_[LEFT_REAR] / (wheel_diameter_ / 2.0));
        joints_[RIGHT_REAR]->SetParam(
                "vel", 0, wheel_speed_[RIGHT_REAR] / (wheel_diameter_ / 2.0));

        last_update_ += common::Time(update_period_);
    }
}

void SkidSteerDrive::get_wheel_velocities(const geometry_msgs::msg::Twist &msg)
{
    double linear_x = msg.linear().x();
    double angular_z = msg.angular().z();

    wheel_speed_[LEFT_FRONT] = linear_x - angular_z * wheel_separation_ / 2.0;
    wheel_speed_[LEFT_REAR] = linear_x - angular_z * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT_FRONT] = linear_x + angular_z * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT_REAR] = linear_x + angular_z * wheel_separation_ / 2.0;
}

void SkidSteerDrive::publish_odometry()
{
    current_time_ = utils::get_sim_time(parent_->GetWorld());

    odometry_sample_.header().stamp().sec(current_time_.sec);
    odometry_sample_.header().stamp().nanosec(current_time_.nsec);

    world_pose_ = get_world_pose();

    odometry_sample_.pose().pose().position().x(world_pose_.Pos().X());
    odometry_sample_.pose().pose().position().y(world_pose_.Pos().Y());

    odometry_sample_.pose().pose().orientation().x(world_pose_.Rot().X());
    odometry_sample_.pose().pose().orientation().y(world_pose_.Rot().Y());
    odometry_sample_.pose().pose().orientation().z(world_pose_.Rot().Z());
    odometry_sample_.pose().pose().orientation().w(world_pose_.Rot().W());

    // get velocity in /odom frame
    get_world_velocity();

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = world_pose_.Rot().Yaw();
    odometry_sample_.twist().twist().linear().x(
            cosf(yaw) * world_linear_.X() + sinf(yaw) * world_linear_.Y());
    odometry_sample_.twist().twist().linear().y(
            cosf(yaw) * world_linear_.Y() - sinf(yaw) * world_linear_.X());

    // set covariance
    odometry_sample_.pose().covariance()[0] = covariance_x_;
    odometry_sample_.pose().covariance()[7] = covariance_y_;
    odometry_sample_.pose().covariance()[14] = 1000000000000.0;
    odometry_sample_.pose().covariance()[21] = 1000000000000.0;
    odometry_sample_.pose().covariance()[28] = 1000000000000.0;
    odometry_sample_.pose().covariance()[35] = covariance_yaw_;

    odometry_sample_.twist().covariance()[0] = covariance_x_;
    odometry_sample_.twist().covariance()[7] = covariance_y_;
    odometry_sample_.twist().covariance()[14] = 1000000000000.0;
    odometry_sample_.twist().covariance()[21] = 1000000000000.0;
    odometry_sample_.twist().covariance()[28] = 1000000000000.0;
    odometry_sample_.twist().covariance()[35] = covariance_yaw_;

    writer_odometry_.write(odometry_sample_);
}

void SkidSteerDrive::publish_joint_state()
{
    current_time_ = utils::get_sim_time(parent_->GetWorld());

    joint_state_sample_.header().stamp().sec(current_time_.sec);
    joint_state_sample_.header().stamp().nanosec(current_time_.nsec);

    for (unsigned int i = 0; i < WHEEL_NUMBER; i++) {
        joint_state_sample_.name()[i] = joints_[i]->GetName();
        joint_state_sample_.position()[i] = get_joint_position(i);
    }

    writer_joint_state_.write(joint_state_sample_);
}

inline ignition::math::Pose3d SkidSteerDrive::get_world_pose()
{
#if GAZEBO_MAJOR_VERSION >= 8
    return parent_->WorldPose();
#else
    return parent_->GetWorldPose().Ign();
#endif
}

inline double SkidSteerDrive::get_joint_position(int index)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return joints_[index]->Position(0);
#else
    return joints_[index]->GetAngle(0).Radian();
#endif
}

inline void SkidSteerDrive::get_world_velocity()
{
#if GAZEBO_MAJOR_VERSION >= 8
    world_linear_ = parent_->WorldLinearVel();
    odometry_sample_.twist().twist().angular().z(
            parent_->WorldAngularVel().Z());
#else
    world_linear_ = parent_->GetWorldLinearVel().Ign();
    odometry_sample_.twist().twist().angular().z(
            parent_->GetWorldAngularVel().Ign().Z());
#endif
}

}  // namespace dds
}  // namespace gazebo
