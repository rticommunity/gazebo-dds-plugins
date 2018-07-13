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

#ifndef DIFF_DRIVE_H
#define DIFF_DRIVE_H

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <dds/core/ddscore.hpp>
#include <dds/dds.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>

#include "geometry_msgs/msg/Twist.hpp"
#include "nav_msgs/msg/Odometry.hpp"
#include "sensor_msgs/msg/JointState.hpp"

namespace gazebo { namespace dds {

const int WHEEL_NUMBER = 2;

class DiffDrive : public ModelPlugin {
public:
    /**
     * @brief Constructor
     */
    DiffDrive();

    /**
     * @brief Destructor
     */
    ~DiffDrive();

    /**
     * @brief Load the plugin inside Gazebo's system
     *
     * @param parent object of Gazebo's model
     * @param sdf object of Gazebo's world
     */
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override;

    /**
     * @brief Reset model information
     */
    void Reset() override;

protected:
    /**
     * @brief Update model
     */
    void update_model();

private:
    /**
     * @brief Publish odometry sample
     */
    void publish_odometry();

    /**
     * @brief Publish JointState sample
     */
    void publish_joint_state();

    /**
     * @brief Obtain velocity of the wheels
     * 
     * msg Message with the new twist of the model
     */
    void get_wheel_velocities(const geometry_msgs::msg::Twist &msg);

    /**
     * @brief Update odometry when it is in encoder mode
     */
    void update_odometry_encoder();

    /**
     * @brief Obtain pose3d of the world
     * 
     * @return pose3d of the world
     */
    ignition::math::Pose3d get_world_pose();

    /**
     * @brief Obtain position of the specific joint
     * 
     * @param index index of the joint in the array of Joint
     * @return position of the specific joint
     */
    double get_joint_position(int index);

    /**
     * @brief Obtain position of the specific joint
     */
    void get_world_velocity();

private:
    ::dds::domain::DomainParticipant participant_;
    ::dds::topic::Topic<nav_msgs::msg::Odometry> topic_odometry_;
    ::dds::topic::Topic<sensor_msgs::msg::JointState> topic_joint_state_;
    ::dds::topic::Topic<geometry_msgs::msg::Twist> topic_twist_;
    ::dds::pub::DataWriter<nav_msgs::msg::Odometry> writer_odometry_;
    ::dds::pub::DataWriter<sensor_msgs::msg::JointState> writer_joint_state_;
    ::dds::sub::DataReader<geometry_msgs::msg::Twist> reader_;
    ::dds::sub::LoanedSamples<geometry_msgs::msg::Twist> twist_samples_;
    nav_msgs::msg::Odometry odometry_sample_;
    sensor_msgs::msg::JointState joint_state_sample_;
    std::string odom_source_;
    physics::JointPtr joints_[WHEEL_NUMBER];
    ignition::math::Quaterniond odometry_orientation_;
    ignition::math::Vector3d pose_encoder_;
    ignition::math::Pose3d world_pose_;
    ignition::math::Vector3d world_linear_;
    common::Time last_update_;
    common::Time last_odom_update_;
    common::Time current_time_;
    physics::ModelPtr parent_;
    event::ConnectionPtr update_connection_;
    double wheel_separation_;
    double wheel_diameter_;
    double wheel_accel_;
    double wheel_torque_;
    double wheel_speed_[WHEEL_NUMBER];
    double wheel_speed_instr_[WHEEL_NUMBER];
    double update_period_;
    bool legacy_mode_;
};

}  // namespace dds
}  // namespace gazebo

#endif  // DIFF_DRIVE_H
