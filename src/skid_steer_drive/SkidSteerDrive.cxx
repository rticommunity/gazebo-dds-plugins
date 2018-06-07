#include <algorithm>
#include <assert.h>

#include "SkidSteerDrive.h"
#include "common/GazeboDdsUtils.cxx"
#include "common/Properties.h"

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/GetMap.h>
// #include <nav_msgs/Odometry.h>

namespace gazebo { 
namespace dds {

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

// Destructor
SkidSteerDrive::~SkidSteerDrive()
{
}

// Load the controller
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

    topic_odometry_ = ::dds::topic::Topic<nav_msgs::msg::Odometry>(
            participant_, topic_name_odometry);

    writer_odometry_ = ::dds::pub::DataWriter<nav_msgs::msg::Odometry>(
            ::dds::pub::Publisher(participant_), topic_odometry_);

    // Obtain jointState topic name from loaded world
    std::string topic_name_joint;
    utils::get_world_parameter<std::string>(
            sdf, topic_name_joint, "topic_name_joint", "JointState");

    topic_joint_state_ = ::dds::topic::Topic<sensor_msgs::msg::JointState>(
            participant_, topic_name_joint);

    writer_joint_state_ = ::dds::pub::DataWriter<sensor_msgs::msg::JointState>(
            ::dds::pub::Publisher(participant_), topic_joint_state_);

    // Obtain twist topic name from loaded world
    std::string topic_name_twist;
    utils::get_world_parameter<std::string>(
            sdf, topic_name_twist, "topic_name_twist", "cmd_vel");

    topic_twist_ = ::dds::topic::Topic<geometry_msgs::msg::Twist>(
            participant_, topic_name_twist);

    rti::core::policy::Property::Entry value(
            { "dds.data_reader.history.depth", "1" });

    rti::core::policy::Property property;
    property.set(value);
    data_reader_qos_ << property;
    reader_ = ::dds::sub::DataReader<geometry_msgs::msg::Twist>(
            ::dds::sub::Subscriber(participant_),
            topic_twist_,
            data_reader_qos_);

    // listen to the world update event
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&SkidSteerDrive::update_model, this));

    gzmsg << "Starting Skid Steer drive Plugin" << std::endl;
    gzmsg << "- Odometry topic name: " << topic_name_odometry << std::endl;
    gzmsg << "- JointState topic name: " << topic_name_joint << std::endl;
    gzmsg << "- Twist topic name: " << topic_name_twist << std::endl;
}

void SkidSteerDrive::Reset()
{
    last_update_ = utils::get_sim_time(parent_->GetWorld());

    joints_[LEFT_FRONT]->SetParam("fmax", 0, wheel_torque_);
    joints_[RIGHT_FRONT]->SetParam("fmax", 0, wheel_torque_);
    joints_[LEFT_REAR]->SetParam("fmax", 0, wheel_torque_);
    joints_[RIGHT_REAR]->SetParam("fmax", 0, wheel_torque_);
}

void SkidSteerDrive::update_model()
{
}

void SkidSteerDrive::get_wheel_velocities(const geometry_msgs::msg::Twist &msg)
{
    wheel_speed_[LEFT_FRONT]
            = msg.linear().x() - msg.angular().z() * wheel_separation_ / 2.0;
    wheel_speed_[LEFT_REAR]
            = msg.linear().x() - msg.angular().z() * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT_FRONT]
            = msg.linear().x() + msg.angular().z() * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT_REAR]
            = msg.linear().x() + msg.angular().z() * wheel_separation_ / 2.0;
}

void SkidSteerDrive::publish_odometry()
{
}

}  // namespace dds
}  // namespace gazebo
