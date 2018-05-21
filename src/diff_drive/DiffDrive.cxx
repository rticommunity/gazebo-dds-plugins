#include <iostream>

#include <gazebo/physics/World.hh>

#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "DiffDrive.h"
#include "common/DataReaderListener.cxx"
#include "common/GazeboDdsUtils.cxx"
#include "common/Properties.h"

namespace gazebo { namespace dds {

enum {
    RIGHT,
    LEFT,
};


GZ_REGISTER_MODEL_PLUGIN(DiffDrive)

DiffDrive::DiffDrive()
        : participant_(::dds::core::null),
          topic_odometry_(::dds::core::null),
          topic_joint_state_(::dds::core::null),
          topic_twist_(::dds::core::null),
          writer_odometry_(::dds::core::null),
          writer_joint_state_(::dds::core::null),
          reader_(::dds::core::null)
{
}

DiffDrive::~DiffDrive()
{
}

void DiffDrive::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    this->parent = parent;

    // Initialize velocity and velocity support
    wheel_speed_[RIGHT] = 0;
    wheel_speed_[LEFT] = 0;

    wheel_speed_instr_[RIGHT] = 0;
    wheel_speed_instr_[LEFT] = 0;

    // Obtain joints from loaded world
    utils::get_world_parameter<double>(
            sdf, wheel_separation_, "wheel_separation", 0.34);
    utils::get_world_parameter<double>(
            sdf, wheel_diameter_, "wheel_diameter", 0.15);
    utils::get_world_parameter<double>(
            sdf, wheel_accel_, "wheel_acceleration", 0.0);
    utils::get_world_parameter<double>(
            sdf, wheel_torque_, "wheel_torque", 5.0);
    utils::get_world_parameter<std::string>(
            sdf, odom_source_, "odometry_source", "world");
    utils::get_world_parameter<std::string>(
            sdf, odom_source_, "odometry_source", "world");
    utils::get_world_parameter<bool>(sdf, legacy_mode_, "legacy_mode", true);



    // Obtain joints from loaded world
    joints_.resize(2);
    joints_[LEFT]
            = utils::get_joint(sdf, parent, "leftJoint", "left_joint");
    joints_[RIGHT] = utils::get_joint(
            sdf, parent, "rightJoint", "right_joint");
    joints_[LEFT]->SetParam("fmax", 0, wheel_torque_);
    joints_[RIGHT]->SetParam("fmax", 0, wheel_torque_);


    // Obtain the domain id from loaded world
    int domain_id;
    utils::get_world_parameter<int>(
            sdf, domain_id, DOMAIN_ID_PROPERTY_NAME.c_str(), 0);

    participant_ = ::dds::domain::find(domain_id);
    if (participant_ == ::dds::core::null) {
        participant_ = ::dds::domain::DomainParticipant(domain_id);
    }

    // Obtain the Odometry topic name from loaded world
    std::string topic_name_odometry;
    utils::get_world_parameter<std::string>(
            sdf, topic_name_odometry, "topic_name_odometry", "OdometryState");

    topic_odometry_ = ::dds::topic::Topic<nav_msgs::msg::Odometry>(
            participant_, topic_name_odometry);

    writer_odometry_ = ::dds::pub::DataWriter<nav_msgs::msg::Odometry>(
            ::dds::pub::Publisher(participant_), topic_odometry_);

    // Obtain the JointState topic name from loaded world
    std::string topic_name_joint;
    utils::get_world_parameter<std::string>(
            sdf, topic_name_joint, "topic_name_joint", "JointState");

    topic_joint_state_ = ::dds::topic::Topic<sensor_msgs::msg::JointState>(
            participant_, topic_name_joint);

    writer_joint_state_ = ::dds::pub::DataWriter<sensor_msgs::msg::JointState>(
            ::dds::pub::Publisher(participant_), topic_joint_state_);

    // Obtain the Twist topic name from loaded world
    std::string topic_name_twist;
    utils::get_world_parameter<std::string>(
            sdf, topic_name_twist, "topic_name_twist", "cmd_vel");

    topic_twist_ = ::dds::topic::Topic<geometry_msgs::msg::Twist>(
            participant_, topic_name_twist);

    reader_ = ::dds::sub::DataReader<geometry_msgs::msg::Twist>(
            ::dds::sub::Subscriber(participant_), topic_twist_);

    reader_.listener(
            new DataReaderListener<geometry_msgs::msg::Twist>(std::bind(
                    &DiffDrive::cmd_callback, this, std::placeholders::_1)),
            ::dds::core::status::StatusMask::data_available());

    // Init samples
    joint_state_sample_.name().resize(2);
    joint_state_sample_.position().resize(2);

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DiffDrive::UpdateChild, this));

    gzmsg << "Starting Differential drive Plugin" << std::endl;
    gzmsg << "- Odometry topic name: " << topic_name_odometry << std::endl;
    gzmsg << "- JointState topic name: " << topic_name_joint << std::endl;
    gzmsg << "- Twist topic name: " << topic_name_twist << std::endl;
}

void DiffDrive::UpdateChild()
{
    publish_odometry();
    publish_joint_state();
}

void DiffDrive::publish_odometry()
{
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->RealTime();
#else
    common::Time current_time = parent->GetWorld()->GetRealTime();
#endif

    // FIX changes local variables
    ignition::math::Quaterniond orientation;
    ignition::math::Vector3d position;

    if (odom_source_ == "encoder") {
        // getting data form encoder integration
        orientation = ignition::math::Quaterniond(0.0, 0.0, 0.0, 0.0);
        position = ignition::math::Vector3d(
                odometry_sample_.pose().pose().position().x(),
                odometry_sample_.pose().pose().position().y(),
                odometry_sample_.pose().pose().position().z());
    }
    if (odom_source_ == "world") {
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = parent->WorldPose();
#else
        ignition::math::Pose3d pose = parent->GetWorldPose().Ign();
#endif
        orientation = ignition::math::Quaterniond(
                pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
        position = ignition::math::Vector3d(
                pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

        odometry_sample_.pose().pose().position().x(position.X());
        odometry_sample_.pose().pose().position().y(position.Y());
        odometry_sample_.pose().pose().position().z(position.Z());

        odometry_sample_.pose().pose().orientation().x(orientation.X());
        odometry_sample_.pose().pose().orientation().y(orientation.Y());
        odometry_sample_.pose().pose().orientation().z(orientation.Z());
        odometry_sample_.pose().pose().orientation().w(orientation.W());

        // get velocity in /odom frame
        ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
        linear = parent->WorldLinearVel();
        odometry_sample_.twist().twist().angular().z(
                parent->WorldAngularVel().Z());
#else
        linear = parent->GetWorldLinearVel().Ign();
        odometry_sample_.twist().twist().angular().z(
                parent->GetWorldAngularVel().Ign().Z());
#endif

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.Rot().Yaw();
        odometry_sample_.twist().twist().linear().x(
                cosf(yaw) * linear.X() + sinf(yaw) * linear.Y());
        odometry_sample_.twist().twist().linear().y(
                cosf(yaw) * linear.Y() - sinf(yaw) * linear.X());
    }

    // set covariance
    odometry_sample_.pose().covariance()[0] = 0.00001;
    odometry_sample_.pose().covariance()[7] = 0.00001;
    odometry_sample_.pose().covariance()[14] = 1000000000000.0;
    odometry_sample_.pose().covariance()[21] = 1000000000000.0;
    odometry_sample_.pose().covariance()[28] = 1000000000000.0;
    odometry_sample_.pose().covariance()[35] = 0.001;

    // set header
    odometry_sample_.header().stamp().sec(current_time.sec);
    odometry_sample_.header().stamp().nanosec(current_time.nsec);
    // FIX frame_id

    writer_odometry_.write(odometry_sample_);
}

void DiffDrive::publish_joint_state()
{
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->RealTime();
#else
    common::Time current_time = parent->GetWorld()->GetRealTime();
#endif

    joint_state_sample_.header().stamp().sec(current_time.sec);
    joint_state_sample_.header().stamp().nanosec(current_time.nsec);
    // FIX frame_id

    for (int i = 0; i < 2; i++) {
#if GAZEBO_MAJOR_VERSION >= 8
        double position = joints_[i]->Position(0);
#else
        double position = joints_[i]->GetAngle(0).Radian();
#endif

        joint_state_sample_.name()[i] = joints_[i]->GetName();
        joint_state_sample_.position()[i] = position;
    }

    writer_joint_state_.write(joint_state_sample_);
}

void DiffDrive::cmd_callback(const geometry_msgs::msg::Twist &msg)
{
}

}  // namespace dds
}  // namespace gazebo
