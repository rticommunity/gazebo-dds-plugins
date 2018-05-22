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

    wheel_speed_[RIGHT] = 0;
    wheel_speed_[LEFT] = 0;

    wheel_speed_instr_[RIGHT] = 0;
    wheel_speed_instr_[LEFT] = 0;

    position_wheel_ = 0.0;
    rotation_wheel_ = 0.0;

    // Obtain joints from loaded world
    utils::get_world_parameter<double>(
            sdf, wheel_separation_, "wheel_separation", 0.34);
    utils::get_world_parameter<double>(
            sdf, wheel_diameter_, "wheel_diameter", 0.15);
    utils::get_world_parameter<double>(
            sdf, wheel_accel_, "wheel_acceleration", 0.0);
    utils::get_world_parameter<double>(sdf, wheel_torque_, "wheel_torque", 5.0);
    utils::get_world_parameter<std::string>(
            sdf, odom_source_, "odometry_source", "world");
    utils::get_world_parameter<bool>(sdf, legacy_mode_, "legacy_mode", true);
    utils::get_world_parameter<double>(
            sdf, update_period_, "update_rate", 100.0);

    if (update_period_ > 0.0) {
        update_period_ = 1.0 / update_period_;
    } else {
        update_period_ = 0.0;
    }

// Obtain the time of simulation
#if GAZEBO_MAJOR_VERSION >= 8
    last_update_ = parent->GetWorld()->SimTime();
#else
    last_update_ = parent->GetWorld()->GetSimTime();
#endif

    // Obtain joints from loaded world
    joints_.resize(2);
    joints_[LEFT] = utils::get_joint(sdf, parent, "leftJoint", "left_joint");
    joints_[RIGHT] = utils::get_joint(sdf, parent, "rightJoint", "right_joint");
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

void DiffDrive::Reset()
{
#if GAZEBO_MAJOR_VERSION >= 8
    last_update_ = parent->GetWorld()->SimTime();
#else
    last_update_ = parent->GetWorld()->GetSimTime();
#endif
    pose_encoder_.X() = 0;
    pose_encoder_.Y() = 0;
    pose_encoder_.Z() = 0;
    position_wheel_ = 0;
    rotation_wheel_ = 0;
    joints_[LEFT]->SetParam("fmax", 0, wheel_torque_);
    joints_[RIGHT]->SetParam("fmax", 0, wheel_torque_);
}

void DiffDrive::UpdateChild()
{
    // Joint::Reset is called after ModelPlugin::Reset, so we need to set
    // maxForce to wheel_torque other than GazeboRosDiffDrive::Reset
    for (int i = 0; i < 2; i++) {
        if (fabs(wheel_torque_ - joints_[i]->GetParam("fmax", 0)) > 1e-6) {
            joints_[i]->SetParam("fmax", 0, wheel_torque_);
        }
    }

    if (odom_source_ == "encoder")
        update_odometry_encoder();
#if GAZEBO_MAJOR_VERSION >= 8
    current_time_ = parent->GetWorld()->SimTime();
#else
    current_time_ = parent->GetWorld()->GetSimTime();
#endif

    diff_time_ = (current_time_ - last_update_).Double();

    if (diff_time_ > update_period_) {
        publish_odometry();
        publish_joint_state();

        ::dds::sub::LoanedSamples<geometry_msgs::msg::Twist> samples
                = reader_.read();
        if (samples.length() > 0) {
            if (samples[0].info().valid()) {
                get_wheel_velocities(samples[0].data());
            }
        }

        double current_speed[2];

        current_speed[LEFT]
                = joints_[LEFT]->GetVelocity(0) * (wheel_diameter_ / 2.0);
        current_speed[RIGHT]
                = joints_[RIGHT]->GetVelocity(0) * (wheel_diameter_ / 2.0);

        if (wheel_accel_ == 0
            || (fabs(wheel_speed_[LEFT] - current_speed[LEFT]) < 0.01)
            || (fabs(wheel_speed_[RIGHT] - current_speed[RIGHT]) < 0.01)) {
            // if max_accel == 0, or target speed is reached
            joints_[LEFT]->SetParam(
                    "vel", 0, wheel_speed_[LEFT] / (wheel_diameter_ / 2.0));
            joints_[RIGHT]->SetParam(
                    "vel", 0, wheel_speed_[RIGHT] / (wheel_diameter_ / 2.0));
        } else {
            if (wheel_speed_[LEFT] >= current_speed[LEFT]) {
                wheel_speed_instr_[LEFT]
                        += fmin(wheel_speed_[LEFT] - current_speed[LEFT],
                                wheel_accel_ * diff_time_);
            } else {
                wheel_speed_instr_[LEFT]
                        += fmax(wheel_speed_[LEFT] - current_speed[LEFT],
                                -wheel_accel_ * diff_time_);
            }

            if (wheel_speed_[RIGHT] > current_speed[RIGHT]) {
                wheel_speed_instr_[RIGHT]
                        += fmin(wheel_speed_[RIGHT] - current_speed[RIGHT],
                                wheel_accel_ * diff_time_);
            } else {
                wheel_speed_instr_[RIGHT]
                        += fmax(wheel_speed_[RIGHT] - current_speed[RIGHT],
                                -wheel_accel_ * diff_time_);
            }
            joints_[LEFT]->SetParam(
                    "vel",
                    0,
                    wheel_speed_instr_[LEFT] / (wheel_diameter_ / 2.0));
            joints_[RIGHT]->SetParam(
                    "vel",
                    0,
                    wheel_speed_instr_[RIGHT] / (wheel_diameter_ / 2.0));
        }
        last_update_ += common::Time(update_period_);
    }
}

void DiffDrive::publish_odometry()
{
#if GAZEBO_MAJOR_VERSION >= 8
    current_time_ = parent->GetWorld()->RealTime();
#else
    current_time_ = parent->GetWorld()->GetRealTime();
#endif

    // FIX no local variable
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
        ignition::math::Vector3d linear;  // FIX no local variable
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
        float yaw = pose.Rot().Yaw();  // FIX no local variable
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
    odometry_sample_.header().stamp().sec(current_time_.sec);
    odometry_sample_.header().stamp().nanosec(current_time_.nsec);
    // FIX frame_id

    writer_odometry_.write(odometry_sample_);
}

void DiffDrive::publish_joint_state()
{
#if GAZEBO_MAJOR_VERSION >= 8
    current_time_ = parent->GetWorld()->RealTime();
#else
    current_time_ = parent->GetWorld()->GetRealTime();
#endif

    joint_state_sample_.header().stamp().sec(current_time_.sec);
    joint_state_sample_.header().stamp().nanosec(current_time_.nsec);
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

void DiffDrive::get_wheel_velocities(const geometry_msgs::msg::Twist &msg)
{
    position_wheel_ = msg.linear().x();
    rotation_wheel_ = msg.angular().z();

    if (legacy_mode_) {
        wheel_speed_[LEFT]
                = position_wheel_ + rotation_wheel_ * wheel_separation_ / 2.0;
        wheel_speed_[RIGHT]
                = position_wheel_ - rotation_wheel_ * wheel_separation_ / 2.0;
    } else {
        wheel_speed_[LEFT]
                = position_wheel_ - rotation_wheel_ * wheel_separation_ / 2.0;
        wheel_speed_[RIGHT]
                = position_wheel_ + rotation_wheel_ * wheel_separation_ / 2.0;
    }
}

void DiffDrive::update_odometry_encoder()
{
// FIX no local variables
#if GAZEBO_MAJOR_VERSION >= 8
    current_time_ = parent->GetWorld()->SimTime();
#else
    current_time_ = parent->GetWorld()->GetSimTime();
#endif

    // FIX no local variables
    diff_time_ = (current_time_ - last_odom_update_).Double();
    last_odom_update_ = current_time_;

    // Book: Sigwart 2011 Autonompus Mobile Robots page:337
    double sl = joints_[LEFT]->GetVelocity(0) * (wheel_diameter_ / 2.0)
            * diff_time_;
    double sr = joints_[RIGHT]->GetVelocity(0) * (wheel_diameter_ / 2.0)
            * diff_time_;
    double ssum = sl + sr;

    double sdiff;
    if (legacy_mode_) {
        sdiff = sl - sr;
    } else {
        sdiff = sr - sl;
    }

    double dx = (ssum) / 2.0
            * cos(pose_encoder_.Z() + (sdiff) / (2.0 * wheel_separation_));
    double dy = (ssum) / 2.0
            * sin(pose_encoder_.Z() + (sdiff) / (2.0 * wheel_separation_));
    double dtheta = (sdiff) / wheel_separation_;

    pose_encoder_.X() += dx;
    pose_encoder_.Y() += dy;
    pose_encoder_.Z() += dtheta;

    double w = dtheta / diff_time_;
    double v = sqrt(dx * dx + dy * dy) / diff_time_;

    ignition::math::Quaterniond qt;
    ignition::math::Vector3d vt;
    qt.Euler(0, 0, pose_encoder_.Z());
    vt = ignition::math::Vector3d(pose_encoder_.X(), pose_encoder_.Y(), 0);

    odometry_sample_.pose().pose().position().x(vt.X());
    odometry_sample_.pose().pose().position().y(vt.Y());
    odometry_sample_.pose().pose().position().z(vt.Z());

    odometry_sample_.pose().pose().orientation().x(qt.X());
    odometry_sample_.pose().pose().orientation().y(qt.Y());
    odometry_sample_.pose().pose().orientation().z(qt.Z());
    odometry_sample_.pose().pose().orientation().w(qt.W());

    odometry_sample_.twist().twist().angular().z(w);
    odometry_sample_.twist().twist().linear().x(dx / diff_time_);
    odometry_sample_.twist().twist().linear().y(dy / diff_time_);
}

}  // namespace dds
}  // namespace gazebo
