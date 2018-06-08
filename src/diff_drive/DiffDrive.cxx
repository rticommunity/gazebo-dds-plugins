#include "common/GazeboDdsUtils.cxx"
#include "common/Properties.h"
#include "DiffDrive.h"

namespace gazebo { namespace dds {

enum WHEEL_POSITION_ENUM {
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
    parent_ = parent;

    wheel_speed_[RIGHT] = 0;
    wheel_speed_[LEFT] = 0;

    wheel_speed_instr_[RIGHT] = 0;
    wheel_speed_instr_[LEFT] = 0;

    // Obtain information of the plugin from loaded world
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

    // Obtain time of the simulation
    last_update_ = utils::get_sim_time(parent_);

    // Obtain joints from loaded world
    joints_[LEFT] = utils::get_joint(sdf, parent_, "leftJoint", "left_joint");
    joints_[RIGHT]
            = utils::get_joint(sdf, parent_, "rightJoint", "right_joint");
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

    // Init samples
    joint_state_sample_.name().resize(2);
    joint_state_sample_.position().resize(2);
    joint_state_sample_.header().frame_id(parent_->GetName() + "/joint");

    odometry_sample_.header().frame_id(parent_->GetName() + "/odometry");
    odometry_sample_.child_frame_id(parent_->GetName() + "/chassis");

    // listen to the world update event
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DiffDrive::update_model, this));

    gzmsg << "Starting Differential drive Plugin" << std::endl;
    gzmsg << "- Odometry topic name: " << topic_name_odometry << std::endl;
    gzmsg << "- JointState topic name: " << topic_name_joint << std::endl;
    gzmsg << "- Twist topic name: " << topic_name_twist << std::endl;
}

void DiffDrive::Reset()
{
    last_update_ = utils::get_sim_time(parent_);

    pose_encoder_.X() = 0;
    pose_encoder_.Y() = 0;
    pose_encoder_.Z() = 0;
    joints_[LEFT]->SetParam("fmax", 0, wheel_torque_);
    joints_[RIGHT]->SetParam("fmax", 0, wheel_torque_);
}

void DiffDrive::update_model()
{
    for (unsigned int i = 0; i < WHEEL_NUMBER; i++) {
        if (fabs(wheel_torque_ - joints_[i]->GetParam("fmax", 0)) > 1e-6) {
            joints_[i]->SetParam("fmax", 0, wheel_torque_);
        }
    }

    if (odom_source_ == "encoder") {
        update_odometry_encoder();
    }

    current_time_ = utils::get_sim_time(parent_);

    double diff_time_ = (current_time_ - last_update_).Double();

    if (diff_time_ > update_period_) {
        publish_odometry();
        publish_joint_state();

        twist_samples_ = reader_.read();

        if (twist_samples_.length() > 0) {
            if (twist_samples_[0].info().valid()) {
                get_wheel_velocities(twist_samples_[0].data());
            }
        }

        double current_speed[WHEEL_NUMBER];

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

            // Update joints velocity
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
    current_time_ = utils::get_sim_time(parent_);

    odometry_sample_.header().stamp().sec(current_time_.sec);
    odometry_sample_.header().stamp().nanosec(current_time_.nsec);

    if (odom_source_ == "world") {
        world_pose_ = get_world_pose();

        odometry_sample_.pose().pose().position().x(world_pose_.Pos().X());
        odometry_sample_.pose().pose().position().y(world_pose_.Pos().Y());
        odometry_sample_.pose().pose().position().z(world_pose_.Pos().Z());

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
    }

    // set covariance
    odometry_sample_.pose().covariance()[0] = 0.00001;
    odometry_sample_.pose().covariance()[7] = 0.00001;
    odometry_sample_.pose().covariance()[14] = 1000000000000.0;
    odometry_sample_.pose().covariance()[21] = 1000000000000.0;
    odometry_sample_.pose().covariance()[28] = 1000000000000.0;
    odometry_sample_.pose().covariance()[35] = 0.001;

    writer_odometry_.write(odometry_sample_);
}

void DiffDrive::publish_joint_state()
{
    current_time_ = utils::get_sim_time(parent_);

    joint_state_sample_.header().stamp().sec(current_time_.sec);
    joint_state_sample_.header().stamp().nanosec(current_time_.nsec);

    for (unsigned int i = 0; i < WHEEL_NUMBER; i++) {
        joint_state_sample_.name()[i] = joints_[i]->GetName();
        joint_state_sample_.position()[i] = get_joint_position(i);
    }

    writer_joint_state_.write(joint_state_sample_);
}

void DiffDrive::get_wheel_velocities(const geometry_msgs::msg::Twist &msg)
{
    if (legacy_mode_) {
        wheel_speed_[LEFT] = msg.linear().x()
                + msg.angular().z() * wheel_separation_ / 2.0;
        wheel_speed_[RIGHT] = msg.linear().x()
                - msg.angular().z() * wheel_separation_ / 2.0;
    } else {
        wheel_speed_[LEFT] = msg.linear().x()
                - msg.angular().z() * wheel_separation_ / 2.0;
        wheel_speed_[RIGHT] = msg.linear().x()
                + msg.angular().z() * wheel_separation_ / 2.0;
    }
}

void DiffDrive::update_odometry_encoder()
{
    current_time_ = utils::get_sim_time(parent_);

    double diff_time_ = (current_time_ - last_odom_update_).Double();
    last_odom_update_ = current_time_;

    // Book: Sigwart 2011 Autonompus Mobile Robots page:337
    double distance_left = joints_[LEFT]->GetVelocity(0)
            * (wheel_diameter_ / 2.0) * diff_time_;
    double distance_right = joints_[RIGHT]->GetVelocity(0)
            * (wheel_diameter_ / 2.0) * diff_time_;

    double distance_diff;
    if (legacy_mode_) {
        distance_diff = distance_left - distance_right;
    } else {
        distance_diff = distance_right - distance_left;
    }

    double derivative_x = (distance_left + distance_right) / 2.0
            * cos(pose_encoder_.Z()
                  + (distance_diff) / (2.0 * wheel_separation_));
    double derivative_y = (distance_left + distance_right) / 2.0
            * sin(pose_encoder_.Z()
                  + (distance_diff) / (2.0 * wheel_separation_));
    double derivative_theta = (distance_diff) / wheel_separation_;

    pose_encoder_.X() += derivative_x;
    pose_encoder_.Y() += derivative_y;
    pose_encoder_.Z() += derivative_theta;

    odometry_orientation_.Euler(0, 0, pose_encoder_.Z());

    odometry_sample_.pose().pose().position().x(pose_encoder_.X());
    odometry_sample_.pose().pose().position().y(pose_encoder_.Y());
    odometry_sample_.pose().pose().position().z(0);

    odometry_sample_.pose().pose().orientation().x(odometry_orientation_.X());
    odometry_sample_.pose().pose().orientation().y(odometry_orientation_.Y());
    odometry_sample_.pose().pose().orientation().z(odometry_orientation_.Z());
    odometry_sample_.pose().pose().orientation().w(odometry_orientation_.W());

    odometry_sample_.twist().twist().angular().z(derivative_theta / diff_time_);
    odometry_sample_.twist().twist().linear().x(derivative_x / diff_time_);
    odometry_sample_.twist().twist().linear().y(derivative_y / diff_time_);
}

inline ignition::math::Pose3d DiffDrive::get_world_pose()
{
#if GAZEBO_MAJOR_VERSION >= 8
    return parent_->WorldPose();
#else
    return parent_->GetWorldPose().Ign();
#endif
}

inline double DiffDrive::get_joint_position(int index)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return joints_[index]->Position(0);
#else
    return joints_[index]->GetAngle(0).Radian();
#endif
}

inline void DiffDrive::get_world_velocity()
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
