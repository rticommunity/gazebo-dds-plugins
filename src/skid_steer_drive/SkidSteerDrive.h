#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <dds/core/ddscore.hpp>
#include <dds/domain/find.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>

#include "geometry_msgs/msg/Twist.hpp"
#include "nav_msgs/msg/Odometry.hpp"
#include "sensor_msgs/msg/JointState.hpp"
// #include <nav_msgs/msg/OccupancyGrid.h>

namespace gazebo { namespace dds {

class SkidSteerDrive : public ModelPlugin {
public:
    SkidSteerDrive();
    ~SkidSteerDrive();
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override;
    void Reset() override;

protected:
    virtual void update_model();

private:
    void publish_odometry();

    /**
     * @brief Publish JointState sample
     */
    void publish_joint_state();

    void get_wheel_velocities(const geometry_msgs::msg::Twist &msg);

private:
    ::dds::domain::DomainParticipant participant_;
    ::dds::topic::Topic<nav_msgs::msg::Odometry> topic_odometry_;
    ::dds::topic::Topic<sensor_msgs::msg::JointState> topic_joint_state_;
    ::dds::topic::Topic<geometry_msgs::msg::Twist> topic_twist_;
    ::dds::pub::DataWriter<nav_msgs::msg::Odometry> writer_odometry_;
    ::dds::pub::DataWriter<sensor_msgs::msg::JointState> writer_joint_state_;
    ::dds::sub::qos::DataReaderQos data_reader_qos_;
    ::dds::sub::DataReader<geometry_msgs::msg::Twist> reader_;

    physics::ModelPtr parent_;
    event::ConnectionPtr update_connection_;

    std::string left_front_joint_name_;
    std::string right_front_joint_name_;
    std::string left_rear_joint_name_;
    std::string right_rear_joint_name_;

    common::Time last_update_;

    double wheel_separation_;
    double wheel_diameter_;
    double wheel_torque_;
    double wheel_speed_[4];
    double update_period_;
    double covariance_x_;
    double covariance_y_;
    double covariance_yaw_;

    physics::JointPtr joints_[4];
};

}  // namespace dds
}  // namespace gazebo
