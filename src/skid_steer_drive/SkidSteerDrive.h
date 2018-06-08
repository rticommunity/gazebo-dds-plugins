#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <dds/core/ddscore.hpp>
#include <dds/domain/find.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>

#include "geometry_msgs/msg/Twist.hpp"
#include "nav_msgs/msg/Odometry.hpp"
#include "sensor_msgs/msg/JointState.hpp"

namespace gazebo { namespace dds {

const int WHEEL_NUMBER = 4;

class SkidSteerDrive : public ModelPlugin {
public:
    /**
     * @brief Constructor
     */
    SkidSteerDrive();

    /**
     * @brief Destructor
     */
    ~SkidSteerDrive();

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
    virtual void update_model();

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
    ::dds::sub::qos::DataReaderQos data_reader_qos_;
    ::dds::sub::DataReader<geometry_msgs::msg::Twist> reader_;
    ::dds::sub::LoanedSamples<geometry_msgs::msg::Twist> twist_samples_;
    nav_msgs::msg::Odometry odometry_sample_;
    sensor_msgs::msg::JointState joint_state_sample_;

    physics::ModelPtr parent_;
    physics::JointPtr joints_[WHEEL_NUMBER];
    event::ConnectionPtr update_connection_;

    std::string left_front_joint_name_;
    std::string right_front_joint_name_;
    std::string left_rear_joint_name_;
    std::string right_rear_joint_name_;

    common::Time last_update_;
    common::Time current_time_;

    ignition::math::Quaterniond odometry_orientation_;
    ignition::math::Pose3d world_pose_;
    ignition::math::Vector3d world_linear_;

    double wheel_separation_;
    double wheel_diameter_;
    double wheel_torque_;
    double wheel_speed_[WHEEL_NUMBER];
    double update_period_;
    double covariance_x_;
    double covariance_y_;
    double covariance_yaw_;

};

}  // namespace dds
}  // namespace gazebo
