#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <dds/core/ddscore.hpp>
#include <dds/domain/find.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>

#include "geometry_msgs/msg/Pose2D.hpp"
#include "geometry_msgs/msg/Twist.hpp"
#include "nav_msgs/msg/Odometry.hpp"
#include "sensor_msgs/msg/JointState.hpp"
#include "common/GazeboDdsUtils.h"

namespace gazebo { namespace dds {

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
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
    
protected:
    /**
     * @brief Update model
     *
     * @param parent object of Gazebo's model
     * @param sdf object of Gazebo's world
     */
      virtual void UpdateChild();

private:
    void PublishOdometry();

    void PublishJointState();

    void CmdCallBack(geometry_msgs::msg::Twist & msg);

private:
    ::dds::domain::DomainParticipant participant_;
    ::dds::topic::Topic<nav_msgs::msg::Odometry> topic_odometry_;
    ::dds::topic::Topic<sensor_msgs::msg::JointState> topic_joint_state_;
    ::dds::topic::Topic<geometry_msgs::msg::Twist> topic_twist_;
    ::dds::pub::DataWriter<nav_msgs::msg::Odometry> writer_odometry_;
    ::dds::pub::DataWriter<sensor_msgs::msg::JointState> writer_joint_state_;
    ::dds::sub::DataReader<geometry_msgs::msg::Twist> reader_;
    nav_msgs::msg::Odometry odometry_sample_;
    sensor_msgs::msg::JointState joint_state_sample_;

    physics::ModelPtr parent;
    event::ConnectionPtr update_connection_;

    double wheel_separation_;
    double wheel_diameter_;
    double wheel_accel_;
    double wheel_torque_;
    double update_period_;
    double wheel_speed_[2];
    double wheel_speed_instr_[2];

    std::vector<physics::JointPtr> joints_;
    geometry_msgs::msg::Pose2D pose_encoder_;
    
};

}  // namespace dds
}  // namespace gazebo
