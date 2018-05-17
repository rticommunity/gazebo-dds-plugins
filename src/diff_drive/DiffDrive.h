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

private:
    ::dds::domain::DomainParticipant participant_;
    ::dds::topic::Topic<nav_msgs::msg::Odometry> topic_odometry_;
    ::dds::topic::Topic<sensor_msgs::msg::JointState> topic_joint_state_;
    ::dds::topic::Topic<geometry_msgs::msg::Twist> topic_twist_;
    ::dds::pub::DataWriter<nav_msgs::msg::Odometry> writer_odometry_;
    ::dds::pub::DataWriter<sensor_msgs::msg::JointState> writer_joint_state_;
    ::dds::pub::DataWriter<geometry_msgs::msg::Twist> reader_;
    nav_msgs::msg::Odometry sample_;
};

}  // namespace dds
}  // namespace gazebo
