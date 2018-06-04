
#include <gazebo/gazebo.hh>
#include <gazebo/plugins/ContactPlugin.hh>

#include <dds/core/ddscore.hpp>
#include <dds/domain/find.hpp>
#include <dds/pub/ddspub.hpp>

#include "gazebo_msgs/msg/ContactState.hpp"
#include "gazebo_msgs/msg/ContactsState.hpp"

namespace gazebo { namespace dds {

class BumperScan : public ContactPlugin {
public:
    /**
     * @brief Constructor
     */
    BumperScan();

    /**
     * @brief Destructor
     */
    virtual ~BumperScan();

    /**
     * @brief Load the plugin inside Gazebo's system
     *
     * @param parent object of Gazebo's sensor
     * @param sdf object of Gazebo's world
     */
    void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override;

private:
    /**
     * @brief Publish the current information of the contact sensor 
     */
    void on_scan();

private:
    ::dds::domain::DomainParticipant participant_;
    ::dds::topic::Topic<gazebo_msgs::msg::ContactsState> topic_;
    ::dds::pub::DataWriter<gazebo_msgs::msg::ContactsState> writer_;
    sensors::ContactSensor* sensor_;
    gazebo_msgs::msg::ContactsState contacts_sample_;
    gazebo_msgs::msg::ContactState contact_state_msg_;
    gazebo::msgs::Contact contact_;
    geometry_msgs::msg::Wrench total_wrench_msg_;
    geometry_msgs::msg::Wrench wrench_msg_;
    geometry_msgs::msg::Vector3 contact_position_msg_;
    geometry_msgs::msg::Vector3 contact_normal_msg_;
    event::ConnectionPtr sensor_connection_;
    common::Time current_time_;
    ignition::math::Quaterniond frame_rot_;
    ignition::math::Vector3d contact_force_;
    ignition::math::Vector3d contact_torque_;
    ignition::math::Vector3d contact_position_;
    ignition::math::Vector3d contact_normal_;
};

}  // namespace dds
}  // namespace gazebo
