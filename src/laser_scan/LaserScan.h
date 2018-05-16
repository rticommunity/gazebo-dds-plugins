#include <gazebo/gazebo.hh>
#include <gazebo/plugins/RayPlugin.hh>

#include <dds/core/ddscore.hpp>
#include <dds/domain/find.hpp>
#include <dds/pub/ddspub.hpp>

#include "LaserScanMsg.hpp"

namespace gazebo { namespace dds {

class LaserScan : public RayPlugin {
public:
    /**
     * @brief Constructor
     */
    LaserScan();

    /**
     * @brief Destructor
     */
    ~LaserScan();

    /**
     * @brief Load the plugin inside Gazebo's system
     *
     * @param parent object of Gazebo's sensor
     * @param sdf object of Gazebo's world
     */
    void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf);

    /**
     * @brief Read the current sensor's information
     *
     * @param msg current information of the sensor
     */
    void OnScan(ConstLaserScanStampedPtr &msg);

private:
    sensors::SensorPtr sensor_;
    ::dds::domain::DomainParticipant participant_;
    ::dds::topic::Topic<sensor_msgs::msg::LaserScanMsg> topic_;
    ::dds::pub::qos::DataWriterQos data_writer_qos_;
    ::dds::pub::DataWriter<sensor_msgs::msg::LaserScanMsg> writer_;
    gazebo::transport::NodePtr gazebo_node_;
    gazebo::transport::SubscriberPtr laser_scan_sub_;
    sensor_msgs::msg::LaserScanMsg sample_;
};

}  // namespace dds
}  // namespace gazebo
