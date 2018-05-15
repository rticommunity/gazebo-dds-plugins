#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <dds/core/ddscore.hpp>
#include <dds/domain/find.hpp>
#include <dds/pub/ddspub.hpp>

#include "Imu.hpp"

namespace gazebo { namespace dds {

class ImuScan : public SensorPlugin {
public:
    /**
     * @brief Constructor
     */
    ImuScan();

    /**
     * @brief Destructor
     */
    ~ImuScan();

    /**
     * @brief Load the plugin inside Gazebo's system
     *
     * @param parent object of Gazebo's sensor
     * @param sdf object of Gazebo's world
     */
    void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf);

protected:
    void OnScan(ConstIMUPtr & msg);

private:
    /// \brief Gaussian noise generator.
    /// \param mu offset value.
    /// \param sigma scaling value.
    double GuassianKernel(double mu, double sigma);

private:
    physics::WorldPtr world_;
    sensors::ImuSensor* sensor_;
    gazebo::transport::NodePtr gazebo_node_;
    gazebo::transport::SubscriberPtr laser_scan_sub_;
    std::string gazebo_topic_name_;
    ignition::math::Pose3d offset_;
    ignition::math::Quaterniond orientation_;

    ::dds::domain::DomainParticipant participant_;
    ::dds::topic::Topic<sensor_msgs::msg::Imu> topic_;
    ::dds::pub::DataWriter<sensor_msgs::msg::Imu> writer_;
    sensor_msgs::msg::Imu sample_;
    double gaussian_noise_;
    unsigned int seed;
};

}  // namespace dds
}  // namespace gazebo
