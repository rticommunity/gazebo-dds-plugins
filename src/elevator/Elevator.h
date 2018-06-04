
#include <gazebo/plugins/ElevatorPlugin.hh>

#include <dds/core/ddscore.hpp>
#include <dds/domain/find.hpp>
#include <dds/sub/ddssub.hpp>

#include "std_msgs/msg/Int32.hpp"

namespace gazebo { namespace dds {

class Elevator : public ElevatorPlugin {

public:
    /**
     * @brief Constructor
     */
    Elevator();

    /**
     * @brief Destructor
     */
    virtual ~Elevator();

    /**
     * @brief Load the plugin inside Gazebo's system
     *
     * @param parent object of Gazebo's model
     * @param sdf object of Gazebo's world
     */
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override;

    /**
     * @brief Receives messages to manage the elevator
     *
     * @param msg current order to the elevator
     */
    void on_msg(const std_msgs::msg::Int32 & msg);

private:
    ::dds::domain::DomainParticipant participant_;
    ::dds::topic::Topic<std_msgs::msg::Int32> topic_;
    ::dds::sub::DataReader<std_msgs::msg::Int32> reader_;
    ::dds::sub::qos::DataReaderQos data_reader_qos_;
};

}  // namespace dds
}  // namespace gazebo
