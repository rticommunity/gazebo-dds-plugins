#include "common/GazeboDdsUtils.cxx"
#include "common/DataReaderListener.cxx"
#include "common/Properties.h"
#include "Elevator.h"

namespace gazebo { namespace dds {

GZ_REGISTER_MODEL_PLUGIN(Elevator);

Elevator::Elevator()
        : participant_(::dds::core::null),
          topic_(::dds::core::null),
          reader_(::dds::core::null)
{
}

Elevator::~Elevator()
{
}

void Elevator::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    ElevatorPlugin::Load(parent, sdf);

    // Obtain the domain id from loaded world
    int domain_id;
    utils::get_world_parameter<int>(
            sdf, domain_id, DOMAIN_ID_PROPERTY_NAME.c_str(), 0);

    participant_ = ::dds::domain::find(domain_id);
    if (participant_ == ::dds::core::null) {
        participant_ = ::dds::domain::DomainParticipant(domain_id);
    }

    // Obtain topic name from loaded world
    std::string topic_name;
    utils::get_world_parameter<std::string>(
            sdf, topic_name, TOPIC_NAME_PROPERTY_NAME.c_str(), "elevator");

    topic_ = ::dds::topic::Topic<std_msgs::msg::Int32>(
            participant_, topic_name);

    rti::core::policy::Property::Entry value(
            { "dds.data_reader.history.depth", "1" });

    rti::core::policy::Property property;
    property.set(value);
    data_reader_qos_ << property;
    reader_ = ::dds::sub::DataReader<std_msgs::msg::Int32>(
            ::dds::sub::Subscriber(participant_), topic_, data_reader_qos_);

    reader_.listener(
            new DataReaderListener<std_msgs::msg::Int32>(
                    std::bind(&Elevator::on_msg, this, std::placeholders::_1)),
            ::dds::core::status::StatusMask::data_available());

    gzmsg << "Starting elevator plugin - Topic name: " << topic_name
          << std::endl;
}

void Elevator::on_msg(const std_msgs::msg::Int32 &msg)
{
    this->MoveToFloor(msg.data());
}

}  // namespace dds
}  // namespace gazebo
