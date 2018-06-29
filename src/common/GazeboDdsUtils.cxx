#ifndef GAZEBO_DDS_UTILS_CXX
#define GAZEBO_DDS_UTILS_CXX

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <dds/dds.hpp>
#include <rti/domain/find.hpp>
#include <dds/core/ddscore.hpp>
#include <dds/pub/ddspub.hpp>

#include "Properties.h"

namespace gazebo { namespace dds { namespace utils {

template <typename T>
void get_world_parameter(
        sdf::ElementPtr sdf,
        T &tag_variable,
        const char *tag_name,
        const T &default_value)
{
    if (sdf->HasElement(tag_name)) {
        tag_variable = sdf->Get<T>(tag_name);
    } else {
        tag_variable = default_value;
        gzwarn << "Missing <" << tag_name
               << ">, set to default: " << tag_variable << std::endl;
    }
}

gazebo::physics::JointPtr get_joint(
        sdf::ElementPtr sdf,
        gazebo::physics::ModelPtr parent,
        const char *tag_name,
        const std::string & joint_default_name)
{
    std::string joint_name;
    get_world_parameter<std::string>(
            sdf, joint_name, tag_name, joint_default_name);
    gazebo::physics::JointPtr joint = parent->GetJoint(joint_name);

    if(!joint){
        char error[200];
        snprintf(error, 200,
                 "Couldn't get wheel hinge joint named %s", joint_name.c_str());
        gzthrow(error);
    }

    return joint;
}

void create_participant(
        int domain_id,
        ::dds::domain::DomainParticipant & participant,
        ::dds::core::QosProvider & qos_provider,
        const std::string & qos_profile)
{
    std::string participant_name = std::to_string(domain_id) +"::"+ qos_profile;

    participant = rti::domain::find_participant_by_name(participant_name);
    if (participant == ::dds::core::null) {
        ::dds::domain::qos::DomainParticipantQos participant_qos
            = qos_provider.participant_qos();

        rti::core::policy::EntityName entity_name(participant_name);

        participant_qos << entity_name;

        participant = ::dds::domain::DomainParticipant(
                domain_id, participant_qos);
    }
}

template <typename T>
void create_topic(
        ::dds::domain::DomainParticipant & participant,
        ::dds::topic::Topic<T> & topic,
        const std::string & topic_name)
{
    topic = ::dds::topic::find<::dds::topic::Topic<T>>(participant, topic_name);
    if (topic == ::dds::core::null) {
        topic = ::dds::topic::Topic<T>(
                participant, topic_name);
    }
}

template <typename T>
void create_writer(
        ::dds::pub::DataWriter<T> & writer,
        ::dds::domain::DomainParticipant & participant,
        ::dds::topic::Topic<T> & topic,
        ::dds::core::QosProvider & qos_provider)
{
    writer = ::dds::pub::DataWriter<T>(
            ::dds::pub::Publisher(
                    participant, qos_provider.publisher_qos()),
            topic,
            qos_provider.datawriter_qos());
}

template <typename T>
void create_writer(
        ::dds::pub::DataWriter<T> & writer,
        ::dds::domain::DomainParticipant & participant,
        ::dds::topic::Topic<T> & topic,
        ::dds::pub::qos::DataWriterQos & data_writer_qos,
        ::dds::pub::qos::PublisherQos & publisher_qos)
{
    writer = ::dds::pub::DataWriter<T>(
            ::dds::pub::Publisher(
                    participant, publisher_qos),
            topic,
            data_writer_qos);
}

common::Time get_sim_time(physics::WorldPtr world)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return world->SimTime();
#else
    return world->GetSimTime();
#endif
}


}  // namespace utils
}  // namespace dds
}  // namespace gazebo

#endif  // GAZEBO_DDS_UTILS_CXX
