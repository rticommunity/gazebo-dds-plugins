/*
 * Copyright 2018 Real-Time Innovations, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GAZEBO_DDS_UTILS_CXX
#define GAZEBO_DDS_UTILS_CXX

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <dds/dds.hpp>
#include <rti/domain/find.hpp>
#include <dds/core/ddscore.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>

#include "Properties.h"

namespace gazebo { namespace dds { namespace utils {

template <typename T>
void get_world_parameter(
        sdf::ElementPtr sdf,
        T & tag_variable,
        const char * element_name,
        const T & element_default)
{
    if (sdf->HasElement(element_name)) {
        tag_variable = sdf->Get<T>(element_name);
    } else {
        tag_variable = element_default;
        gzwarn << "Missing <" << element_name
               << ">, set to default: " << tag_variable << std::endl;
    }
}

void find_domain_participant(
        int domain_id,
        ::dds::domain::DomainParticipant & participant,
        ::dds::core::QosProvider & qos_provider,
        const std::string & qos_profile)
{
    std::string participant_name
            = std::to_string(domain_id) + "::" + qos_profile;

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
void find_topic(
        const ::dds::domain::DomainParticipant & participant,
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
void create_datawriter(
        ::dds::pub::DataWriter<T> & writer,
        const ::dds::domain::DomainParticipant & participant,
        const ::dds::topic::Topic<T> & topic,
        ::dds::core::QosProvider & qos_provider)
{
    writer = ::dds::pub::DataWriter<T>(
            ::dds::pub::Publisher(
                    participant, qos_provider.publisher_qos()),
            topic,
            qos_provider.datawriter_qos());
}

template <typename T>
void create_datawriter(
        ::dds::pub::DataWriter<T> & writer,
        const ::dds::domain::DomainParticipant & participant,
        const ::dds::topic::Topic<T> & topic,
        const ::dds::pub::qos::DataWriterQos & data_writer_qos,
        const ::dds::pub::qos::PublisherQos & publisher_qos)
{
    writer = ::dds::pub::DataWriter<T>(
            ::dds::pub::Publisher(
                    participant, publisher_qos),
            topic,
            data_writer_qos);
}

template <typename T>
void create_datareader(
        ::dds::sub::DataReader<T> & reader,
        const ::dds::domain::DomainParticipant & participant,
        const ::dds::topic::Topic<T> & topic,
        ::dds::core::QosProvider & qos_provider)
{
    reader = ::dds::sub::DataReader<T>(
            ::dds::sub::Subscriber(
                    participant, qos_provider.subscriber_qos()),
            topic,
            qos_provider.datareader_qos());
}

template <typename T>
void create_datareader(
        ::dds::sub::DataReader<T> & reader,
        const ::dds::domain::DomainParticipant & participant,
        const ::dds::topic::Topic<T> & topic,
        const ::dds::sub::qos::DataReaderQos & data_reader_qos,
        const ::dds::sub::qos::SubscriberQos & subscriber_qos)
{
    reader = ::dds::sub::DataReader<T>(
            ::dds::sub::Subscriber(
                    participant, subscriber_qos),
            topic,
            data_reader_qos);
}

void set_unbounded_sequence_allocated_size(
        ::dds::pub::qos::DataWriterQos &data_writer_qos,
        int pool_size)
{
    rti::core::policy::Property::Entry value(
            { "dds.data_writer.history.memory_manager.fast_pool.pool_"
              "buffer_max_size",
              std::to_string(pool_size) });

    rti::core::policy::Property property;
    property.set(value);
    data_writer_qos << property;
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
