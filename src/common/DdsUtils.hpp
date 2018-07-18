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

#ifndef DDS_UTILS_HPP
#define DDS_UTILS_HPP

#include <csignal>
#include <iostream>

#include <dds/dds.hpp>
#include <rti/domain/find.hpp>
#include <dds/core/ddscore.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>

namespace gazebo { namespace dds { namespace utils {

/**
 * This global variable keeps the main thread sleeping until a signal changes
 * its value to false.
 */
bool exit_application = false;
/**
 * This method changes sets exit_service to true. It is triggered by
 * @param signal Unused int parameters that identifies the captured signal.
 */
inline void signal_handler(int signal)
{
    // Log registration of Web Server
    std::cout << "Received " << std::to_string(signal) << " signal"
              << std::endl;
    exit_application = true;
}

/**
 * This method registers the signal_handler() with all the exit signals, which
 * triggers a call to the method if any event happens.
 */
inline void setup_signal_handler()
{
#ifndef RTI_WIN32
    signal(SIGHUP,  signal_handler); //Terminal is closed
    signal(SIGQUIT, signal_handler); //Quit
#endif
    signal(SIGTERM, signal_handler); //Terminate
    signal(SIGINT,  signal_handler); //Interrupt
    signal(SIGABRT, signal_handler); //Abort
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

template <typename T>
void wait_for_publication_matched(
        const ::dds::pub::DataWriter<T> &writer,
        const ::dds::core::Duration &duration)
{
    ::dds::core::cond::StatusCondition status_condition(writer);
    status_condition.enabled_statuses(
            ::dds::core::status::StatusMask::publication_matched());

    ::dds::core::cond::WaitSet waitset;
    waitset.attach_condition(status_condition);

    waitset.wait(duration);
}

}  // namespace utils
}  // namespace dds
}  // namespace gazebo

#endif // DDS_UTILS_HPP
