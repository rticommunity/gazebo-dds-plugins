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
#include <rti/request/rtirequest.hpp>
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

/**
 * @brief Look for an existed domain participant.
 * If this method doesn't find it, builds it.
 *
 * @param domain_id domain id for the domain participant
 * @param participant the found/built domain participant
 * @param qos_provider object that contains the loaded profiles
 * @param qos_profile the name of the profile that will be used 
 * in the domain participant
 */
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

/**
 * @brief Look for an existed topic.
 * If this method doesn't find it, builds it.
 *
 * @param participant domain participant that the topic will use.
 * @param topic the found/built topic
 * @param topic_name name that the topic will have.
 */
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

/**
 * @brief Create a data writer
 *
 * @param writer the created data writer
 * @param participant domain participant that the data writer will use.
 * @param topic topic that the data writer will use.
 * @param qos_provider object that contains the loaded profiles
 */
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

/**
 * @brief Create a data writer
 *
 * @param writer the created data writer
 * @param participant domain participant that the data writer will use.
 * @param topic topic that the data writer will use.
 * @param data_writer_qos qos of the data writer
 * @param publisher_qos qos of the publisher
 */
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

/**
 * @brief Create a data reader
 *
 * @param writer the created data reader
 * @param participant domain participant that the data reader will use.
 * @param topic topic that the data reader will use.
 * @param qos_provider object that contains the loaded profiles
 */
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

/**
 * @brief Create a data reader
 *
 * @param reader the created data reader
 * @param participant domain participant that the data reader will use.
 * @param topic topic that the data reader will use.
 * @param data_reader_qos qos of the data reader
 * @param subscriber_qos qos of the subscriber
 */
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

/**
 * @brief Create a requester
 *
 * @param requester the created requester
 * @param participant domain participant that the requester will use.
 * @param service_name service name that the requester will use.
 */
template <typename T, typename T2>
void create_requester(
        rti::request::Requester<T,T2> & requester,
        const ::dds::domain::DomainParticipant & participant,
        const std::string & service_name)
{
    rti::request::RequesterParams requester_params(participant);
    requester_params.service_name(service_name);

    requester = rti::request::Requester<T,T2>(requester_params);
}

/**
 * @brief Create a replier
 *
 * @param replier the created replier
 * @param participant domain participant that the replier will use.
 * @param service_name service name that the replier will use.
 */
template <typename T, typename T2>
void create_replier(
        rti::request::Replier<T,T2> & replier,
        const ::dds::domain::DomainParticipant & participant,
        const std::string & service_name)
{
    rti::request::ReplierParams replier_params(participant);
    replier_params.service_name(service_name);
    
    replier = rti::request::Replier<T,T2>(replier_params);
}

/**
 * @brief Call a service 
 *
 * @param requester object that will call the service
 * @param request message that the requester will send to the replier
 */
template <typename T, typename T2>
T2 call_service(
        rti::request::Requester<T,T2> & requester,
        T & request)
{
    T2 result_reply;

    // Send the request
    requester.send_request(request);

    // Receive replies
    const ::dds::core::Duration MAX_WAIT = ::dds::core::Duration::from_secs(2);

    bool in_progress = true;
    while (in_progress) {
        auto replies = requester.receive_replies(MAX_WAIT);

        if (replies.length() == 0) {
            throw std::runtime_error("Timed out waiting for replies");
            return result_reply;
        }

        for (const auto &reply : replies) {
            if (reply.info().valid()) {
                result_reply = reply.data();
                in_progress = false;
            }
        }
    }

    return result_reply;
}

/**
 * @brief Set the buffer size for the unbounded sequence.
 *
 * @param data_writer_qos qos that will set the buffer size
 * @param pool_size size of the buffer that will be used
 */
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

/**
 * @brief Wait for publication matched
 *
 * @param writer data writer that will wait until a publication matched
 * @param duration time of wait
 */
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
