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

#include <dds/core/ddscore.hpp>
#include <dds/pub/ddspub.hpp>

#ifndef DATA_WRITER_LISTENER_H
#define DATA_WRITER_LISTENER_H

namespace gazebo { namespace dds {

template <typename T>
class DataWriterListener : public ::dds::pub::NoOpDataWriterListener<T> {
public:
    /**
     * @brief Constructor
     */
    DataWriterListener();

    /**
     * @brief Constructor
     *
     * @param on_connect on connect callback
     * @param on_disconnect on disconnect callback
     */
    DataWriterListener(
            const std::function<void()> &on_connect,
            const std::function<void()> &on_disconnect);

    /**
     * @brief On publication matched
     *
     * @param writer datawriter that did match
     * @param status status of the match
     */
    virtual void on_publication_matched(
            ::dds::pub::DataWriter<T> &writer,
            const ::dds::core::status::PublicationMatchedStatus &status);

private:
    std::function<void()> on_connect_;
    std::function<void()> on_disconnect_;
};

}  // namespace dds
}  // namespace gazebo

#endif  // DATA_WRITER_LISTENER_H