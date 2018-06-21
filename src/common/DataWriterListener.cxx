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

#include "DataWriterListener.h"

#ifndef DATA_WRITER_LISTENER_CXX
#define DATA_WRITER_LISTENER_CXX

namespace gazebo { namespace dds {

template <typename T>
DataWriterListener<T>::DataWriterListener()
{
}

template <typename T>
DataWriterListener<T>::DataWriterListener(
        const std::function<void()> &on_connect,
        const std::function<void()> &on_disconnect)
        : on_connect_(on_connect), on_disconnect_(on_disconnect)
{
}

template <typename T>
void DataWriterListener<T>::on_publication_matched(
        ::dds::pub::DataWriter<T> &writer,
        const ::dds::core::status::PublicationMatchedStatus &status)
{
    if (status.current_count_change() < 0) {
        on_disconnect_();
    } else {
        on_connect_();
    }
}

}  // namespace dds
}  // namespace gazebo

#endif  // DATA_WRITER_LISTENER_CXX
