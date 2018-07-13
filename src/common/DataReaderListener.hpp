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

#ifndef DATA_READER_LISTENER_H
#define DATA_READER_LISTENER_H

#include <dds/sub/ddssub.hpp>

namespace gazebo { namespace dds {

template <typename T>
class DataReaderListener : public ::dds::sub::NoOpDataReaderListener<T> {
public:
    /**
     * @brief Constructor
     *
     * @param on_data on data callback
     */
    DataReaderListener(const std::function<void(const T &sample)> &on_data)
            : on_data_(on_data)
    {
    }

    /**
     * @brief On data available
     *
     * @param reader datareader that have data available
     */
    virtual void on_data_available(::dds::sub::DataReader<T> &reader)
    {
        ::dds::sub::LoanedSamples<T> samples = reader.take();
        for (auto sample : samples) {
            // If the reference we get is valid data, it means we have actual
            // data available, otherwise we got metadata.
            if (sample.info().valid()) {
                on_data_(sample.data());
            }
        }
    }

private:
    std::function<void(const T &sample)> on_data_;
};

}  // namespace dds
}  // namespace gazebo

#endif  // DATA_READER_LISTENER_H
