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

#ifndef REPLIER_LISTENER_H
#define REPLIER_LISTENER_H

#include <dds/core/ddscore.hpp>
#include <dds/pub/ddspub.hpp>

#include <rti/request/ReplierListener.hpp>

namespace gazebo { namespace dds {

template <typename T, typename T2>
class ReplierListener : public rti::request::ReplierListener< T, T2 > {
public:
    /**
     * @brief Constructor
     */
    ReplierListener()
    {
    }

    /**
     * @brief Constructor
     *
     * @param on_connect on connect callback
     * @param on_disconnect on disconnect callback
     */
    ReplierListener(
            const std::function<T2(const T &)> &on_request)
            : on_request_(on_request)
    {
    }

    /**
     * @brief On request available
     *
     * @param replier that did match
     */
    virtual void on_request_available(
            rti::request::Replier< T, T2 > & replier)
    {
        /*
        * Receive requests and process them
        */
        const auto MAX_WAIT = ::dds::core::Duration::from_secs(5);

        requests_ = replier.receive_requests(MAX_WAIT);
        while (requests_.length() > 0) {
            for (unsigned int i=0 ; i< requests_.length(); i++) {
                if (requests_[i].info().valid()) {
                    T2 reply = on_request_(requests_[i].data());
                    replier.send_reply(reply, requests_[i].info());
                }
            }
            requests_ = replier.receive_requests(MAX_WAIT);
        }
    }

private:
    std::function<T2(const T &)> on_request_;
    ::dds::sub::LoanedSamples<T> requests_;
};

}  // namespace dds
}  // namespace gazebo

#endif  // REPLIER_LISTENER_H
