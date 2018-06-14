/*
 * Copyright 2018 Real-Time Innovations, Inc.
 * Copyright 2012 Open Source Robotics Foundation
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

#include <gazebo/common/Time.hh>
#include <gazebo/plugins/CameraPlugin.hh>

#include <dds/core/ddscore.hpp>
#include <dds/domain/find.hpp>
#include <dds/pub/ddspub.hpp>

#include "sensor_msgs/msg/CameraInfo.hpp"
#include "sensor_msgs/msg/Image.hpp"

namespace gazebo { namespace dds {

class GazeboCameraUtils {
public:
    /**
     * @brief Constructor
     */
    GazeboCameraUtils();

    /**
     * @brief Destructor
     */
    ~GazeboCameraUtils();

    void load_sdf(
            sensors::SensorPtr parent,
            sdf::ElementPtr sdf,
            const std::string &camera_name = "");

    void load_sdf(
            sensors::SensorPtr parent,
            sdf::ElementPtr sdf,
            const std::string &camera_name,
            double hack_baseline);

protected:
    void publish_image(
            const unsigned char *_src,
            common::Time &last_update_time);

    void publish_camera_info(common::Time &last_update_time);

    void init_samples();

protected:
    ::dds::domain::DomainParticipant participant_;
    ::dds::topic::Topic<sensor_msgs::msg::Image> topic_image_;
    ::dds::topic::Topic<sensor_msgs::msg::CameraInfo> topic_camera_info_;
    ::dds::pub::qos::DataWriterQos writer_image_qos_;
    ::dds::pub::DataWriter<sensor_msgs::msg::Image> writer_image_;
    ::dds::pub::DataWriter<sensor_msgs::msg::CameraInfo> writer_camera_info_;
    sensor_msgs::msg::Image sample_image_;
    sensor_msgs::msg::CameraInfo sample_camera_info_;

    std::string topic_name_image_;
    std::string topic_name_camera_info_;
    std::string frame_name_;
    double update_period_;
    double cx_prime_;
    double cx_;
    double cy_;
    double focal_length_;
    double hack_baseline_;
    double distortion_k1_;
    double distortion_k2_;
    double distortion_k3_;
    double distortion_t1_;
    double distortion_t2_;
    bool border_crop_;
    std::string format_;
    int skip_;
    unsigned int width_;
    unsigned int height_;
    unsigned int depth_;
    sensors::SensorPtr parentSensor_;
    rendering::CameraPtr camera_;
    common::Time sensor_update_time_;

private:
    common::Time last_info_update_time_;
};

}  // namespace dds
}  // namespace gazebo
