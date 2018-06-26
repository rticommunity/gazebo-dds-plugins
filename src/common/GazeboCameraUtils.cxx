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

#ifndef GAZEBO_CAMERA_UTILS_CXX
#define GAZEBO_CAMERA_UTILS_CXX

#include "GazeboDdsUtils.cxx"
#include "Properties.h"
#include "GazeboCameraUtils.h"

#include <gazebo/rendering/Distortion.hh>

namespace gazebo { namespace dds {

GazeboCameraUtils::GazeboCameraUtils()
        : participant_(::dds::core::null),
          topic_image_(::dds::core::null),
          writer_image_(::dds::core::null),
          topic_camera_info_(::dds::core::null),
          writer_camera_info_(::dds::core::null),
          last_info_update_time_(common::Time(0)),
          last_update_time_(common::Time(0)),
          height_(0),
          width_(0),
          skip_(0)
{
}

GazeboCameraUtils::~GazeboCameraUtils()
{
}

void GazeboCameraUtils::load_sdf(
        sensors::SensorPtr parent,
        sdf::ElementPtr sdf,
        double hack_baseline)
{
    // Obtain information of the plugin from loaded world
    utils::get_world_parameter<double>(sdf, cx_prime_, "cx_prime", 0);
    utils::get_world_parameter<double>(sdf, cx_, "cx", 0);
    utils::get_world_parameter<double>(sdf, cy_, "cy", 0);
    utils::get_world_parameter<double>(sdf, focal_length_, "focal_length", 0);

    utils::get_world_parameter<double>(sdf, distortion_k1_, "distortion_k1", 0);
    utils::get_world_parameter<double>(sdf, distortion_k2_, "distortion_k2", 0);
    utils::get_world_parameter<double>(sdf, distortion_k3_, "distortion_k3", 0);
    utils::get_world_parameter<double>(sdf, distortion_t1_, "distortion_t1", 0);
    utils::get_world_parameter<double>(sdf, distortion_t2_, "distortion_t2", 0);

    utils::get_world_parameter<std::string>(
            sdf, frame_name_, "frame_name", "/world");

    utils::get_world_parameter<bool>(sdf, border_crop_, "border_crop", true);
    utils::get_world_parameter<double>(sdf, update_period_, "update_rate", 0);
    hack_baseline_ = hack_baseline;

    // Obtain the domain id from loaded world
    int domain_id;
    utils::get_world_parameter<int>(
            sdf, domain_id, DOMAIN_ID_PROPERTY_NAME.c_str(), 0);

    participant_ = ::dds::domain::find(domain_id);
    if (participant_ == ::dds::core::null) {
        participant_ = ::dds::domain::DomainParticipant(domain_id);
    }

    // Obtain camera info topic name from loaded world
    utils::get_world_parameter<std::string>(
            sdf,
            topic_name_camera_info_,
            "topic_name_camera_info",
            "camera_info");

    auto index = camera_->Name().find_last_of("::");
    std::string camera_name = camera_->Name().substr(++index);
    topic_name_camera_info_ = camera_name+ "/"+ topic_name_camera_info_;

    topic_camera_info_ = ::dds::topic::find<
            ::dds::topic::Topic<sensor_msgs::msg::CameraInfo>>(
            participant_, topic_name_camera_info_);
    if (topic_camera_info_ == ::dds::core::null) {
        topic_camera_info_ = ::dds::topic::Topic<sensor_msgs::msg::CameraInfo>(
                participant_, topic_name_camera_info_);
    }

    writer_camera_info_ = ::dds::pub::DataWriter<sensor_msgs::msg::CameraInfo>(
            ::dds::pub::Publisher(participant_), topic_camera_info_);

    // Obtain image topic name from loaded world
    utils::get_world_parameter<std::string>(
            sdf, topic_name_image_, "topic_name_image", "image");

    topic_name_image_ = camera_name+ "/"+ topic_name_image_;

    topic_image_ = ::dds::topic::Topic<sensor_msgs::msg::Image>(
            participant_, topic_name_image_);

    topic_image_
            = ::dds::topic::find<::dds::topic::Topic<sensor_msgs::msg::Image>>(
                    participant_, topic_name_image_);
    if (topic_image_ == ::dds::core::null) {
        topic_image_ = ::dds::topic::Topic<sensor_msgs::msg::Image>(
                participant_, topic_name_image_);
    }

    // Change the maximum size of the sequences
    rti::core::policy::Property::Entry value(
            { "dds.data_writer.history.memory_manager.fast_pool.pool_"
              "buffer_max_size",
              "4096" });

    rti::core::policy::Property property;
    property.set(value);
    writer_image_qos_ << property;

    writer_image_ = ::dds::pub::DataWriter<sensor_msgs::msg::Image>(
            ::dds::pub::Publisher(participant_),
            topic_image_,
            writer_image_qos_);
}

void GazeboCameraUtils::publish_image(
        const unsigned char * raw_image,
        common::Time &last_update_time)
{
    sensor_update_time_ = last_update_time;
    if (height_ > 0 && width_ > 0) {
        if (sensor_update_time_ - last_info_update_time_ >= update_period_) {
            sample_image_.header().stamp().sec(sensor_update_time_.sec);
            sample_image_.header().stamp().nanosec(sensor_update_time_.nsec);

            memcpy(&sample_image_.data()[0],
                   raw_image,
                   skip_ * width_ * height_);

            writer_image_.write(sample_image_);
        }
    }
}

void GazeboCameraUtils::publish_camera_info(common::Time &last_update_time)
{
    if (height_ > 0 && width_ > 0) {
        sensor_update_time_ = parent_sensor_->LastMeasurementTime();
        if (sensor_update_time_ - last_info_update_time_ >= update_period_) {
            sample_camera_info_.header().stamp().sec(sensor_update_time_.sec);
            sample_camera_info_.header().stamp().nanosec(
                    sensor_update_time_.nsec);

            writer_camera_info_.write(sample_camera_info_);
            last_info_update_time_ = sensor_update_time_;
        }
    }
}

void GazeboCameraUtils::init_samples()
{
    if (update_period_ > 0.0) {
        update_period_ = 1.0 / update_period_;
    } else {
        update_period_ = 0.0;
    }

    if (format_ == "R8G8B8" || format_ == "B8G8R8") {
        skip_ = 3;
    } else {
        skip_ = 1;
    }

    /// Calculate camera information
    if (cx_prime_ == 0) {
        cx_prime_ = (static_cast<double>(width_) + 1.0) / 2.0;
    }
    if (cx_ == 0) {
        cx_ = (static_cast<double>(width_) + 1.0) / 2.0;
    }
    if (cy_ == 0) {
        cy_ = (static_cast<double>(height_) + 1.0) / 2.0;
    }

    double hfov = camera_->HFOV().Radian();
    double computed_focal_length
            = (static_cast<double>(width_)) / (2.0 * tan(hfov / 2.0));

    if (focal_length_ == 0) {
        focal_length_ = computed_focal_length;
    }

    sample_camera_info_.distortion_model("plumb_bob");

    if (camera_->LensDistortion()) {
        camera_->LensDistortion()->SetCrop(border_crop_);
    }

    // Init camera info sample
    sample_camera_info_.header().frame_id(frame_name_);

    sample_camera_info_.height(height_);
    sample_camera_info_.width(width_);

    // Set distortion
    sample_camera_info_.d().resize(5);
    sample_camera_info_.d()[0] = distortion_k1_;
    sample_camera_info_.d()[1] = distortion_k2_;
    sample_camera_info_.d()[2] = distortion_t1_;
    sample_camera_info_.d()[3] = distortion_t2_;
    sample_camera_info_.d()[4] = distortion_k3_;

    // Set camera matrix
    sample_camera_info_.k()[0] = focal_length_;
    sample_camera_info_.k()[1] = 0.0;
    sample_camera_info_.k()[2] = cx_;
    sample_camera_info_.k()[3] = 0.0;
    sample_camera_info_.k()[4] = focal_length_;
    sample_camera_info_.k()[5] = cy_;
    sample_camera_info_.k()[6] = 0.0;
    sample_camera_info_.k()[7] = 0.0;
    sample_camera_info_.k()[8] = 1.0;

    // Set rectification
    sample_camera_info_.r()[0] = 1.0;
    sample_camera_info_.r()[1] = 0.0;
    sample_camera_info_.r()[2] = 0.0;
    sample_camera_info_.r()[3] = 0.0;
    sample_camera_info_.r()[4] = 1.0;
    sample_camera_info_.r()[5] = 0.0;
    sample_camera_info_.r()[6] = 0.0;
    sample_camera_info_.r()[7] = 0.0;
    sample_camera_info_.r()[8] = 1.0;

    // Set projection matrix
    sample_camera_info_.p()[0] = focal_length_;
    sample_camera_info_.p()[1] = 0.0;
    sample_camera_info_.p()[2] = cx_;
    sample_camera_info_.p()[3] = -focal_length_ * hack_baseline_;
    sample_camera_info_.p()[4] = 0.0;
    sample_camera_info_.p()[5] = focal_length_;
    sample_camera_info_.p()[6] = cy_;
    sample_camera_info_.p()[7] = 0.0;
    sample_camera_info_.p()[8] = 0.0;
    sample_camera_info_.p()[9] = 0.0;
    sample_camera_info_.p()[10] = 1.0;
    sample_camera_info_.p()[11] = 0.0;

    // Init sample image
    sample_image_.header().frame_id(frame_name_);
    sample_image_.encoding(format_);
    sample_image_.height(height_);
    sample_image_.width(width_);
    sample_image_.step(skip_ * width_);
    sample_image_.is_bigendian(0);

    sample_image_.data().resize(skip_ * width_ * height_);
}

}  // namespace dds
}  // namespace gazebo

#endif  // GAZEBO_CAMERA_UTILS_CXX
