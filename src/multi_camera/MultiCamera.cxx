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

#include "common/GazeboDdsUtils.cxx"
#include "MultiCamera.h"

namespace gazebo { namespace dds {

GZ_REGISTER_SENSOR_PLUGIN(MultiCamera)

MultiCamera::MultiCamera() : last_update_time_(common::Time(0))
{
}

MultiCamera::~MultiCamera()
{
}

void MultiCamera::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
    // CameraPlugin::Load(parent, sdf);

    // // CameraPlugin information into GazeboCameraUtils
    // parent_sensor_ = parentSensor;
    // width_ = width;
    // height_ = height;
    // depth_ = depth;
    // format_ = format;
    // camera_ = camera;

    // double baseline;
    // utils::get_world_parameter<double>(sdf, baseline, "hack_baseline", 0);

    // GazeboCameraUtils::load_sdf(parent, sdf, baseline);
    // GazeboCameraUtils::init_samples();

    // gzmsg << "Starting MultiCamera Plugin" << std::endl;
    // gzmsg << "* Publications:" << std::endl;
    // gzmsg << "  - " << topic_name_camera_info_
    //       << " [sensor_msgs/msg/CameraInfo]" << std::endl;
    // gzmsg << "  - " << topic_name_image_ << " [sensor_msgs/msg/Image]"
    //       << std::endl;
}

void MultiCamera::on_new_frame(
        const unsigned char *image,
        unsigned int width,
        unsigned int height,
        unsigned int depth,
        const std::string &format)
{
}

void MultiCamera::on_new_frame_left(
        const unsigned char *image,
        unsigned int width,
        unsigned int height,
        unsigned int depth,
        const std::string &format)
{
}

void MultiCamera::on_new_frame_right(
        const unsigned char *image,
        unsigned int width,
        unsigned int height,
        unsigned int depth,
        const std::string &format)
{
}

}  // namespace dds
}  // namespace gazebo
