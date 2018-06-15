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
#include <gazebo/sensors/MultiCameraSensor.hh>
#include "MultiCamera.h"

namespace gazebo { namespace dds {

GZ_REGISTER_SENSOR_PLUGIN(MultiCamera)

MultiCamera::MultiCamera() : last_update_time_(common::Time(0))
{
}

MultiCamera::~MultiCamera()
{
}

void MultiCamera::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
    parent_sensor_ = dynamic_cast<sensors::MultiCameraSensor*>(sensor.get());
    double baseline;
    GazeboCameraUtils* util;

    for (unsigned int i = 0; i < parent_sensor_->CameraCount(); ++i){

        util = new GazeboCameraUtils();
        util->parent_sensor_ = sensor;
        util->width_   = parent_sensor_->Camera(i)->ImageWidth();
        util->height_  = parent_sensor_->Camera(i)->ImageHeight();
        util->depth_   = parent_sensor_->Camera(i)->ImageDepth();
        util->format_  = parent_sensor_->Camera(i)->ImageFormat();
        util->camera_  = parent_sensor_->Camera(i);

        if (util->camera_->Name().find("left") != std::string::npos){
            util->load_sdf(sensor, sdf, 0.0);
        }
        else if (util->camera_->Name().find("right") != std::string::npos){
            utils::get_world_parameter<double>(sdf, baseline, "hack_baseline", 0);
            util->load_sdf(sensor, sdf, baseline);
        }

        util->init_samples();
        utils.push_back(util);
    }

    parent_sensor_->SetActive(true);
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
