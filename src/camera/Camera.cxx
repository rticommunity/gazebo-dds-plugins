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

#include "Camera.h"

namespace gazebo { namespace dds {

GZ_REGISTER_SENSOR_PLUGIN(Camera)

Camera::Camera()
        : CameraPlugin(),
        GazeboCameraUtils()
{
}

Camera::~Camera()
{
}

void Camera::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
    CameraPlugin::Load(parent, sdf);

    parentSensor_ = parentSensor;
    width_ = width;
    height_ = height;
    depth_ = depth;
    format_ = format;
    camera_ = camera;
    
    
    load_sdf(parent, sdf);
    
    init();
}

void Camera::OnNewFrame(const unsigned char * image,
    unsigned int width, unsigned int height, unsigned int depth,
    const std::string & format)
{
    common::Time sensor_update_time = parentSensor_->LastMeasurementTime();

    if (sensor_update_time - this->last_update_time_ >= this->update_period_){
        publish_image(image, sensor_update_time);
        publish_camera_info(sensor_update_time);
        last_update_time_ = sensor_update_time;
    }
}

}  // namespace dds
}  // namespace gazebo
