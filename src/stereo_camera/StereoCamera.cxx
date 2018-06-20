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

#include <gazebo/sensors/MultiCameraSensor.hh>

#include "common/GazeboCameraUtils.cxx"
#include "common/GazeboDdsUtils.cxx"
#include "StereoCamera.h"

namespace gazebo { namespace dds {

GZ_REGISTER_SENSOR_PLUGIN(StereoCamera)

StereoCamera::StereoCamera()
{
}

StereoCamera::~StereoCamera()
{
}

void StereoCamera::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
    parent_sensor_ = dynamic_cast<sensors::MultiCameraSensor *>(sensor.get());
    sensor_ = sensor;
    sdf_ = sdf;

    if (parent_sensor_->CameraCount() != CAMERA_NUMBER) {
        gzthrow("Stereo camera plugin needs to have two cameras");
    }

    for (unsigned int i = 0; i < CAMERA_NUMBER; ++i) {
        if (parent_sensor_->Camera(i)->Name().find("left")
            != std::string::npos) {
            init_camera(parent_sensor_->Camera(i), camera_left_);
        } else if (
                parent_sensor_->Camera(i)->Name().find("right")
                != std::string::npos) {
            init_camera(parent_sensor_->Camera(i), camera_right_);
        }
    }

    parent_sensor_->SetActive(true);

    gzmsg << "Starting stereo camera plugin" << std::endl;
    gzmsg << "* Publications:" << std::endl;
    gzmsg << "  - " << camera_left_.topic_name_camera_info_
          << " [sensor_msgs/msg/CameraInfo]" << std::endl;
    gzmsg << "  - " << camera_left_.topic_name_image_
          << " [sensor_msgs/msg/Image]" << std::endl;
    gzmsg << "  - " << camera_right_.topic_name_camera_info_
          << " [sensor_msgs/msg/CameraInfo]" << std::endl;
    gzmsg << "  - " << camera_right_.topic_name_image_
          << " [sensor_msgs/msg/Image]" << std::endl;
}

void StereoCamera::on_new_frame(
        const unsigned char *image,
        GazeboCameraUtils &camera_utils)
{
    common::Time sensor_update_time = parent_sensor_->LastMeasurementTime();

    if (sensor_update_time - camera_utils.last_update_time_
        >= camera_utils.update_period_) {
        camera_utils.publish_image(image, sensor_update_time);
        camera_utils.publish_camera_info(sensor_update_time);
        camera_utils.last_update_time_ = sensor_update_time;
    }
}

void StereoCamera::on_new_frame_left(
        const unsigned char *image,
        unsigned int width,
        unsigned int height,
        unsigned int depth,
        const std::string &format)
{
    on_new_frame(image, camera_left_);
}

void StereoCamera::on_new_frame_right(
        const unsigned char *image,
        unsigned int width,
        unsigned int height,
        unsigned int depth,
        const std::string &format)
{
    on_new_frame(image, camera_right_);
}

void StereoCamera::init_camera(
        rendering::CameraPtr camera,
        GazeboCameraUtils &camera_utils)
{
    // Initialize the information of the camera
    double baseline;
    camera_utils.parent_sensor_ = sensor_;
    camera_utils.width_ = camera->ImageWidth();
    camera_utils.height_ = camera->ImageHeight();
    camera_utils.depth_ = camera->ImageDepth();
    camera_utils.format_ = camera->ImageFormat();
    camera_utils.camera_ = camera;

    // Load the sdf information of the plugin
    if (camera_utils.camera_->Name().find("left") != std::string::npos) {
        camera_utils.load_sdf(sensor_, sdf_, 0.0);
        camera_utils.new_frame_connection_
                = camera_utils.camera_->ConnectNewImageFrame(std::bind(
                        &StereoCamera::on_new_frame_left,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2,
                        std::placeholders::_3,
                        std::placeholders::_4,
                        std::placeholders::_5));
    } else if (
            camera_utils.camera_->Name().find("right") != std::string::npos) {
        utils::get_world_parameter<double>(sdf_, baseline, "hack_baseline", 0);
        camera_utils.load_sdf(sensor_, sdf_, baseline);
        camera_utils.new_frame_connection_
                = camera_utils.camera_->ConnectNewImageFrame(std::bind(
                        &StereoCamera::on_new_frame_right,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2,
                        std::placeholders::_3,
                        std::placeholders::_4,
                        std::placeholders::_5));
    }

    camera_utils.init_samples();
}

}  // namespace dds
}  // namespace gazebo
