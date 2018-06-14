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

#include <gazebo/plugins/CameraPlugin.hh>

#include "common/GazeboCameraUtils.h"

namespace gazebo { namespace dds {

class Camera : public CameraPlugin, GazeboCameraUtils
{
public:
    /**
     * @brief Constructor
     */
    Camera();

    /**
     * @brief Destructor
     */
    ~Camera();

    /**
     * @brief Load the plugin inside Gazebo's system
     *
     * @param parent object of Gazebo's sensor
     * @param sdf object of Gazebo's world
     */
    void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override;

protected:

    /**
     * @brief Update the controller
     *
     * @param image raw information of the current image
     * @param width width of the current image
     * @param height height of the current image
     * @param depth  depth of the current image
     * @param format format of the current image
     */
    virtual void OnNewFrame(
            const unsigned char * image,
            unsigned int width,
            unsigned int height,
            unsigned int depth,
            const std::string & format) override;

private:
    common::Time last_update_time_;
};

}  // namespace dds
}  // namespace gazebo
