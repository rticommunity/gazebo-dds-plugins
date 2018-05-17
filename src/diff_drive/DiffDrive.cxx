#include <iostream>

#include <gazebo/physics/World.hh>

#include "common/Properties.h"
#include "DiffDrive.h"

namespace gazebo { namespace dds {

GZ_REGISTER_MODEL_PLUGIN(DiffDrive)

DiffDrive::DiffDrive()
        : participant_(::dds::core::null),
          topic_odometry_(::dds::core::null),
          topic_joint_state_(::dds::core::null),
          topic_twist_(::dds::core::null),
          writer_odometry_(::dds::core::null),
          writer_joint_state_(::dds::core::null),
          reader_(::dds::core::null)

{
}

DiffDrive::~DiffDrive()
{
}

void DiffDrive::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
//     gzmsg << "Starting Imu Plugin - Topic name: " << topic_name << std::endl;
}

void DiffDrive::PublishOdometry()
{
}

void DiffDrive::PublishJointState()
{
}

}  // namespace dds
}  // namespace gazebo
