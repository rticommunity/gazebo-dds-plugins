#ifndef GAZEBO_DDS_UTILS_CXX
#define GAZEBO_DDS_UTILS_CXX

#include "GazeboDdsUtils.h"

template <class T>
void GazeboDdsUtils::GetParameter(
        sdf::ElementPtr sdf,
        T &tag_variable,
        const char *tag_name,
        const T &default_value)
{
    if (sdf->HasElement(tag_name)) {
        tag_variable = sdf->Get<T>(tag_name);
    } else {
        tag_variable = default_value;
        gzwarn << "Missing <" << tag_name
               << ">, set to default: " << tag_variable << std::endl;
    }
}

gazebo::physics::JointPtr GazeboDdsUtils::GetJoint(
        sdf::ElementPtr sdf,
        gazebo::physics::ModelPtr parent,
        const char * tag_name,
        const std::string & joint_default_name)
{
    std::string joint_name;
    GetParameter<std::string>(sdf,joint_name, tag_name, joint_default_name);
    gazebo::physics::JointPtr joint = parent->GetJoint(joint_name);

    return joint;
}

#endif  // DATA_WRITER_LISTENER_CXX