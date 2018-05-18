#ifndef GAZEBO_DDS_UTILS_H
#define GAZEBO_DDS_UTILS_H

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

class GazeboDdsUtils {
public:
    /**
     * Read the follwoing tag_name parameter or sets a default_value
     * @param _value
     * @param _tag_name
     * @param _default
     * @retun sdf tag value
     **/
    template <typename T>
    static void GetParameter(
            sdf::ElementPtr sdf,
            T &value,
            const char *tag_name,
            const T &default_value);

    static gazebo::physics::JointPtr GetJoint(
            sdf::ElementPtr sdf,
            gazebo::physics::ModelPtr parent,
            const char * tag_name,
            const std::string & joint_default_name);
};

#endif  // GAZEBO_DDS_UTILS_H
