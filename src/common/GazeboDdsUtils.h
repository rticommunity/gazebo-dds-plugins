#ifndef GAZEBO_DDS_UTILS_H
#define GAZEBO_DDS_UTILS_H

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
    static void GetParam(
            sdf::ElementPtr sdf,
            T &value,
            const char *tag_name,
            const T &default_value);
};

#endif  // GAZEBO_DDS_UTILS_H
