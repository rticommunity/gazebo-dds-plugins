#ifndef GAZEBO_DDS_UTILS_CXX
#define GAZEBO_DDS_UTILS_CXX

#include "GazeboDdsUtils.h"

template <class T>
void GazeboDdsUtils::GetParam(
        sdf::ElementPtr sdf,
        T & tag_variable,
        const char *tag_name,
        const T &default_value)
{
    tag_variable = default_value;
    if (sdf->HasElement(tag_name)) {
        tag_variable = sdf->Get<T>(tag_name);
    } else {
        tag_variable = 0;
        gzwarn << "Missing <" << tag_name << ">, set to default: " << tag_variable
               << std::endl;
    }
}

#endif  // DATA_WRITER_LISTENER_CXX