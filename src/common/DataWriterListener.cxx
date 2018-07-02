#include "DataWriterListener.h"

namespace gazebo { namespace dds {

template <typename T>
DataWriterListener<T>::DataWriterListener()
{
}

template <typename T>
DataWriterListener<T>::DataWriterListener(
        const std::function<void()> &on_connect,
        const std::function<void()> &on_disconnect)
        : on_connect_(on_connect), on_disconnect_(on_disconnect)
{
}

template <typename T>
void DataWriterListener<T>::on_publication_matched(
        ::dds::pub::DataWriter<T> &writer,
        const ::dds::core::status::PublicationMatchedStatus &status)
{
    if (status.current_count_change() < 0) {
        on_disconnect_();
    } else {
        on_connect_();
    }
}

}  // namespace dds
}  // namespace gazebo
