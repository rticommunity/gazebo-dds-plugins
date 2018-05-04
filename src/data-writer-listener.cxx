#ifndef DATA_WRITER_LISTENER_CXX
#define DATA_WRITER_LISTENER_CXX

#include "../include/data-writer-listener.h"

template <class Type>
DataWriterListener<Type>::DataWriterListener()
{
}

template <class Type>
DataWriterListener<Type>::DataWriterListener(
        std::function<void()> on_con,
        std::function<void()> on_discon)
{
    on_connect_ = on_con;
    on_disconnect_ = on_discon;
}

template <class Type>
void DataWriterListener<Type>::on_publication_matched(
        dds::pub::DataWriter<Type> &writer,
        const dds::core::status::PublicationMatchedStatus &status)
{
    if (status.current_count_change() < 0) {
        on_disconnect_();
    } else {
        on_connect_();
    }
}

#endif
