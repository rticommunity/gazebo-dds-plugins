#ifndef DATA_WRITER_LISTENER_CXX
#define DATA_WRITER_LISTENER_CXX

#include "../include/data-writer-listener.h"

template <class Type>
DataWriterListener<Type>::DataWriterListener()
{
}

template <class Type>
void DataWriterListener<Type>::on_publication_matched(
        dds::pub::DataWriter<Type> &writer,
        const dds::core::status::PublicationMatchedStatus &status)
{
    // if (status.current_count_change() < 0) {
    //     onDisconnect();
    // } else {
    //     onConnect();
    // }
}

#endif
