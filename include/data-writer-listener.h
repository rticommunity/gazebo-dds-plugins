#ifndef DATA_WRITER_LISTENER_H
#define DATA_WRITER_LISTENER_H

#include <dds/core/ddscore.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <rti/core/ListenerBinder.hpp>
#include <rti/util/util.hpp>

#include <dds/dds.hpp>


template <class Type>
class DataWriterListener : public dds::pub::NoOpDataWriterListener<Type> {
public:
    DataWriterListener();
    virtual void on_publication_matched(
            dds::pub::DataWriter<Type> &writer,
            const dds::core::status::PublicationMatchedStatus &status);

private:
};

#endif