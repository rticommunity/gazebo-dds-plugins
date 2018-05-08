#ifndef DATA_WRITER_LISTENER_H
#define DATA_WRITER_LISTENER_H

#include <dds/core/ddscore.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <rti/core/ListenerBinder.hpp>

#include <rti/util/util.hpp>

#include <dds/dds.hpp>


template <typename Type>
class DataWriterListener : public dds::pub::NoOpDataWriterListener<Type> {
public:
    DataWriterListener();

    DataWriterListener(std::function<void()> on_con,
    std::function<void()> on_discon);

    virtual void on_publication_matched(
            dds::pub::DataWriter<Type> &writer,
            const dds::core::status::PublicationMatchedStatus &status);

private:
    std::function<void()> on_connect_;
    std::function<void()> on_disconnect_;
};

#endif