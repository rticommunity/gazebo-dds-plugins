#ifndef DATA_WRITER_LISTENER_H
#define DATA_WRITER_LISTENER_H

#include <dds/core/ddscore.hpp>
#include <dds/pub/ddspub.hpp>

template <typename T>
class DataWriterListener : public dds::pub::NoOpDataWriterListener<T> {
public:
    /**
     * @brief Constructor
     */
    DataWriterListener();

    /**
     * @brief Constructor
     *
     * @param on_connect on connect callback
     * @param on_disconnect on disconnect callback
     */
    DataWriterListener(
            std::function<void()> on_connect,
            std::function<void()> on_disconnect);

    /**
     * @brief On publication matched
     *
     * @param writer datawriter that did match
     * @param status status of the match
     */
    virtual void on_publication_matched(
            dds::pub::DataWriter<T> &writer,
            const dds::core::status::PublicationMatchedStatus &status);

private:
    std::function<void()> on_connect_;
    std::function<void()> on_disconnect_;
};

#endif  // DATA_WRITER_LISTENER_H
