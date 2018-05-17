#ifndef DATA_READER_LISTENER_H
#define DATA_READER_LISTENER_H

#include <dds/sub/ddssub.hpp>

template <typename T>
class DataReaderListener : public dds::sub::NoOpDataReaderListener<T> {
public:
    /**
     * @brief Constructor
     *
     * @param on_dana on data callback
     */
    DataReaderListener(const std::function<void(T)> &on_data);

    /**
     * @brief On data available
     *
     * @param reader datareader that have data available
     */
    virtual void on_data_available(dds::sub::DataReader<T>& reader);

private:
    std::function<void(T)> on_data_;
};

#endif  // DATA_READER_LISTENER_H
