#include <dds/sub/ddssub.hpp>

namespace gazebo { namespace dds {

template <typename T>
class DataReaderListener : public ::dds::sub::NoOpDataReaderListener<T> {
public:
    /**
     * @brief Constructor
     *
     * @param on_data on data callback
     */
    DataReaderListener(const std::function<void(const T &sample)> &on_data);

    /**
     * @brief On data available
     *
     * @param reader datareader that have data available
     */
    virtual void on_data_available(::dds::sub::DataReader<T>& reader);

private:
    std::function<void(const T &sample)> on_data_;
};

}  // namespace dds
}  // namespace gazebo
