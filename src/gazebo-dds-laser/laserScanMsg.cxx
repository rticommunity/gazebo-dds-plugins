

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from laserScanMsg.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#include <iosfwd>
#include <iomanip>
#include "laserScanMsg.hpp"
#include "laserScanMsgImplPlugin.h"

#include <rti/util/ostream_operators.hpp>

// ---- Time: 

Time::Time() :
    m_sec_ (0u) ,
    m_nsec_ (0u) {
}   

Time::Time (
    uint32_t sec_param,
    uint32_t nsec_param)
    :
        m_sec_( sec_param ),
        m_nsec_( nsec_param ) {
}
#ifdef RTI_CXX11_RVALUE_REFERENCES
#ifdef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
Time::Time(Time&& other_) OMG_NOEXCEPT  :m_sec_ (std::move(other_.m_sec_))
,
m_nsec_ (std::move(other_.m_nsec_))
{
} 

Time& Time::operator=(Time&&  other_) OMG_NOEXCEPT {
    Time tmp(std::move(other_));
    swap(tmp); 
    return *this;
}
#endif
#endif   

void Time::swap(Time& other_)  OMG_NOEXCEPT 
{
    using std::swap;
    swap(m_sec_, other_.m_sec_);
    swap(m_nsec_, other_.m_nsec_);
}  

bool Time::operator == (const Time& other_) const {
    if (m_sec_ != other_.m_sec_) {
        return false;
    }
    if (m_nsec_ != other_.m_nsec_) {
        return false;
    }
    return true;
}
bool Time::operator != (const Time& other_) const {
    return !this->operator ==(other_);
}

// --- Getters and Setters: -------------------------------------------------
uint32_t Time::sec() const OMG_NOEXCEPT{
    return m_sec_;
}

void Time::sec(uint32_t value) {
    m_sec_ = value;
}

uint32_t Time::nsec() const OMG_NOEXCEPT{
    return m_nsec_;
}

void Time::nsec(uint32_t value) {
    m_nsec_ = value;
}

std::ostream& operator << (std::ostream& o,const Time& sample)
{
    rti::util::StreamFlagSaver flag_saver (o);
    o <<"[";
    o << "sec: " << sample.sec()<<", ";
    o << "nsec: " << sample.nsec() ;
    o <<"]";
    return o;
}

// ---- Header: 

Header::Header() :
    m_seq_ (0u) {
}   

Header::Header (
    uint32_t seq_param,
    const Time& stamp_param,
    const dds::core::string& frame_id_param)
    :
        m_seq_( seq_param ),
        m_stamp_( stamp_param ),
        m_frame_id_( frame_id_param ) {
}
#ifdef RTI_CXX11_RVALUE_REFERENCES
#ifdef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
Header::Header(Header&& other_) OMG_NOEXCEPT  :m_seq_ (std::move(other_.m_seq_))
,
m_stamp_ (std::move(other_.m_stamp_))
,
m_frame_id_ (std::move(other_.m_frame_id_))
{
} 

Header& Header::operator=(Header&&  other_) OMG_NOEXCEPT {
    Header tmp(std::move(other_));
    swap(tmp); 
    return *this;
}
#endif
#endif   

void Header::swap(Header& other_)  OMG_NOEXCEPT 
{
    using std::swap;
    swap(m_seq_, other_.m_seq_);
    swap(m_stamp_, other_.m_stamp_);
    swap(m_frame_id_, other_.m_frame_id_);
}  

bool Header::operator == (const Header& other_) const {
    if (m_seq_ != other_.m_seq_) {
        return false;
    }
    if (m_stamp_ != other_.m_stamp_) {
        return false;
    }
    if (m_frame_id_ != other_.m_frame_id_) {
        return false;
    }
    return true;
}
bool Header::operator != (const Header& other_) const {
    return !this->operator ==(other_);
}

// --- Getters and Setters: -------------------------------------------------
uint32_t Header::seq() const OMG_NOEXCEPT{
    return m_seq_;
}

void Header::seq(uint32_t value) {
    m_seq_ = value;
}

Time& Header::stamp() OMG_NOEXCEPT {
    return m_stamp_;
}

const Time& Header::stamp() const OMG_NOEXCEPT {
    return m_stamp_;
}

void Header::stamp(const Time& value) {
    m_stamp_ = value;
}

dds::core::string& Header::frame_id() OMG_NOEXCEPT {
    return m_frame_id_;
}

const dds::core::string& Header::frame_id() const OMG_NOEXCEPT {
    return m_frame_id_;
}

void Header::frame_id(const dds::core::string& value) {
    m_frame_id_ = value;
}

std::ostream& operator << (std::ostream& o,const Header& sample)
{
    rti::util::StreamFlagSaver flag_saver (o);
    o <<"[";
    o << "seq: " << sample.seq()<<", ";
    o << "stamp: " << sample.stamp()<<", ";
    o << "frame_id: " << sample.frame_id() ;
    o <<"]";
    return o;
}

// ---- laser_Scan_msg: 

laser_Scan_msg::laser_Scan_msg() :
    m_angle_min_ (0.0f) ,
    m_angle_max_ (0.0f) ,
    m_angle_increment_ (0.0f) ,
    m_time_increment_ (0.0f) ,
    m_scan_time_ (0.0f) ,
    m_range_min_ (0.0f) ,
    m_range_max_ (0.0f) {
}   

laser_Scan_msg::laser_Scan_msg (
    const Header& header_param,
    float angle_min_param,
    float angle_max_param,
    float angle_increment_param,
    float time_increment_param,
    float scan_time_param,
    float range_min_param,
    float range_max_param,
    const dds::core::vector<float>& ranges_param,
    const dds::core::vector<float>& intensities_param)
    :
        m_header_( header_param ),
        m_angle_min_( angle_min_param ),
        m_angle_max_( angle_max_param ),
        m_angle_increment_( angle_increment_param ),
        m_time_increment_( time_increment_param ),
        m_scan_time_( scan_time_param ),
        m_range_min_( range_min_param ),
        m_range_max_( range_max_param ),
        m_ranges_( ranges_param ),
        m_intensities_( intensities_param ) {
}
#ifdef RTI_CXX11_RVALUE_REFERENCES
#ifdef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
laser_Scan_msg::laser_Scan_msg(laser_Scan_msg&& other_) OMG_NOEXCEPT  :m_header_ (std::move(other_.m_header_))
,
m_angle_min_ (std::move(other_.m_angle_min_))
,
m_angle_max_ (std::move(other_.m_angle_max_))
,
m_angle_increment_ (std::move(other_.m_angle_increment_))
,
m_time_increment_ (std::move(other_.m_time_increment_))
,
m_scan_time_ (std::move(other_.m_scan_time_))
,
m_range_min_ (std::move(other_.m_range_min_))
,
m_range_max_ (std::move(other_.m_range_max_))
,
m_ranges_ (std::move(other_.m_ranges_))
,
m_intensities_ (std::move(other_.m_intensities_))
{
} 

laser_Scan_msg& laser_Scan_msg::operator=(laser_Scan_msg&&  other_) OMG_NOEXCEPT {
    laser_Scan_msg tmp(std::move(other_));
    swap(tmp); 
    return *this;
}
#endif
#endif   

void laser_Scan_msg::swap(laser_Scan_msg& other_)  OMG_NOEXCEPT 
{
    using std::swap;
    swap(m_header_, other_.m_header_);
    swap(m_angle_min_, other_.m_angle_min_);
    swap(m_angle_max_, other_.m_angle_max_);
    swap(m_angle_increment_, other_.m_angle_increment_);
    swap(m_time_increment_, other_.m_time_increment_);
    swap(m_scan_time_, other_.m_scan_time_);
    swap(m_range_min_, other_.m_range_min_);
    swap(m_range_max_, other_.m_range_max_);
    swap(m_ranges_, other_.m_ranges_);
    swap(m_intensities_, other_.m_intensities_);
}  

bool laser_Scan_msg::operator == (const laser_Scan_msg& other_) const {
    if (m_header_ != other_.m_header_) {
        return false;
    }
    if (m_angle_min_ != other_.m_angle_min_) {
        return false;
    }
    if (m_angle_max_ != other_.m_angle_max_) {
        return false;
    }
    if (m_angle_increment_ != other_.m_angle_increment_) {
        return false;
    }
    if (m_time_increment_ != other_.m_time_increment_) {
        return false;
    }
    if (m_scan_time_ != other_.m_scan_time_) {
        return false;
    }
    if (m_range_min_ != other_.m_range_min_) {
        return false;
    }
    if (m_range_max_ != other_.m_range_max_) {
        return false;
    }
    if (m_ranges_ != other_.m_ranges_) {
        return false;
    }
    if (m_intensities_ != other_.m_intensities_) {
        return false;
    }
    return true;
}
bool laser_Scan_msg::operator != (const laser_Scan_msg& other_) const {
    return !this->operator ==(other_);
}

// --- Getters and Setters: -------------------------------------------------
Header& laser_Scan_msg::header() OMG_NOEXCEPT {
    return m_header_;
}

const Header& laser_Scan_msg::header() const OMG_NOEXCEPT {
    return m_header_;
}

void laser_Scan_msg::header(const Header& value) {
    m_header_ = value;
}

float laser_Scan_msg::angle_min() const OMG_NOEXCEPT{
    return m_angle_min_;
}

void laser_Scan_msg::angle_min(float value) {
    m_angle_min_ = value;
}

float laser_Scan_msg::angle_max() const OMG_NOEXCEPT{
    return m_angle_max_;
}

void laser_Scan_msg::angle_max(float value) {
    m_angle_max_ = value;
}

float laser_Scan_msg::angle_increment() const OMG_NOEXCEPT{
    return m_angle_increment_;
}

void laser_Scan_msg::angle_increment(float value) {
    m_angle_increment_ = value;
}

float laser_Scan_msg::time_increment() const OMG_NOEXCEPT{
    return m_time_increment_;
}

void laser_Scan_msg::time_increment(float value) {
    m_time_increment_ = value;
}

float laser_Scan_msg::scan_time() const OMG_NOEXCEPT{
    return m_scan_time_;
}

void laser_Scan_msg::scan_time(float value) {
    m_scan_time_ = value;
}

float laser_Scan_msg::range_min() const OMG_NOEXCEPT{
    return m_range_min_;
}

void laser_Scan_msg::range_min(float value) {
    m_range_min_ = value;
}

float laser_Scan_msg::range_max() const OMG_NOEXCEPT{
    return m_range_max_;
}

void laser_Scan_msg::range_max(float value) {
    m_range_max_ = value;
}

dds::core::vector<float>& laser_Scan_msg::ranges() OMG_NOEXCEPT {
    return m_ranges_;
}

const dds::core::vector<float>& laser_Scan_msg::ranges() const OMG_NOEXCEPT {
    return m_ranges_;
}

void laser_Scan_msg::ranges(const dds::core::vector<float>& value) {
    m_ranges_ = value;
}

dds::core::vector<float>& laser_Scan_msg::intensities() OMG_NOEXCEPT {
    return m_intensities_;
}

const dds::core::vector<float>& laser_Scan_msg::intensities() const OMG_NOEXCEPT {
    return m_intensities_;
}

void laser_Scan_msg::intensities(const dds::core::vector<float>& value) {
    m_intensities_ = value;
}

std::ostream& operator << (std::ostream& o,const laser_Scan_msg& sample)
{
    rti::util::StreamFlagSaver flag_saver (o);
    o <<"[";
    o << "header: " << sample.header()<<", ";
    o << "angle_min: " << std::setprecision(9) <<sample.angle_min()<<", ";
    o << "angle_max: " << std::setprecision(9) <<sample.angle_max()<<", ";
    o << "angle_increment: " << std::setprecision(9) <<sample.angle_increment()<<", ";
    o << "time_increment: " << std::setprecision(9) <<sample.time_increment()<<", ";
    o << "scan_time: " << std::setprecision(9) <<sample.scan_time()<<", ";
    o << "range_min: " << std::setprecision(9) <<sample.range_min()<<", ";
    o << "range_max: " << std::setprecision(9) <<sample.range_max()<<", ";
    o << "ranges: " << sample.ranges()<<", ";
    o << "intensities: " << sample.intensities() ;
    o <<"]";
    return o;
}

// --- Type traits: -------------------------------------------------

namespace rti { 
    namespace topic {

        const dds::core::xtypes::StructType& dynamic_type<Time>::get()
        {
            return static_cast<const dds::core::xtypes::StructType&>(
                rti::core::native_conversions::cast_from_native<dds::core::xtypes::DynamicType>(
                    *(Time_c_get_typecode())));
        }

        const dds::core::xtypes::StructType& dynamic_type<Header>::get()
        {
            return static_cast<const dds::core::xtypes::StructType&>(
                rti::core::native_conversions::cast_from_native<dds::core::xtypes::DynamicType>(
                    *(Header_c_get_typecode())));
        }

        const dds::core::xtypes::StructType& dynamic_type<laser_Scan_msg>::get()
        {
            return static_cast<const dds::core::xtypes::StructType&>(
                rti::core::native_conversions::cast_from_native<dds::core::xtypes::DynamicType>(
                    *(laser_Scan_msg_c_get_typecode())));
        }

    }
}  

namespace dds { 
    namespace topic {
        void topic_type_support<Time>:: register_type(
            dds::domain::DomainParticipant& participant,
            const std::string& type_name){

            rti::domain::register_type_plugin(
                participant,
                type_name,
                Time_cPlugin_new,
                Time_cPlugin_delete);
        }

        void topic_type_support<Time>::initialize_sample(Time& sample){

            Time_c* native_sample=reinterpret_cast<Time_c*> (&sample);

            struct DDS_TypeDeallocationParams_t deAllocParams = {RTI_FALSE, RTI_FALSE};
            Time_c_finalize_w_params(native_sample,&deAllocParams);

            struct DDS_TypeAllocationParams_t allocParams = {RTI_FALSE, RTI_FALSE, RTI_TRUE}; 
            RTIBool ok=Time_c_initialize_w_params(native_sample,&allocParams);
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to initialize_w_params");

        } 

        std::vector<char>& topic_type_support<Time>::to_cdr_buffer(
            std::vector<char>& buffer, const Time& sample)
        {
            // First get the length of the buffer
            unsigned int length = 0;
            RTIBool ok = Time_cPlugin_serialize_to_cdr_buffer(
                NULL, &length,reinterpret_cast<const Time_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to calculate cdr buffer size");

            // Create a vector with that size and copy the cdr buffer into it
            buffer.resize(length);
            ok = Time_cPlugin_serialize_to_cdr_buffer(
                &buffer[0], &length, reinterpret_cast<const Time_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to copy cdr buffer");

            return buffer;

        }

        void topic_type_support<Time>::from_cdr_buffer(Time& sample, 
        const std::vector<char>& buffer)
        {

            RTIBool ok  = Time_cPlugin_deserialize_from_cdr_buffer(
                reinterpret_cast<Time_c*> (&sample), &buffer[0], 
                static_cast<unsigned int>(buffer.size()));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to create Time from cdr buffer");
        }

        void topic_type_support<Header>:: register_type(
            dds::domain::DomainParticipant& participant,
            const std::string& type_name){

            rti::domain::register_type_plugin(
                participant,
                type_name,
                Header_cPlugin_new,
                Header_cPlugin_delete);
        }

        void topic_type_support<Header>::initialize_sample(Header& sample){

            Header_c* native_sample=reinterpret_cast<Header_c*> (&sample);

            struct DDS_TypeDeallocationParams_t deAllocParams = {RTI_FALSE, RTI_FALSE};
            Header_c_finalize_w_params(native_sample,&deAllocParams);

            struct DDS_TypeAllocationParams_t allocParams = {RTI_FALSE, RTI_FALSE, RTI_TRUE}; 
            RTIBool ok=Header_c_initialize_w_params(native_sample,&allocParams);
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to initialize_w_params");

        } 

        std::vector<char>& topic_type_support<Header>::to_cdr_buffer(
            std::vector<char>& buffer, const Header& sample)
        {
            // First get the length of the buffer
            unsigned int length = 0;
            RTIBool ok = Header_cPlugin_serialize_to_cdr_buffer(
                NULL, &length,reinterpret_cast<const Header_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to calculate cdr buffer size");

            // Create a vector with that size and copy the cdr buffer into it
            buffer.resize(length);
            ok = Header_cPlugin_serialize_to_cdr_buffer(
                &buffer[0], &length, reinterpret_cast<const Header_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to copy cdr buffer");

            return buffer;

        }

        void topic_type_support<Header>::from_cdr_buffer(Header& sample, 
        const std::vector<char>& buffer)
        {

            RTIBool ok  = Header_cPlugin_deserialize_from_cdr_buffer(
                reinterpret_cast<Header_c*> (&sample), &buffer[0], 
                static_cast<unsigned int>(buffer.size()));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to create Header from cdr buffer");
        }

        void topic_type_support<laser_Scan_msg>:: register_type(
            dds::domain::DomainParticipant& participant,
            const std::string& type_name){

            rti::domain::register_type_plugin(
                participant,
                type_name,
                laser_Scan_msg_cPlugin_new,
                laser_Scan_msg_cPlugin_delete);
        }

        void topic_type_support<laser_Scan_msg>::initialize_sample(laser_Scan_msg& sample){

            laser_Scan_msg_c* native_sample=reinterpret_cast<laser_Scan_msg_c*> (&sample);

            struct DDS_TypeDeallocationParams_t deAllocParams = {RTI_FALSE, RTI_FALSE};
            laser_Scan_msg_c_finalize_w_params(native_sample,&deAllocParams);

            struct DDS_TypeAllocationParams_t allocParams = {RTI_FALSE, RTI_FALSE, RTI_TRUE}; 
            RTIBool ok=laser_Scan_msg_c_initialize_w_params(native_sample,&allocParams);
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to initialize_w_params");

        } 

        std::vector<char>& topic_type_support<laser_Scan_msg>::to_cdr_buffer(
            std::vector<char>& buffer, const laser_Scan_msg& sample)
        {
            // First get the length of the buffer
            unsigned int length = 0;
            RTIBool ok = laser_Scan_msg_cPlugin_serialize_to_cdr_buffer(
                NULL, &length,reinterpret_cast<const laser_Scan_msg_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to calculate cdr buffer size");

            // Create a vector with that size and copy the cdr buffer into it
            buffer.resize(length);
            ok = laser_Scan_msg_cPlugin_serialize_to_cdr_buffer(
                &buffer[0], &length, reinterpret_cast<const laser_Scan_msg_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to copy cdr buffer");

            return buffer;

        }

        void topic_type_support<laser_Scan_msg>::from_cdr_buffer(laser_Scan_msg& sample, 
        const std::vector<char>& buffer)
        {

            RTIBool ok  = laser_Scan_msg_cPlugin_deserialize_from_cdr_buffer(
                reinterpret_cast<laser_Scan_msg_c*> (&sample), &buffer[0], 
                static_cast<unsigned int>(buffer.size()));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to create laser_Scan_msg from cdr buffer");
        }

    }
}  

