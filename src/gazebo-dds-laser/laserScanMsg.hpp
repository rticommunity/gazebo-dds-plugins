

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from laserScanMsg.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef laserScanMsg_1972817541_hpp
#define laserScanMsg_1972817541_hpp

#include <iosfwd>
#include "laserScanMsgImpl.h"

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef RTIUSERDllExport
#define RTIUSERDllExport __declspec(dllexport)
#endif

#include "dds/domain/DomainParticipant.hpp"
#include "dds/topic/TopicTraits.hpp"
#include "dds/core/SafeEnumeration.hpp"
#include "dds/core/String.hpp"
#include "dds/core/array.hpp"
#include "dds/core/vector.hpp"
#include "dds/core/Optional.hpp"
#include "dds/core/xtypes/DynamicType.hpp"
#include "dds/core/xtypes/StructType.hpp"
#include "dds/core/xtypes/UnionType.hpp"
#include "dds/core/xtypes/EnumType.hpp"
#include "dds/core/xtypes/AliasType.hpp"
#include "rti/core/array.hpp"
#include "rti/util/StreamFlagSaver.hpp"
#include "rti/domain/PluginSupport.hpp"
#include "rti/core/LongDouble.hpp"
#include "rti/core/Pointer.hpp"
#include "rti/topic/TopicTraits.hpp"
#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef RTIUSERDllExport
#define RTIUSERDllExport
#endif

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif

class NDDSUSERDllExport Time {

  public:
    Time();

    Time(
        uint32_t sec_param,
        uint32_t nsec_param);

    #ifdef RTI_CXX11_RVALUE_REFERENCES
    #ifndef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
    Time (Time&&) = default;
    Time& operator=(Time&&) = default;
    Time& operator=(const Time&) = default;
    Time(const Time&) = default;
    #else
    Time(Time&& other_) OMG_NOEXCEPT;  
    Time& operator=(Time&&  other_) OMG_NOEXCEPT;
    #endif
    #endif 

    uint32_t sec() const OMG_NOEXCEPT;
    void sec(uint32_t value);

    uint32_t nsec() const OMG_NOEXCEPT;
    void nsec(uint32_t value);

    bool operator == (const Time& other_) const;
    bool operator != (const Time& other_) const;

    void swap(Time& other_) OMG_NOEXCEPT;

  private:

    uint32_t m_sec_;
    uint32_t m_nsec_;

};

inline void swap(Time& a, Time& b)  OMG_NOEXCEPT 
{
    a.swap(b);
}

NDDSUSERDllExport std::ostream& operator<<(std::ostream& o,const Time& sample);

class NDDSUSERDllExport Header {

  public:
    Header();

    Header(
        uint32_t seq_param,
        const Time& stamp_param,
        const dds::core::string& frame_id_param);

    #ifdef RTI_CXX11_RVALUE_REFERENCES
    #ifndef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
    Header (Header&&) = default;
    Header& operator=(Header&&) = default;
    Header& operator=(const Header&) = default;
    Header(const Header&) = default;
    #else
    Header(Header&& other_) OMG_NOEXCEPT;  
    Header& operator=(Header&&  other_) OMG_NOEXCEPT;
    #endif
    #endif 

    uint32_t seq() const OMG_NOEXCEPT;
    void seq(uint32_t value);

    Time& stamp() OMG_NOEXCEPT; 
    const Time& stamp() const OMG_NOEXCEPT;
    void stamp(const Time& value);

    dds::core::string& frame_id() OMG_NOEXCEPT; 
    const dds::core::string& frame_id() const OMG_NOEXCEPT;
    void frame_id(const dds::core::string& value);

    bool operator == (const Header& other_) const;
    bool operator != (const Header& other_) const;

    void swap(Header& other_) OMG_NOEXCEPT;

  private:

    uint32_t m_seq_;
    Time m_stamp_;
    dds::core::string m_frame_id_;

};

inline void swap(Header& a, Header& b)  OMG_NOEXCEPT 
{
    a.swap(b);
}

NDDSUSERDllExport std::ostream& operator<<(std::ostream& o,const Header& sample);

class NDDSUSERDllExport laser_Scan_msg {

  public:
    laser_Scan_msg();

    laser_Scan_msg(
        const Header& header_param,
        float angle_min_param,
        float angle_max_param,
        float angle_increment_param,
        float time_increment_param,
        float scan_time_param,
        float range_min_param,
        float range_max_param,
        const dds::core::vector<float>& ranges_param,
        const dds::core::vector<float>& intensities_param);

    #ifdef RTI_CXX11_RVALUE_REFERENCES
    #ifndef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
    laser_Scan_msg (laser_Scan_msg&&) = default;
    laser_Scan_msg& operator=(laser_Scan_msg&&) = default;
    laser_Scan_msg& operator=(const laser_Scan_msg&) = default;
    laser_Scan_msg(const laser_Scan_msg&) = default;
    #else
    laser_Scan_msg(laser_Scan_msg&& other_) OMG_NOEXCEPT;  
    laser_Scan_msg& operator=(laser_Scan_msg&&  other_) OMG_NOEXCEPT;
    #endif
    #endif 

    Header& header() OMG_NOEXCEPT; 
    const Header& header() const OMG_NOEXCEPT;
    void header(const Header& value);

    float angle_min() const OMG_NOEXCEPT;
    void angle_min(float value);

    float angle_max() const OMG_NOEXCEPT;
    void angle_max(float value);

    float angle_increment() const OMG_NOEXCEPT;
    void angle_increment(float value);

    float time_increment() const OMG_NOEXCEPT;
    void time_increment(float value);

    float scan_time() const OMG_NOEXCEPT;
    void scan_time(float value);

    float range_min() const OMG_NOEXCEPT;
    void range_min(float value);

    float range_max() const OMG_NOEXCEPT;
    void range_max(float value);

    dds::core::vector<float>& ranges() OMG_NOEXCEPT; 
    const dds::core::vector<float>& ranges() const OMG_NOEXCEPT;
    void ranges(const dds::core::vector<float>& value);

    dds::core::vector<float>& intensities() OMG_NOEXCEPT; 
    const dds::core::vector<float>& intensities() const OMG_NOEXCEPT;
    void intensities(const dds::core::vector<float>& value);

    bool operator == (const laser_Scan_msg& other_) const;
    bool operator != (const laser_Scan_msg& other_) const;

    void swap(laser_Scan_msg& other_) OMG_NOEXCEPT;

  private:

    Header m_header_;
    float m_angle_min_;
    float m_angle_max_;
    float m_angle_increment_;
    float m_time_increment_;
    float m_scan_time_;
    float m_range_min_;
    float m_range_max_;
    dds::core::vector<float> m_ranges_;
    dds::core::vector<float> m_intensities_;

};

inline void swap(laser_Scan_msg& a, laser_Scan_msg& b)  OMG_NOEXCEPT 
{
    a.swap(b);
}

NDDSUSERDllExport std::ostream& operator<<(std::ostream& o,const laser_Scan_msg& sample);

namespace dds { 
    namespace topic {

        template<>
        struct topic_type_name<Time> {
            NDDSUSERDllExport static std::string value() {
                return "Time";
            }
        };

        template<>
        struct is_topic_type<Time> : public dds::core::true_type {};

        template<>
        struct topic_type_support<Time> {

            NDDSUSERDllExport static void initialize_sample(Time& sample);

            NDDSUSERDllExport static void register_type(
                dds::domain::DomainParticipant& participant,
                const std::string & type_name);

            NDDSUSERDllExport static std::vector<char>& to_cdr_buffer(
                std::vector<char>& buffer, const Time& sample);

            NDDSUSERDllExport static void from_cdr_buffer(Time& sample, const std::vector<char>& buffer);

            static const rti::topic::TypePluginKind::type type_plugin_kind = 
            rti::topic::TypePluginKind::NON_STL;
        };

        template<>
        struct topic_type_name<Header> {
            NDDSUSERDllExport static std::string value() {
                return "Header";
            }
        };

        template<>
        struct is_topic_type<Header> : public dds::core::true_type {};

        template<>
        struct topic_type_support<Header> {

            NDDSUSERDllExport static void initialize_sample(Header& sample);

            NDDSUSERDllExport static void register_type(
                dds::domain::DomainParticipant& participant,
                const std::string & type_name);

            NDDSUSERDllExport static std::vector<char>& to_cdr_buffer(
                std::vector<char>& buffer, const Header& sample);

            NDDSUSERDllExport static void from_cdr_buffer(Header& sample, const std::vector<char>& buffer);

            static const rti::topic::TypePluginKind::type type_plugin_kind = 
            rti::topic::TypePluginKind::NON_STL;
        };

        template<>
        struct topic_type_name<laser_Scan_msg> {
            NDDSUSERDllExport static std::string value() {
                return "laser_Scan_msg";
            }
        };

        template<>
        struct is_topic_type<laser_Scan_msg> : public dds::core::true_type {};

        template<>
        struct topic_type_support<laser_Scan_msg> {

            NDDSUSERDllExport static void initialize_sample(laser_Scan_msg& sample);

            NDDSUSERDllExport static void register_type(
                dds::domain::DomainParticipant& participant,
                const std::string & type_name);

            NDDSUSERDllExport static std::vector<char>& to_cdr_buffer(
                std::vector<char>& buffer, const laser_Scan_msg& sample);

            NDDSUSERDllExport static void from_cdr_buffer(laser_Scan_msg& sample, const std::vector<char>& buffer);

            static const rti::topic::TypePluginKind::type type_plugin_kind = 
            rti::topic::TypePluginKind::NON_STL;
        };

    }
}

namespace rti { 
    namespace topic {
        template<>
        struct dynamic_type<Time> {
            typedef dds::core::xtypes::StructType type;
            NDDSUSERDllExport static const dds::core::xtypes::StructType& get();
        };

        template<>
        struct impl_type<Time> {
            typedef Time_c type;
        };

        template<>
        struct dynamic_type<Header> {
            typedef dds::core::xtypes::StructType type;
            NDDSUSERDllExport static const dds::core::xtypes::StructType& get();
        };

        template<>
        struct impl_type<Header> {
            typedef Header_c type;
        };

        template<>
        struct dynamic_type<laser_Scan_msg> {
            typedef dds::core::xtypes::StructType type;
            NDDSUSERDllExport static const dds::core::xtypes::StructType& get();
        };

        template<>
        struct impl_type<laser_Scan_msg> {
            typedef laser_Scan_msg_c type;
        };

    }
}

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

#endif // laserScanMsg_1972817541_hpp

