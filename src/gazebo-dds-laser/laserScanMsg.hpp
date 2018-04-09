

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from laserScanMsg.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef laserScanMsg_1972819042_hpp
#define laserScanMsg_1972819042_hpp

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

    Time m_stamp_;
    dds::core::string m_frame_id_;

};

inline void swap(Header& a, Header& b)  OMG_NOEXCEPT 
{
    a.swap(b);
}

NDDSUSERDllExport std::ostream& operator<<(std::ostream& o,const Header& sample);

class NDDSUSERDllExport Position {

  public:
    Position();

    Position(
        float x_param,
        float y_param,
        float z_param);

    #ifdef RTI_CXX11_RVALUE_REFERENCES
    #ifndef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
    Position (Position&&) = default;
    Position& operator=(Position&&) = default;
    Position& operator=(const Position&) = default;
    Position(const Position&) = default;
    #else
    Position(Position&& other_) OMG_NOEXCEPT;  
    Position& operator=(Position&&  other_) OMG_NOEXCEPT;
    #endif
    #endif 

    float x() const OMG_NOEXCEPT;
    void x(float value);

    float y() const OMG_NOEXCEPT;
    void y(float value);

    float z() const OMG_NOEXCEPT;
    void z(float value);

    bool operator == (const Position& other_) const;
    bool operator != (const Position& other_) const;

    void swap(Position& other_) OMG_NOEXCEPT;

  private:

    float m_x_;
    float m_y_;
    float m_z_;

};

inline void swap(Position& a, Position& b)  OMG_NOEXCEPT 
{
    a.swap(b);
}

NDDSUSERDllExport std::ostream& operator<<(std::ostream& o,const Position& sample);

class NDDSUSERDllExport Orientation {

  public:
    Orientation();

    Orientation(
        float x_param,
        float y_param,
        float z_param,
        float w_param);

    #ifdef RTI_CXX11_RVALUE_REFERENCES
    #ifndef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
    Orientation (Orientation&&) = default;
    Orientation& operator=(Orientation&&) = default;
    Orientation& operator=(const Orientation&) = default;
    Orientation(const Orientation&) = default;
    #else
    Orientation(Orientation&& other_) OMG_NOEXCEPT;  
    Orientation& operator=(Orientation&&  other_) OMG_NOEXCEPT;
    #endif
    #endif 

    float x() const OMG_NOEXCEPT;
    void x(float value);

    float y() const OMG_NOEXCEPT;
    void y(float value);

    float z() const OMG_NOEXCEPT;
    void z(float value);

    float w() const OMG_NOEXCEPT;
    void w(float value);

    bool operator == (const Orientation& other_) const;
    bool operator != (const Orientation& other_) const;

    void swap(Orientation& other_) OMG_NOEXCEPT;

  private:

    float m_x_;
    float m_y_;
    float m_z_;
    float m_w_;

};

inline void swap(Orientation& a, Orientation& b)  OMG_NOEXCEPT 
{
    a.swap(b);
}

NDDSUSERDllExport std::ostream& operator<<(std::ostream& o,const Orientation& sample);

class NDDSUSERDllExport World_Pose {

  public:
    World_Pose();

    World_Pose(
        const Position& position_param,
        const Orientation& orientation_param);

    #ifdef RTI_CXX11_RVALUE_REFERENCES
    #ifndef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
    World_Pose (World_Pose&&) = default;
    World_Pose& operator=(World_Pose&&) = default;
    World_Pose& operator=(const World_Pose&) = default;
    World_Pose(const World_Pose&) = default;
    #else
    World_Pose(World_Pose&& other_) OMG_NOEXCEPT;  
    World_Pose& operator=(World_Pose&&  other_) OMG_NOEXCEPT;
    #endif
    #endif 

    Position& position() OMG_NOEXCEPT; 
    const Position& position() const OMG_NOEXCEPT;
    void position(const Position& value);

    Orientation& orientation() OMG_NOEXCEPT; 
    const Orientation& orientation() const OMG_NOEXCEPT;
    void orientation(const Orientation& value);

    bool operator == (const World_Pose& other_) const;
    bool operator != (const World_Pose& other_) const;

    void swap(World_Pose& other_) OMG_NOEXCEPT;

  private:

    Position m_position_;
    Orientation m_orientation_;

};

inline void swap(World_Pose& a, World_Pose& b)  OMG_NOEXCEPT 
{
    a.swap(b);
}

NDDSUSERDllExport std::ostream& operator<<(std::ostream& o,const World_Pose& sample);

class NDDSUSERDllExport laser_Scan_msg {

  public:
    laser_Scan_msg();

    laser_Scan_msg(
        int32_t laser_id_param,
        const Header& header_param,
        const World_Pose& world_pose_param,
        float angle_min_param,
        float angle_max_param,
        float angle_step_param,
        float range_min_param,
        float range_max_param,
        int32_t count_param,
        float vertical_angle_min_param,
        float vertical_angle_max_param,
        float vertical_angle_step_param,
        float vertical_count_param,
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

    int32_t laser_id() const OMG_NOEXCEPT;
    void laser_id(int32_t value);

    Header& header() OMG_NOEXCEPT; 
    const Header& header() const OMG_NOEXCEPT;
    void header(const Header& value);

    World_Pose& world_pose() OMG_NOEXCEPT; 
    const World_Pose& world_pose() const OMG_NOEXCEPT;
    void world_pose(const World_Pose& value);

    float angle_min() const OMG_NOEXCEPT;
    void angle_min(float value);

    float angle_max() const OMG_NOEXCEPT;
    void angle_max(float value);

    float angle_step() const OMG_NOEXCEPT;
    void angle_step(float value);

    float range_min() const OMG_NOEXCEPT;
    void range_min(float value);

    float range_max() const OMG_NOEXCEPT;
    void range_max(float value);

    int32_t count() const OMG_NOEXCEPT;
    void count(int32_t value);

    float vertical_angle_min() const OMG_NOEXCEPT;
    void vertical_angle_min(float value);

    float vertical_angle_max() const OMG_NOEXCEPT;
    void vertical_angle_max(float value);

    float vertical_angle_step() const OMG_NOEXCEPT;
    void vertical_angle_step(float value);

    float vertical_count() const OMG_NOEXCEPT;
    void vertical_count(float value);

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

    int32_t m_laser_id_;
    Header m_header_;
    World_Pose m_world_pose_;
    float m_angle_min_;
    float m_angle_max_;
    float m_angle_step_;
    float m_range_min_;
    float m_range_max_;
    int32_t m_count_;
    float m_vertical_angle_min_;
    float m_vertical_angle_max_;
    float m_vertical_angle_step_;
    float m_vertical_count_;
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
        struct topic_type_name<Position> {
            NDDSUSERDllExport static std::string value() {
                return "Position";
            }
        };

        template<>
        struct is_topic_type<Position> : public dds::core::true_type {};

        template<>
        struct topic_type_support<Position> {

            NDDSUSERDllExport static void initialize_sample(Position& sample);

            NDDSUSERDllExport static void register_type(
                dds::domain::DomainParticipant& participant,
                const std::string & type_name);

            NDDSUSERDllExport static std::vector<char>& to_cdr_buffer(
                std::vector<char>& buffer, const Position& sample);

            NDDSUSERDllExport static void from_cdr_buffer(Position& sample, const std::vector<char>& buffer);

            static const rti::topic::TypePluginKind::type type_plugin_kind = 
            rti::topic::TypePluginKind::NON_STL;
        };

        template<>
        struct topic_type_name<Orientation> {
            NDDSUSERDllExport static std::string value() {
                return "Orientation";
            }
        };

        template<>
        struct is_topic_type<Orientation> : public dds::core::true_type {};

        template<>
        struct topic_type_support<Orientation> {

            NDDSUSERDllExport static void initialize_sample(Orientation& sample);

            NDDSUSERDllExport static void register_type(
                dds::domain::DomainParticipant& participant,
                const std::string & type_name);

            NDDSUSERDllExport static std::vector<char>& to_cdr_buffer(
                std::vector<char>& buffer, const Orientation& sample);

            NDDSUSERDllExport static void from_cdr_buffer(Orientation& sample, const std::vector<char>& buffer);

            static const rti::topic::TypePluginKind::type type_plugin_kind = 
            rti::topic::TypePluginKind::NON_STL;
        };

        template<>
        struct topic_type_name<World_Pose> {
            NDDSUSERDllExport static std::string value() {
                return "World_Pose";
            }
        };

        template<>
        struct is_topic_type<World_Pose> : public dds::core::true_type {};

        template<>
        struct topic_type_support<World_Pose> {

            NDDSUSERDllExport static void initialize_sample(World_Pose& sample);

            NDDSUSERDllExport static void register_type(
                dds::domain::DomainParticipant& participant,
                const std::string & type_name);

            NDDSUSERDllExport static std::vector<char>& to_cdr_buffer(
                std::vector<char>& buffer, const World_Pose& sample);

            NDDSUSERDllExport static void from_cdr_buffer(World_Pose& sample, const std::vector<char>& buffer);

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
        struct dynamic_type<Position> {
            typedef dds::core::xtypes::StructType type;
            NDDSUSERDllExport static const dds::core::xtypes::StructType& get();
        };

        template<>
        struct impl_type<Position> {
            typedef Position_c type;
        };

        template<>
        struct dynamic_type<Orientation> {
            typedef dds::core::xtypes::StructType type;
            NDDSUSERDllExport static const dds::core::xtypes::StructType& get();
        };

        template<>
        struct impl_type<Orientation> {
            typedef Orientation_c type;
        };

        template<>
        struct dynamic_type<World_Pose> {
            typedef dds::core::xtypes::StructType type;
            NDDSUSERDllExport static const dds::core::xtypes::StructType& get();
        };

        template<>
        struct impl_type<World_Pose> {
            typedef World_Pose_c type;
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

#endif // laserScanMsg_1972819042_hpp

