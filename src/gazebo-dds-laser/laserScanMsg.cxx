

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

Header::Header() {
}   

Header::Header (
    const Time& stamp_param,
    const dds::core::string& frame_id_param)
    :
        m_stamp_( stamp_param ),
        m_frame_id_( frame_id_param ) {
}
#ifdef RTI_CXX11_RVALUE_REFERENCES
#ifdef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
Header::Header(Header&& other_) OMG_NOEXCEPT  :m_stamp_ (std::move(other_.m_stamp_))
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
    swap(m_stamp_, other_.m_stamp_);
    swap(m_frame_id_, other_.m_frame_id_);
}  

bool Header::operator == (const Header& other_) const {
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
    o << "stamp: " << sample.stamp()<<", ";
    o << "frame_id: " << sample.frame_id() ;
    o <<"]";
    return o;
}

// ---- Position: 

Position::Position() :
    m_x_ (0.0f) ,
    m_y_ (0.0f) ,
    m_z_ (0.0f) {
}   

Position::Position (
    float x_param,
    float y_param,
    float z_param)
    :
        m_x_( x_param ),
        m_y_( y_param ),
        m_z_( z_param ) {
}
#ifdef RTI_CXX11_RVALUE_REFERENCES
#ifdef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
Position::Position(Position&& other_) OMG_NOEXCEPT  :m_x_ (std::move(other_.m_x_))
,
m_y_ (std::move(other_.m_y_))
,
m_z_ (std::move(other_.m_z_))
{
} 

Position& Position::operator=(Position&&  other_) OMG_NOEXCEPT {
    Position tmp(std::move(other_));
    swap(tmp); 
    return *this;
}
#endif
#endif   

void Position::swap(Position& other_)  OMG_NOEXCEPT 
{
    using std::swap;
    swap(m_x_, other_.m_x_);
    swap(m_y_, other_.m_y_);
    swap(m_z_, other_.m_z_);
}  

bool Position::operator == (const Position& other_) const {
    if (m_x_ != other_.m_x_) {
        return false;
    }
    if (m_y_ != other_.m_y_) {
        return false;
    }
    if (m_z_ != other_.m_z_) {
        return false;
    }
    return true;
}
bool Position::operator != (const Position& other_) const {
    return !this->operator ==(other_);
}

// --- Getters and Setters: -------------------------------------------------
float Position::x() const OMG_NOEXCEPT{
    return m_x_;
}

void Position::x(float value) {
    m_x_ = value;
}

float Position::y() const OMG_NOEXCEPT{
    return m_y_;
}

void Position::y(float value) {
    m_y_ = value;
}

float Position::z() const OMG_NOEXCEPT{
    return m_z_;
}

void Position::z(float value) {
    m_z_ = value;
}

std::ostream& operator << (std::ostream& o,const Position& sample)
{
    rti::util::StreamFlagSaver flag_saver (o);
    o <<"[";
    o << "x: " << std::setprecision(9) <<sample.x()<<", ";
    o << "y: " << std::setprecision(9) <<sample.y()<<", ";
    o << "z: " << std::setprecision(9) <<sample.z() ;
    o <<"]";
    return o;
}

// ---- Orientation: 

Orientation::Orientation() :
    m_x_ (0.0f) ,
    m_y_ (0.0f) ,
    m_z_ (0.0f) ,
    m_w_ (0.0f) {
}   

Orientation::Orientation (
    float x_param,
    float y_param,
    float z_param,
    float w_param)
    :
        m_x_( x_param ),
        m_y_( y_param ),
        m_z_( z_param ),
        m_w_( w_param ) {
}
#ifdef RTI_CXX11_RVALUE_REFERENCES
#ifdef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
Orientation::Orientation(Orientation&& other_) OMG_NOEXCEPT  :m_x_ (std::move(other_.m_x_))
,
m_y_ (std::move(other_.m_y_))
,
m_z_ (std::move(other_.m_z_))
,
m_w_ (std::move(other_.m_w_))
{
} 

Orientation& Orientation::operator=(Orientation&&  other_) OMG_NOEXCEPT {
    Orientation tmp(std::move(other_));
    swap(tmp); 
    return *this;
}
#endif
#endif   

void Orientation::swap(Orientation& other_)  OMG_NOEXCEPT 
{
    using std::swap;
    swap(m_x_, other_.m_x_);
    swap(m_y_, other_.m_y_);
    swap(m_z_, other_.m_z_);
    swap(m_w_, other_.m_w_);
}  

bool Orientation::operator == (const Orientation& other_) const {
    if (m_x_ != other_.m_x_) {
        return false;
    }
    if (m_y_ != other_.m_y_) {
        return false;
    }
    if (m_z_ != other_.m_z_) {
        return false;
    }
    if (m_w_ != other_.m_w_) {
        return false;
    }
    return true;
}
bool Orientation::operator != (const Orientation& other_) const {
    return !this->operator ==(other_);
}

// --- Getters and Setters: -------------------------------------------------
float Orientation::x() const OMG_NOEXCEPT{
    return m_x_;
}

void Orientation::x(float value) {
    m_x_ = value;
}

float Orientation::y() const OMG_NOEXCEPT{
    return m_y_;
}

void Orientation::y(float value) {
    m_y_ = value;
}

float Orientation::z() const OMG_NOEXCEPT{
    return m_z_;
}

void Orientation::z(float value) {
    m_z_ = value;
}

float Orientation::w() const OMG_NOEXCEPT{
    return m_w_;
}

void Orientation::w(float value) {
    m_w_ = value;
}

std::ostream& operator << (std::ostream& o,const Orientation& sample)
{
    rti::util::StreamFlagSaver flag_saver (o);
    o <<"[";
    o << "x: " << std::setprecision(9) <<sample.x()<<", ";
    o << "y: " << std::setprecision(9) <<sample.y()<<", ";
    o << "z: " << std::setprecision(9) <<sample.z()<<", ";
    o << "w: " << std::setprecision(9) <<sample.w() ;
    o <<"]";
    return o;
}

// ---- World_Pose: 

World_Pose::World_Pose() {
}   

World_Pose::World_Pose (
    const Position& position_param,
    const Orientation& orientation_param)
    :
        m_position_( position_param ),
        m_orientation_( orientation_param ) {
}
#ifdef RTI_CXX11_RVALUE_REFERENCES
#ifdef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
World_Pose::World_Pose(World_Pose&& other_) OMG_NOEXCEPT  :m_position_ (std::move(other_.m_position_))
,
m_orientation_ (std::move(other_.m_orientation_))
{
} 

World_Pose& World_Pose::operator=(World_Pose&&  other_) OMG_NOEXCEPT {
    World_Pose tmp(std::move(other_));
    swap(tmp); 
    return *this;
}
#endif
#endif   

void World_Pose::swap(World_Pose& other_)  OMG_NOEXCEPT 
{
    using std::swap;
    swap(m_position_, other_.m_position_);
    swap(m_orientation_, other_.m_orientation_);
}  

bool World_Pose::operator == (const World_Pose& other_) const {
    if (m_position_ != other_.m_position_) {
        return false;
    }
    if (m_orientation_ != other_.m_orientation_) {
        return false;
    }
    return true;
}
bool World_Pose::operator != (const World_Pose& other_) const {
    return !this->operator ==(other_);
}

// --- Getters and Setters: -------------------------------------------------
Position& World_Pose::position() OMG_NOEXCEPT {
    return m_position_;
}

const Position& World_Pose::position() const OMG_NOEXCEPT {
    return m_position_;
}

void World_Pose::position(const Position& value) {
    m_position_ = value;
}

Orientation& World_Pose::orientation() OMG_NOEXCEPT {
    return m_orientation_;
}

const Orientation& World_Pose::orientation() const OMG_NOEXCEPT {
    return m_orientation_;
}

void World_Pose::orientation(const Orientation& value) {
    m_orientation_ = value;
}

std::ostream& operator << (std::ostream& o,const World_Pose& sample)
{
    rti::util::StreamFlagSaver flag_saver (o);
    o <<"[";
    o << "position: " << sample.position()<<", ";
    o << "orientation: " << sample.orientation() ;
    o <<"]";
    return o;
}

// ---- laser_Scan_msg: 

laser_Scan_msg::laser_Scan_msg() :
    m_laser_id_ (0) ,
    m_angle_min_ (0.0f) ,
    m_angle_max_ (0.0f) ,
    m_angle_step_ (0.0f) ,
    m_range_min_ (0.0f) ,
    m_range_max_ (0.0f) ,
    m_count_ (0) ,
    m_vertical_angle_min_ (0.0f) ,
    m_vertical_angle_max_ (0.0f) ,
    m_vertical_angle_step_ (0.0f) ,
    m_vertical_count_ (0.0f) {
}   

laser_Scan_msg::laser_Scan_msg (
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
    const dds::core::vector<float>& intensities_param)
    :
        m_laser_id_( laser_id_param ),
        m_header_( header_param ),
        m_world_pose_( world_pose_param ),
        m_angle_min_( angle_min_param ),
        m_angle_max_( angle_max_param ),
        m_angle_step_( angle_step_param ),
        m_range_min_( range_min_param ),
        m_range_max_( range_max_param ),
        m_count_( count_param ),
        m_vertical_angle_min_( vertical_angle_min_param ),
        m_vertical_angle_max_( vertical_angle_max_param ),
        m_vertical_angle_step_( vertical_angle_step_param ),
        m_vertical_count_( vertical_count_param ),
        m_ranges_( ranges_param ),
        m_intensities_( intensities_param ) {
}
#ifdef RTI_CXX11_RVALUE_REFERENCES
#ifdef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
laser_Scan_msg::laser_Scan_msg(laser_Scan_msg&& other_) OMG_NOEXCEPT  :m_laser_id_ (std::move(other_.m_laser_id_))
,
m_header_ (std::move(other_.m_header_))
,
m_world_pose_ (std::move(other_.m_world_pose_))
,
m_angle_min_ (std::move(other_.m_angle_min_))
,
m_angle_max_ (std::move(other_.m_angle_max_))
,
m_angle_step_ (std::move(other_.m_angle_step_))
,
m_range_min_ (std::move(other_.m_range_min_))
,
m_range_max_ (std::move(other_.m_range_max_))
,
m_count_ (std::move(other_.m_count_))
,
m_vertical_angle_min_ (std::move(other_.m_vertical_angle_min_))
,
m_vertical_angle_max_ (std::move(other_.m_vertical_angle_max_))
,
m_vertical_angle_step_ (std::move(other_.m_vertical_angle_step_))
,
m_vertical_count_ (std::move(other_.m_vertical_count_))
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
    swap(m_laser_id_, other_.m_laser_id_);
    swap(m_header_, other_.m_header_);
    swap(m_world_pose_, other_.m_world_pose_);
    swap(m_angle_min_, other_.m_angle_min_);
    swap(m_angle_max_, other_.m_angle_max_);
    swap(m_angle_step_, other_.m_angle_step_);
    swap(m_range_min_, other_.m_range_min_);
    swap(m_range_max_, other_.m_range_max_);
    swap(m_count_, other_.m_count_);
    swap(m_vertical_angle_min_, other_.m_vertical_angle_min_);
    swap(m_vertical_angle_max_, other_.m_vertical_angle_max_);
    swap(m_vertical_angle_step_, other_.m_vertical_angle_step_);
    swap(m_vertical_count_, other_.m_vertical_count_);
    swap(m_ranges_, other_.m_ranges_);
    swap(m_intensities_, other_.m_intensities_);
}  

bool laser_Scan_msg::operator == (const laser_Scan_msg& other_) const {
    if (m_laser_id_ != other_.m_laser_id_) {
        return false;
    }
    if (m_header_ != other_.m_header_) {
        return false;
    }
    if (m_world_pose_ != other_.m_world_pose_) {
        return false;
    }
    if (m_angle_min_ != other_.m_angle_min_) {
        return false;
    }
    if (m_angle_max_ != other_.m_angle_max_) {
        return false;
    }
    if (m_angle_step_ != other_.m_angle_step_) {
        return false;
    }
    if (m_range_min_ != other_.m_range_min_) {
        return false;
    }
    if (m_range_max_ != other_.m_range_max_) {
        return false;
    }
    if (m_count_ != other_.m_count_) {
        return false;
    }
    if (m_vertical_angle_min_ != other_.m_vertical_angle_min_) {
        return false;
    }
    if (m_vertical_angle_max_ != other_.m_vertical_angle_max_) {
        return false;
    }
    if (m_vertical_angle_step_ != other_.m_vertical_angle_step_) {
        return false;
    }
    if (m_vertical_count_ != other_.m_vertical_count_) {
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
int32_t laser_Scan_msg::laser_id() const OMG_NOEXCEPT{
    return m_laser_id_;
}

void laser_Scan_msg::laser_id(int32_t value) {
    m_laser_id_ = value;
}

Header& laser_Scan_msg::header() OMG_NOEXCEPT {
    return m_header_;
}

const Header& laser_Scan_msg::header() const OMG_NOEXCEPT {
    return m_header_;
}

void laser_Scan_msg::header(const Header& value) {
    m_header_ = value;
}

World_Pose& laser_Scan_msg::world_pose() OMG_NOEXCEPT {
    return m_world_pose_;
}

const World_Pose& laser_Scan_msg::world_pose() const OMG_NOEXCEPT {
    return m_world_pose_;
}

void laser_Scan_msg::world_pose(const World_Pose& value) {
    m_world_pose_ = value;
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

float laser_Scan_msg::angle_step() const OMG_NOEXCEPT{
    return m_angle_step_;
}

void laser_Scan_msg::angle_step(float value) {
    m_angle_step_ = value;
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

int32_t laser_Scan_msg::count() const OMG_NOEXCEPT{
    return m_count_;
}

void laser_Scan_msg::count(int32_t value) {
    m_count_ = value;
}

float laser_Scan_msg::vertical_angle_min() const OMG_NOEXCEPT{
    return m_vertical_angle_min_;
}

void laser_Scan_msg::vertical_angle_min(float value) {
    m_vertical_angle_min_ = value;
}

float laser_Scan_msg::vertical_angle_max() const OMG_NOEXCEPT{
    return m_vertical_angle_max_;
}

void laser_Scan_msg::vertical_angle_max(float value) {
    m_vertical_angle_max_ = value;
}

float laser_Scan_msg::vertical_angle_step() const OMG_NOEXCEPT{
    return m_vertical_angle_step_;
}

void laser_Scan_msg::vertical_angle_step(float value) {
    m_vertical_angle_step_ = value;
}

float laser_Scan_msg::vertical_count() const OMG_NOEXCEPT{
    return m_vertical_count_;
}

void laser_Scan_msg::vertical_count(float value) {
    m_vertical_count_ = value;
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
    o << "laser_id: " << sample.laser_id()<<", ";
    o << "header: " << sample.header()<<", ";
    o << "world_pose: " << sample.world_pose()<<", ";
    o << "angle_min: " << std::setprecision(9) <<sample.angle_min()<<", ";
    o << "angle_max: " << std::setprecision(9) <<sample.angle_max()<<", ";
    o << "angle_step: " << std::setprecision(9) <<sample.angle_step()<<", ";
    o << "range_min: " << std::setprecision(9) <<sample.range_min()<<", ";
    o << "range_max: " << std::setprecision(9) <<sample.range_max()<<", ";
    o << "count: " << sample.count()<<", ";
    o << "vertical_angle_min: " << std::setprecision(9) <<sample.vertical_angle_min()<<", ";
    o << "vertical_angle_max: " << std::setprecision(9) <<sample.vertical_angle_max()<<", ";
    o << "vertical_angle_step: " << std::setprecision(9) <<sample.vertical_angle_step()<<", ";
    o << "vertical_count: " << std::setprecision(9) <<sample.vertical_count()<<", ";
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

        const dds::core::xtypes::StructType& dynamic_type<Position>::get()
        {
            return static_cast<const dds::core::xtypes::StructType&>(
                rti::core::native_conversions::cast_from_native<dds::core::xtypes::DynamicType>(
                    *(Position_c_get_typecode())));
        }

        const dds::core::xtypes::StructType& dynamic_type<Orientation>::get()
        {
            return static_cast<const dds::core::xtypes::StructType&>(
                rti::core::native_conversions::cast_from_native<dds::core::xtypes::DynamicType>(
                    *(Orientation_c_get_typecode())));
        }

        const dds::core::xtypes::StructType& dynamic_type<World_Pose>::get()
        {
            return static_cast<const dds::core::xtypes::StructType&>(
                rti::core::native_conversions::cast_from_native<dds::core::xtypes::DynamicType>(
                    *(World_Pose_c_get_typecode())));
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

        void topic_type_support<Position>:: register_type(
            dds::domain::DomainParticipant& participant,
            const std::string& type_name){

            rti::domain::register_type_plugin(
                participant,
                type_name,
                Position_cPlugin_new,
                Position_cPlugin_delete);
        }

        void topic_type_support<Position>::initialize_sample(Position& sample){

            Position_c* native_sample=reinterpret_cast<Position_c*> (&sample);

            struct DDS_TypeDeallocationParams_t deAllocParams = {RTI_FALSE, RTI_FALSE};
            Position_c_finalize_w_params(native_sample,&deAllocParams);

            struct DDS_TypeAllocationParams_t allocParams = {RTI_FALSE, RTI_FALSE, RTI_TRUE}; 
            RTIBool ok=Position_c_initialize_w_params(native_sample,&allocParams);
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to initialize_w_params");

        } 

        std::vector<char>& topic_type_support<Position>::to_cdr_buffer(
            std::vector<char>& buffer, const Position& sample)
        {
            // First get the length of the buffer
            unsigned int length = 0;
            RTIBool ok = Position_cPlugin_serialize_to_cdr_buffer(
                NULL, &length,reinterpret_cast<const Position_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to calculate cdr buffer size");

            // Create a vector with that size and copy the cdr buffer into it
            buffer.resize(length);
            ok = Position_cPlugin_serialize_to_cdr_buffer(
                &buffer[0], &length, reinterpret_cast<const Position_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to copy cdr buffer");

            return buffer;

        }

        void topic_type_support<Position>::from_cdr_buffer(Position& sample, 
        const std::vector<char>& buffer)
        {

            RTIBool ok  = Position_cPlugin_deserialize_from_cdr_buffer(
                reinterpret_cast<Position_c*> (&sample), &buffer[0], 
                static_cast<unsigned int>(buffer.size()));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to create Position from cdr buffer");
        }

        void topic_type_support<Orientation>:: register_type(
            dds::domain::DomainParticipant& participant,
            const std::string& type_name){

            rti::domain::register_type_plugin(
                participant,
                type_name,
                Orientation_cPlugin_new,
                Orientation_cPlugin_delete);
        }

        void topic_type_support<Orientation>::initialize_sample(Orientation& sample){

            Orientation_c* native_sample=reinterpret_cast<Orientation_c*> (&sample);

            struct DDS_TypeDeallocationParams_t deAllocParams = {RTI_FALSE, RTI_FALSE};
            Orientation_c_finalize_w_params(native_sample,&deAllocParams);

            struct DDS_TypeAllocationParams_t allocParams = {RTI_FALSE, RTI_FALSE, RTI_TRUE}; 
            RTIBool ok=Orientation_c_initialize_w_params(native_sample,&allocParams);
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to initialize_w_params");

        } 

        std::vector<char>& topic_type_support<Orientation>::to_cdr_buffer(
            std::vector<char>& buffer, const Orientation& sample)
        {
            // First get the length of the buffer
            unsigned int length = 0;
            RTIBool ok = Orientation_cPlugin_serialize_to_cdr_buffer(
                NULL, &length,reinterpret_cast<const Orientation_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to calculate cdr buffer size");

            // Create a vector with that size and copy the cdr buffer into it
            buffer.resize(length);
            ok = Orientation_cPlugin_serialize_to_cdr_buffer(
                &buffer[0], &length, reinterpret_cast<const Orientation_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to copy cdr buffer");

            return buffer;

        }

        void topic_type_support<Orientation>::from_cdr_buffer(Orientation& sample, 
        const std::vector<char>& buffer)
        {

            RTIBool ok  = Orientation_cPlugin_deserialize_from_cdr_buffer(
                reinterpret_cast<Orientation_c*> (&sample), &buffer[0], 
                static_cast<unsigned int>(buffer.size()));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to create Orientation from cdr buffer");
        }

        void topic_type_support<World_Pose>:: register_type(
            dds::domain::DomainParticipant& participant,
            const std::string& type_name){

            rti::domain::register_type_plugin(
                participant,
                type_name,
                World_Pose_cPlugin_new,
                World_Pose_cPlugin_delete);
        }

        void topic_type_support<World_Pose>::initialize_sample(World_Pose& sample){

            World_Pose_c* native_sample=reinterpret_cast<World_Pose_c*> (&sample);

            struct DDS_TypeDeallocationParams_t deAllocParams = {RTI_FALSE, RTI_FALSE};
            World_Pose_c_finalize_w_params(native_sample,&deAllocParams);

            struct DDS_TypeAllocationParams_t allocParams = {RTI_FALSE, RTI_FALSE, RTI_TRUE}; 
            RTIBool ok=World_Pose_c_initialize_w_params(native_sample,&allocParams);
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to initialize_w_params");

        } 

        std::vector<char>& topic_type_support<World_Pose>::to_cdr_buffer(
            std::vector<char>& buffer, const World_Pose& sample)
        {
            // First get the length of the buffer
            unsigned int length = 0;
            RTIBool ok = World_Pose_cPlugin_serialize_to_cdr_buffer(
                NULL, &length,reinterpret_cast<const World_Pose_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to calculate cdr buffer size");

            // Create a vector with that size and copy the cdr buffer into it
            buffer.resize(length);
            ok = World_Pose_cPlugin_serialize_to_cdr_buffer(
                &buffer[0], &length, reinterpret_cast<const World_Pose_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to copy cdr buffer");

            return buffer;

        }

        void topic_type_support<World_Pose>::from_cdr_buffer(World_Pose& sample, 
        const std::vector<char>& buffer)
        {

            RTIBool ok  = World_Pose_cPlugin_deserialize_from_cdr_buffer(
                reinterpret_cast<World_Pose_c*> (&sample), &buffer[0], 
                static_cast<unsigned int>(buffer.size()));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to create World_Pose from cdr buffer");
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

