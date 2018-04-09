

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from laserScanMsgImpl.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef laserScanMsgImpl_1972818959_h
#define laserScanMsgImpl_1972818959_h

#ifndef NDDS_STANDALONE_TYPE
#ifndef ndds_c_h
#include "ndds/ndds_c.h"
#endif
#else
#include "ndds_standalone_type.h"
#endif

extern const char *Time_cTYPENAME;

typedef struct Time_c {

    DDS_UnsignedLong   sec ;
    DDS_UnsignedLong   nsec ;

    Time_c() {}

} Time_c ;
#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif

NDDSUSERDllExport DDS_TypeCode* Time_c_get_typecode(void); /* Type code */

DDS_SEQUENCE(Time_cSeq, Time_c);

NDDSUSERDllExport
RTIBool Time_c_initialize(
    Time_c* self);

NDDSUSERDllExport
RTIBool Time_c_initialize_ex(
    Time_c* self,RTIBool allocatePointers,RTIBool allocateMemory);

NDDSUSERDllExport
RTIBool Time_c_initialize_w_params(
    Time_c* self,
    const struct DDS_TypeAllocationParams_t * allocParams);  

NDDSUSERDllExport
void Time_c_finalize(
    Time_c* self);

NDDSUSERDllExport
void Time_c_finalize_ex(
    Time_c* self,RTIBool deletePointers);

NDDSUSERDllExport
void Time_c_finalize_w_params(
    Time_c* self,
    const struct DDS_TypeDeallocationParams_t * deallocParams);

NDDSUSERDllExport
void Time_c_finalize_optional_members(
    Time_c* self, RTIBool deletePointers);  

NDDSUSERDllExport
RTIBool Time_c_copy(
    Time_c* dst,
    const Time_c* src);

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

extern const char *Header_cTYPENAME;

typedef struct Header_c {

    Time_c   stamp ;
    DDS_Char *   frame_id ;

    Header_c() {}

} Header_c ;
#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif

NDDSUSERDllExport DDS_TypeCode* Header_c_get_typecode(void); /* Type code */

DDS_SEQUENCE(Header_cSeq, Header_c);

NDDSUSERDllExport
RTIBool Header_c_initialize(
    Header_c* self);

NDDSUSERDllExport
RTIBool Header_c_initialize_ex(
    Header_c* self,RTIBool allocatePointers,RTIBool allocateMemory);

NDDSUSERDllExport
RTIBool Header_c_initialize_w_params(
    Header_c* self,
    const struct DDS_TypeAllocationParams_t * allocParams);  

NDDSUSERDllExport
void Header_c_finalize(
    Header_c* self);

NDDSUSERDllExport
void Header_c_finalize_ex(
    Header_c* self,RTIBool deletePointers);

NDDSUSERDllExport
void Header_c_finalize_w_params(
    Header_c* self,
    const struct DDS_TypeDeallocationParams_t * deallocParams);

NDDSUSERDllExport
void Header_c_finalize_optional_members(
    Header_c* self, RTIBool deletePointers);  

NDDSUSERDllExport
RTIBool Header_c_copy(
    Header_c* dst,
    const Header_c* src);

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

extern const char *Position_cTYPENAME;

typedef struct Position_c {

    DDS_Float   x ;
    DDS_Float   y ;
    DDS_Float   z ;

    Position_c() {}

} Position_c ;
#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif

NDDSUSERDllExport DDS_TypeCode* Position_c_get_typecode(void); /* Type code */

DDS_SEQUENCE(Position_cSeq, Position_c);

NDDSUSERDllExport
RTIBool Position_c_initialize(
    Position_c* self);

NDDSUSERDllExport
RTIBool Position_c_initialize_ex(
    Position_c* self,RTIBool allocatePointers,RTIBool allocateMemory);

NDDSUSERDllExport
RTIBool Position_c_initialize_w_params(
    Position_c* self,
    const struct DDS_TypeAllocationParams_t * allocParams);  

NDDSUSERDllExport
void Position_c_finalize(
    Position_c* self);

NDDSUSERDllExport
void Position_c_finalize_ex(
    Position_c* self,RTIBool deletePointers);

NDDSUSERDllExport
void Position_c_finalize_w_params(
    Position_c* self,
    const struct DDS_TypeDeallocationParams_t * deallocParams);

NDDSUSERDllExport
void Position_c_finalize_optional_members(
    Position_c* self, RTIBool deletePointers);  

NDDSUSERDllExport
RTIBool Position_c_copy(
    Position_c* dst,
    const Position_c* src);

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

extern const char *Orientation_cTYPENAME;

typedef struct Orientation_c {

    DDS_Float   x ;
    DDS_Float   y ;
    DDS_Float   z ;
    DDS_Float   w ;

    Orientation_c() {}

} Orientation_c ;
#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif

NDDSUSERDllExport DDS_TypeCode* Orientation_c_get_typecode(void); /* Type code */

DDS_SEQUENCE(Orientation_cSeq, Orientation_c);

NDDSUSERDllExport
RTIBool Orientation_c_initialize(
    Orientation_c* self);

NDDSUSERDllExport
RTIBool Orientation_c_initialize_ex(
    Orientation_c* self,RTIBool allocatePointers,RTIBool allocateMemory);

NDDSUSERDllExport
RTIBool Orientation_c_initialize_w_params(
    Orientation_c* self,
    const struct DDS_TypeAllocationParams_t * allocParams);  

NDDSUSERDllExport
void Orientation_c_finalize(
    Orientation_c* self);

NDDSUSERDllExport
void Orientation_c_finalize_ex(
    Orientation_c* self,RTIBool deletePointers);

NDDSUSERDllExport
void Orientation_c_finalize_w_params(
    Orientation_c* self,
    const struct DDS_TypeDeallocationParams_t * deallocParams);

NDDSUSERDllExport
void Orientation_c_finalize_optional_members(
    Orientation_c* self, RTIBool deletePointers);  

NDDSUSERDllExport
RTIBool Orientation_c_copy(
    Orientation_c* dst,
    const Orientation_c* src);

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

extern const char *World_Pose_cTYPENAME;

typedef struct World_Pose_c {

    Position_c   position ;
    Orientation_c   orientation ;

    World_Pose_c() {}

} World_Pose_c ;
#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif

NDDSUSERDllExport DDS_TypeCode* World_Pose_c_get_typecode(void); /* Type code */

DDS_SEQUENCE(World_Pose_cSeq, World_Pose_c);

NDDSUSERDllExport
RTIBool World_Pose_c_initialize(
    World_Pose_c* self);

NDDSUSERDllExport
RTIBool World_Pose_c_initialize_ex(
    World_Pose_c* self,RTIBool allocatePointers,RTIBool allocateMemory);

NDDSUSERDllExport
RTIBool World_Pose_c_initialize_w_params(
    World_Pose_c* self,
    const struct DDS_TypeAllocationParams_t * allocParams);  

NDDSUSERDllExport
void World_Pose_c_finalize(
    World_Pose_c* self);

NDDSUSERDllExport
void World_Pose_c_finalize_ex(
    World_Pose_c* self,RTIBool deletePointers);

NDDSUSERDllExport
void World_Pose_c_finalize_w_params(
    World_Pose_c* self,
    const struct DDS_TypeDeallocationParams_t * deallocParams);

NDDSUSERDllExport
void World_Pose_c_finalize_optional_members(
    World_Pose_c* self, RTIBool deletePointers);  

NDDSUSERDllExport
RTIBool World_Pose_c_copy(
    World_Pose_c* dst,
    const World_Pose_c* src);

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

extern const char *laser_Scan_msg_cTYPENAME;

typedef struct laser_Scan_msg_c {

    Header_c   header ;
    World_Pose_c   world_pose ;
    DDS_Float   angle_min ;
    DDS_Float   angle_max ;
    DDS_Float   angle_step ;
    DDS_Float   range_min ;
    DDS_Float   range_max ;
    DDS_Long   count ;
    DDS_Float   vertical_angle_min ;
    DDS_Float   vertical_angle_max ;
    DDS_Float   vertical_angle_step ;
    DDS_Float   vertical_count ;
    struct    DDS_FloatSeq  ranges ;
    struct    DDS_FloatSeq  intensities ;

    laser_Scan_msg_c() {}

} laser_Scan_msg_c ;
#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif

NDDSUSERDllExport DDS_TypeCode* laser_Scan_msg_c_get_typecode(void); /* Type code */

DDS_SEQUENCE(laser_Scan_msg_cSeq, laser_Scan_msg_c);

NDDSUSERDllExport
RTIBool laser_Scan_msg_c_initialize(
    laser_Scan_msg_c* self);

NDDSUSERDllExport
RTIBool laser_Scan_msg_c_initialize_ex(
    laser_Scan_msg_c* self,RTIBool allocatePointers,RTIBool allocateMemory);

NDDSUSERDllExport
RTIBool laser_Scan_msg_c_initialize_w_params(
    laser_Scan_msg_c* self,
    const struct DDS_TypeAllocationParams_t * allocParams);  

NDDSUSERDllExport
void laser_Scan_msg_c_finalize(
    laser_Scan_msg_c* self);

NDDSUSERDllExport
void laser_Scan_msg_c_finalize_ex(
    laser_Scan_msg_c* self,RTIBool deletePointers);

NDDSUSERDllExport
void laser_Scan_msg_c_finalize_w_params(
    laser_Scan_msg_c* self,
    const struct DDS_TypeDeallocationParams_t * deallocParams);

NDDSUSERDllExport
void laser_Scan_msg_c_finalize_optional_members(
    laser_Scan_msg_c* self, RTIBool deletePointers);  

NDDSUSERDllExport
RTIBool laser_Scan_msg_c_copy(
    laser_Scan_msg_c* dst,
    const laser_Scan_msg_c* src);

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

#endif /* laserScanMsgImpl */

