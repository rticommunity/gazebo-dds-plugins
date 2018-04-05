

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from laserScanMsgImpl.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef laserScanMsgImpl_1972817541_h
#define laserScanMsgImpl_1972817541_h

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

    DDS_UnsignedLong   seq ;
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

extern const char *laser_Scan_msg_cTYPENAME;

typedef struct laser_Scan_msg_c {

    Header_c   header ;
    DDS_Float   angle_min ;
    DDS_Float   angle_max ;
    DDS_Float   angle_increment ;
    DDS_Float   time_increment ;
    DDS_Float   scan_time ;
    DDS_Float   range_min ;
    DDS_Float   range_max ;
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

