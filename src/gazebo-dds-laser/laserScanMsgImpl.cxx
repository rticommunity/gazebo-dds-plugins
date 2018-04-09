

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from laserScanMsgImpl.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef NDDS_STANDALONE_TYPE
#ifndef ndds_c_h
#include "ndds/ndds_c.h"
#endif

#ifndef cdr_type_h
#include "cdr/cdr_type.h"
#endif    

#ifndef osapi_heap_h
#include "osapi/osapi_heap.h" 
#endif
#else
#include "ndds_standalone_type.h"
#endif

#include "laserScanMsgImpl.h"

/* ========================================================================= */
const char *Time_cTYPENAME = "Time";

DDS_TypeCode* Time_c_get_typecode()
{
    static RTIBool is_initialized = RTI_FALSE;

    static DDS_TypeCode_Member Time_c_g_tc_members[2]=
    {

        {
            (char *)"sec",/* Member name */
            {
                0,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"nsec",/* Member name */
            {
                1,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }
    };

    static DDS_TypeCode Time_c_g_tc =
    {{
            DDS_TK_STRUCT,/* Kind */
            DDS_BOOLEAN_FALSE, /* Ignored */
            -1, /*Ignored*/
            (char *)"Time", /* Name */
            NULL, /* Ignored */      
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            2, /* Number of members */
            Time_c_g_tc_members, /* Members */
            DDS_VM_NONE  /* Ignored */         
        }}; /* Type code for Time_c*/

    if (is_initialized) {
        return &Time_c_g_tc;
    }

    Time_c_g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_ulong;

    Time_c_g_tc_members[1]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_ulong;

    is_initialized = RTI_TRUE;

    return &Time_c_g_tc;
}

RTIBool Time_c_initialize(
    Time_c* sample) {
    return Time_c_initialize_ex(sample,RTI_TRUE,RTI_TRUE);
}

RTIBool Time_c_initialize_ex(
    Time_c* sample,RTIBool allocatePointers, RTIBool allocateMemory)
{

    struct DDS_TypeAllocationParams_t allocParams =
    DDS_TYPE_ALLOCATION_PARAMS_DEFAULT;

    allocParams.allocate_pointers =  (DDS_Boolean)allocatePointers;
    allocParams.allocate_memory = (DDS_Boolean)allocateMemory;

    return Time_c_initialize_w_params(
        sample,&allocParams);

}

RTIBool Time_c_initialize_w_params(
    Time_c* sample, const struct DDS_TypeAllocationParams_t * allocParams)
{

    if (sample == NULL) {
        return RTI_FALSE;
    }
    if (allocParams == NULL) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initUnsignedLong(&sample->sec)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initUnsignedLong(&sample->nsec)) {
        return RTI_FALSE;
    }

    return RTI_TRUE;
}

void Time_c_finalize(
    Time_c* sample)
{

    Time_c_finalize_ex(sample,RTI_TRUE);
}

void Time_c_finalize_ex(
    Time_c* sample,RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParams =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;

    if (sample==NULL) {
        return;
    } 

    deallocParams.delete_pointers = (DDS_Boolean)deletePointers;

    Time_c_finalize_w_params(
        sample,&deallocParams);
}

void Time_c_finalize_w_params(
    Time_c* sample,const struct DDS_TypeDeallocationParams_t * deallocParams)
{

    if (sample==NULL) {
        return;
    }

    if (deallocParams == NULL) {
        return;
    }

}

void Time_c_finalize_optional_members(
    Time_c* sample, RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParamsTmp =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;
    struct DDS_TypeDeallocationParams_t * deallocParams =
    &deallocParamsTmp;

    if (sample==NULL) {
        return;
    } 
    if (deallocParams) {} /* To avoid warnings */

    deallocParamsTmp.delete_pointers = (DDS_Boolean)deletePointers;
    deallocParamsTmp.delete_optional_members = DDS_BOOLEAN_TRUE;

}

RTIBool Time_c_copy(
    Time_c* dst,
    const Time_c* src)
{

    if (dst == NULL || src == NULL) {
        return RTI_FALSE;
    }

    if (!RTICdrType_copyUnsignedLong (
        &dst->sec, &src->sec)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyUnsignedLong (
        &dst->nsec, &src->nsec)) { 
        return RTI_FALSE;
    }

    return RTI_TRUE;

}

/**
* <<IMPLEMENTATION>>
*
* Defines:  TSeq, T
*
* Configure and implement 'Time_c' sequence class.
*/
#define T Time_c
#define TSeq Time_cSeq

#define T_initialize_w_params Time_c_initialize_w_params

#define T_finalize_w_params   Time_c_finalize_w_params
#define T_copy       Time_c_copy

#ifndef NDDS_STANDALONE_TYPE
#include "dds_c/generic/dds_c_sequence_TSeq.gen"
#else
#include "dds_c_sequence_TSeq.gen"
#endif

#undef T_copy
#undef T_finalize_w_params

#undef T_initialize_w_params

#undef TSeq
#undef T

/* ========================================================================= */
const char *Header_cTYPENAME = "Header";

DDS_TypeCode* Header_c_get_typecode()
{
    static RTIBool is_initialized = RTI_FALSE;

    static DDS_TypeCode Header_c_g_tc_frame_id_string = DDS_INITIALIZE_STRING_TYPECODE(RTI_INT32_MAX);
    static DDS_TypeCode_Member Header_c_g_tc_members[2]=
    {

        {
            (char *)"stamp",/* Member name */
            {
                0,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"frame_id",/* Member name */
            {
                1,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }
    };

    static DDS_TypeCode Header_c_g_tc =
    {{
            DDS_TK_STRUCT,/* Kind */
            DDS_BOOLEAN_FALSE, /* Ignored */
            -1, /*Ignored*/
            (char *)"Header", /* Name */
            NULL, /* Ignored */      
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            2, /* Number of members */
            Header_c_g_tc_members, /* Members */
            DDS_VM_NONE  /* Ignored */         
        }}; /* Type code for Header_c*/

    if (is_initialized) {
        return &Header_c_g_tc;
    }

    Header_c_g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)Time_c_get_typecode();

    Header_c_g_tc_members[1]._representation._typeCode = (RTICdrTypeCode *)&Header_c_g_tc_frame_id_string;

    is_initialized = RTI_TRUE;

    return &Header_c_g_tc;
}

RTIBool Header_c_initialize(
    Header_c* sample) {
    return Header_c_initialize_ex(sample,RTI_TRUE,RTI_TRUE);
}

RTIBool Header_c_initialize_ex(
    Header_c* sample,RTIBool allocatePointers, RTIBool allocateMemory)
{

    struct DDS_TypeAllocationParams_t allocParams =
    DDS_TYPE_ALLOCATION_PARAMS_DEFAULT;

    allocParams.allocate_pointers =  (DDS_Boolean)allocatePointers;
    allocParams.allocate_memory = (DDS_Boolean)allocateMemory;

    return Header_c_initialize_w_params(
        sample,&allocParams);

}

RTIBool Header_c_initialize_w_params(
    Header_c* sample, const struct DDS_TypeAllocationParams_t * allocParams)
{

    if (sample == NULL) {
        return RTI_FALSE;
    }
    if (allocParams == NULL) {
        return RTI_FALSE;
    }

    if (!Time_c_initialize_w_params(&sample->stamp,
    allocParams)) {
        return RTI_FALSE;
    }
    if (allocParams->allocate_memory){
        sample->frame_id= DDS_String_alloc ((0));
        if (sample->frame_id == NULL) {
            return RTI_FALSE;
        }

    } else {
        if (sample->frame_id!= NULL) { 
            sample->frame_id[0] = '\0';
        }
    }

    return RTI_TRUE;
}

void Header_c_finalize(
    Header_c* sample)
{

    Header_c_finalize_ex(sample,RTI_TRUE);
}

void Header_c_finalize_ex(
    Header_c* sample,RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParams =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;

    if (sample==NULL) {
        return;
    } 

    deallocParams.delete_pointers = (DDS_Boolean)deletePointers;

    Header_c_finalize_w_params(
        sample,&deallocParams);
}

void Header_c_finalize_w_params(
    Header_c* sample,const struct DDS_TypeDeallocationParams_t * deallocParams)
{

    if (sample==NULL) {
        return;
    }

    if (deallocParams == NULL) {
        return;
    }

    Time_c_finalize_w_params(&sample->stamp,deallocParams);

    if (sample->frame_id != NULL) {
        DDS_String_free(sample->frame_id);
        sample->frame_id=NULL;

    }
}

void Header_c_finalize_optional_members(
    Header_c* sample, RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParamsTmp =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;
    struct DDS_TypeDeallocationParams_t * deallocParams =
    &deallocParamsTmp;

    if (sample==NULL) {
        return;
    } 
    if (deallocParams) {} /* To avoid warnings */

    deallocParamsTmp.delete_pointers = (DDS_Boolean)deletePointers;
    deallocParamsTmp.delete_optional_members = DDS_BOOLEAN_TRUE;

    Time_c_finalize_optional_members(&sample->stamp, deallocParams->delete_pointers);
}

RTIBool Header_c_copy(
    Header_c* dst,
    const Header_c* src)
{

    if (dst == NULL || src == NULL) {
        return RTI_FALSE;
    }

    if (!Time_c_copy(
        &dst->stamp,(const Time_c*)&src->stamp)) {
        return RTI_FALSE;
    } 
    if (!RTICdrType_copyStringEx (
        &dst->frame_id, src->frame_id, 
        (RTI_INT32_MAX-1) + 1,RTI_TRUE)){
        return RTI_FALSE;
    }

    return RTI_TRUE;

}

/**
* <<IMPLEMENTATION>>
*
* Defines:  TSeq, T
*
* Configure and implement 'Header_c' sequence class.
*/
#define T Header_c
#define TSeq Header_cSeq

#define T_initialize_w_params Header_c_initialize_w_params

#define T_finalize_w_params   Header_c_finalize_w_params
#define T_copy       Header_c_copy

#ifndef NDDS_STANDALONE_TYPE
#include "dds_c/generic/dds_c_sequence_TSeq.gen"
#else
#include "dds_c_sequence_TSeq.gen"
#endif

#undef T_copy
#undef T_finalize_w_params

#undef T_initialize_w_params

#undef TSeq
#undef T

/* ========================================================================= */
const char *Position_cTYPENAME = "Position";

DDS_TypeCode* Position_c_get_typecode()
{
    static RTIBool is_initialized = RTI_FALSE;

    static DDS_TypeCode_Member Position_c_g_tc_members[3]=
    {

        {
            (char *)"x",/* Member name */
            {
                0,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"y",/* Member name */
            {
                1,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"z",/* Member name */
            {
                2,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }
    };

    static DDS_TypeCode Position_c_g_tc =
    {{
            DDS_TK_STRUCT,/* Kind */
            DDS_BOOLEAN_FALSE, /* Ignored */
            -1, /*Ignored*/
            (char *)"Position", /* Name */
            NULL, /* Ignored */      
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            3, /* Number of members */
            Position_c_g_tc_members, /* Members */
            DDS_VM_NONE  /* Ignored */         
        }}; /* Type code for Position_c*/

    if (is_initialized) {
        return &Position_c_g_tc;
    }

    Position_c_g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    Position_c_g_tc_members[1]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    Position_c_g_tc_members[2]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    is_initialized = RTI_TRUE;

    return &Position_c_g_tc;
}

RTIBool Position_c_initialize(
    Position_c* sample) {
    return Position_c_initialize_ex(sample,RTI_TRUE,RTI_TRUE);
}

RTIBool Position_c_initialize_ex(
    Position_c* sample,RTIBool allocatePointers, RTIBool allocateMemory)
{

    struct DDS_TypeAllocationParams_t allocParams =
    DDS_TYPE_ALLOCATION_PARAMS_DEFAULT;

    allocParams.allocate_pointers =  (DDS_Boolean)allocatePointers;
    allocParams.allocate_memory = (DDS_Boolean)allocateMemory;

    return Position_c_initialize_w_params(
        sample,&allocParams);

}

RTIBool Position_c_initialize_w_params(
    Position_c* sample, const struct DDS_TypeAllocationParams_t * allocParams)
{

    if (sample == NULL) {
        return RTI_FALSE;
    }
    if (allocParams == NULL) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->x)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->y)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->z)) {
        return RTI_FALSE;
    }

    return RTI_TRUE;
}

void Position_c_finalize(
    Position_c* sample)
{

    Position_c_finalize_ex(sample,RTI_TRUE);
}

void Position_c_finalize_ex(
    Position_c* sample,RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParams =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;

    if (sample==NULL) {
        return;
    } 

    deallocParams.delete_pointers = (DDS_Boolean)deletePointers;

    Position_c_finalize_w_params(
        sample,&deallocParams);
}

void Position_c_finalize_w_params(
    Position_c* sample,const struct DDS_TypeDeallocationParams_t * deallocParams)
{

    if (sample==NULL) {
        return;
    }

    if (deallocParams == NULL) {
        return;
    }

}

void Position_c_finalize_optional_members(
    Position_c* sample, RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParamsTmp =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;
    struct DDS_TypeDeallocationParams_t * deallocParams =
    &deallocParamsTmp;

    if (sample==NULL) {
        return;
    } 
    if (deallocParams) {} /* To avoid warnings */

    deallocParamsTmp.delete_pointers = (DDS_Boolean)deletePointers;
    deallocParamsTmp.delete_optional_members = DDS_BOOLEAN_TRUE;

}

RTIBool Position_c_copy(
    Position_c* dst,
    const Position_c* src)
{

    if (dst == NULL || src == NULL) {
        return RTI_FALSE;
    }

    if (!RTICdrType_copyFloat (
        &dst->x, &src->x)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->y, &src->y)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->z, &src->z)) { 
        return RTI_FALSE;
    }

    return RTI_TRUE;

}

/**
* <<IMPLEMENTATION>>
*
* Defines:  TSeq, T
*
* Configure and implement 'Position_c' sequence class.
*/
#define T Position_c
#define TSeq Position_cSeq

#define T_initialize_w_params Position_c_initialize_w_params

#define T_finalize_w_params   Position_c_finalize_w_params
#define T_copy       Position_c_copy

#ifndef NDDS_STANDALONE_TYPE
#include "dds_c/generic/dds_c_sequence_TSeq.gen"
#else
#include "dds_c_sequence_TSeq.gen"
#endif

#undef T_copy
#undef T_finalize_w_params

#undef T_initialize_w_params

#undef TSeq
#undef T

/* ========================================================================= */
const char *Orientation_cTYPENAME = "Orientation";

DDS_TypeCode* Orientation_c_get_typecode()
{
    static RTIBool is_initialized = RTI_FALSE;

    static DDS_TypeCode_Member Orientation_c_g_tc_members[4]=
    {

        {
            (char *)"x",/* Member name */
            {
                0,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"y",/* Member name */
            {
                1,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"z",/* Member name */
            {
                2,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"w",/* Member name */
            {
                3,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }
    };

    static DDS_TypeCode Orientation_c_g_tc =
    {{
            DDS_TK_STRUCT,/* Kind */
            DDS_BOOLEAN_FALSE, /* Ignored */
            -1, /*Ignored*/
            (char *)"Orientation", /* Name */
            NULL, /* Ignored */      
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            4, /* Number of members */
            Orientation_c_g_tc_members, /* Members */
            DDS_VM_NONE  /* Ignored */         
        }}; /* Type code for Orientation_c*/

    if (is_initialized) {
        return &Orientation_c_g_tc;
    }

    Orientation_c_g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    Orientation_c_g_tc_members[1]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    Orientation_c_g_tc_members[2]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    Orientation_c_g_tc_members[3]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    is_initialized = RTI_TRUE;

    return &Orientation_c_g_tc;
}

RTIBool Orientation_c_initialize(
    Orientation_c* sample) {
    return Orientation_c_initialize_ex(sample,RTI_TRUE,RTI_TRUE);
}

RTIBool Orientation_c_initialize_ex(
    Orientation_c* sample,RTIBool allocatePointers, RTIBool allocateMemory)
{

    struct DDS_TypeAllocationParams_t allocParams =
    DDS_TYPE_ALLOCATION_PARAMS_DEFAULT;

    allocParams.allocate_pointers =  (DDS_Boolean)allocatePointers;
    allocParams.allocate_memory = (DDS_Boolean)allocateMemory;

    return Orientation_c_initialize_w_params(
        sample,&allocParams);

}

RTIBool Orientation_c_initialize_w_params(
    Orientation_c* sample, const struct DDS_TypeAllocationParams_t * allocParams)
{

    if (sample == NULL) {
        return RTI_FALSE;
    }
    if (allocParams == NULL) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->x)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->y)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->z)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->w)) {
        return RTI_FALSE;
    }

    return RTI_TRUE;
}

void Orientation_c_finalize(
    Orientation_c* sample)
{

    Orientation_c_finalize_ex(sample,RTI_TRUE);
}

void Orientation_c_finalize_ex(
    Orientation_c* sample,RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParams =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;

    if (sample==NULL) {
        return;
    } 

    deallocParams.delete_pointers = (DDS_Boolean)deletePointers;

    Orientation_c_finalize_w_params(
        sample,&deallocParams);
}

void Orientation_c_finalize_w_params(
    Orientation_c* sample,const struct DDS_TypeDeallocationParams_t * deallocParams)
{

    if (sample==NULL) {
        return;
    }

    if (deallocParams == NULL) {
        return;
    }

}

void Orientation_c_finalize_optional_members(
    Orientation_c* sample, RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParamsTmp =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;
    struct DDS_TypeDeallocationParams_t * deallocParams =
    &deallocParamsTmp;

    if (sample==NULL) {
        return;
    } 
    if (deallocParams) {} /* To avoid warnings */

    deallocParamsTmp.delete_pointers = (DDS_Boolean)deletePointers;
    deallocParamsTmp.delete_optional_members = DDS_BOOLEAN_TRUE;

}

RTIBool Orientation_c_copy(
    Orientation_c* dst,
    const Orientation_c* src)
{

    if (dst == NULL || src == NULL) {
        return RTI_FALSE;
    }

    if (!RTICdrType_copyFloat (
        &dst->x, &src->x)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->y, &src->y)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->z, &src->z)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->w, &src->w)) { 
        return RTI_FALSE;
    }

    return RTI_TRUE;

}

/**
* <<IMPLEMENTATION>>
*
* Defines:  TSeq, T
*
* Configure and implement 'Orientation_c' sequence class.
*/
#define T Orientation_c
#define TSeq Orientation_cSeq

#define T_initialize_w_params Orientation_c_initialize_w_params

#define T_finalize_w_params   Orientation_c_finalize_w_params
#define T_copy       Orientation_c_copy

#ifndef NDDS_STANDALONE_TYPE
#include "dds_c/generic/dds_c_sequence_TSeq.gen"
#else
#include "dds_c_sequence_TSeq.gen"
#endif

#undef T_copy
#undef T_finalize_w_params

#undef T_initialize_w_params

#undef TSeq
#undef T

/* ========================================================================= */
const char *World_Pose_cTYPENAME = "World_Pose";

DDS_TypeCode* World_Pose_c_get_typecode()
{
    static RTIBool is_initialized = RTI_FALSE;

    static DDS_TypeCode_Member World_Pose_c_g_tc_members[2]=
    {

        {
            (char *)"position",/* Member name */
            {
                0,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"orientation",/* Member name */
            {
                1,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }
    };

    static DDS_TypeCode World_Pose_c_g_tc =
    {{
            DDS_TK_STRUCT,/* Kind */
            DDS_BOOLEAN_FALSE, /* Ignored */
            -1, /*Ignored*/
            (char *)"World_Pose", /* Name */
            NULL, /* Ignored */      
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            2, /* Number of members */
            World_Pose_c_g_tc_members, /* Members */
            DDS_VM_NONE  /* Ignored */         
        }}; /* Type code for World_Pose_c*/

    if (is_initialized) {
        return &World_Pose_c_g_tc;
    }

    World_Pose_c_g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)Position_c_get_typecode();

    World_Pose_c_g_tc_members[1]._representation._typeCode = (RTICdrTypeCode *)Orientation_c_get_typecode();

    is_initialized = RTI_TRUE;

    return &World_Pose_c_g_tc;
}

RTIBool World_Pose_c_initialize(
    World_Pose_c* sample) {
    return World_Pose_c_initialize_ex(sample,RTI_TRUE,RTI_TRUE);
}

RTIBool World_Pose_c_initialize_ex(
    World_Pose_c* sample,RTIBool allocatePointers, RTIBool allocateMemory)
{

    struct DDS_TypeAllocationParams_t allocParams =
    DDS_TYPE_ALLOCATION_PARAMS_DEFAULT;

    allocParams.allocate_pointers =  (DDS_Boolean)allocatePointers;
    allocParams.allocate_memory = (DDS_Boolean)allocateMemory;

    return World_Pose_c_initialize_w_params(
        sample,&allocParams);

}

RTIBool World_Pose_c_initialize_w_params(
    World_Pose_c* sample, const struct DDS_TypeAllocationParams_t * allocParams)
{

    if (sample == NULL) {
        return RTI_FALSE;
    }
    if (allocParams == NULL) {
        return RTI_FALSE;
    }

    if (!Position_c_initialize_w_params(&sample->position,
    allocParams)) {
        return RTI_FALSE;
    }
    if (!Orientation_c_initialize_w_params(&sample->orientation,
    allocParams)) {
        return RTI_FALSE;
    }
    return RTI_TRUE;
}

void World_Pose_c_finalize(
    World_Pose_c* sample)
{

    World_Pose_c_finalize_ex(sample,RTI_TRUE);
}

void World_Pose_c_finalize_ex(
    World_Pose_c* sample,RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParams =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;

    if (sample==NULL) {
        return;
    } 

    deallocParams.delete_pointers = (DDS_Boolean)deletePointers;

    World_Pose_c_finalize_w_params(
        sample,&deallocParams);
}

void World_Pose_c_finalize_w_params(
    World_Pose_c* sample,const struct DDS_TypeDeallocationParams_t * deallocParams)
{

    if (sample==NULL) {
        return;
    }

    if (deallocParams == NULL) {
        return;
    }

    Position_c_finalize_w_params(&sample->position,deallocParams);

    Orientation_c_finalize_w_params(&sample->orientation,deallocParams);

}

void World_Pose_c_finalize_optional_members(
    World_Pose_c* sample, RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParamsTmp =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;
    struct DDS_TypeDeallocationParams_t * deallocParams =
    &deallocParamsTmp;

    if (sample==NULL) {
        return;
    } 
    if (deallocParams) {} /* To avoid warnings */

    deallocParamsTmp.delete_pointers = (DDS_Boolean)deletePointers;
    deallocParamsTmp.delete_optional_members = DDS_BOOLEAN_TRUE;

    Position_c_finalize_optional_members(&sample->position, deallocParams->delete_pointers);
    Orientation_c_finalize_optional_members(&sample->orientation, deallocParams->delete_pointers);
}

RTIBool World_Pose_c_copy(
    World_Pose_c* dst,
    const World_Pose_c* src)
{

    if (dst == NULL || src == NULL) {
        return RTI_FALSE;
    }

    if (!Position_c_copy(
        &dst->position,(const Position_c*)&src->position)) {
        return RTI_FALSE;
    } 
    if (!Orientation_c_copy(
        &dst->orientation,(const Orientation_c*)&src->orientation)) {
        return RTI_FALSE;
    } 

    return RTI_TRUE;

}

/**
* <<IMPLEMENTATION>>
*
* Defines:  TSeq, T
*
* Configure and implement 'World_Pose_c' sequence class.
*/
#define T World_Pose_c
#define TSeq World_Pose_cSeq

#define T_initialize_w_params World_Pose_c_initialize_w_params

#define T_finalize_w_params   World_Pose_c_finalize_w_params
#define T_copy       World_Pose_c_copy

#ifndef NDDS_STANDALONE_TYPE
#include "dds_c/generic/dds_c_sequence_TSeq.gen"
#else
#include "dds_c_sequence_TSeq.gen"
#endif

#undef T_copy
#undef T_finalize_w_params

#undef T_initialize_w_params

#undef TSeq
#undef T

/* ========================================================================= */
const char *laser_Scan_msg_cTYPENAME = "laser_Scan_msg";

DDS_TypeCode* laser_Scan_msg_c_get_typecode()
{
    static RTIBool is_initialized = RTI_FALSE;

    static DDS_TypeCode laser_Scan_msg_c_g_tc_ranges_sequence = DDS_INITIALIZE_SEQUENCE_TYPECODE(RTI_INT32_MAX,NULL);
    static DDS_TypeCode laser_Scan_msg_c_g_tc_intensities_sequence = DDS_INITIALIZE_SEQUENCE_TYPECODE(RTI_INT32_MAX,NULL);
    static DDS_TypeCode_Member laser_Scan_msg_c_g_tc_members[14]=
    {

        {
            (char *)"header",/* Member name */
            {
                0,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"world_pose",/* Member name */
            {
                1,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"angle_min",/* Member name */
            {
                2,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"angle_max",/* Member name */
            {
                3,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"angle_step",/* Member name */
            {
                4,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"range_min",/* Member name */
            {
                5,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"range_max",/* Member name */
            {
                6,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"count",/* Member name */
            {
                7,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"vertical_angle_min",/* Member name */
            {
                8,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"vertical_angle_max",/* Member name */
            {
                9,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"vertical_angle_step",/* Member name */
            {
                10,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"vertical_count",/* Member name */
            {
                11,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"ranges",/* Member name */
            {
                12,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"intensities",/* Member name */
            {
                13,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }
    };

    static DDS_TypeCode laser_Scan_msg_c_g_tc =
    {{
            DDS_TK_STRUCT,/* Kind */
            DDS_BOOLEAN_FALSE, /* Ignored */
            -1, /*Ignored*/
            (char *)"laser_Scan_msg", /* Name */
            NULL, /* Ignored */      
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            14, /* Number of members */
            laser_Scan_msg_c_g_tc_members, /* Members */
            DDS_VM_NONE  /* Ignored */         
        }}; /* Type code for laser_Scan_msg_c*/

    if (is_initialized) {
        return &laser_Scan_msg_c_g_tc;
    }

    laser_Scan_msg_c_g_tc_ranges_sequence._data._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_intensities_sequence._data._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)Header_c_get_typecode();

    laser_Scan_msg_c_g_tc_members[1]._representation._typeCode = (RTICdrTypeCode *)World_Pose_c_get_typecode();

    laser_Scan_msg_c_g_tc_members[2]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[3]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[4]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[5]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[6]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[7]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;

    laser_Scan_msg_c_g_tc_members[8]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[9]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[10]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[11]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[12]._representation._typeCode = (RTICdrTypeCode *)& laser_Scan_msg_c_g_tc_ranges_sequence;
    laser_Scan_msg_c_g_tc_members[13]._representation._typeCode = (RTICdrTypeCode *)& laser_Scan_msg_c_g_tc_intensities_sequence;

    is_initialized = RTI_TRUE;

    return &laser_Scan_msg_c_g_tc;
}

RTIBool laser_Scan_msg_c_initialize(
    laser_Scan_msg_c* sample) {
    return laser_Scan_msg_c_initialize_ex(sample,RTI_TRUE,RTI_TRUE);
}

RTIBool laser_Scan_msg_c_initialize_ex(
    laser_Scan_msg_c* sample,RTIBool allocatePointers, RTIBool allocateMemory)
{

    struct DDS_TypeAllocationParams_t allocParams =
    DDS_TYPE_ALLOCATION_PARAMS_DEFAULT;

    allocParams.allocate_pointers =  (DDS_Boolean)allocatePointers;
    allocParams.allocate_memory = (DDS_Boolean)allocateMemory;

    return laser_Scan_msg_c_initialize_w_params(
        sample,&allocParams);

}

RTIBool laser_Scan_msg_c_initialize_w_params(
    laser_Scan_msg_c* sample, const struct DDS_TypeAllocationParams_t * allocParams)
{

    void* buffer = NULL;
    if (buffer) {} /* To avoid warnings */

    if (sample == NULL) {
        return RTI_FALSE;
    }
    if (allocParams == NULL) {
        return RTI_FALSE;
    }

    if (!Header_c_initialize_w_params(&sample->header,
    allocParams)) {
        return RTI_FALSE;
    }
    if (!World_Pose_c_initialize_w_params(&sample->world_pose,
    allocParams)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->angle_min)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->angle_max)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->angle_step)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->range_min)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->range_max)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initLong(&sample->count)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->vertical_angle_min)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->vertical_angle_max)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->vertical_angle_step)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->vertical_count)) {
        return RTI_FALSE;
    }

    if (allocParams->allocate_memory) {
        DDS_FloatSeq_initialize(&sample->ranges  );
        DDS_FloatSeq_set_absolute_maximum(&sample->ranges , RTI_INT32_MAX);
        if (!DDS_FloatSeq_set_maximum(&sample->ranges , (0))) {
            return RTI_FALSE;
        }
    } else { 
        DDS_FloatSeq_set_length(&sample->ranges, 0);
    }
    if (allocParams->allocate_memory) {
        DDS_FloatSeq_initialize(&sample->intensities  );
        DDS_FloatSeq_set_absolute_maximum(&sample->intensities , RTI_INT32_MAX);
        if (!DDS_FloatSeq_set_maximum(&sample->intensities , (0))) {
            return RTI_FALSE;
        }
    } else { 
        DDS_FloatSeq_set_length(&sample->intensities, 0);
    }
    return RTI_TRUE;
}

void laser_Scan_msg_c_finalize(
    laser_Scan_msg_c* sample)
{

    laser_Scan_msg_c_finalize_ex(sample,RTI_TRUE);
}

void laser_Scan_msg_c_finalize_ex(
    laser_Scan_msg_c* sample,RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParams =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;

    if (sample==NULL) {
        return;
    } 

    deallocParams.delete_pointers = (DDS_Boolean)deletePointers;

    laser_Scan_msg_c_finalize_w_params(
        sample,&deallocParams);
}

void laser_Scan_msg_c_finalize_w_params(
    laser_Scan_msg_c* sample,const struct DDS_TypeDeallocationParams_t * deallocParams)
{

    if (sample==NULL) {
        return;
    }

    if (deallocParams == NULL) {
        return;
    }

    Header_c_finalize_w_params(&sample->header,deallocParams);

    World_Pose_c_finalize_w_params(&sample->world_pose,deallocParams);

    DDS_FloatSeq_finalize(&sample->ranges);

    DDS_FloatSeq_finalize(&sample->intensities);

}

void laser_Scan_msg_c_finalize_optional_members(
    laser_Scan_msg_c* sample, RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParamsTmp =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;
    struct DDS_TypeDeallocationParams_t * deallocParams =
    &deallocParamsTmp;

    if (sample==NULL) {
        return;
    } 
    if (deallocParams) {} /* To avoid warnings */

    deallocParamsTmp.delete_pointers = (DDS_Boolean)deletePointers;
    deallocParamsTmp.delete_optional_members = DDS_BOOLEAN_TRUE;

    Header_c_finalize_optional_members(&sample->header, deallocParams->delete_pointers);
    World_Pose_c_finalize_optional_members(&sample->world_pose, deallocParams->delete_pointers);
}

RTIBool laser_Scan_msg_c_copy(
    laser_Scan_msg_c* dst,
    const laser_Scan_msg_c* src)
{

    if (dst == NULL || src == NULL) {
        return RTI_FALSE;
    }

    if (!Header_c_copy(
        &dst->header,(const Header_c*)&src->header)) {
        return RTI_FALSE;
    } 
    if (!World_Pose_c_copy(
        &dst->world_pose,(const World_Pose_c*)&src->world_pose)) {
        return RTI_FALSE;
    } 
    if (!RTICdrType_copyFloat (
        &dst->angle_min, &src->angle_min)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->angle_max, &src->angle_max)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->angle_step, &src->angle_step)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->range_min, &src->range_min)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->range_max, &src->range_max)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyLong (
        &dst->count, &src->count)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->vertical_angle_min, &src->vertical_angle_min)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->vertical_angle_max, &src->vertical_angle_max)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->vertical_angle_step, &src->vertical_angle_step)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->vertical_count, &src->vertical_count)) { 
        return RTI_FALSE;
    }
    if (!DDS_FloatSeq_copy(&dst->ranges ,
    &src->ranges )) {
        return RTI_FALSE;
    }
    if (!DDS_FloatSeq_copy(&dst->intensities ,
    &src->intensities )) {
        return RTI_FALSE;
    }

    return RTI_TRUE;

}

/**
* <<IMPLEMENTATION>>
*
* Defines:  TSeq, T
*
* Configure and implement 'laser_Scan_msg_c' sequence class.
*/
#define T laser_Scan_msg_c
#define TSeq laser_Scan_msg_cSeq

#define T_initialize_w_params laser_Scan_msg_c_initialize_w_params

#define T_finalize_w_params   laser_Scan_msg_c_finalize_w_params
#define T_copy       laser_Scan_msg_c_copy

#ifndef NDDS_STANDALONE_TYPE
#include "dds_c/generic/dds_c_sequence_TSeq.gen"
#else
#include "dds_c_sequence_TSeq.gen"
#endif

#undef T_copy
#undef T_finalize_w_params

#undef T_initialize_w_params

#undef TSeq
#undef T

