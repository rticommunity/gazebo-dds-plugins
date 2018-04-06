

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
    static DDS_TypeCode_Member Header_c_g_tc_members[3]=
    {

        {
            (char *)"seq",/* Member name */
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
            (char *)"stamp",/* Member name */
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
            (char *)"frame_id",/* Member name */
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
            3, /* Number of members */
            Header_c_g_tc_members, /* Members */
            DDS_VM_NONE  /* Ignored */         
        }}; /* Type code for Header_c*/

    if (is_initialized) {
        return &Header_c_g_tc;
    }

    Header_c_g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_ulong;

    Header_c_g_tc_members[1]._representation._typeCode = (RTICdrTypeCode *)Time_c_get_typecode();

    Header_c_g_tc_members[2]._representation._typeCode = (RTICdrTypeCode *)&Header_c_g_tc_frame_id_string;

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

    if (!RTICdrType_initUnsignedLong(&sample->seq)) {
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

    if (!RTICdrType_copyUnsignedLong (
        &dst->seq, &src->seq)) { 
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
const char *laser_Scan_msg_cTYPENAME = "laser_Scan_msg";

DDS_TypeCode* laser_Scan_msg_c_get_typecode()
{
    static RTIBool is_initialized = RTI_FALSE;

    static DDS_TypeCode laser_Scan_msg_c_g_tc_ranges_sequence = DDS_INITIALIZE_SEQUENCE_TYPECODE(RTI_INT32_MAX,NULL);
    static DDS_TypeCode laser_Scan_msg_c_g_tc_intensities_sequence = DDS_INITIALIZE_SEQUENCE_TYPECODE(RTI_INT32_MAX,NULL);
    static DDS_TypeCode_Member laser_Scan_msg_c_g_tc_members[10]=
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
            (char *)"angle_min",/* Member name */
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
            (char *)"angle_max",/* Member name */
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
            (char *)"angle_increment",/* Member name */
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
            (char *)"time_increment",/* Member name */
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
            (char *)"scan_time",/* Member name */
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
            (char *)"range_min",/* Member name */
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
            (char *)"range_max",/* Member name */
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
            (char *)"ranges",/* Member name */
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
            (char *)"intensities",/* Member name */
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
            10, /* Number of members */
            laser_Scan_msg_c_g_tc_members, /* Members */
            DDS_VM_NONE  /* Ignored */         
        }}; /* Type code for laser_Scan_msg_c*/

    if (is_initialized) {
        return &laser_Scan_msg_c_g_tc;
    }

    laser_Scan_msg_c_g_tc_ranges_sequence._data._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_intensities_sequence._data._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)Header_c_get_typecode();

    laser_Scan_msg_c_g_tc_members[1]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[2]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[3]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[4]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[5]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[6]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[7]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;

    laser_Scan_msg_c_g_tc_members[8]._representation._typeCode = (RTICdrTypeCode *)& laser_Scan_msg_c_g_tc_ranges_sequence;
    laser_Scan_msg_c_g_tc_members[9]._representation._typeCode = (RTICdrTypeCode *)& laser_Scan_msg_c_g_tc_intensities_sequence;

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

    if (!RTICdrType_initFloat(&sample->angle_min)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->angle_max)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->angle_increment)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->time_increment)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->scan_time)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->range_min)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initFloat(&sample->range_max)) {
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
    if (!RTICdrType_copyFloat (
        &dst->angle_min, &src->angle_min)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->angle_max, &src->angle_max)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->angle_increment, &src->angle_increment)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->time_increment, &src->time_increment)) { 
        return RTI_FALSE;
    }
    if (!RTICdrType_copyFloat (
        &dst->scan_time, &src->scan_time)) { 
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

