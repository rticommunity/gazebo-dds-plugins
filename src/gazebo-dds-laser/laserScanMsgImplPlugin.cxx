
/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from laserScanMsgImpl.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#include <string.h>

#ifndef ndds_c_h
#include "ndds/ndds_c.h"
#endif

#ifndef osapi_type_h
#include "osapi/osapi_type.h"
#endif
#ifndef osapi_heap_h
#include "osapi/osapi_heap.h"
#endif

#ifndef osapi_utility_h
#include "osapi/osapi_utility.h"
#endif

#ifndef cdr_type_h
#include "cdr/cdr_type.h"
#endif

#ifndef cdr_type_object_h
#include "cdr/cdr_typeObject.h"
#endif

#ifndef cdr_encapsulation_h
#include "cdr/cdr_encapsulation.h"
#endif

#ifndef cdr_stream_h
#include "cdr/cdr_stream.h"
#endif

#ifndef cdr_log_h
#include "cdr/cdr_log.h"
#endif

#ifndef pres_typePlugin_h
#include "pres/pres_typePlugin.h"
#endif

#define RTI_CDR_CURRENT_SUBMODULE RTI_CDR_SUBMODULE_MASK_STREAM

#include "laserScanMsgImplPlugin.h"

/* ----------------------------------------------------------------------------
*  Type Time_c
* -------------------------------------------------------------------------- */

/* -----------------------------------------------------------------------------
Support functions:
* -------------------------------------------------------------------------- */

Time_c*
Time_cPluginSupport_create_data_w_params(
    const struct DDS_TypeAllocationParams_t * alloc_params) 
{
    Time_c *sample = NULL;

    RTIOsapiHeap_allocateStructure(&(sample),Time_c);
    if (sample == NULL) {
        return NULL;
    }

    if (!Time_c_initialize_w_params(sample,alloc_params)) {
        RTIOsapiHeap_freeStructure(sample);
        sample=NULL;
    }
    return sample; 
} 

Time_c *
Time_cPluginSupport_create_data_ex(RTIBool allocate_pointers) 
{
    Time_c *sample = NULL;

    RTIOsapiHeap_allocateStructure(&(sample),Time_c);

    if(sample == NULL) {
        return NULL;
    }

    if (!Time_c_initialize_ex(sample,allocate_pointers, RTI_TRUE)) {
        RTIOsapiHeap_freeStructure(sample);
        sample=NULL;
    }

    return sample; 
}

Time_c *
Time_cPluginSupport_create_data(void)
{
    return Time_cPluginSupport_create_data_ex(RTI_TRUE);
}

void 
Time_cPluginSupport_destroy_data_w_params(
    Time_c *sample,
    const struct DDS_TypeDeallocationParams_t * dealloc_params) {

    Time_c_finalize_w_params(sample,dealloc_params);

    RTIOsapiHeap_freeStructure(sample);
    sample=NULL;
}

void 
Time_cPluginSupport_destroy_data_ex(
    Time_c *sample,RTIBool deallocate_pointers) {

    Time_c_finalize_ex(sample,deallocate_pointers);

    RTIOsapiHeap_freeStructure(sample);
    sample=NULL;
}

void 
Time_cPluginSupport_destroy_data(
    Time_c *sample) {

    Time_cPluginSupport_destroy_data_ex(sample,RTI_TRUE);

}

RTIBool 
Time_cPluginSupport_copy_data(
    Time_c *dst,
    const Time_c *src)
{
    return Time_c_copy(dst,(const Time_c*) src);
}

void 
Time_cPluginSupport_print_data(
    const Time_c *sample,
    const char *desc,
    unsigned int indent_level)
{

    RTICdrType_printIndent(indent_level);

    if (desc != NULL) {
        RTILog_debug("%s:\n", desc);
    } else {
        RTILog_debug("\n");
    }

    if (sample == NULL) {
        RTILog_debug("NULL\n");
        return;
    }

    RTICdrType_printUnsignedLong(
        &sample->sec, "sec", indent_level + 1);    

    RTICdrType_printUnsignedLong(
        &sample->nsec, "nsec", indent_level + 1);    

}

/* ----------------------------------------------------------------------------
Callback functions:
* ---------------------------------------------------------------------------- */

PRESTypePluginParticipantData 
Time_cPlugin_on_participant_attached(
    void *registration_data,
    const struct PRESTypePluginParticipantInfo *participant_info,
    RTIBool top_level_registration,
    void *container_plugin_context,
    RTICdrTypeCode *type_code)
{
    if (registration_data) {} /* To avoid warnings */
    if (participant_info) {} /* To avoid warnings */
    if (top_level_registration) {} /* To avoid warnings */
    if (container_plugin_context) {} /* To avoid warnings */
    if (type_code) {} /* To avoid warnings */

    return PRESTypePluginDefaultParticipantData_new(participant_info);

}

void 
Time_cPlugin_on_participant_detached(
    PRESTypePluginParticipantData participant_data)
{

    PRESTypePluginDefaultParticipantData_delete(participant_data);
}

PRESTypePluginEndpointData
Time_cPlugin_on_endpoint_attached(
    PRESTypePluginParticipantData participant_data,
    const struct PRESTypePluginEndpointInfo *endpoint_info,
    RTIBool top_level_registration, 
    void *containerPluginContext)
{
    PRESTypePluginEndpointData epd = NULL;

    unsigned int serializedSampleMaxSize;

    if (top_level_registration) {} /* To avoid warnings */
    if (containerPluginContext) {} /* To avoid warnings */

    epd = PRESTypePluginDefaultEndpointData_new(
        participant_data,
        endpoint_info,
        (PRESTypePluginDefaultEndpointDataCreateSampleFunction)
        Time_cPluginSupport_create_data,
        (PRESTypePluginDefaultEndpointDataDestroySampleFunction)
        Time_cPluginSupport_destroy_data,
        NULL , NULL );

    if (epd == NULL) {
        return NULL;
    } 

    if (endpoint_info->endpointKind == PRES_TYPEPLUGIN_ENDPOINT_WRITER) {
        serializedSampleMaxSize = Time_cPlugin_get_serialized_sample_max_size(
            epd,RTI_FALSE,RTI_CDR_ENCAPSULATION_ID_CDR_BE,0);

        PRESTypePluginDefaultEndpointData_setMaxSizeSerializedSample(epd, serializedSampleMaxSize);

        if (PRESTypePluginDefaultEndpointData_createWriterPool(
            epd,
            endpoint_info,
            (PRESTypePluginGetSerializedSampleMaxSizeFunction)
            Time_cPlugin_get_serialized_sample_max_size, epd,
            (PRESTypePluginGetSerializedSampleSizeFunction)
            Time_cPlugin_get_serialized_sample_size,
            epd) == RTI_FALSE) {
            PRESTypePluginDefaultEndpointData_delete(epd);
            return NULL;
        }
    }

    return epd;    
}

void 
Time_cPlugin_on_endpoint_detached(
    PRESTypePluginEndpointData endpoint_data)
{  

    PRESTypePluginDefaultEndpointData_delete(endpoint_data);
}

void    
Time_cPlugin_return_sample(
    PRESTypePluginEndpointData endpoint_data,
    Time_c *sample,
    void *handle)
{

    Time_c_finalize_optional_members(sample, RTI_TRUE);

    PRESTypePluginDefaultEndpointData_returnSample(
        endpoint_data, sample, handle);
}

RTIBool 
Time_cPlugin_copy_sample(
    PRESTypePluginEndpointData endpoint_data,
    Time_c *dst,
    const Time_c *src)
{
    if (endpoint_data) {} /* To avoid warnings */
    return Time_cPluginSupport_copy_data(dst,src);
}

/* ----------------------------------------------------------------------------
(De)Serialize functions:
* ------------------------------------------------------------------------- */
unsigned int 
Time_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

RTIBool 
Time_cPlugin_serialize(
    PRESTypePluginEndpointData endpoint_data,
    const Time_c *sample, 
    struct RTICdrStream *stream,    
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;
    RTIBool retval = RTI_TRUE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(serialize_encapsulation) {
        if (!RTICdrStream_serializeAndSetCdrEncapsulation(stream , encapsulation_id)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if(serialize_sample) {

        if (!RTICdrStream_serializeUnsignedLong(
            stream, &sample->sec)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeUnsignedLong(
            stream, &sample->nsec)) {
            return RTI_FALSE;
        }

    }

    if(serialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return retval;
}

RTIBool 
Time_cPlugin_deserialize_sample(
    PRESTypePluginEndpointData endpoint_data,
    Time_c *sample,
    struct RTICdrStream *stream,   
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos)
{

    char * position = NULL;

    RTIBool done = RTI_FALSE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */
    if(deserialize_encapsulation) {

        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }
    if(deserialize_sample) {

        Time_c_initialize_ex(sample, RTI_FALSE, RTI_FALSE);

        if (!RTICdrStream_deserializeUnsignedLong(
            stream, &sample->sec)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeUnsignedLong(
            stream, &sample->nsec)) {
            goto fin; 
        }
    }

    done = RTI_TRUE;
  fin:
    if (done != RTI_TRUE && 
    RTICdrStream_getRemainder(stream) >=
    RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
        return RTI_FALSE;   
    }
    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;

}

RTIBool
Time_cPlugin_serialize_to_cdr_buffer(
    char * buffer,
    unsigned int * length,
    const Time_c *sample)
{
    struct RTICdrStream stream;
    struct PRESTypePluginDefaultEndpointData epd;
    RTIBool result;

    if (length == NULL) {
        return RTI_FALSE;
    }

    epd._maxSizeSerializedSample =
    Time_cPlugin_get_serialized_sample_max_size(
        NULL, RTI_TRUE, RTICdrEncapsulation_getNativeCdrEncapsulationId(), 0);

    if (buffer == NULL) {
        *length = 
        Time_cPlugin_get_serialized_sample_size(
            (PRESTypePluginEndpointData)&epd,
            RTI_TRUE,
            RTICdrEncapsulation_getNativeCdrEncapsulationId(),
            0,
            sample);

        if (*length == 0) {
            return RTI_FALSE;
        }

        return RTI_TRUE;
    }    

    RTICdrStream_init(&stream);
    RTICdrStream_set(&stream, (char *)buffer, *length);

    result = Time_cPlugin_serialize(
        (PRESTypePluginEndpointData)&epd, sample, &stream, 
        RTI_TRUE, RTICdrEncapsulation_getNativeCdrEncapsulationId(), 
        RTI_TRUE, NULL);  

    *length = RTICdrStream_getCurrentPositionOffset(&stream);
    return result;     
}

RTIBool
Time_cPlugin_deserialize_from_cdr_buffer(
    Time_c *sample,
    const char * buffer,
    unsigned int length)
{
    struct RTICdrStream stream;

    RTICdrStream_init(&stream);
    RTICdrStream_set(&stream, (char *)buffer, length);

    Time_c_finalize_optional_members(sample, RTI_TRUE);
    return Time_cPlugin_deserialize_sample( 
        NULL, sample,
        &stream, RTI_TRUE, RTI_TRUE, 
        NULL);
}

DDS_ReturnCode_t
Time_cPlugin_data_to_string(
    const Time_c *sample,
    char *str,
    DDS_UnsignedLong *str_size, 
    const struct DDS_PrintFormatProperty *property)
{
    DDS_DynamicData *data = NULL;
    char *buffer = NULL;
    unsigned int length = 0;
    struct DDS_PrintFormat printFormat;
    DDS_ReturnCode_t retCode = DDS_RETCODE_ERROR;

    if (sample == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (str_size == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (property == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (!Time_cPlugin_serialize_to_cdr_buffer(
        NULL, 
        &length, 
        sample)) {
        return DDS_RETCODE_ERROR;
    }

    RTIOsapiHeap_allocateBuffer(&buffer, length, RTI_OSAPI_ALIGNMENT_DEFAULT);
    if (buffer == NULL) {
        return DDS_RETCODE_ERROR;
    }

    if (!Time_cPlugin_serialize_to_cdr_buffer(
        buffer, 
        &length, 
        sample)) {
        RTIOsapiHeap_freeBuffer(buffer);
        return DDS_RETCODE_ERROR;
    }

    data = DDS_DynamicData_new(
        Time_c_get_typecode(), 
        &DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
    if (data == NULL) {
        RTIOsapiHeap_freeBuffer(buffer);
        return DDS_RETCODE_ERROR;
    }

    retCode = DDS_DynamicData_from_cdr_buffer(data, buffer, length);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    retCode = DDS_PrintFormatProperty_to_print_format(
        property, 
        &printFormat);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    retCode = DDS_DynamicDataFormatter_to_string_w_format(
        data, 
        str,
        str_size, 
        &printFormat);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    RTIOsapiHeap_freeBuffer(buffer);
    DDS_DynamicData_delete(data);
    return DDS_RETCODE_OK;
}

RTIBool 
Time_cPlugin_deserialize(
    PRESTypePluginEndpointData endpoint_data,
    Time_c **sample,
    RTIBool * drop_sample,
    struct RTICdrStream *stream,   
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos)
{

    RTIBool result;
    const char *METHOD_NAME = "Time_cPlugin_deserialize";
    if (drop_sample) {} /* To avoid warnings */

    stream->_xTypesState.unassignable = RTI_FALSE;
    result= Time_cPlugin_deserialize_sample( 
        endpoint_data, (sample != NULL)?*sample:NULL,
        stream, deserialize_encapsulation, deserialize_sample, 
        endpoint_plugin_qos);
    if (result) {
        if (stream->_xTypesState.unassignable) {
            result = RTI_FALSE;
        }
    }
    if (!result && stream->_xTypesState.unassignable ) {

        RTICdrLog_exception(
            METHOD_NAME, 
            &RTI_CDR_LOG_UNASSIGNABLE_SAMPLE_OF_TYPE_s, 
            "Time_c");

    }

    return result;

}

RTIBool Time_cPlugin_skip(
    PRESTypePluginEndpointData endpoint_data,
    struct RTICdrStream *stream,   
    RTIBool skip_encapsulation,
    RTIBool skip_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    RTIBool done = RTI_FALSE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(skip_encapsulation) {
        if (!RTICdrStream_skipEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if (skip_sample) {

        if (!RTICdrStream_skipUnsignedLong (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipUnsignedLong (stream)) {
            goto fin; 
        }
    }

    done = RTI_TRUE;
  fin:
    if (done != RTI_TRUE && 
    RTICdrStream_getRemainder(stream) >=
    RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
        return RTI_FALSE;   
    }
    if(skip_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

unsigned int 
Time_cPlugin_get_serialized_sample_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */ 
    if (overflow) {} /* To avoid warnings */

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment +=RTICdrType_getUnsignedLongMaxSizeSerialized(
        current_alignment);

    current_alignment +=RTICdrType_getUnsignedLongMaxSizeSerialized(
        current_alignment);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return  current_alignment - initial_alignment;
}

unsigned int 
Time_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{
    unsigned int size;
    RTIBool overflow = RTI_FALSE;

    size = Time_cPlugin_get_serialized_sample_max_size_ex(
        endpoint_data,&overflow,include_encapsulation,encapsulation_id,current_alignment);

    if (overflow) {
        size = RTI_CDR_MAX_SERIALIZED_SIZE;
    }

    return size;
}

unsigned int 
Time_cPlugin_get_serialized_sample_min_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */ 

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment +=RTICdrType_getUnsignedLongMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getUnsignedLongMaxSizeSerialized(
        current_alignment);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return  current_alignment - initial_alignment;
}

/* Returns the size of the sample in its serialized form (in bytes).
* It can also be an estimation in excess of the real buffer needed 
* during a call to the serialize() function.
* The value reported does not have to include the space for the
* encapsulation flags.
*/
unsigned int
Time_cPlugin_get_serialized_sample_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment,
    const Time_c * sample) 
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;
    struct PRESTypePluginDefaultEndpointData epd;   

    if (sample==NULL) {
        return 0;
    }
    if (endpoint_data == NULL) {
        endpoint_data = (PRESTypePluginEndpointData) &epd;
        PRESTypePluginDefaultEndpointData_setBaseAlignment(
            endpoint_data,
            current_alignment);        
    }

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
        PRESTypePluginDefaultEndpointData_setBaseAlignment(
            endpoint_data,
            current_alignment);
    }

    current_alignment += RTICdrType_getUnsignedLongMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getUnsignedLongMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return current_alignment - initial_alignment;
}

/* --------------------------------------------------------------------------------------
Key Management functions:
* -------------------------------------------------------------------------------------- */

PRESTypePluginKeyKind 
Time_cPlugin_get_key_kind(void)
{
    return PRES_TYPEPLUGIN_NO_KEY;
}

RTIBool 
Time_cPlugin_serialize_key(
    PRESTypePluginEndpointData endpoint_data,
    const Time_c *sample, 
    struct RTICdrStream *stream,    
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_key,
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    if(serialize_encapsulation) {
        if (!RTICdrStream_serializeAndSetCdrEncapsulation(stream , encapsulation_id)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if(serialize_key) {

        if (!Time_cPlugin_serialize(
            endpoint_data,
            sample,
            stream,
            RTI_FALSE, encapsulation_id,
            RTI_TRUE,
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

    }

    if(serialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

RTIBool Time_cPlugin_deserialize_key_sample(
    PRESTypePluginEndpointData endpoint_data,
    Time_c *sample, 
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos)
{

    char * position = NULL;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(deserialize_encapsulation) {

        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }
    if (deserialize_key) {

        if (!Time_cPlugin_deserialize_sample(
            endpoint_data, sample, stream, 
            RTI_FALSE, RTI_TRUE, 
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }
    }

    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;

}

RTIBool Time_cPlugin_deserialize_key(
    PRESTypePluginEndpointData endpoint_data,
    Time_c **sample, 
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos)
{
    RTIBool result;
    if (drop_sample) {} /* To avoid warnings */
    stream->_xTypesState.unassignable = RTI_FALSE;
    result= Time_cPlugin_deserialize_key_sample(
        endpoint_data, (sample != NULL)?*sample:NULL, stream,
        deserialize_encapsulation, deserialize_key, endpoint_plugin_qos);
    if (result) {
        if (stream->_xTypesState.unassignable) {
            result = RTI_FALSE;
        }
    }

    return result;    

}

unsigned int
Time_cPlugin_get_serialized_key_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */
    if (overflow) {} /* To avoid warnings */

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment += Time_cPlugin_get_serialized_sample_max_size_ex(
        endpoint_data, overflow,RTI_FALSE, encapsulation_id, current_alignment);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return current_alignment - initial_alignment;
}

unsigned int
Time_cPlugin_get_serialized_key_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{
    unsigned int size;
    RTIBool overflow = RTI_FALSE;

    size = Time_cPlugin_get_serialized_key_max_size_ex(
        endpoint_data,&overflow,include_encapsulation,encapsulation_id,current_alignment);

    if (overflow) {
        size = RTI_CDR_MAX_SERIALIZED_SIZE;
    }

    return size;
}

RTIBool 
Time_cPlugin_serialized_sample_to_key(
    PRESTypePluginEndpointData endpoint_data,
    Time_c *sample,
    struct RTICdrStream *stream, 
    RTIBool deserialize_encapsulation,  
    RTIBool deserialize_key, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    RTIBool done = RTI_FALSE;
    RTIBool error = RTI_FALSE;

    if (stream == NULL) {
        error = RTI_TRUE;
        goto fin;
    }
    if(deserialize_encapsulation) {
        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }
        position = RTICdrStream_resetAlignment(stream);
    }

    if (deserialize_key) {

        if (!Time_cPlugin_deserialize_sample(
            endpoint_data, sample, stream, RTI_FALSE, 
            RTI_TRUE, endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

    }

    done = RTI_TRUE;
  fin:
    if(!error) {
        if (done != RTI_TRUE && 
        RTICdrStream_getRemainder(stream) >=
        RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
            return RTI_FALSE;   
        }
    } else {
        return RTI_FALSE;
    }       

    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

/* ------------------------------------------------------------------------
* Plug-in Installation Methods
* ------------------------------------------------------------------------ */
struct PRESTypePlugin *Time_cPlugin_new(void) 
{ 
    struct PRESTypePlugin *plugin = NULL;
    const struct PRESTypePluginVersion PLUGIN_VERSION = 
    PRES_TYPE_PLUGIN_VERSION_2_0;

    RTIOsapiHeap_allocateStructure(
        &plugin, struct PRESTypePlugin);

    if (plugin == NULL) {
        return NULL;
    }

    plugin->version = PLUGIN_VERSION;

    /* set up parent's function pointers */
    plugin->onParticipantAttached =
    (PRESTypePluginOnParticipantAttachedCallback)
    Time_cPlugin_on_participant_attached;
    plugin->onParticipantDetached =
    (PRESTypePluginOnParticipantDetachedCallback)
    Time_cPlugin_on_participant_detached;
    plugin->onEndpointAttached =
    (PRESTypePluginOnEndpointAttachedCallback)
    Time_cPlugin_on_endpoint_attached;
    plugin->onEndpointDetached =
    (PRESTypePluginOnEndpointDetachedCallback)
    Time_cPlugin_on_endpoint_detached;

    plugin->copySampleFnc =
    (PRESTypePluginCopySampleFunction)
    Time_cPlugin_copy_sample;
    plugin->createSampleFnc =
    (PRESTypePluginCreateSampleFunction)
    Time_cPlugin_create_sample;
    plugin->destroySampleFnc =
    (PRESTypePluginDestroySampleFunction)
    Time_cPlugin_destroy_sample;
    plugin->finalizeOptionalMembersFnc =
    (PRESTypePluginFinalizeOptionalMembersFunction)
    Time_c_finalize_optional_members;

    plugin->serializeFnc =
    (PRESTypePluginSerializeFunction)
    Time_cPlugin_serialize;
    plugin->deserializeFnc =
    (PRESTypePluginDeserializeFunction)
    Time_cPlugin_deserialize;
    plugin->getSerializedSampleMaxSizeFnc =
    (PRESTypePluginGetSerializedSampleMaxSizeFunction)
    Time_cPlugin_get_serialized_sample_max_size;
    plugin->getSerializedSampleMinSizeFnc =
    (PRESTypePluginGetSerializedSampleMinSizeFunction)
    Time_cPlugin_get_serialized_sample_min_size;

    plugin->getSampleFnc =
    (PRESTypePluginGetSampleFunction)
    Time_cPlugin_get_sample;
    plugin->returnSampleFnc =
    (PRESTypePluginReturnSampleFunction)
    Time_cPlugin_return_sample;

    plugin->getKeyKindFnc =
    (PRESTypePluginGetKeyKindFunction)
    Time_cPlugin_get_key_kind;

    /* These functions are only used for keyed types. As this is not a keyed
    type they are all set to NULL
    */
    plugin->serializeKeyFnc = NULL ;    
    plugin->deserializeKeyFnc = NULL;  
    plugin->getKeyFnc = NULL;
    plugin->returnKeyFnc = NULL;
    plugin->instanceToKeyFnc = NULL;
    plugin->keyToInstanceFnc = NULL;
    plugin->getSerializedKeyMaxSizeFnc = NULL;
    plugin->instanceToKeyHashFnc = NULL;
    plugin->serializedSampleToKeyHashFnc = NULL;
    plugin->serializedKeyToKeyHashFnc = NULL;    
    plugin->typeCode =  (struct RTICdrTypeCode *)Time_c_get_typecode();

    plugin->languageKind = PRES_TYPEPLUGIN_CPP_LANG;

    /* Serialized buffer */
    plugin->getBuffer = 
    (PRESTypePluginGetBufferFunction)
    Time_cPlugin_get_buffer;
    plugin->returnBuffer = 
    (PRESTypePluginReturnBufferFunction)
    Time_cPlugin_return_buffer;
    plugin->getSerializedSampleSizeFnc =
    (PRESTypePluginGetSerializedSampleSizeFunction)
    Time_cPlugin_get_serialized_sample_size;

    plugin->endpointTypeName = Time_cTYPENAME;

    return plugin;
}

void
Time_cPlugin_delete(struct PRESTypePlugin *plugin)
{
    RTIOsapiHeap_freeStructure(plugin);
} 

/* ----------------------------------------------------------------------------
*  Type Header_c
* -------------------------------------------------------------------------- */

/* -----------------------------------------------------------------------------
Support functions:
* -------------------------------------------------------------------------- */

Header_c*
Header_cPluginSupport_create_data_w_params(
    const struct DDS_TypeAllocationParams_t * alloc_params) 
{
    Header_c *sample = NULL;

    RTIOsapiHeap_allocateStructure(&(sample),Header_c);
    if (sample == NULL) {
        return NULL;
    }

    if (!Header_c_initialize_w_params(sample,alloc_params)) {
        RTIOsapiHeap_freeStructure(sample);
        sample=NULL;
    }
    return sample; 
} 

Header_c *
Header_cPluginSupport_create_data_ex(RTIBool allocate_pointers) 
{
    Header_c *sample = NULL;

    RTIOsapiHeap_allocateStructure(&(sample),Header_c);

    if(sample == NULL) {
        return NULL;
    }

    if (!Header_c_initialize_ex(sample,allocate_pointers, RTI_TRUE)) {
        RTIOsapiHeap_freeStructure(sample);
        sample=NULL;
    }

    return sample; 
}

Header_c *
Header_cPluginSupport_create_data(void)
{
    return Header_cPluginSupport_create_data_ex(RTI_TRUE);
}

void 
Header_cPluginSupport_destroy_data_w_params(
    Header_c *sample,
    const struct DDS_TypeDeallocationParams_t * dealloc_params) {

    Header_c_finalize_w_params(sample,dealloc_params);

    RTIOsapiHeap_freeStructure(sample);
    sample=NULL;
}

void 
Header_cPluginSupport_destroy_data_ex(
    Header_c *sample,RTIBool deallocate_pointers) {

    Header_c_finalize_ex(sample,deallocate_pointers);

    RTIOsapiHeap_freeStructure(sample);
    sample=NULL;
}

void 
Header_cPluginSupport_destroy_data(
    Header_c *sample) {

    Header_cPluginSupport_destroy_data_ex(sample,RTI_TRUE);

}

RTIBool 
Header_cPluginSupport_copy_data(
    Header_c *dst,
    const Header_c *src)
{
    return Header_c_copy(dst,(const Header_c*) src);
}

void 
Header_cPluginSupport_print_data(
    const Header_c *sample,
    const char *desc,
    unsigned int indent_level)
{

    RTICdrType_printIndent(indent_level);

    if (desc != NULL) {
        RTILog_debug("%s:\n", desc);
    } else {
        RTILog_debug("\n");
    }

    if (sample == NULL) {
        RTILog_debug("NULL\n");
        return;
    }

    Time_cPluginSupport_print_data(
        (const Time_c*) &sample->stamp, "stamp", indent_level + 1);

    if (sample->frame_id==NULL) {
        RTICdrType_printString(
            NULL,"frame_id", indent_level + 1);
    } else {
        RTICdrType_printString(
            sample->frame_id,"frame_id", indent_level + 1);    
    }

}

/* ----------------------------------------------------------------------------
Callback functions:
* ---------------------------------------------------------------------------- */

PRESTypePluginParticipantData 
Header_cPlugin_on_participant_attached(
    void *registration_data,
    const struct PRESTypePluginParticipantInfo *participant_info,
    RTIBool top_level_registration,
    void *container_plugin_context,
    RTICdrTypeCode *type_code)
{
    if (registration_data) {} /* To avoid warnings */
    if (participant_info) {} /* To avoid warnings */
    if (top_level_registration) {} /* To avoid warnings */
    if (container_plugin_context) {} /* To avoid warnings */
    if (type_code) {} /* To avoid warnings */

    return PRESTypePluginDefaultParticipantData_new(participant_info);

}

void 
Header_cPlugin_on_participant_detached(
    PRESTypePluginParticipantData participant_data)
{

    PRESTypePluginDefaultParticipantData_delete(participant_data);
}

PRESTypePluginEndpointData
Header_cPlugin_on_endpoint_attached(
    PRESTypePluginParticipantData participant_data,
    const struct PRESTypePluginEndpointInfo *endpoint_info,
    RTIBool top_level_registration, 
    void *containerPluginContext)
{
    PRESTypePluginEndpointData epd = NULL;

    unsigned int serializedSampleMaxSize;

    if (top_level_registration) {} /* To avoid warnings */
    if (containerPluginContext) {} /* To avoid warnings */

    epd = PRESTypePluginDefaultEndpointData_new(
        participant_data,
        endpoint_info,
        (PRESTypePluginDefaultEndpointDataCreateSampleFunction)
        Header_cPluginSupport_create_data,
        (PRESTypePluginDefaultEndpointDataDestroySampleFunction)
        Header_cPluginSupport_destroy_data,
        NULL , NULL );

    if (epd == NULL) {
        return NULL;
    } 

    if (endpoint_info->endpointKind == PRES_TYPEPLUGIN_ENDPOINT_WRITER) {
        serializedSampleMaxSize = Header_cPlugin_get_serialized_sample_max_size(
            epd,RTI_FALSE,RTI_CDR_ENCAPSULATION_ID_CDR_BE,0);

        PRESTypePluginDefaultEndpointData_setMaxSizeSerializedSample(epd, serializedSampleMaxSize);

        if (PRESTypePluginDefaultEndpointData_createWriterPool(
            epd,
            endpoint_info,
            (PRESTypePluginGetSerializedSampleMaxSizeFunction)
            Header_cPlugin_get_serialized_sample_max_size, epd,
            (PRESTypePluginGetSerializedSampleSizeFunction)
            Header_cPlugin_get_serialized_sample_size,
            epd) == RTI_FALSE) {
            PRESTypePluginDefaultEndpointData_delete(epd);
            return NULL;
        }
    }

    return epd;    
}

void 
Header_cPlugin_on_endpoint_detached(
    PRESTypePluginEndpointData endpoint_data)
{  

    PRESTypePluginDefaultEndpointData_delete(endpoint_data);
}

void    
Header_cPlugin_return_sample(
    PRESTypePluginEndpointData endpoint_data,
    Header_c *sample,
    void *handle)
{

    Header_c_finalize_optional_members(sample, RTI_TRUE);

    PRESTypePluginDefaultEndpointData_returnSample(
        endpoint_data, sample, handle);
}

RTIBool 
Header_cPlugin_copy_sample(
    PRESTypePluginEndpointData endpoint_data,
    Header_c *dst,
    const Header_c *src)
{
    if (endpoint_data) {} /* To avoid warnings */
    return Header_cPluginSupport_copy_data(dst,src);
}

/* ----------------------------------------------------------------------------
(De)Serialize functions:
* ------------------------------------------------------------------------- */
unsigned int 
Header_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

RTIBool 
Header_cPlugin_serialize(
    PRESTypePluginEndpointData endpoint_data,
    const Header_c *sample, 
    struct RTICdrStream *stream,    
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;
    RTIBool retval = RTI_TRUE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(serialize_encapsulation) {
        if (!RTICdrStream_serializeAndSetCdrEncapsulation(stream , encapsulation_id)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if(serialize_sample) {

        if(!Time_cPlugin_serialize(
            endpoint_data,
            (const Time_c*) &sample->stamp,
            stream,
            RTI_FALSE, encapsulation_id,
            RTI_TRUE,
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeString(
            stream, sample->frame_id, (RTI_INT32_MAX-1) + 1)) {
            return RTI_FALSE;
        }

    }

    if(serialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return retval;
}

RTIBool 
Header_cPlugin_deserialize_sample(
    PRESTypePluginEndpointData endpoint_data,
    Header_c *sample,
    struct RTICdrStream *stream,   
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos)
{

    char * position = NULL;

    RTIBool done = RTI_FALSE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */
    if(deserialize_encapsulation) {

        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }
    if(deserialize_sample) {

        Header_c_initialize_ex(sample, RTI_FALSE, RTI_FALSE);

        if(!Time_cPlugin_deserialize_sample(
            endpoint_data,
            &sample->stamp,
            stream,
            RTI_FALSE, RTI_TRUE,
            endpoint_plugin_qos)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeStringEx(
            stream,&sample->frame_id, (RTI_INT32_MAX-1) + 1,RTI_TRUE)) {
            goto fin; 
        }
    }

    done = RTI_TRUE;
  fin:
    if (done != RTI_TRUE && 
    RTICdrStream_getRemainder(stream) >=
    RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
        return RTI_FALSE;   
    }
    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;

}

RTIBool
Header_cPlugin_serialize_to_cdr_buffer(
    char * buffer,
    unsigned int * length,
    const Header_c *sample)
{
    struct RTICdrStream stream;
    struct PRESTypePluginDefaultEndpointData epd;
    RTIBool result;

    if (length == NULL) {
        return RTI_FALSE;
    }

    epd._maxSizeSerializedSample =
    Header_cPlugin_get_serialized_sample_max_size(
        NULL, RTI_TRUE, RTICdrEncapsulation_getNativeCdrEncapsulationId(), 0);

    if (buffer == NULL) {
        *length = 
        Header_cPlugin_get_serialized_sample_size(
            (PRESTypePluginEndpointData)&epd,
            RTI_TRUE,
            RTICdrEncapsulation_getNativeCdrEncapsulationId(),
            0,
            sample);

        if (*length == 0) {
            return RTI_FALSE;
        }

        return RTI_TRUE;
    }    

    RTICdrStream_init(&stream);
    RTICdrStream_set(&stream, (char *)buffer, *length);

    result = Header_cPlugin_serialize(
        (PRESTypePluginEndpointData)&epd, sample, &stream, 
        RTI_TRUE, RTICdrEncapsulation_getNativeCdrEncapsulationId(), 
        RTI_TRUE, NULL);  

    *length = RTICdrStream_getCurrentPositionOffset(&stream);
    return result;     
}

RTIBool
Header_cPlugin_deserialize_from_cdr_buffer(
    Header_c *sample,
    const char * buffer,
    unsigned int length)
{
    struct RTICdrStream stream;

    RTICdrStream_init(&stream);
    RTICdrStream_set(&stream, (char *)buffer, length);

    Header_c_finalize_optional_members(sample, RTI_TRUE);
    return Header_cPlugin_deserialize_sample( 
        NULL, sample,
        &stream, RTI_TRUE, RTI_TRUE, 
        NULL);
}

DDS_ReturnCode_t
Header_cPlugin_data_to_string(
    const Header_c *sample,
    char *str,
    DDS_UnsignedLong *str_size, 
    const struct DDS_PrintFormatProperty *property)
{
    DDS_DynamicData *data = NULL;
    char *buffer = NULL;
    unsigned int length = 0;
    struct DDS_PrintFormat printFormat;
    DDS_ReturnCode_t retCode = DDS_RETCODE_ERROR;

    if (sample == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (str_size == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (property == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (!Header_cPlugin_serialize_to_cdr_buffer(
        NULL, 
        &length, 
        sample)) {
        return DDS_RETCODE_ERROR;
    }

    RTIOsapiHeap_allocateBuffer(&buffer, length, RTI_OSAPI_ALIGNMENT_DEFAULT);
    if (buffer == NULL) {
        return DDS_RETCODE_ERROR;
    }

    if (!Header_cPlugin_serialize_to_cdr_buffer(
        buffer, 
        &length, 
        sample)) {
        RTIOsapiHeap_freeBuffer(buffer);
        return DDS_RETCODE_ERROR;
    }

    data = DDS_DynamicData_new(
        Header_c_get_typecode(), 
        &DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
    if (data == NULL) {
        RTIOsapiHeap_freeBuffer(buffer);
        return DDS_RETCODE_ERROR;
    }

    retCode = DDS_DynamicData_from_cdr_buffer(data, buffer, length);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    retCode = DDS_PrintFormatProperty_to_print_format(
        property, 
        &printFormat);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    retCode = DDS_DynamicDataFormatter_to_string_w_format(
        data, 
        str,
        str_size, 
        &printFormat);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    RTIOsapiHeap_freeBuffer(buffer);
    DDS_DynamicData_delete(data);
    return DDS_RETCODE_OK;
}

RTIBool 
Header_cPlugin_deserialize(
    PRESTypePluginEndpointData endpoint_data,
    Header_c **sample,
    RTIBool * drop_sample,
    struct RTICdrStream *stream,   
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos)
{

    RTIBool result;
    const char *METHOD_NAME = "Header_cPlugin_deserialize";
    if (drop_sample) {} /* To avoid warnings */

    stream->_xTypesState.unassignable = RTI_FALSE;
    result= Header_cPlugin_deserialize_sample( 
        endpoint_data, (sample != NULL)?*sample:NULL,
        stream, deserialize_encapsulation, deserialize_sample, 
        endpoint_plugin_qos);
    if (result) {
        if (stream->_xTypesState.unassignable) {
            result = RTI_FALSE;
        }
    }
    if (!result && stream->_xTypesState.unassignable ) {

        RTICdrLog_exception(
            METHOD_NAME, 
            &RTI_CDR_LOG_UNASSIGNABLE_SAMPLE_OF_TYPE_s, 
            "Header_c");

    }

    return result;

}

RTIBool Header_cPlugin_skip(
    PRESTypePluginEndpointData endpoint_data,
    struct RTICdrStream *stream,   
    RTIBool skip_encapsulation,
    RTIBool skip_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    RTIBool done = RTI_FALSE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(skip_encapsulation) {
        if (!RTICdrStream_skipEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if (skip_sample) {

        if (!Time_cPlugin_skip(
            endpoint_data,
            stream, 
            RTI_FALSE, RTI_TRUE, 
            endpoint_plugin_qos)) {
            goto fin; 
        }
        if (!RTICdrStream_skipString (stream, (RTI_INT32_MAX-1)+1)) {
            goto fin; 
        }
    }

    done = RTI_TRUE;
  fin:
    if (done != RTI_TRUE && 
    RTICdrStream_getRemainder(stream) >=
    RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
        return RTI_FALSE;   
    }
    if(skip_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

unsigned int 
Header_cPlugin_get_serialized_sample_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    if (endpoint_data) {} /* To avoid warnings */
    if (include_encapsulation) {}
    if (encapsulation_id) {}
    if (current_alignment) {}

    if (overflow != NULL) {
        *overflow = RTI_TRUE;
    }

    return RTI_CDR_MAX_SERIALIZED_SIZE;

}

unsigned int 
Header_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{
    unsigned int size;
    RTIBool overflow = RTI_FALSE;

    size = Header_cPlugin_get_serialized_sample_max_size_ex(
        endpoint_data,&overflow,include_encapsulation,encapsulation_id,current_alignment);

    if (overflow) {
        size = RTI_CDR_MAX_SERIALIZED_SIZE;
    }

    return size;
}

unsigned int 
Header_cPlugin_get_serialized_sample_min_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */ 

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment +=Time_cPlugin_get_serialized_sample_min_size(
        endpoint_data,RTI_FALSE,encapsulation_id,current_alignment);
    current_alignment +=RTICdrType_getStringMaxSizeSerialized(
        current_alignment, 1);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return  current_alignment - initial_alignment;
}

/* Returns the size of the sample in its serialized form (in bytes).
* It can also be an estimation in excess of the real buffer needed 
* during a call to the serialize() function.
* The value reported does not have to include the space for the
* encapsulation flags.
*/
unsigned int
Header_cPlugin_get_serialized_sample_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment,
    const Header_c * sample) 
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;
    struct PRESTypePluginDefaultEndpointData epd;   

    if (sample==NULL) {
        return 0;
    }
    if (endpoint_data == NULL) {
        endpoint_data = (PRESTypePluginEndpointData) &epd;
        PRESTypePluginDefaultEndpointData_setBaseAlignment(
            endpoint_data,
            current_alignment);        
    }

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
        PRESTypePluginDefaultEndpointData_setBaseAlignment(
            endpoint_data,
            current_alignment);
    }

    current_alignment += Time_cPlugin_get_serialized_sample_size(
        endpoint_data,RTI_FALSE, encapsulation_id,
        current_alignment, (const Time_c*) &sample->stamp);

    current_alignment += RTICdrType_getStringSerializedSize(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment), sample->frame_id);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return current_alignment - initial_alignment;
}

/* --------------------------------------------------------------------------------------
Key Management functions:
* -------------------------------------------------------------------------------------- */

PRESTypePluginKeyKind 
Header_cPlugin_get_key_kind(void)
{
    return PRES_TYPEPLUGIN_NO_KEY;
}

RTIBool 
Header_cPlugin_serialize_key(
    PRESTypePluginEndpointData endpoint_data,
    const Header_c *sample, 
    struct RTICdrStream *stream,    
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_key,
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    if(serialize_encapsulation) {
        if (!RTICdrStream_serializeAndSetCdrEncapsulation(stream , encapsulation_id)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if(serialize_key) {

        if (!Header_cPlugin_serialize(
            endpoint_data,
            sample,
            stream,
            RTI_FALSE, encapsulation_id,
            RTI_TRUE,
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

    }

    if(serialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

RTIBool Header_cPlugin_deserialize_key_sample(
    PRESTypePluginEndpointData endpoint_data,
    Header_c *sample, 
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos)
{

    char * position = NULL;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(deserialize_encapsulation) {

        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }
    if (deserialize_key) {

        if (!Header_cPlugin_deserialize_sample(
            endpoint_data, sample, stream, 
            RTI_FALSE, RTI_TRUE, 
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }
    }

    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;

}

RTIBool Header_cPlugin_deserialize_key(
    PRESTypePluginEndpointData endpoint_data,
    Header_c **sample, 
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos)
{
    RTIBool result;
    if (drop_sample) {} /* To avoid warnings */
    stream->_xTypesState.unassignable = RTI_FALSE;
    result= Header_cPlugin_deserialize_key_sample(
        endpoint_data, (sample != NULL)?*sample:NULL, stream,
        deserialize_encapsulation, deserialize_key, endpoint_plugin_qos);
    if (result) {
        if (stream->_xTypesState.unassignable) {
            result = RTI_FALSE;
        }
    }

    return result;    

}

unsigned int
Header_cPlugin_get_serialized_key_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */
    if (overflow) {} /* To avoid warnings */

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment += Header_cPlugin_get_serialized_sample_max_size_ex(
        endpoint_data, overflow,RTI_FALSE, encapsulation_id, current_alignment);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return current_alignment - initial_alignment;
}

unsigned int
Header_cPlugin_get_serialized_key_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{
    unsigned int size;
    RTIBool overflow = RTI_FALSE;

    size = Header_cPlugin_get_serialized_key_max_size_ex(
        endpoint_data,&overflow,include_encapsulation,encapsulation_id,current_alignment);

    if (overflow) {
        size = RTI_CDR_MAX_SERIALIZED_SIZE;
    }

    return size;
}

RTIBool 
Header_cPlugin_serialized_sample_to_key(
    PRESTypePluginEndpointData endpoint_data,
    Header_c *sample,
    struct RTICdrStream *stream, 
    RTIBool deserialize_encapsulation,  
    RTIBool deserialize_key, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    RTIBool done = RTI_FALSE;
    RTIBool error = RTI_FALSE;

    if (stream == NULL) {
        error = RTI_TRUE;
        goto fin;
    }
    if(deserialize_encapsulation) {
        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }
        position = RTICdrStream_resetAlignment(stream);
    }

    if (deserialize_key) {

        if (!Header_cPlugin_deserialize_sample(
            endpoint_data, sample, stream, RTI_FALSE, 
            RTI_TRUE, endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

    }

    done = RTI_TRUE;
  fin:
    if(!error) {
        if (done != RTI_TRUE && 
        RTICdrStream_getRemainder(stream) >=
        RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
            return RTI_FALSE;   
        }
    } else {
        return RTI_FALSE;
    }       

    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

/* ------------------------------------------------------------------------
* Plug-in Installation Methods
* ------------------------------------------------------------------------ */
struct PRESTypePlugin *Header_cPlugin_new(void) 
{ 
    struct PRESTypePlugin *plugin = NULL;
    const struct PRESTypePluginVersion PLUGIN_VERSION = 
    PRES_TYPE_PLUGIN_VERSION_2_0;

    RTIOsapiHeap_allocateStructure(
        &plugin, struct PRESTypePlugin);

    if (plugin == NULL) {
        return NULL;
    }

    plugin->version = PLUGIN_VERSION;

    /* set up parent's function pointers */
    plugin->onParticipantAttached =
    (PRESTypePluginOnParticipantAttachedCallback)
    Header_cPlugin_on_participant_attached;
    plugin->onParticipantDetached =
    (PRESTypePluginOnParticipantDetachedCallback)
    Header_cPlugin_on_participant_detached;
    plugin->onEndpointAttached =
    (PRESTypePluginOnEndpointAttachedCallback)
    Header_cPlugin_on_endpoint_attached;
    plugin->onEndpointDetached =
    (PRESTypePluginOnEndpointDetachedCallback)
    Header_cPlugin_on_endpoint_detached;

    plugin->copySampleFnc =
    (PRESTypePluginCopySampleFunction)
    Header_cPlugin_copy_sample;
    plugin->createSampleFnc =
    (PRESTypePluginCreateSampleFunction)
    Header_cPlugin_create_sample;
    plugin->destroySampleFnc =
    (PRESTypePluginDestroySampleFunction)
    Header_cPlugin_destroy_sample;
    plugin->finalizeOptionalMembersFnc =
    (PRESTypePluginFinalizeOptionalMembersFunction)
    Header_c_finalize_optional_members;

    plugin->serializeFnc =
    (PRESTypePluginSerializeFunction)
    Header_cPlugin_serialize;
    plugin->deserializeFnc =
    (PRESTypePluginDeserializeFunction)
    Header_cPlugin_deserialize;
    plugin->getSerializedSampleMaxSizeFnc =
    (PRESTypePluginGetSerializedSampleMaxSizeFunction)
    Header_cPlugin_get_serialized_sample_max_size;
    plugin->getSerializedSampleMinSizeFnc =
    (PRESTypePluginGetSerializedSampleMinSizeFunction)
    Header_cPlugin_get_serialized_sample_min_size;

    plugin->getSampleFnc =
    (PRESTypePluginGetSampleFunction)
    Header_cPlugin_get_sample;
    plugin->returnSampleFnc =
    (PRESTypePluginReturnSampleFunction)
    Header_cPlugin_return_sample;

    plugin->getKeyKindFnc =
    (PRESTypePluginGetKeyKindFunction)
    Header_cPlugin_get_key_kind;

    /* These functions are only used for keyed types. As this is not a keyed
    type they are all set to NULL
    */
    plugin->serializeKeyFnc = NULL ;    
    plugin->deserializeKeyFnc = NULL;  
    plugin->getKeyFnc = NULL;
    plugin->returnKeyFnc = NULL;
    plugin->instanceToKeyFnc = NULL;
    plugin->keyToInstanceFnc = NULL;
    plugin->getSerializedKeyMaxSizeFnc = NULL;
    plugin->instanceToKeyHashFnc = NULL;
    plugin->serializedSampleToKeyHashFnc = NULL;
    plugin->serializedKeyToKeyHashFnc = NULL;    
    plugin->typeCode =  (struct RTICdrTypeCode *)Header_c_get_typecode();

    plugin->languageKind = PRES_TYPEPLUGIN_CPP_LANG;

    /* Serialized buffer */
    plugin->getBuffer = 
    (PRESTypePluginGetBufferFunction)
    Header_cPlugin_get_buffer;
    plugin->returnBuffer = 
    (PRESTypePluginReturnBufferFunction)
    Header_cPlugin_return_buffer;
    plugin->getSerializedSampleSizeFnc =
    (PRESTypePluginGetSerializedSampleSizeFunction)
    Header_cPlugin_get_serialized_sample_size;

    plugin->endpointTypeName = Header_cTYPENAME;

    return plugin;
}

void
Header_cPlugin_delete(struct PRESTypePlugin *plugin)
{
    RTIOsapiHeap_freeStructure(plugin);
} 

/* ----------------------------------------------------------------------------
*  Type Position_c
* -------------------------------------------------------------------------- */

/* -----------------------------------------------------------------------------
Support functions:
* -------------------------------------------------------------------------- */

Position_c*
Position_cPluginSupport_create_data_w_params(
    const struct DDS_TypeAllocationParams_t * alloc_params) 
{
    Position_c *sample = NULL;

    RTIOsapiHeap_allocateStructure(&(sample),Position_c);
    if (sample == NULL) {
        return NULL;
    }

    if (!Position_c_initialize_w_params(sample,alloc_params)) {
        RTIOsapiHeap_freeStructure(sample);
        sample=NULL;
    }
    return sample; 
} 

Position_c *
Position_cPluginSupport_create_data_ex(RTIBool allocate_pointers) 
{
    Position_c *sample = NULL;

    RTIOsapiHeap_allocateStructure(&(sample),Position_c);

    if(sample == NULL) {
        return NULL;
    }

    if (!Position_c_initialize_ex(sample,allocate_pointers, RTI_TRUE)) {
        RTIOsapiHeap_freeStructure(sample);
        sample=NULL;
    }

    return sample; 
}

Position_c *
Position_cPluginSupport_create_data(void)
{
    return Position_cPluginSupport_create_data_ex(RTI_TRUE);
}

void 
Position_cPluginSupport_destroy_data_w_params(
    Position_c *sample,
    const struct DDS_TypeDeallocationParams_t * dealloc_params) {

    Position_c_finalize_w_params(sample,dealloc_params);

    RTIOsapiHeap_freeStructure(sample);
    sample=NULL;
}

void 
Position_cPluginSupport_destroy_data_ex(
    Position_c *sample,RTIBool deallocate_pointers) {

    Position_c_finalize_ex(sample,deallocate_pointers);

    RTIOsapiHeap_freeStructure(sample);
    sample=NULL;
}

void 
Position_cPluginSupport_destroy_data(
    Position_c *sample) {

    Position_cPluginSupport_destroy_data_ex(sample,RTI_TRUE);

}

RTIBool 
Position_cPluginSupport_copy_data(
    Position_c *dst,
    const Position_c *src)
{
    return Position_c_copy(dst,(const Position_c*) src);
}

void 
Position_cPluginSupport_print_data(
    const Position_c *sample,
    const char *desc,
    unsigned int indent_level)
{

    RTICdrType_printIndent(indent_level);

    if (desc != NULL) {
        RTILog_debug("%s:\n", desc);
    } else {
        RTILog_debug("\n");
    }

    if (sample == NULL) {
        RTILog_debug("NULL\n");
        return;
    }

    RTICdrType_printFloat(
        &sample->x, "x", indent_level + 1);    

    RTICdrType_printFloat(
        &sample->y, "y", indent_level + 1);    

    RTICdrType_printFloat(
        &sample->z, "z", indent_level + 1);    

}

/* ----------------------------------------------------------------------------
Callback functions:
* ---------------------------------------------------------------------------- */

PRESTypePluginParticipantData 
Position_cPlugin_on_participant_attached(
    void *registration_data,
    const struct PRESTypePluginParticipantInfo *participant_info,
    RTIBool top_level_registration,
    void *container_plugin_context,
    RTICdrTypeCode *type_code)
{
    if (registration_data) {} /* To avoid warnings */
    if (participant_info) {} /* To avoid warnings */
    if (top_level_registration) {} /* To avoid warnings */
    if (container_plugin_context) {} /* To avoid warnings */
    if (type_code) {} /* To avoid warnings */

    return PRESTypePluginDefaultParticipantData_new(participant_info);

}

void 
Position_cPlugin_on_participant_detached(
    PRESTypePluginParticipantData participant_data)
{

    PRESTypePluginDefaultParticipantData_delete(participant_data);
}

PRESTypePluginEndpointData
Position_cPlugin_on_endpoint_attached(
    PRESTypePluginParticipantData participant_data,
    const struct PRESTypePluginEndpointInfo *endpoint_info,
    RTIBool top_level_registration, 
    void *containerPluginContext)
{
    PRESTypePluginEndpointData epd = NULL;

    unsigned int serializedSampleMaxSize;

    if (top_level_registration) {} /* To avoid warnings */
    if (containerPluginContext) {} /* To avoid warnings */

    epd = PRESTypePluginDefaultEndpointData_new(
        participant_data,
        endpoint_info,
        (PRESTypePluginDefaultEndpointDataCreateSampleFunction)
        Position_cPluginSupport_create_data,
        (PRESTypePluginDefaultEndpointDataDestroySampleFunction)
        Position_cPluginSupport_destroy_data,
        NULL , NULL );

    if (epd == NULL) {
        return NULL;
    } 

    if (endpoint_info->endpointKind == PRES_TYPEPLUGIN_ENDPOINT_WRITER) {
        serializedSampleMaxSize = Position_cPlugin_get_serialized_sample_max_size(
            epd,RTI_FALSE,RTI_CDR_ENCAPSULATION_ID_CDR_BE,0);

        PRESTypePluginDefaultEndpointData_setMaxSizeSerializedSample(epd, serializedSampleMaxSize);

        if (PRESTypePluginDefaultEndpointData_createWriterPool(
            epd,
            endpoint_info,
            (PRESTypePluginGetSerializedSampleMaxSizeFunction)
            Position_cPlugin_get_serialized_sample_max_size, epd,
            (PRESTypePluginGetSerializedSampleSizeFunction)
            Position_cPlugin_get_serialized_sample_size,
            epd) == RTI_FALSE) {
            PRESTypePluginDefaultEndpointData_delete(epd);
            return NULL;
        }
    }

    return epd;    
}

void 
Position_cPlugin_on_endpoint_detached(
    PRESTypePluginEndpointData endpoint_data)
{  

    PRESTypePluginDefaultEndpointData_delete(endpoint_data);
}

void    
Position_cPlugin_return_sample(
    PRESTypePluginEndpointData endpoint_data,
    Position_c *sample,
    void *handle)
{

    Position_c_finalize_optional_members(sample, RTI_TRUE);

    PRESTypePluginDefaultEndpointData_returnSample(
        endpoint_data, sample, handle);
}

RTIBool 
Position_cPlugin_copy_sample(
    PRESTypePluginEndpointData endpoint_data,
    Position_c *dst,
    const Position_c *src)
{
    if (endpoint_data) {} /* To avoid warnings */
    return Position_cPluginSupport_copy_data(dst,src);
}

/* ----------------------------------------------------------------------------
(De)Serialize functions:
* ------------------------------------------------------------------------- */
unsigned int 
Position_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

RTIBool 
Position_cPlugin_serialize(
    PRESTypePluginEndpointData endpoint_data,
    const Position_c *sample, 
    struct RTICdrStream *stream,    
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;
    RTIBool retval = RTI_TRUE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(serialize_encapsulation) {
        if (!RTICdrStream_serializeAndSetCdrEncapsulation(stream , encapsulation_id)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if(serialize_sample) {

        if (!RTICdrStream_serializeFloat(
            stream, &sample->x)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeFloat(
            stream, &sample->y)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeFloat(
            stream, &sample->z)) {
            return RTI_FALSE;
        }

    }

    if(serialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return retval;
}

RTIBool 
Position_cPlugin_deserialize_sample(
    PRESTypePluginEndpointData endpoint_data,
    Position_c *sample,
    struct RTICdrStream *stream,   
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos)
{

    char * position = NULL;

    RTIBool done = RTI_FALSE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */
    if(deserialize_encapsulation) {

        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }
    if(deserialize_sample) {

        Position_c_initialize_ex(sample, RTI_FALSE, RTI_FALSE);

        if (!RTICdrStream_deserializeFloat(
            stream, &sample->x)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeFloat(
            stream, &sample->y)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeFloat(
            stream, &sample->z)) {
            goto fin; 
        }
    }

    done = RTI_TRUE;
  fin:
    if (done != RTI_TRUE && 
    RTICdrStream_getRemainder(stream) >=
    RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
        return RTI_FALSE;   
    }
    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;

}

RTIBool
Position_cPlugin_serialize_to_cdr_buffer(
    char * buffer,
    unsigned int * length,
    const Position_c *sample)
{
    struct RTICdrStream stream;
    struct PRESTypePluginDefaultEndpointData epd;
    RTIBool result;

    if (length == NULL) {
        return RTI_FALSE;
    }

    epd._maxSizeSerializedSample =
    Position_cPlugin_get_serialized_sample_max_size(
        NULL, RTI_TRUE, RTICdrEncapsulation_getNativeCdrEncapsulationId(), 0);

    if (buffer == NULL) {
        *length = 
        Position_cPlugin_get_serialized_sample_size(
            (PRESTypePluginEndpointData)&epd,
            RTI_TRUE,
            RTICdrEncapsulation_getNativeCdrEncapsulationId(),
            0,
            sample);

        if (*length == 0) {
            return RTI_FALSE;
        }

        return RTI_TRUE;
    }    

    RTICdrStream_init(&stream);
    RTICdrStream_set(&stream, (char *)buffer, *length);

    result = Position_cPlugin_serialize(
        (PRESTypePluginEndpointData)&epd, sample, &stream, 
        RTI_TRUE, RTICdrEncapsulation_getNativeCdrEncapsulationId(), 
        RTI_TRUE, NULL);  

    *length = RTICdrStream_getCurrentPositionOffset(&stream);
    return result;     
}

RTIBool
Position_cPlugin_deserialize_from_cdr_buffer(
    Position_c *sample,
    const char * buffer,
    unsigned int length)
{
    struct RTICdrStream stream;

    RTICdrStream_init(&stream);
    RTICdrStream_set(&stream, (char *)buffer, length);

    Position_c_finalize_optional_members(sample, RTI_TRUE);
    return Position_cPlugin_deserialize_sample( 
        NULL, sample,
        &stream, RTI_TRUE, RTI_TRUE, 
        NULL);
}

DDS_ReturnCode_t
Position_cPlugin_data_to_string(
    const Position_c *sample,
    char *str,
    DDS_UnsignedLong *str_size, 
    const struct DDS_PrintFormatProperty *property)
{
    DDS_DynamicData *data = NULL;
    char *buffer = NULL;
    unsigned int length = 0;
    struct DDS_PrintFormat printFormat;
    DDS_ReturnCode_t retCode = DDS_RETCODE_ERROR;

    if (sample == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (str_size == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (property == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (!Position_cPlugin_serialize_to_cdr_buffer(
        NULL, 
        &length, 
        sample)) {
        return DDS_RETCODE_ERROR;
    }

    RTIOsapiHeap_allocateBuffer(&buffer, length, RTI_OSAPI_ALIGNMENT_DEFAULT);
    if (buffer == NULL) {
        return DDS_RETCODE_ERROR;
    }

    if (!Position_cPlugin_serialize_to_cdr_buffer(
        buffer, 
        &length, 
        sample)) {
        RTIOsapiHeap_freeBuffer(buffer);
        return DDS_RETCODE_ERROR;
    }

    data = DDS_DynamicData_new(
        Position_c_get_typecode(), 
        &DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
    if (data == NULL) {
        RTIOsapiHeap_freeBuffer(buffer);
        return DDS_RETCODE_ERROR;
    }

    retCode = DDS_DynamicData_from_cdr_buffer(data, buffer, length);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    retCode = DDS_PrintFormatProperty_to_print_format(
        property, 
        &printFormat);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    retCode = DDS_DynamicDataFormatter_to_string_w_format(
        data, 
        str,
        str_size, 
        &printFormat);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    RTIOsapiHeap_freeBuffer(buffer);
    DDS_DynamicData_delete(data);
    return DDS_RETCODE_OK;
}

RTIBool 
Position_cPlugin_deserialize(
    PRESTypePluginEndpointData endpoint_data,
    Position_c **sample,
    RTIBool * drop_sample,
    struct RTICdrStream *stream,   
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos)
{

    RTIBool result;
    const char *METHOD_NAME = "Position_cPlugin_deserialize";
    if (drop_sample) {} /* To avoid warnings */

    stream->_xTypesState.unassignable = RTI_FALSE;
    result= Position_cPlugin_deserialize_sample( 
        endpoint_data, (sample != NULL)?*sample:NULL,
        stream, deserialize_encapsulation, deserialize_sample, 
        endpoint_plugin_qos);
    if (result) {
        if (stream->_xTypesState.unassignable) {
            result = RTI_FALSE;
        }
    }
    if (!result && stream->_xTypesState.unassignable ) {

        RTICdrLog_exception(
            METHOD_NAME, 
            &RTI_CDR_LOG_UNASSIGNABLE_SAMPLE_OF_TYPE_s, 
            "Position_c");

    }

    return result;

}

RTIBool Position_cPlugin_skip(
    PRESTypePluginEndpointData endpoint_data,
    struct RTICdrStream *stream,   
    RTIBool skip_encapsulation,
    RTIBool skip_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    RTIBool done = RTI_FALSE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(skip_encapsulation) {
        if (!RTICdrStream_skipEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if (skip_sample) {

        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
    }

    done = RTI_TRUE;
  fin:
    if (done != RTI_TRUE && 
    RTICdrStream_getRemainder(stream) >=
    RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
        return RTI_FALSE;   
    }
    if(skip_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

unsigned int 
Position_cPlugin_get_serialized_sample_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */ 
    if (overflow) {} /* To avoid warnings */

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);

    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);

    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return  current_alignment - initial_alignment;
}

unsigned int 
Position_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{
    unsigned int size;
    RTIBool overflow = RTI_FALSE;

    size = Position_cPlugin_get_serialized_sample_max_size_ex(
        endpoint_data,&overflow,include_encapsulation,encapsulation_id,current_alignment);

    if (overflow) {
        size = RTI_CDR_MAX_SERIALIZED_SIZE;
    }

    return size;
}

unsigned int 
Position_cPlugin_get_serialized_sample_min_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */ 

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return  current_alignment - initial_alignment;
}

/* Returns the size of the sample in its serialized form (in bytes).
* It can also be an estimation in excess of the real buffer needed 
* during a call to the serialize() function.
* The value reported does not have to include the space for the
* encapsulation flags.
*/
unsigned int
Position_cPlugin_get_serialized_sample_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment,
    const Position_c * sample) 
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;
    struct PRESTypePluginDefaultEndpointData epd;   

    if (sample==NULL) {
        return 0;
    }
    if (endpoint_data == NULL) {
        endpoint_data = (PRESTypePluginEndpointData) &epd;
        PRESTypePluginDefaultEndpointData_setBaseAlignment(
            endpoint_data,
            current_alignment);        
    }

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
        PRESTypePluginDefaultEndpointData_setBaseAlignment(
            endpoint_data,
            current_alignment);
    }

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return current_alignment - initial_alignment;
}

/* --------------------------------------------------------------------------------------
Key Management functions:
* -------------------------------------------------------------------------------------- */

PRESTypePluginKeyKind 
Position_cPlugin_get_key_kind(void)
{
    return PRES_TYPEPLUGIN_NO_KEY;
}

RTIBool 
Position_cPlugin_serialize_key(
    PRESTypePluginEndpointData endpoint_data,
    const Position_c *sample, 
    struct RTICdrStream *stream,    
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_key,
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    if(serialize_encapsulation) {
        if (!RTICdrStream_serializeAndSetCdrEncapsulation(stream , encapsulation_id)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if(serialize_key) {

        if (!Position_cPlugin_serialize(
            endpoint_data,
            sample,
            stream,
            RTI_FALSE, encapsulation_id,
            RTI_TRUE,
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

    }

    if(serialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

RTIBool Position_cPlugin_deserialize_key_sample(
    PRESTypePluginEndpointData endpoint_data,
    Position_c *sample, 
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos)
{

    char * position = NULL;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(deserialize_encapsulation) {

        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }
    if (deserialize_key) {

        if (!Position_cPlugin_deserialize_sample(
            endpoint_data, sample, stream, 
            RTI_FALSE, RTI_TRUE, 
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }
    }

    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;

}

RTIBool Position_cPlugin_deserialize_key(
    PRESTypePluginEndpointData endpoint_data,
    Position_c **sample, 
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos)
{
    RTIBool result;
    if (drop_sample) {} /* To avoid warnings */
    stream->_xTypesState.unassignable = RTI_FALSE;
    result= Position_cPlugin_deserialize_key_sample(
        endpoint_data, (sample != NULL)?*sample:NULL, stream,
        deserialize_encapsulation, deserialize_key, endpoint_plugin_qos);
    if (result) {
        if (stream->_xTypesState.unassignable) {
            result = RTI_FALSE;
        }
    }

    return result;    

}

unsigned int
Position_cPlugin_get_serialized_key_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */
    if (overflow) {} /* To avoid warnings */

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment += Position_cPlugin_get_serialized_sample_max_size_ex(
        endpoint_data, overflow,RTI_FALSE, encapsulation_id, current_alignment);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return current_alignment - initial_alignment;
}

unsigned int
Position_cPlugin_get_serialized_key_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{
    unsigned int size;
    RTIBool overflow = RTI_FALSE;

    size = Position_cPlugin_get_serialized_key_max_size_ex(
        endpoint_data,&overflow,include_encapsulation,encapsulation_id,current_alignment);

    if (overflow) {
        size = RTI_CDR_MAX_SERIALIZED_SIZE;
    }

    return size;
}

RTIBool 
Position_cPlugin_serialized_sample_to_key(
    PRESTypePluginEndpointData endpoint_data,
    Position_c *sample,
    struct RTICdrStream *stream, 
    RTIBool deserialize_encapsulation,  
    RTIBool deserialize_key, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    RTIBool done = RTI_FALSE;
    RTIBool error = RTI_FALSE;

    if (stream == NULL) {
        error = RTI_TRUE;
        goto fin;
    }
    if(deserialize_encapsulation) {
        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }
        position = RTICdrStream_resetAlignment(stream);
    }

    if (deserialize_key) {

        if (!Position_cPlugin_deserialize_sample(
            endpoint_data, sample, stream, RTI_FALSE, 
            RTI_TRUE, endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

    }

    done = RTI_TRUE;
  fin:
    if(!error) {
        if (done != RTI_TRUE && 
        RTICdrStream_getRemainder(stream) >=
        RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
            return RTI_FALSE;   
        }
    } else {
        return RTI_FALSE;
    }       

    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

/* ------------------------------------------------------------------------
* Plug-in Installation Methods
* ------------------------------------------------------------------------ */
struct PRESTypePlugin *Position_cPlugin_new(void) 
{ 
    struct PRESTypePlugin *plugin = NULL;
    const struct PRESTypePluginVersion PLUGIN_VERSION = 
    PRES_TYPE_PLUGIN_VERSION_2_0;

    RTIOsapiHeap_allocateStructure(
        &plugin, struct PRESTypePlugin);

    if (plugin == NULL) {
        return NULL;
    }

    plugin->version = PLUGIN_VERSION;

    /* set up parent's function pointers */
    plugin->onParticipantAttached =
    (PRESTypePluginOnParticipantAttachedCallback)
    Position_cPlugin_on_participant_attached;
    plugin->onParticipantDetached =
    (PRESTypePluginOnParticipantDetachedCallback)
    Position_cPlugin_on_participant_detached;
    plugin->onEndpointAttached =
    (PRESTypePluginOnEndpointAttachedCallback)
    Position_cPlugin_on_endpoint_attached;
    plugin->onEndpointDetached =
    (PRESTypePluginOnEndpointDetachedCallback)
    Position_cPlugin_on_endpoint_detached;

    plugin->copySampleFnc =
    (PRESTypePluginCopySampleFunction)
    Position_cPlugin_copy_sample;
    plugin->createSampleFnc =
    (PRESTypePluginCreateSampleFunction)
    Position_cPlugin_create_sample;
    plugin->destroySampleFnc =
    (PRESTypePluginDestroySampleFunction)
    Position_cPlugin_destroy_sample;
    plugin->finalizeOptionalMembersFnc =
    (PRESTypePluginFinalizeOptionalMembersFunction)
    Position_c_finalize_optional_members;

    plugin->serializeFnc =
    (PRESTypePluginSerializeFunction)
    Position_cPlugin_serialize;
    plugin->deserializeFnc =
    (PRESTypePluginDeserializeFunction)
    Position_cPlugin_deserialize;
    plugin->getSerializedSampleMaxSizeFnc =
    (PRESTypePluginGetSerializedSampleMaxSizeFunction)
    Position_cPlugin_get_serialized_sample_max_size;
    plugin->getSerializedSampleMinSizeFnc =
    (PRESTypePluginGetSerializedSampleMinSizeFunction)
    Position_cPlugin_get_serialized_sample_min_size;

    plugin->getSampleFnc =
    (PRESTypePluginGetSampleFunction)
    Position_cPlugin_get_sample;
    plugin->returnSampleFnc =
    (PRESTypePluginReturnSampleFunction)
    Position_cPlugin_return_sample;

    plugin->getKeyKindFnc =
    (PRESTypePluginGetKeyKindFunction)
    Position_cPlugin_get_key_kind;

    /* These functions are only used for keyed types. As this is not a keyed
    type they are all set to NULL
    */
    plugin->serializeKeyFnc = NULL ;    
    plugin->deserializeKeyFnc = NULL;  
    plugin->getKeyFnc = NULL;
    plugin->returnKeyFnc = NULL;
    plugin->instanceToKeyFnc = NULL;
    plugin->keyToInstanceFnc = NULL;
    plugin->getSerializedKeyMaxSizeFnc = NULL;
    plugin->instanceToKeyHashFnc = NULL;
    plugin->serializedSampleToKeyHashFnc = NULL;
    plugin->serializedKeyToKeyHashFnc = NULL;    
    plugin->typeCode =  (struct RTICdrTypeCode *)Position_c_get_typecode();

    plugin->languageKind = PRES_TYPEPLUGIN_CPP_LANG;

    /* Serialized buffer */
    plugin->getBuffer = 
    (PRESTypePluginGetBufferFunction)
    Position_cPlugin_get_buffer;
    plugin->returnBuffer = 
    (PRESTypePluginReturnBufferFunction)
    Position_cPlugin_return_buffer;
    plugin->getSerializedSampleSizeFnc =
    (PRESTypePluginGetSerializedSampleSizeFunction)
    Position_cPlugin_get_serialized_sample_size;

    plugin->endpointTypeName = Position_cTYPENAME;

    return plugin;
}

void
Position_cPlugin_delete(struct PRESTypePlugin *plugin)
{
    RTIOsapiHeap_freeStructure(plugin);
} 

/* ----------------------------------------------------------------------------
*  Type Orientation_c
* -------------------------------------------------------------------------- */

/* -----------------------------------------------------------------------------
Support functions:
* -------------------------------------------------------------------------- */

Orientation_c*
Orientation_cPluginSupport_create_data_w_params(
    const struct DDS_TypeAllocationParams_t * alloc_params) 
{
    Orientation_c *sample = NULL;

    RTIOsapiHeap_allocateStructure(&(sample),Orientation_c);
    if (sample == NULL) {
        return NULL;
    }

    if (!Orientation_c_initialize_w_params(sample,alloc_params)) {
        RTIOsapiHeap_freeStructure(sample);
        sample=NULL;
    }
    return sample; 
} 

Orientation_c *
Orientation_cPluginSupport_create_data_ex(RTIBool allocate_pointers) 
{
    Orientation_c *sample = NULL;

    RTIOsapiHeap_allocateStructure(&(sample),Orientation_c);

    if(sample == NULL) {
        return NULL;
    }

    if (!Orientation_c_initialize_ex(sample,allocate_pointers, RTI_TRUE)) {
        RTIOsapiHeap_freeStructure(sample);
        sample=NULL;
    }

    return sample; 
}

Orientation_c *
Orientation_cPluginSupport_create_data(void)
{
    return Orientation_cPluginSupport_create_data_ex(RTI_TRUE);
}

void 
Orientation_cPluginSupport_destroy_data_w_params(
    Orientation_c *sample,
    const struct DDS_TypeDeallocationParams_t * dealloc_params) {

    Orientation_c_finalize_w_params(sample,dealloc_params);

    RTIOsapiHeap_freeStructure(sample);
    sample=NULL;
}

void 
Orientation_cPluginSupport_destroy_data_ex(
    Orientation_c *sample,RTIBool deallocate_pointers) {

    Orientation_c_finalize_ex(sample,deallocate_pointers);

    RTIOsapiHeap_freeStructure(sample);
    sample=NULL;
}

void 
Orientation_cPluginSupport_destroy_data(
    Orientation_c *sample) {

    Orientation_cPluginSupport_destroy_data_ex(sample,RTI_TRUE);

}

RTIBool 
Orientation_cPluginSupport_copy_data(
    Orientation_c *dst,
    const Orientation_c *src)
{
    return Orientation_c_copy(dst,(const Orientation_c*) src);
}

void 
Orientation_cPluginSupport_print_data(
    const Orientation_c *sample,
    const char *desc,
    unsigned int indent_level)
{

    RTICdrType_printIndent(indent_level);

    if (desc != NULL) {
        RTILog_debug("%s:\n", desc);
    } else {
        RTILog_debug("\n");
    }

    if (sample == NULL) {
        RTILog_debug("NULL\n");
        return;
    }

    RTICdrType_printFloat(
        &sample->x, "x", indent_level + 1);    

    RTICdrType_printFloat(
        &sample->y, "y", indent_level + 1);    

    RTICdrType_printFloat(
        &sample->z, "z", indent_level + 1);    

    RTICdrType_printFloat(
        &sample->w, "w", indent_level + 1);    

}

/* ----------------------------------------------------------------------------
Callback functions:
* ---------------------------------------------------------------------------- */

PRESTypePluginParticipantData 
Orientation_cPlugin_on_participant_attached(
    void *registration_data,
    const struct PRESTypePluginParticipantInfo *participant_info,
    RTIBool top_level_registration,
    void *container_plugin_context,
    RTICdrTypeCode *type_code)
{
    if (registration_data) {} /* To avoid warnings */
    if (participant_info) {} /* To avoid warnings */
    if (top_level_registration) {} /* To avoid warnings */
    if (container_plugin_context) {} /* To avoid warnings */
    if (type_code) {} /* To avoid warnings */

    return PRESTypePluginDefaultParticipantData_new(participant_info);

}

void 
Orientation_cPlugin_on_participant_detached(
    PRESTypePluginParticipantData participant_data)
{

    PRESTypePluginDefaultParticipantData_delete(participant_data);
}

PRESTypePluginEndpointData
Orientation_cPlugin_on_endpoint_attached(
    PRESTypePluginParticipantData participant_data,
    const struct PRESTypePluginEndpointInfo *endpoint_info,
    RTIBool top_level_registration, 
    void *containerPluginContext)
{
    PRESTypePluginEndpointData epd = NULL;

    unsigned int serializedSampleMaxSize;

    if (top_level_registration) {} /* To avoid warnings */
    if (containerPluginContext) {} /* To avoid warnings */

    epd = PRESTypePluginDefaultEndpointData_new(
        participant_data,
        endpoint_info,
        (PRESTypePluginDefaultEndpointDataCreateSampleFunction)
        Orientation_cPluginSupport_create_data,
        (PRESTypePluginDefaultEndpointDataDestroySampleFunction)
        Orientation_cPluginSupport_destroy_data,
        NULL , NULL );

    if (epd == NULL) {
        return NULL;
    } 

    if (endpoint_info->endpointKind == PRES_TYPEPLUGIN_ENDPOINT_WRITER) {
        serializedSampleMaxSize = Orientation_cPlugin_get_serialized_sample_max_size(
            epd,RTI_FALSE,RTI_CDR_ENCAPSULATION_ID_CDR_BE,0);

        PRESTypePluginDefaultEndpointData_setMaxSizeSerializedSample(epd, serializedSampleMaxSize);

        if (PRESTypePluginDefaultEndpointData_createWriterPool(
            epd,
            endpoint_info,
            (PRESTypePluginGetSerializedSampleMaxSizeFunction)
            Orientation_cPlugin_get_serialized_sample_max_size, epd,
            (PRESTypePluginGetSerializedSampleSizeFunction)
            Orientation_cPlugin_get_serialized_sample_size,
            epd) == RTI_FALSE) {
            PRESTypePluginDefaultEndpointData_delete(epd);
            return NULL;
        }
    }

    return epd;    
}

void 
Orientation_cPlugin_on_endpoint_detached(
    PRESTypePluginEndpointData endpoint_data)
{  

    PRESTypePluginDefaultEndpointData_delete(endpoint_data);
}

void    
Orientation_cPlugin_return_sample(
    PRESTypePluginEndpointData endpoint_data,
    Orientation_c *sample,
    void *handle)
{

    Orientation_c_finalize_optional_members(sample, RTI_TRUE);

    PRESTypePluginDefaultEndpointData_returnSample(
        endpoint_data, sample, handle);
}

RTIBool 
Orientation_cPlugin_copy_sample(
    PRESTypePluginEndpointData endpoint_data,
    Orientation_c *dst,
    const Orientation_c *src)
{
    if (endpoint_data) {} /* To avoid warnings */
    return Orientation_cPluginSupport_copy_data(dst,src);
}

/* ----------------------------------------------------------------------------
(De)Serialize functions:
* ------------------------------------------------------------------------- */
unsigned int 
Orientation_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

RTIBool 
Orientation_cPlugin_serialize(
    PRESTypePluginEndpointData endpoint_data,
    const Orientation_c *sample, 
    struct RTICdrStream *stream,    
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;
    RTIBool retval = RTI_TRUE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(serialize_encapsulation) {
        if (!RTICdrStream_serializeAndSetCdrEncapsulation(stream , encapsulation_id)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if(serialize_sample) {

        if (!RTICdrStream_serializeFloat(
            stream, &sample->x)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeFloat(
            stream, &sample->y)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeFloat(
            stream, &sample->z)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeFloat(
            stream, &sample->w)) {
            return RTI_FALSE;
        }

    }

    if(serialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return retval;
}

RTIBool 
Orientation_cPlugin_deserialize_sample(
    PRESTypePluginEndpointData endpoint_data,
    Orientation_c *sample,
    struct RTICdrStream *stream,   
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos)
{

    char * position = NULL;

    RTIBool done = RTI_FALSE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */
    if(deserialize_encapsulation) {

        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }
    if(deserialize_sample) {

        Orientation_c_initialize_ex(sample, RTI_FALSE, RTI_FALSE);

        if (!RTICdrStream_deserializeFloat(
            stream, &sample->x)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeFloat(
            stream, &sample->y)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeFloat(
            stream, &sample->z)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeFloat(
            stream, &sample->w)) {
            goto fin; 
        }
    }

    done = RTI_TRUE;
  fin:
    if (done != RTI_TRUE && 
    RTICdrStream_getRemainder(stream) >=
    RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
        return RTI_FALSE;   
    }
    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;

}

RTIBool
Orientation_cPlugin_serialize_to_cdr_buffer(
    char * buffer,
    unsigned int * length,
    const Orientation_c *sample)
{
    struct RTICdrStream stream;
    struct PRESTypePluginDefaultEndpointData epd;
    RTIBool result;

    if (length == NULL) {
        return RTI_FALSE;
    }

    epd._maxSizeSerializedSample =
    Orientation_cPlugin_get_serialized_sample_max_size(
        NULL, RTI_TRUE, RTICdrEncapsulation_getNativeCdrEncapsulationId(), 0);

    if (buffer == NULL) {
        *length = 
        Orientation_cPlugin_get_serialized_sample_size(
            (PRESTypePluginEndpointData)&epd,
            RTI_TRUE,
            RTICdrEncapsulation_getNativeCdrEncapsulationId(),
            0,
            sample);

        if (*length == 0) {
            return RTI_FALSE;
        }

        return RTI_TRUE;
    }    

    RTICdrStream_init(&stream);
    RTICdrStream_set(&stream, (char *)buffer, *length);

    result = Orientation_cPlugin_serialize(
        (PRESTypePluginEndpointData)&epd, sample, &stream, 
        RTI_TRUE, RTICdrEncapsulation_getNativeCdrEncapsulationId(), 
        RTI_TRUE, NULL);  

    *length = RTICdrStream_getCurrentPositionOffset(&stream);
    return result;     
}

RTIBool
Orientation_cPlugin_deserialize_from_cdr_buffer(
    Orientation_c *sample,
    const char * buffer,
    unsigned int length)
{
    struct RTICdrStream stream;

    RTICdrStream_init(&stream);
    RTICdrStream_set(&stream, (char *)buffer, length);

    Orientation_c_finalize_optional_members(sample, RTI_TRUE);
    return Orientation_cPlugin_deserialize_sample( 
        NULL, sample,
        &stream, RTI_TRUE, RTI_TRUE, 
        NULL);
}

DDS_ReturnCode_t
Orientation_cPlugin_data_to_string(
    const Orientation_c *sample,
    char *str,
    DDS_UnsignedLong *str_size, 
    const struct DDS_PrintFormatProperty *property)
{
    DDS_DynamicData *data = NULL;
    char *buffer = NULL;
    unsigned int length = 0;
    struct DDS_PrintFormat printFormat;
    DDS_ReturnCode_t retCode = DDS_RETCODE_ERROR;

    if (sample == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (str_size == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (property == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (!Orientation_cPlugin_serialize_to_cdr_buffer(
        NULL, 
        &length, 
        sample)) {
        return DDS_RETCODE_ERROR;
    }

    RTIOsapiHeap_allocateBuffer(&buffer, length, RTI_OSAPI_ALIGNMENT_DEFAULT);
    if (buffer == NULL) {
        return DDS_RETCODE_ERROR;
    }

    if (!Orientation_cPlugin_serialize_to_cdr_buffer(
        buffer, 
        &length, 
        sample)) {
        RTIOsapiHeap_freeBuffer(buffer);
        return DDS_RETCODE_ERROR;
    }

    data = DDS_DynamicData_new(
        Orientation_c_get_typecode(), 
        &DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
    if (data == NULL) {
        RTIOsapiHeap_freeBuffer(buffer);
        return DDS_RETCODE_ERROR;
    }

    retCode = DDS_DynamicData_from_cdr_buffer(data, buffer, length);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    retCode = DDS_PrintFormatProperty_to_print_format(
        property, 
        &printFormat);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    retCode = DDS_DynamicDataFormatter_to_string_w_format(
        data, 
        str,
        str_size, 
        &printFormat);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    RTIOsapiHeap_freeBuffer(buffer);
    DDS_DynamicData_delete(data);
    return DDS_RETCODE_OK;
}

RTIBool 
Orientation_cPlugin_deserialize(
    PRESTypePluginEndpointData endpoint_data,
    Orientation_c **sample,
    RTIBool * drop_sample,
    struct RTICdrStream *stream,   
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos)
{

    RTIBool result;
    const char *METHOD_NAME = "Orientation_cPlugin_deserialize";
    if (drop_sample) {} /* To avoid warnings */

    stream->_xTypesState.unassignable = RTI_FALSE;
    result= Orientation_cPlugin_deserialize_sample( 
        endpoint_data, (sample != NULL)?*sample:NULL,
        stream, deserialize_encapsulation, deserialize_sample, 
        endpoint_plugin_qos);
    if (result) {
        if (stream->_xTypesState.unassignable) {
            result = RTI_FALSE;
        }
    }
    if (!result && stream->_xTypesState.unassignable ) {

        RTICdrLog_exception(
            METHOD_NAME, 
            &RTI_CDR_LOG_UNASSIGNABLE_SAMPLE_OF_TYPE_s, 
            "Orientation_c");

    }

    return result;

}

RTIBool Orientation_cPlugin_skip(
    PRESTypePluginEndpointData endpoint_data,
    struct RTICdrStream *stream,   
    RTIBool skip_encapsulation,
    RTIBool skip_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    RTIBool done = RTI_FALSE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(skip_encapsulation) {
        if (!RTICdrStream_skipEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if (skip_sample) {

        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
    }

    done = RTI_TRUE;
  fin:
    if (done != RTI_TRUE && 
    RTICdrStream_getRemainder(stream) >=
    RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
        return RTI_FALSE;   
    }
    if(skip_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

unsigned int 
Orientation_cPlugin_get_serialized_sample_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */ 
    if (overflow) {} /* To avoid warnings */

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);

    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);

    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);

    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return  current_alignment - initial_alignment;
}

unsigned int 
Orientation_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{
    unsigned int size;
    RTIBool overflow = RTI_FALSE;

    size = Orientation_cPlugin_get_serialized_sample_max_size_ex(
        endpoint_data,&overflow,include_encapsulation,encapsulation_id,current_alignment);

    if (overflow) {
        size = RTI_CDR_MAX_SERIALIZED_SIZE;
    }

    return size;
}

unsigned int 
Orientation_cPlugin_get_serialized_sample_min_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */ 

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return  current_alignment - initial_alignment;
}

/* Returns the size of the sample in its serialized form (in bytes).
* It can also be an estimation in excess of the real buffer needed 
* during a call to the serialize() function.
* The value reported does not have to include the space for the
* encapsulation flags.
*/
unsigned int
Orientation_cPlugin_get_serialized_sample_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment,
    const Orientation_c * sample) 
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;
    struct PRESTypePluginDefaultEndpointData epd;   

    if (sample==NULL) {
        return 0;
    }
    if (endpoint_data == NULL) {
        endpoint_data = (PRESTypePluginEndpointData) &epd;
        PRESTypePluginDefaultEndpointData_setBaseAlignment(
            endpoint_data,
            current_alignment);        
    }

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
        PRESTypePluginDefaultEndpointData_setBaseAlignment(
            endpoint_data,
            current_alignment);
    }

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return current_alignment - initial_alignment;
}

/* --------------------------------------------------------------------------------------
Key Management functions:
* -------------------------------------------------------------------------------------- */

PRESTypePluginKeyKind 
Orientation_cPlugin_get_key_kind(void)
{
    return PRES_TYPEPLUGIN_NO_KEY;
}

RTIBool 
Orientation_cPlugin_serialize_key(
    PRESTypePluginEndpointData endpoint_data,
    const Orientation_c *sample, 
    struct RTICdrStream *stream,    
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_key,
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    if(serialize_encapsulation) {
        if (!RTICdrStream_serializeAndSetCdrEncapsulation(stream , encapsulation_id)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if(serialize_key) {

        if (!Orientation_cPlugin_serialize(
            endpoint_data,
            sample,
            stream,
            RTI_FALSE, encapsulation_id,
            RTI_TRUE,
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

    }

    if(serialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

RTIBool Orientation_cPlugin_deserialize_key_sample(
    PRESTypePluginEndpointData endpoint_data,
    Orientation_c *sample, 
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos)
{

    char * position = NULL;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(deserialize_encapsulation) {

        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }
    if (deserialize_key) {

        if (!Orientation_cPlugin_deserialize_sample(
            endpoint_data, sample, stream, 
            RTI_FALSE, RTI_TRUE, 
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }
    }

    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;

}

RTIBool Orientation_cPlugin_deserialize_key(
    PRESTypePluginEndpointData endpoint_data,
    Orientation_c **sample, 
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos)
{
    RTIBool result;
    if (drop_sample) {} /* To avoid warnings */
    stream->_xTypesState.unassignable = RTI_FALSE;
    result= Orientation_cPlugin_deserialize_key_sample(
        endpoint_data, (sample != NULL)?*sample:NULL, stream,
        deserialize_encapsulation, deserialize_key, endpoint_plugin_qos);
    if (result) {
        if (stream->_xTypesState.unassignable) {
            result = RTI_FALSE;
        }
    }

    return result;    

}

unsigned int
Orientation_cPlugin_get_serialized_key_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */
    if (overflow) {} /* To avoid warnings */

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment += Orientation_cPlugin_get_serialized_sample_max_size_ex(
        endpoint_data, overflow,RTI_FALSE, encapsulation_id, current_alignment);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return current_alignment - initial_alignment;
}

unsigned int
Orientation_cPlugin_get_serialized_key_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{
    unsigned int size;
    RTIBool overflow = RTI_FALSE;

    size = Orientation_cPlugin_get_serialized_key_max_size_ex(
        endpoint_data,&overflow,include_encapsulation,encapsulation_id,current_alignment);

    if (overflow) {
        size = RTI_CDR_MAX_SERIALIZED_SIZE;
    }

    return size;
}

RTIBool 
Orientation_cPlugin_serialized_sample_to_key(
    PRESTypePluginEndpointData endpoint_data,
    Orientation_c *sample,
    struct RTICdrStream *stream, 
    RTIBool deserialize_encapsulation,  
    RTIBool deserialize_key, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    RTIBool done = RTI_FALSE;
    RTIBool error = RTI_FALSE;

    if (stream == NULL) {
        error = RTI_TRUE;
        goto fin;
    }
    if(deserialize_encapsulation) {
        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }
        position = RTICdrStream_resetAlignment(stream);
    }

    if (deserialize_key) {

        if (!Orientation_cPlugin_deserialize_sample(
            endpoint_data, sample, stream, RTI_FALSE, 
            RTI_TRUE, endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

    }

    done = RTI_TRUE;
  fin:
    if(!error) {
        if (done != RTI_TRUE && 
        RTICdrStream_getRemainder(stream) >=
        RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
            return RTI_FALSE;   
        }
    } else {
        return RTI_FALSE;
    }       

    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

/* ------------------------------------------------------------------------
* Plug-in Installation Methods
* ------------------------------------------------------------------------ */
struct PRESTypePlugin *Orientation_cPlugin_new(void) 
{ 
    struct PRESTypePlugin *plugin = NULL;
    const struct PRESTypePluginVersion PLUGIN_VERSION = 
    PRES_TYPE_PLUGIN_VERSION_2_0;

    RTIOsapiHeap_allocateStructure(
        &plugin, struct PRESTypePlugin);

    if (plugin == NULL) {
        return NULL;
    }

    plugin->version = PLUGIN_VERSION;

    /* set up parent's function pointers */
    plugin->onParticipantAttached =
    (PRESTypePluginOnParticipantAttachedCallback)
    Orientation_cPlugin_on_participant_attached;
    plugin->onParticipantDetached =
    (PRESTypePluginOnParticipantDetachedCallback)
    Orientation_cPlugin_on_participant_detached;
    plugin->onEndpointAttached =
    (PRESTypePluginOnEndpointAttachedCallback)
    Orientation_cPlugin_on_endpoint_attached;
    plugin->onEndpointDetached =
    (PRESTypePluginOnEndpointDetachedCallback)
    Orientation_cPlugin_on_endpoint_detached;

    plugin->copySampleFnc =
    (PRESTypePluginCopySampleFunction)
    Orientation_cPlugin_copy_sample;
    plugin->createSampleFnc =
    (PRESTypePluginCreateSampleFunction)
    Orientation_cPlugin_create_sample;
    plugin->destroySampleFnc =
    (PRESTypePluginDestroySampleFunction)
    Orientation_cPlugin_destroy_sample;
    plugin->finalizeOptionalMembersFnc =
    (PRESTypePluginFinalizeOptionalMembersFunction)
    Orientation_c_finalize_optional_members;

    plugin->serializeFnc =
    (PRESTypePluginSerializeFunction)
    Orientation_cPlugin_serialize;
    plugin->deserializeFnc =
    (PRESTypePluginDeserializeFunction)
    Orientation_cPlugin_deserialize;
    plugin->getSerializedSampleMaxSizeFnc =
    (PRESTypePluginGetSerializedSampleMaxSizeFunction)
    Orientation_cPlugin_get_serialized_sample_max_size;
    plugin->getSerializedSampleMinSizeFnc =
    (PRESTypePluginGetSerializedSampleMinSizeFunction)
    Orientation_cPlugin_get_serialized_sample_min_size;

    plugin->getSampleFnc =
    (PRESTypePluginGetSampleFunction)
    Orientation_cPlugin_get_sample;
    plugin->returnSampleFnc =
    (PRESTypePluginReturnSampleFunction)
    Orientation_cPlugin_return_sample;

    plugin->getKeyKindFnc =
    (PRESTypePluginGetKeyKindFunction)
    Orientation_cPlugin_get_key_kind;

    /* These functions are only used for keyed types. As this is not a keyed
    type they are all set to NULL
    */
    plugin->serializeKeyFnc = NULL ;    
    plugin->deserializeKeyFnc = NULL;  
    plugin->getKeyFnc = NULL;
    plugin->returnKeyFnc = NULL;
    plugin->instanceToKeyFnc = NULL;
    plugin->keyToInstanceFnc = NULL;
    plugin->getSerializedKeyMaxSizeFnc = NULL;
    plugin->instanceToKeyHashFnc = NULL;
    plugin->serializedSampleToKeyHashFnc = NULL;
    plugin->serializedKeyToKeyHashFnc = NULL;    
    plugin->typeCode =  (struct RTICdrTypeCode *)Orientation_c_get_typecode();

    plugin->languageKind = PRES_TYPEPLUGIN_CPP_LANG;

    /* Serialized buffer */
    plugin->getBuffer = 
    (PRESTypePluginGetBufferFunction)
    Orientation_cPlugin_get_buffer;
    plugin->returnBuffer = 
    (PRESTypePluginReturnBufferFunction)
    Orientation_cPlugin_return_buffer;
    plugin->getSerializedSampleSizeFnc =
    (PRESTypePluginGetSerializedSampleSizeFunction)
    Orientation_cPlugin_get_serialized_sample_size;

    plugin->endpointTypeName = Orientation_cTYPENAME;

    return plugin;
}

void
Orientation_cPlugin_delete(struct PRESTypePlugin *plugin)
{
    RTIOsapiHeap_freeStructure(plugin);
} 

/* ----------------------------------------------------------------------------
*  Type World_Pose_c
* -------------------------------------------------------------------------- */

/* -----------------------------------------------------------------------------
Support functions:
* -------------------------------------------------------------------------- */

World_Pose_c*
World_Pose_cPluginSupport_create_data_w_params(
    const struct DDS_TypeAllocationParams_t * alloc_params) 
{
    World_Pose_c *sample = NULL;

    RTIOsapiHeap_allocateStructure(&(sample),World_Pose_c);
    if (sample == NULL) {
        return NULL;
    }

    if (!World_Pose_c_initialize_w_params(sample,alloc_params)) {
        RTIOsapiHeap_freeStructure(sample);
        sample=NULL;
    }
    return sample; 
} 

World_Pose_c *
World_Pose_cPluginSupport_create_data_ex(RTIBool allocate_pointers) 
{
    World_Pose_c *sample = NULL;

    RTIOsapiHeap_allocateStructure(&(sample),World_Pose_c);

    if(sample == NULL) {
        return NULL;
    }

    if (!World_Pose_c_initialize_ex(sample,allocate_pointers, RTI_TRUE)) {
        RTIOsapiHeap_freeStructure(sample);
        sample=NULL;
    }

    return sample; 
}

World_Pose_c *
World_Pose_cPluginSupport_create_data(void)
{
    return World_Pose_cPluginSupport_create_data_ex(RTI_TRUE);
}

void 
World_Pose_cPluginSupport_destroy_data_w_params(
    World_Pose_c *sample,
    const struct DDS_TypeDeallocationParams_t * dealloc_params) {

    World_Pose_c_finalize_w_params(sample,dealloc_params);

    RTIOsapiHeap_freeStructure(sample);
    sample=NULL;
}

void 
World_Pose_cPluginSupport_destroy_data_ex(
    World_Pose_c *sample,RTIBool deallocate_pointers) {

    World_Pose_c_finalize_ex(sample,deallocate_pointers);

    RTIOsapiHeap_freeStructure(sample);
    sample=NULL;
}

void 
World_Pose_cPluginSupport_destroy_data(
    World_Pose_c *sample) {

    World_Pose_cPluginSupport_destroy_data_ex(sample,RTI_TRUE);

}

RTIBool 
World_Pose_cPluginSupport_copy_data(
    World_Pose_c *dst,
    const World_Pose_c *src)
{
    return World_Pose_c_copy(dst,(const World_Pose_c*) src);
}

void 
World_Pose_cPluginSupport_print_data(
    const World_Pose_c *sample,
    const char *desc,
    unsigned int indent_level)
{

    RTICdrType_printIndent(indent_level);

    if (desc != NULL) {
        RTILog_debug("%s:\n", desc);
    } else {
        RTILog_debug("\n");
    }

    if (sample == NULL) {
        RTILog_debug("NULL\n");
        return;
    }

    Position_cPluginSupport_print_data(
        (const Position_c*) &sample->position, "position", indent_level + 1);

    Orientation_cPluginSupport_print_data(
        (const Orientation_c*) &sample->orientation, "orientation", indent_level + 1);

}

/* ----------------------------------------------------------------------------
Callback functions:
* ---------------------------------------------------------------------------- */

PRESTypePluginParticipantData 
World_Pose_cPlugin_on_participant_attached(
    void *registration_data,
    const struct PRESTypePluginParticipantInfo *participant_info,
    RTIBool top_level_registration,
    void *container_plugin_context,
    RTICdrTypeCode *type_code)
{
    if (registration_data) {} /* To avoid warnings */
    if (participant_info) {} /* To avoid warnings */
    if (top_level_registration) {} /* To avoid warnings */
    if (container_plugin_context) {} /* To avoid warnings */
    if (type_code) {} /* To avoid warnings */

    return PRESTypePluginDefaultParticipantData_new(participant_info);

}

void 
World_Pose_cPlugin_on_participant_detached(
    PRESTypePluginParticipantData participant_data)
{

    PRESTypePluginDefaultParticipantData_delete(participant_data);
}

PRESTypePluginEndpointData
World_Pose_cPlugin_on_endpoint_attached(
    PRESTypePluginParticipantData participant_data,
    const struct PRESTypePluginEndpointInfo *endpoint_info,
    RTIBool top_level_registration, 
    void *containerPluginContext)
{
    PRESTypePluginEndpointData epd = NULL;

    unsigned int serializedSampleMaxSize;

    if (top_level_registration) {} /* To avoid warnings */
    if (containerPluginContext) {} /* To avoid warnings */

    epd = PRESTypePluginDefaultEndpointData_new(
        participant_data,
        endpoint_info,
        (PRESTypePluginDefaultEndpointDataCreateSampleFunction)
        World_Pose_cPluginSupport_create_data,
        (PRESTypePluginDefaultEndpointDataDestroySampleFunction)
        World_Pose_cPluginSupport_destroy_data,
        NULL , NULL );

    if (epd == NULL) {
        return NULL;
    } 

    if (endpoint_info->endpointKind == PRES_TYPEPLUGIN_ENDPOINT_WRITER) {
        serializedSampleMaxSize = World_Pose_cPlugin_get_serialized_sample_max_size(
            epd,RTI_FALSE,RTI_CDR_ENCAPSULATION_ID_CDR_BE,0);

        PRESTypePluginDefaultEndpointData_setMaxSizeSerializedSample(epd, serializedSampleMaxSize);

        if (PRESTypePluginDefaultEndpointData_createWriterPool(
            epd,
            endpoint_info,
            (PRESTypePluginGetSerializedSampleMaxSizeFunction)
            World_Pose_cPlugin_get_serialized_sample_max_size, epd,
            (PRESTypePluginGetSerializedSampleSizeFunction)
            World_Pose_cPlugin_get_serialized_sample_size,
            epd) == RTI_FALSE) {
            PRESTypePluginDefaultEndpointData_delete(epd);
            return NULL;
        }
    }

    return epd;    
}

void 
World_Pose_cPlugin_on_endpoint_detached(
    PRESTypePluginEndpointData endpoint_data)
{  

    PRESTypePluginDefaultEndpointData_delete(endpoint_data);
}

void    
World_Pose_cPlugin_return_sample(
    PRESTypePluginEndpointData endpoint_data,
    World_Pose_c *sample,
    void *handle)
{

    World_Pose_c_finalize_optional_members(sample, RTI_TRUE);

    PRESTypePluginDefaultEndpointData_returnSample(
        endpoint_data, sample, handle);
}

RTIBool 
World_Pose_cPlugin_copy_sample(
    PRESTypePluginEndpointData endpoint_data,
    World_Pose_c *dst,
    const World_Pose_c *src)
{
    if (endpoint_data) {} /* To avoid warnings */
    return World_Pose_cPluginSupport_copy_data(dst,src);
}

/* ----------------------------------------------------------------------------
(De)Serialize functions:
* ------------------------------------------------------------------------- */
unsigned int 
World_Pose_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

RTIBool 
World_Pose_cPlugin_serialize(
    PRESTypePluginEndpointData endpoint_data,
    const World_Pose_c *sample, 
    struct RTICdrStream *stream,    
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;
    RTIBool retval = RTI_TRUE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(serialize_encapsulation) {
        if (!RTICdrStream_serializeAndSetCdrEncapsulation(stream , encapsulation_id)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if(serialize_sample) {

        if(!Position_cPlugin_serialize(
            endpoint_data,
            (const Position_c*) &sample->position,
            stream,
            RTI_FALSE, encapsulation_id,
            RTI_TRUE,
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

        if(!Orientation_cPlugin_serialize(
            endpoint_data,
            (const Orientation_c*) &sample->orientation,
            stream,
            RTI_FALSE, encapsulation_id,
            RTI_TRUE,
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

    }

    if(serialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return retval;
}

RTIBool 
World_Pose_cPlugin_deserialize_sample(
    PRESTypePluginEndpointData endpoint_data,
    World_Pose_c *sample,
    struct RTICdrStream *stream,   
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos)
{

    char * position = NULL;

    RTIBool done = RTI_FALSE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */
    if(deserialize_encapsulation) {

        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }
    if(deserialize_sample) {

        World_Pose_c_initialize_ex(sample, RTI_FALSE, RTI_FALSE);

        if(!Position_cPlugin_deserialize_sample(
            endpoint_data,
            &sample->position,
            stream,
            RTI_FALSE, RTI_TRUE,
            endpoint_plugin_qos)) {
            goto fin; 
        }
        if(!Orientation_cPlugin_deserialize_sample(
            endpoint_data,
            &sample->orientation,
            stream,
            RTI_FALSE, RTI_TRUE,
            endpoint_plugin_qos)) {
            goto fin; 
        }
    }

    done = RTI_TRUE;
  fin:
    if (done != RTI_TRUE && 
    RTICdrStream_getRemainder(stream) >=
    RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
        return RTI_FALSE;   
    }
    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;

}

RTIBool
World_Pose_cPlugin_serialize_to_cdr_buffer(
    char * buffer,
    unsigned int * length,
    const World_Pose_c *sample)
{
    struct RTICdrStream stream;
    struct PRESTypePluginDefaultEndpointData epd;
    RTIBool result;

    if (length == NULL) {
        return RTI_FALSE;
    }

    epd._maxSizeSerializedSample =
    World_Pose_cPlugin_get_serialized_sample_max_size(
        NULL, RTI_TRUE, RTICdrEncapsulation_getNativeCdrEncapsulationId(), 0);

    if (buffer == NULL) {
        *length = 
        World_Pose_cPlugin_get_serialized_sample_size(
            (PRESTypePluginEndpointData)&epd,
            RTI_TRUE,
            RTICdrEncapsulation_getNativeCdrEncapsulationId(),
            0,
            sample);

        if (*length == 0) {
            return RTI_FALSE;
        }

        return RTI_TRUE;
    }    

    RTICdrStream_init(&stream);
    RTICdrStream_set(&stream, (char *)buffer, *length);

    result = World_Pose_cPlugin_serialize(
        (PRESTypePluginEndpointData)&epd, sample, &stream, 
        RTI_TRUE, RTICdrEncapsulation_getNativeCdrEncapsulationId(), 
        RTI_TRUE, NULL);  

    *length = RTICdrStream_getCurrentPositionOffset(&stream);
    return result;     
}

RTIBool
World_Pose_cPlugin_deserialize_from_cdr_buffer(
    World_Pose_c *sample,
    const char * buffer,
    unsigned int length)
{
    struct RTICdrStream stream;

    RTICdrStream_init(&stream);
    RTICdrStream_set(&stream, (char *)buffer, length);

    World_Pose_c_finalize_optional_members(sample, RTI_TRUE);
    return World_Pose_cPlugin_deserialize_sample( 
        NULL, sample,
        &stream, RTI_TRUE, RTI_TRUE, 
        NULL);
}

DDS_ReturnCode_t
World_Pose_cPlugin_data_to_string(
    const World_Pose_c *sample,
    char *str,
    DDS_UnsignedLong *str_size, 
    const struct DDS_PrintFormatProperty *property)
{
    DDS_DynamicData *data = NULL;
    char *buffer = NULL;
    unsigned int length = 0;
    struct DDS_PrintFormat printFormat;
    DDS_ReturnCode_t retCode = DDS_RETCODE_ERROR;

    if (sample == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (str_size == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (property == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (!World_Pose_cPlugin_serialize_to_cdr_buffer(
        NULL, 
        &length, 
        sample)) {
        return DDS_RETCODE_ERROR;
    }

    RTIOsapiHeap_allocateBuffer(&buffer, length, RTI_OSAPI_ALIGNMENT_DEFAULT);
    if (buffer == NULL) {
        return DDS_RETCODE_ERROR;
    }

    if (!World_Pose_cPlugin_serialize_to_cdr_buffer(
        buffer, 
        &length, 
        sample)) {
        RTIOsapiHeap_freeBuffer(buffer);
        return DDS_RETCODE_ERROR;
    }

    data = DDS_DynamicData_new(
        World_Pose_c_get_typecode(), 
        &DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
    if (data == NULL) {
        RTIOsapiHeap_freeBuffer(buffer);
        return DDS_RETCODE_ERROR;
    }

    retCode = DDS_DynamicData_from_cdr_buffer(data, buffer, length);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    retCode = DDS_PrintFormatProperty_to_print_format(
        property, 
        &printFormat);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    retCode = DDS_DynamicDataFormatter_to_string_w_format(
        data, 
        str,
        str_size, 
        &printFormat);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    RTIOsapiHeap_freeBuffer(buffer);
    DDS_DynamicData_delete(data);
    return DDS_RETCODE_OK;
}

RTIBool 
World_Pose_cPlugin_deserialize(
    PRESTypePluginEndpointData endpoint_data,
    World_Pose_c **sample,
    RTIBool * drop_sample,
    struct RTICdrStream *stream,   
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos)
{

    RTIBool result;
    const char *METHOD_NAME = "World_Pose_cPlugin_deserialize";
    if (drop_sample) {} /* To avoid warnings */

    stream->_xTypesState.unassignable = RTI_FALSE;
    result= World_Pose_cPlugin_deserialize_sample( 
        endpoint_data, (sample != NULL)?*sample:NULL,
        stream, deserialize_encapsulation, deserialize_sample, 
        endpoint_plugin_qos);
    if (result) {
        if (stream->_xTypesState.unassignable) {
            result = RTI_FALSE;
        }
    }
    if (!result && stream->_xTypesState.unassignable ) {

        RTICdrLog_exception(
            METHOD_NAME, 
            &RTI_CDR_LOG_UNASSIGNABLE_SAMPLE_OF_TYPE_s, 
            "World_Pose_c");

    }

    return result;

}

RTIBool World_Pose_cPlugin_skip(
    PRESTypePluginEndpointData endpoint_data,
    struct RTICdrStream *stream,   
    RTIBool skip_encapsulation,
    RTIBool skip_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    RTIBool done = RTI_FALSE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(skip_encapsulation) {
        if (!RTICdrStream_skipEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if (skip_sample) {

        if (!Position_cPlugin_skip(
            endpoint_data,
            stream, 
            RTI_FALSE, RTI_TRUE, 
            endpoint_plugin_qos)) {
            goto fin; 
        }
        if (!Orientation_cPlugin_skip(
            endpoint_data,
            stream, 
            RTI_FALSE, RTI_TRUE, 
            endpoint_plugin_qos)) {
            goto fin; 
        }
    }

    done = RTI_TRUE;
  fin:
    if (done != RTI_TRUE && 
    RTICdrStream_getRemainder(stream) >=
    RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
        return RTI_FALSE;   
    }
    if(skip_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

unsigned int 
World_Pose_cPlugin_get_serialized_sample_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment +=Position_cPlugin_get_serialized_sample_max_size_ex(
        endpoint_data, overflow, RTI_FALSE,encapsulation_id,current_alignment);

    current_alignment +=Orientation_cPlugin_get_serialized_sample_max_size_ex(
        endpoint_data, overflow, RTI_FALSE,encapsulation_id,current_alignment);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return  current_alignment - initial_alignment;
}

unsigned int 
World_Pose_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{
    unsigned int size;
    RTIBool overflow = RTI_FALSE;

    size = World_Pose_cPlugin_get_serialized_sample_max_size_ex(
        endpoint_data,&overflow,include_encapsulation,encapsulation_id,current_alignment);

    if (overflow) {
        size = RTI_CDR_MAX_SERIALIZED_SIZE;
    }

    return size;
}

unsigned int 
World_Pose_cPlugin_get_serialized_sample_min_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */ 

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment +=Position_cPlugin_get_serialized_sample_min_size(
        endpoint_data,RTI_FALSE,encapsulation_id,current_alignment);
    current_alignment +=Orientation_cPlugin_get_serialized_sample_min_size(
        endpoint_data,RTI_FALSE,encapsulation_id,current_alignment);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return  current_alignment - initial_alignment;
}

/* Returns the size of the sample in its serialized form (in bytes).
* It can also be an estimation in excess of the real buffer needed 
* during a call to the serialize() function.
* The value reported does not have to include the space for the
* encapsulation flags.
*/
unsigned int
World_Pose_cPlugin_get_serialized_sample_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment,
    const World_Pose_c * sample) 
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;
    struct PRESTypePluginDefaultEndpointData epd;   

    if (sample==NULL) {
        return 0;
    }
    if (endpoint_data == NULL) {
        endpoint_data = (PRESTypePluginEndpointData) &epd;
        PRESTypePluginDefaultEndpointData_setBaseAlignment(
            endpoint_data,
            current_alignment);        
    }

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
        PRESTypePluginDefaultEndpointData_setBaseAlignment(
            endpoint_data,
            current_alignment);
    }

    current_alignment += Position_cPlugin_get_serialized_sample_size(
        endpoint_data,RTI_FALSE, encapsulation_id,
        current_alignment, (const Position_c*) &sample->position);

    current_alignment += Orientation_cPlugin_get_serialized_sample_size(
        endpoint_data,RTI_FALSE, encapsulation_id,
        current_alignment, (const Orientation_c*) &sample->orientation);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return current_alignment - initial_alignment;
}

/* --------------------------------------------------------------------------------------
Key Management functions:
* -------------------------------------------------------------------------------------- */

PRESTypePluginKeyKind 
World_Pose_cPlugin_get_key_kind(void)
{
    return PRES_TYPEPLUGIN_NO_KEY;
}

RTIBool 
World_Pose_cPlugin_serialize_key(
    PRESTypePluginEndpointData endpoint_data,
    const World_Pose_c *sample, 
    struct RTICdrStream *stream,    
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_key,
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    if(serialize_encapsulation) {
        if (!RTICdrStream_serializeAndSetCdrEncapsulation(stream , encapsulation_id)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if(serialize_key) {

        if (!World_Pose_cPlugin_serialize(
            endpoint_data,
            sample,
            stream,
            RTI_FALSE, encapsulation_id,
            RTI_TRUE,
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

    }

    if(serialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

RTIBool World_Pose_cPlugin_deserialize_key_sample(
    PRESTypePluginEndpointData endpoint_data,
    World_Pose_c *sample, 
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos)
{

    char * position = NULL;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(deserialize_encapsulation) {

        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }
    if (deserialize_key) {

        if (!World_Pose_cPlugin_deserialize_sample(
            endpoint_data, sample, stream, 
            RTI_FALSE, RTI_TRUE, 
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }
    }

    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;

}

RTIBool World_Pose_cPlugin_deserialize_key(
    PRESTypePluginEndpointData endpoint_data,
    World_Pose_c **sample, 
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos)
{
    RTIBool result;
    if (drop_sample) {} /* To avoid warnings */
    stream->_xTypesState.unassignable = RTI_FALSE;
    result= World_Pose_cPlugin_deserialize_key_sample(
        endpoint_data, (sample != NULL)?*sample:NULL, stream,
        deserialize_encapsulation, deserialize_key, endpoint_plugin_qos);
    if (result) {
        if (stream->_xTypesState.unassignable) {
            result = RTI_FALSE;
        }
    }

    return result;    

}

unsigned int
World_Pose_cPlugin_get_serialized_key_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */
    if (overflow) {} /* To avoid warnings */

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment += World_Pose_cPlugin_get_serialized_sample_max_size_ex(
        endpoint_data, overflow,RTI_FALSE, encapsulation_id, current_alignment);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return current_alignment - initial_alignment;
}

unsigned int
World_Pose_cPlugin_get_serialized_key_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{
    unsigned int size;
    RTIBool overflow = RTI_FALSE;

    size = World_Pose_cPlugin_get_serialized_key_max_size_ex(
        endpoint_data,&overflow,include_encapsulation,encapsulation_id,current_alignment);

    if (overflow) {
        size = RTI_CDR_MAX_SERIALIZED_SIZE;
    }

    return size;
}

RTIBool 
World_Pose_cPlugin_serialized_sample_to_key(
    PRESTypePluginEndpointData endpoint_data,
    World_Pose_c *sample,
    struct RTICdrStream *stream, 
    RTIBool deserialize_encapsulation,  
    RTIBool deserialize_key, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    RTIBool done = RTI_FALSE;
    RTIBool error = RTI_FALSE;

    if (stream == NULL) {
        error = RTI_TRUE;
        goto fin;
    }
    if(deserialize_encapsulation) {
        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }
        position = RTICdrStream_resetAlignment(stream);
    }

    if (deserialize_key) {

        if (!World_Pose_cPlugin_deserialize_sample(
            endpoint_data, sample, stream, RTI_FALSE, 
            RTI_TRUE, endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

    }

    done = RTI_TRUE;
  fin:
    if(!error) {
        if (done != RTI_TRUE && 
        RTICdrStream_getRemainder(stream) >=
        RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
            return RTI_FALSE;   
        }
    } else {
        return RTI_FALSE;
    }       

    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

/* ------------------------------------------------------------------------
* Plug-in Installation Methods
* ------------------------------------------------------------------------ */
struct PRESTypePlugin *World_Pose_cPlugin_new(void) 
{ 
    struct PRESTypePlugin *plugin = NULL;
    const struct PRESTypePluginVersion PLUGIN_VERSION = 
    PRES_TYPE_PLUGIN_VERSION_2_0;

    RTIOsapiHeap_allocateStructure(
        &plugin, struct PRESTypePlugin);

    if (plugin == NULL) {
        return NULL;
    }

    plugin->version = PLUGIN_VERSION;

    /* set up parent's function pointers */
    plugin->onParticipantAttached =
    (PRESTypePluginOnParticipantAttachedCallback)
    World_Pose_cPlugin_on_participant_attached;
    plugin->onParticipantDetached =
    (PRESTypePluginOnParticipantDetachedCallback)
    World_Pose_cPlugin_on_participant_detached;
    plugin->onEndpointAttached =
    (PRESTypePluginOnEndpointAttachedCallback)
    World_Pose_cPlugin_on_endpoint_attached;
    plugin->onEndpointDetached =
    (PRESTypePluginOnEndpointDetachedCallback)
    World_Pose_cPlugin_on_endpoint_detached;

    plugin->copySampleFnc =
    (PRESTypePluginCopySampleFunction)
    World_Pose_cPlugin_copy_sample;
    plugin->createSampleFnc =
    (PRESTypePluginCreateSampleFunction)
    World_Pose_cPlugin_create_sample;
    plugin->destroySampleFnc =
    (PRESTypePluginDestroySampleFunction)
    World_Pose_cPlugin_destroy_sample;
    plugin->finalizeOptionalMembersFnc =
    (PRESTypePluginFinalizeOptionalMembersFunction)
    World_Pose_c_finalize_optional_members;

    plugin->serializeFnc =
    (PRESTypePluginSerializeFunction)
    World_Pose_cPlugin_serialize;
    plugin->deserializeFnc =
    (PRESTypePluginDeserializeFunction)
    World_Pose_cPlugin_deserialize;
    plugin->getSerializedSampleMaxSizeFnc =
    (PRESTypePluginGetSerializedSampleMaxSizeFunction)
    World_Pose_cPlugin_get_serialized_sample_max_size;
    plugin->getSerializedSampleMinSizeFnc =
    (PRESTypePluginGetSerializedSampleMinSizeFunction)
    World_Pose_cPlugin_get_serialized_sample_min_size;

    plugin->getSampleFnc =
    (PRESTypePluginGetSampleFunction)
    World_Pose_cPlugin_get_sample;
    plugin->returnSampleFnc =
    (PRESTypePluginReturnSampleFunction)
    World_Pose_cPlugin_return_sample;

    plugin->getKeyKindFnc =
    (PRESTypePluginGetKeyKindFunction)
    World_Pose_cPlugin_get_key_kind;

    /* These functions are only used for keyed types. As this is not a keyed
    type they are all set to NULL
    */
    plugin->serializeKeyFnc = NULL ;    
    plugin->deserializeKeyFnc = NULL;  
    plugin->getKeyFnc = NULL;
    plugin->returnKeyFnc = NULL;
    plugin->instanceToKeyFnc = NULL;
    plugin->keyToInstanceFnc = NULL;
    plugin->getSerializedKeyMaxSizeFnc = NULL;
    plugin->instanceToKeyHashFnc = NULL;
    plugin->serializedSampleToKeyHashFnc = NULL;
    plugin->serializedKeyToKeyHashFnc = NULL;    
    plugin->typeCode =  (struct RTICdrTypeCode *)World_Pose_c_get_typecode();

    plugin->languageKind = PRES_TYPEPLUGIN_CPP_LANG;

    /* Serialized buffer */
    plugin->getBuffer = 
    (PRESTypePluginGetBufferFunction)
    World_Pose_cPlugin_get_buffer;
    plugin->returnBuffer = 
    (PRESTypePluginReturnBufferFunction)
    World_Pose_cPlugin_return_buffer;
    plugin->getSerializedSampleSizeFnc =
    (PRESTypePluginGetSerializedSampleSizeFunction)
    World_Pose_cPlugin_get_serialized_sample_size;

    plugin->endpointTypeName = World_Pose_cTYPENAME;

    return plugin;
}

void
World_Pose_cPlugin_delete(struct PRESTypePlugin *plugin)
{
    RTIOsapiHeap_freeStructure(plugin);
} 

/* ----------------------------------------------------------------------------
*  Type laser_Scan_msg_c
* -------------------------------------------------------------------------- */

/* -----------------------------------------------------------------------------
Support functions:
* -------------------------------------------------------------------------- */

laser_Scan_msg_c*
laser_Scan_msg_cPluginSupport_create_data_w_params(
    const struct DDS_TypeAllocationParams_t * alloc_params) 
{
    laser_Scan_msg_c *sample = NULL;

    RTIOsapiHeap_allocateStructure(&(sample),laser_Scan_msg_c);
    if (sample == NULL) {
        return NULL;
    }

    if (!laser_Scan_msg_c_initialize_w_params(sample,alloc_params)) {
        RTIOsapiHeap_freeStructure(sample);
        sample=NULL;
    }
    return sample; 
} 

laser_Scan_msg_c *
laser_Scan_msg_cPluginSupport_create_data_ex(RTIBool allocate_pointers) 
{
    laser_Scan_msg_c *sample = NULL;

    RTIOsapiHeap_allocateStructure(&(sample),laser_Scan_msg_c);

    if(sample == NULL) {
        return NULL;
    }

    if (!laser_Scan_msg_c_initialize_ex(sample,allocate_pointers, RTI_TRUE)) {
        RTIOsapiHeap_freeStructure(sample);
        sample=NULL;
    }

    return sample; 
}

laser_Scan_msg_c *
laser_Scan_msg_cPluginSupport_create_data(void)
{
    return laser_Scan_msg_cPluginSupport_create_data_ex(RTI_TRUE);
}

void 
laser_Scan_msg_cPluginSupport_destroy_data_w_params(
    laser_Scan_msg_c *sample,
    const struct DDS_TypeDeallocationParams_t * dealloc_params) {

    laser_Scan_msg_c_finalize_w_params(sample,dealloc_params);

    RTIOsapiHeap_freeStructure(sample);
    sample=NULL;
}

void 
laser_Scan_msg_cPluginSupport_destroy_data_ex(
    laser_Scan_msg_c *sample,RTIBool deallocate_pointers) {

    laser_Scan_msg_c_finalize_ex(sample,deallocate_pointers);

    RTIOsapiHeap_freeStructure(sample);
    sample=NULL;
}

void 
laser_Scan_msg_cPluginSupport_destroy_data(
    laser_Scan_msg_c *sample) {

    laser_Scan_msg_cPluginSupport_destroy_data_ex(sample,RTI_TRUE);

}

RTIBool 
laser_Scan_msg_cPluginSupport_copy_data(
    laser_Scan_msg_c *dst,
    const laser_Scan_msg_c *src)
{
    return laser_Scan_msg_c_copy(dst,(const laser_Scan_msg_c*) src);
}

void 
laser_Scan_msg_cPluginSupport_print_data(
    const laser_Scan_msg_c *sample,
    const char *desc,
    unsigned int indent_level)
{

    RTICdrType_printIndent(indent_level);

    if (desc != NULL) {
        RTILog_debug("%s:\n", desc);
    } else {
        RTILog_debug("\n");
    }

    if (sample == NULL) {
        RTILog_debug("NULL\n");
        return;
    }

    Header_cPluginSupport_print_data(
        (const Header_c*) &sample->header, "header", indent_level + 1);

    World_Pose_cPluginSupport_print_data(
        (const World_Pose_c*) &sample->world_pose, "world_pose", indent_level + 1);

    RTICdrType_printFloat(
        &sample->angle_min, "angle_min", indent_level + 1);    

    RTICdrType_printFloat(
        &sample->angle_max, "angle_max", indent_level + 1);    

    RTICdrType_printFloat(
        &sample->angle_step, "angle_step", indent_level + 1);    

    RTICdrType_printFloat(
        &sample->range_min, "range_min", indent_level + 1);    

    RTICdrType_printFloat(
        &sample->range_max, "range_max", indent_level + 1);    

    RTICdrType_printLong(
        &sample->count, "count", indent_level + 1);    

    RTICdrType_printFloat(
        &sample->vertical_angle_min, "vertical_angle_min", indent_level + 1);    

    RTICdrType_printFloat(
        &sample->vertical_angle_max, "vertical_angle_max", indent_level + 1);    

    RTICdrType_printFloat(
        &sample->vertical_angle_step, "vertical_angle_step", indent_level + 1);    

    RTICdrType_printFloat(
        &sample->vertical_count, "vertical_count", indent_level + 1);    

    if (DDS_FloatSeq_get_contiguous_bufferI(&sample->ranges) != NULL) {
        RTICdrType_printArray(
            DDS_FloatSeq_get_contiguous_bufferI(&sample->ranges),
            DDS_FloatSeq_get_length(&sample->ranges),
            RTI_CDR_FLOAT_SIZE,
            (RTICdrTypePrintFunction)RTICdrType_printFloat,
            "ranges", indent_level + 1);
    } else {
        RTICdrType_printPointerArray(
            DDS_FloatSeq_get_discontiguous_bufferI(&sample->ranges),
            DDS_FloatSeq_get_length(&sample->ranges ),
            (RTICdrTypePrintFunction)RTICdrType_printFloat,
            "ranges", indent_level + 1);
    }

    if (DDS_FloatSeq_get_contiguous_bufferI(&sample->intensities) != NULL) {
        RTICdrType_printArray(
            DDS_FloatSeq_get_contiguous_bufferI(&sample->intensities),
            DDS_FloatSeq_get_length(&sample->intensities),
            RTI_CDR_FLOAT_SIZE,
            (RTICdrTypePrintFunction)RTICdrType_printFloat,
            "intensities", indent_level + 1);
    } else {
        RTICdrType_printPointerArray(
            DDS_FloatSeq_get_discontiguous_bufferI(&sample->intensities),
            DDS_FloatSeq_get_length(&sample->intensities ),
            (RTICdrTypePrintFunction)RTICdrType_printFloat,
            "intensities", indent_level + 1);
    }

}

/* ----------------------------------------------------------------------------
Callback functions:
* ---------------------------------------------------------------------------- */

PRESTypePluginParticipantData 
laser_Scan_msg_cPlugin_on_participant_attached(
    void *registration_data,
    const struct PRESTypePluginParticipantInfo *participant_info,
    RTIBool top_level_registration,
    void *container_plugin_context,
    RTICdrTypeCode *type_code)
{
    if (registration_data) {} /* To avoid warnings */
    if (participant_info) {} /* To avoid warnings */
    if (top_level_registration) {} /* To avoid warnings */
    if (container_plugin_context) {} /* To avoid warnings */
    if (type_code) {} /* To avoid warnings */

    return PRESTypePluginDefaultParticipantData_new(participant_info);

}

void 
laser_Scan_msg_cPlugin_on_participant_detached(
    PRESTypePluginParticipantData participant_data)
{

    PRESTypePluginDefaultParticipantData_delete(participant_data);
}

PRESTypePluginEndpointData
laser_Scan_msg_cPlugin_on_endpoint_attached(
    PRESTypePluginParticipantData participant_data,
    const struct PRESTypePluginEndpointInfo *endpoint_info,
    RTIBool top_level_registration, 
    void *containerPluginContext)
{
    PRESTypePluginEndpointData epd = NULL;

    unsigned int serializedSampleMaxSize;

    if (top_level_registration) {} /* To avoid warnings */
    if (containerPluginContext) {} /* To avoid warnings */

    epd = PRESTypePluginDefaultEndpointData_new(
        participant_data,
        endpoint_info,
        (PRESTypePluginDefaultEndpointDataCreateSampleFunction)
        laser_Scan_msg_cPluginSupport_create_data,
        (PRESTypePluginDefaultEndpointDataDestroySampleFunction)
        laser_Scan_msg_cPluginSupport_destroy_data,
        NULL , NULL );

    if (epd == NULL) {
        return NULL;
    } 

    if (endpoint_info->endpointKind == PRES_TYPEPLUGIN_ENDPOINT_WRITER) {
        serializedSampleMaxSize = laser_Scan_msg_cPlugin_get_serialized_sample_max_size(
            epd,RTI_FALSE,RTI_CDR_ENCAPSULATION_ID_CDR_BE,0);

        PRESTypePluginDefaultEndpointData_setMaxSizeSerializedSample(epd, serializedSampleMaxSize);

        if (PRESTypePluginDefaultEndpointData_createWriterPool(
            epd,
            endpoint_info,
            (PRESTypePluginGetSerializedSampleMaxSizeFunction)
            laser_Scan_msg_cPlugin_get_serialized_sample_max_size, epd,
            (PRESTypePluginGetSerializedSampleSizeFunction)
            laser_Scan_msg_cPlugin_get_serialized_sample_size,
            epd) == RTI_FALSE) {
            PRESTypePluginDefaultEndpointData_delete(epd);
            return NULL;
        }
    }

    return epd;    
}

void 
laser_Scan_msg_cPlugin_on_endpoint_detached(
    PRESTypePluginEndpointData endpoint_data)
{  

    PRESTypePluginDefaultEndpointData_delete(endpoint_data);
}

void    
laser_Scan_msg_cPlugin_return_sample(
    PRESTypePluginEndpointData endpoint_data,
    laser_Scan_msg_c *sample,
    void *handle)
{

    laser_Scan_msg_c_finalize_optional_members(sample, RTI_TRUE);

    PRESTypePluginDefaultEndpointData_returnSample(
        endpoint_data, sample, handle);
}

RTIBool 
laser_Scan_msg_cPlugin_copy_sample(
    PRESTypePluginEndpointData endpoint_data,
    laser_Scan_msg_c *dst,
    const laser_Scan_msg_c *src)
{
    if (endpoint_data) {} /* To avoid warnings */
    return laser_Scan_msg_cPluginSupport_copy_data(dst,src);
}

/* ----------------------------------------------------------------------------
(De)Serialize functions:
* ------------------------------------------------------------------------- */
unsigned int 
laser_Scan_msg_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

RTIBool 
laser_Scan_msg_cPlugin_serialize(
    PRESTypePluginEndpointData endpoint_data,
    const laser_Scan_msg_c *sample, 
    struct RTICdrStream *stream,    
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;
    RTIBool retval = RTI_TRUE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(serialize_encapsulation) {
        if (!RTICdrStream_serializeAndSetCdrEncapsulation(stream , encapsulation_id)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if(serialize_sample) {

        if(!Header_cPlugin_serialize(
            endpoint_data,
            (const Header_c*) &sample->header,
            stream,
            RTI_FALSE, encapsulation_id,
            RTI_TRUE,
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

        if(!World_Pose_cPlugin_serialize(
            endpoint_data,
            (const World_Pose_c*) &sample->world_pose,
            stream,
            RTI_FALSE, encapsulation_id,
            RTI_TRUE,
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeFloat(
            stream, &sample->angle_min)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeFloat(
            stream, &sample->angle_max)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeFloat(
            stream, &sample->angle_step)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeFloat(
            stream, &sample->range_min)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeFloat(
            stream, &sample->range_max)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeLong(
            stream, &sample->count)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeFloat(
            stream, &sample->vertical_angle_min)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeFloat(
            stream, &sample->vertical_angle_max)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeFloat(
            stream, &sample->vertical_angle_step)) {
            return RTI_FALSE;
        }

        if (!RTICdrStream_serializeFloat(
            stream, &sample->vertical_count)) {
            return RTI_FALSE;
        }

        if (DDS_FloatSeq_get_contiguous_bufferI(&sample->ranges) != NULL) {
            if (!RTICdrStream_serializePrimitiveSequence(
                stream,
                DDS_FloatSeq_get_contiguous_bufferI(&sample->ranges),
                DDS_FloatSeq_get_length(&sample->ranges),
                (RTI_INT32_MAX-1),
                RTI_CDR_FLOAT_TYPE)) {
                return RTI_FALSE;
            } 
        } else {
            if (!RTICdrStream_serializePrimitivePointerSequence(
                stream,
                (const void **) DDS_FloatSeq_get_discontiguous_bufferI(&sample->ranges),
                DDS_FloatSeq_get_length(&sample->ranges),
                (RTI_INT32_MAX-1), 
                RTI_CDR_FLOAT_TYPE)) {
                return RTI_FALSE;
            } 
        }

        if (DDS_FloatSeq_get_contiguous_bufferI(&sample->intensities) != NULL) {
            if (!RTICdrStream_serializePrimitiveSequence(
                stream,
                DDS_FloatSeq_get_contiguous_bufferI(&sample->intensities),
                DDS_FloatSeq_get_length(&sample->intensities),
                (RTI_INT32_MAX-1),
                RTI_CDR_FLOAT_TYPE)) {
                return RTI_FALSE;
            } 
        } else {
            if (!RTICdrStream_serializePrimitivePointerSequence(
                stream,
                (const void **) DDS_FloatSeq_get_discontiguous_bufferI(&sample->intensities),
                DDS_FloatSeq_get_length(&sample->intensities),
                (RTI_INT32_MAX-1), 
                RTI_CDR_FLOAT_TYPE)) {
                return RTI_FALSE;
            } 
        }

    }

    if(serialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return retval;
}

RTIBool 
laser_Scan_msg_cPlugin_deserialize_sample(
    PRESTypePluginEndpointData endpoint_data,
    laser_Scan_msg_c *sample,
    struct RTICdrStream *stream,   
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos)
{

    char * position = NULL;

    RTIBool done = RTI_FALSE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */
    if(deserialize_encapsulation) {

        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }
    if(deserialize_sample) {

        laser_Scan_msg_c_initialize_ex(sample, RTI_FALSE, RTI_FALSE);

        if(!Header_cPlugin_deserialize_sample(
            endpoint_data,
            &sample->header,
            stream,
            RTI_FALSE, RTI_TRUE,
            endpoint_plugin_qos)) {
            goto fin; 
        }
        if(!World_Pose_cPlugin_deserialize_sample(
            endpoint_data,
            &sample->world_pose,
            stream,
            RTI_FALSE, RTI_TRUE,
            endpoint_plugin_qos)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeFloat(
            stream, &sample->angle_min)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeFloat(
            stream, &sample->angle_max)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeFloat(
            stream, &sample->angle_step)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeFloat(
            stream, &sample->range_min)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeFloat(
            stream, &sample->range_max)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeLong(
            stream, &sample->count)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeFloat(
            stream, &sample->vertical_angle_min)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeFloat(
            stream, &sample->vertical_angle_max)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeFloat(
            stream, &sample->vertical_angle_step)) {
            goto fin; 
        }
        if (!RTICdrStream_deserializeFloat(
            stream, &sample->vertical_count)) {
            goto fin; 
        }
        {
            RTICdrUnsignedLong sequence_length;
            if (!RTICdrStream_lookUnsignedLong(stream,&sequence_length)) {
                goto fin; 
            }
            if (!DDS_FloatSeq_set_maximum(&sample->ranges,sequence_length)) {
                return RTI_FALSE;
            }
            if (DDS_FloatSeq_get_contiguous_bufferI(&sample->ranges) != NULL) {
                if (!RTICdrStream_deserializePrimitiveSequence(
                    stream,
                    DDS_FloatSeq_get_contiguous_bufferI(&sample->ranges),
                    &sequence_length,
                    DDS_FloatSeq_get_maximum(&sample->ranges),
                    RTI_CDR_FLOAT_TYPE)){
                    goto fin; 
                }
            } else {
                if (!RTICdrStream_deserializePrimitivePointerSequence(
                    stream,
                    (void **) DDS_FloatSeq_get_discontiguous_bufferI(&sample->ranges),
                    &sequence_length,
                    DDS_FloatSeq_get_maximum(&sample->ranges),
                    RTI_CDR_FLOAT_TYPE)){
                    goto fin; 
                }
            }
            if (!DDS_FloatSeq_set_length(&sample->ranges, sequence_length)) {
                return RTI_FALSE;
            }

        }
        {
            RTICdrUnsignedLong sequence_length;
            if (!RTICdrStream_lookUnsignedLong(stream,&sequence_length)) {
                goto fin; 
            }
            if (!DDS_FloatSeq_set_maximum(&sample->intensities,sequence_length)) {
                return RTI_FALSE;
            }
            if (DDS_FloatSeq_get_contiguous_bufferI(&sample->intensities) != NULL) {
                if (!RTICdrStream_deserializePrimitiveSequence(
                    stream,
                    DDS_FloatSeq_get_contiguous_bufferI(&sample->intensities),
                    &sequence_length,
                    DDS_FloatSeq_get_maximum(&sample->intensities),
                    RTI_CDR_FLOAT_TYPE)){
                    goto fin; 
                }
            } else {
                if (!RTICdrStream_deserializePrimitivePointerSequence(
                    stream,
                    (void **) DDS_FloatSeq_get_discontiguous_bufferI(&sample->intensities),
                    &sequence_length,
                    DDS_FloatSeq_get_maximum(&sample->intensities),
                    RTI_CDR_FLOAT_TYPE)){
                    goto fin; 
                }
            }
            if (!DDS_FloatSeq_set_length(&sample->intensities, sequence_length)) {
                return RTI_FALSE;
            }

        }
    }

    done = RTI_TRUE;
  fin:
    if (done != RTI_TRUE && 
    RTICdrStream_getRemainder(stream) >=
    RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
        return RTI_FALSE;   
    }
    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;

}

RTIBool
laser_Scan_msg_cPlugin_serialize_to_cdr_buffer(
    char * buffer,
    unsigned int * length,
    const laser_Scan_msg_c *sample)
{
    struct RTICdrStream stream;
    struct PRESTypePluginDefaultEndpointData epd;
    RTIBool result;

    if (length == NULL) {
        return RTI_FALSE;
    }

    epd._maxSizeSerializedSample =
    laser_Scan_msg_cPlugin_get_serialized_sample_max_size(
        NULL, RTI_TRUE, RTICdrEncapsulation_getNativeCdrEncapsulationId(), 0);

    if (buffer == NULL) {
        *length = 
        laser_Scan_msg_cPlugin_get_serialized_sample_size(
            (PRESTypePluginEndpointData)&epd,
            RTI_TRUE,
            RTICdrEncapsulation_getNativeCdrEncapsulationId(),
            0,
            sample);

        if (*length == 0) {
            return RTI_FALSE;
        }

        return RTI_TRUE;
    }    

    RTICdrStream_init(&stream);
    RTICdrStream_set(&stream, (char *)buffer, *length);

    result = laser_Scan_msg_cPlugin_serialize(
        (PRESTypePluginEndpointData)&epd, sample, &stream, 
        RTI_TRUE, RTICdrEncapsulation_getNativeCdrEncapsulationId(), 
        RTI_TRUE, NULL);  

    *length = RTICdrStream_getCurrentPositionOffset(&stream);
    return result;     
}

RTIBool
laser_Scan_msg_cPlugin_deserialize_from_cdr_buffer(
    laser_Scan_msg_c *sample,
    const char * buffer,
    unsigned int length)
{
    struct RTICdrStream stream;

    RTICdrStream_init(&stream);
    RTICdrStream_set(&stream, (char *)buffer, length);

    laser_Scan_msg_c_finalize_optional_members(sample, RTI_TRUE);
    return laser_Scan_msg_cPlugin_deserialize_sample( 
        NULL, sample,
        &stream, RTI_TRUE, RTI_TRUE, 
        NULL);
}

DDS_ReturnCode_t
laser_Scan_msg_cPlugin_data_to_string(
    const laser_Scan_msg_c *sample,
    char *str,
    DDS_UnsignedLong *str_size, 
    const struct DDS_PrintFormatProperty *property)
{
    DDS_DynamicData *data = NULL;
    char *buffer = NULL;
    unsigned int length = 0;
    struct DDS_PrintFormat printFormat;
    DDS_ReturnCode_t retCode = DDS_RETCODE_ERROR;

    if (sample == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (str_size == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (property == NULL) {
        return DDS_RETCODE_BAD_PARAMETER;
    }

    if (!laser_Scan_msg_cPlugin_serialize_to_cdr_buffer(
        NULL, 
        &length, 
        sample)) {
        return DDS_RETCODE_ERROR;
    }

    RTIOsapiHeap_allocateBuffer(&buffer, length, RTI_OSAPI_ALIGNMENT_DEFAULT);
    if (buffer == NULL) {
        return DDS_RETCODE_ERROR;
    }

    if (!laser_Scan_msg_cPlugin_serialize_to_cdr_buffer(
        buffer, 
        &length, 
        sample)) {
        RTIOsapiHeap_freeBuffer(buffer);
        return DDS_RETCODE_ERROR;
    }

    data = DDS_DynamicData_new(
        laser_Scan_msg_c_get_typecode(), 
        &DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
    if (data == NULL) {
        RTIOsapiHeap_freeBuffer(buffer);
        return DDS_RETCODE_ERROR;
    }

    retCode = DDS_DynamicData_from_cdr_buffer(data, buffer, length);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    retCode = DDS_PrintFormatProperty_to_print_format(
        property, 
        &printFormat);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    retCode = DDS_DynamicDataFormatter_to_string_w_format(
        data, 
        str,
        str_size, 
        &printFormat);
    if (retCode != DDS_RETCODE_OK) {
        RTIOsapiHeap_freeBuffer(buffer);
        DDS_DynamicData_delete(data);
        return retCode;
    }

    RTIOsapiHeap_freeBuffer(buffer);
    DDS_DynamicData_delete(data);
    return DDS_RETCODE_OK;
}

RTIBool 
laser_Scan_msg_cPlugin_deserialize(
    PRESTypePluginEndpointData endpoint_data,
    laser_Scan_msg_c **sample,
    RTIBool * drop_sample,
    struct RTICdrStream *stream,   
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos)
{

    RTIBool result;
    const char *METHOD_NAME = "laser_Scan_msg_cPlugin_deserialize";
    if (drop_sample) {} /* To avoid warnings */

    stream->_xTypesState.unassignable = RTI_FALSE;
    result= laser_Scan_msg_cPlugin_deserialize_sample( 
        endpoint_data, (sample != NULL)?*sample:NULL,
        stream, deserialize_encapsulation, deserialize_sample, 
        endpoint_plugin_qos);
    if (result) {
        if (stream->_xTypesState.unassignable) {
            result = RTI_FALSE;
        }
    }
    if (!result && stream->_xTypesState.unassignable ) {

        RTICdrLog_exception(
            METHOD_NAME, 
            &RTI_CDR_LOG_UNASSIGNABLE_SAMPLE_OF_TYPE_s, 
            "laser_Scan_msg_c");

    }

    return result;

}

RTIBool laser_Scan_msg_cPlugin_skip(
    PRESTypePluginEndpointData endpoint_data,
    struct RTICdrStream *stream,   
    RTIBool skip_encapsulation,
    RTIBool skip_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    RTIBool done = RTI_FALSE;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(skip_encapsulation) {
        if (!RTICdrStream_skipEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if (skip_sample) {

        if (!Header_cPlugin_skip(
            endpoint_data,
            stream, 
            RTI_FALSE, RTI_TRUE, 
            endpoint_plugin_qos)) {
            goto fin; 
        }
        if (!World_Pose_cPlugin_skip(
            endpoint_data,
            stream, 
            RTI_FALSE, RTI_TRUE, 
            endpoint_plugin_qos)) {
            goto fin; 
        }
        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipLong (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
        if (!RTICdrStream_skipFloat (stream)) {
            goto fin; 
        }
        {
            RTICdrUnsignedLong sequence_length;
            if (!RTICdrStream_skipPrimitiveSequence(
                stream,
                &sequence_length,
                RTI_CDR_FLOAT_TYPE)){
                goto fin; 
            }
        }
        {
            RTICdrUnsignedLong sequence_length;
            if (!RTICdrStream_skipPrimitiveSequence(
                stream,
                &sequence_length,
                RTI_CDR_FLOAT_TYPE)){
                goto fin; 
            }
        }
    }

    done = RTI_TRUE;
  fin:
    if (done != RTI_TRUE && 
    RTICdrStream_getRemainder(stream) >=
    RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
        return RTI_FALSE;   
    }
    if(skip_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

unsigned int 
laser_Scan_msg_cPlugin_get_serialized_sample_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    if (endpoint_data) {} /* To avoid warnings */
    if (include_encapsulation) {}
    if (encapsulation_id) {}
    if (current_alignment) {}

    if (overflow != NULL) {
        *overflow = RTI_TRUE;
    }

    return RTI_CDR_MAX_SERIALIZED_SIZE;

}

unsigned int 
laser_Scan_msg_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{
    unsigned int size;
    RTIBool overflow = RTI_FALSE;

    size = laser_Scan_msg_cPlugin_get_serialized_sample_max_size_ex(
        endpoint_data,&overflow,include_encapsulation,encapsulation_id,current_alignment);

    if (overflow) {
        size = RTI_CDR_MAX_SERIALIZED_SIZE;
    }

    return size;
}

unsigned int 
laser_Scan_msg_cPlugin_get_serialized_sample_min_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */ 

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment +=Header_cPlugin_get_serialized_sample_min_size(
        endpoint_data,RTI_FALSE,encapsulation_id,current_alignment);
    current_alignment +=World_Pose_cPlugin_get_serialized_sample_min_size(
        endpoint_data,RTI_FALSE,encapsulation_id,current_alignment);
    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
    current_alignment +=RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
    current_alignment +=    RTICdrType_getPrimitiveSequenceMaxSizeSerialized(
        current_alignment,0, RTI_CDR_FLOAT_TYPE);
    current_alignment +=    RTICdrType_getPrimitiveSequenceMaxSizeSerialized(
        current_alignment,0, RTI_CDR_FLOAT_TYPE);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return  current_alignment - initial_alignment;
}

/* Returns the size of the sample in its serialized form (in bytes).
* It can also be an estimation in excess of the real buffer needed 
* during a call to the serialize() function.
* The value reported does not have to include the space for the
* encapsulation flags.
*/
unsigned int
laser_Scan_msg_cPlugin_get_serialized_sample_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment,
    const laser_Scan_msg_c * sample) 
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;
    struct PRESTypePluginDefaultEndpointData epd;   

    if (sample==NULL) {
        return 0;
    }
    if (endpoint_data == NULL) {
        endpoint_data = (PRESTypePluginEndpointData) &epd;
        PRESTypePluginDefaultEndpointData_setBaseAlignment(
            endpoint_data,
            current_alignment);        
    }

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
        PRESTypePluginDefaultEndpointData_setBaseAlignment(
            endpoint_data,
            current_alignment);
    }

    current_alignment += Header_cPlugin_get_serialized_sample_size(
        endpoint_data,RTI_FALSE, encapsulation_id,
        current_alignment, (const Header_c*) &sample->header);

    current_alignment += World_Pose_cPlugin_get_serialized_sample_size(
        endpoint_data,RTI_FALSE, encapsulation_id,
        current_alignment, (const World_Pose_c*) &sample->world_pose);

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getLongMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment));

    current_alignment += RTICdrType_getPrimitiveSequenceSerializedSize(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment), 
            DDS_FloatSeq_get_length(&sample->ranges),
            RTI_CDR_FLOAT_TYPE);

    current_alignment += RTICdrType_getPrimitiveSequenceSerializedSize(
        PRESTypePluginDefaultEndpointData_getAlignment(
            endpoint_data, current_alignment), 
            DDS_FloatSeq_get_length(&sample->intensities),
            RTI_CDR_FLOAT_TYPE);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return current_alignment - initial_alignment;
}

/* --------------------------------------------------------------------------------------
Key Management functions:
* -------------------------------------------------------------------------------------- */

PRESTypePluginKeyKind 
laser_Scan_msg_cPlugin_get_key_kind(void)
{
    return PRES_TYPEPLUGIN_NO_KEY;
}

RTIBool 
laser_Scan_msg_cPlugin_serialize_key(
    PRESTypePluginEndpointData endpoint_data,
    const laser_Scan_msg_c *sample, 
    struct RTICdrStream *stream,    
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_key,
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    if(serialize_encapsulation) {
        if (!RTICdrStream_serializeAndSetCdrEncapsulation(stream , encapsulation_id)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }

    if(serialize_key) {

        if (!laser_Scan_msg_cPlugin_serialize(
            endpoint_data,
            sample,
            stream,
            RTI_FALSE, encapsulation_id,
            RTI_TRUE,
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

    }

    if(serialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

RTIBool laser_Scan_msg_cPlugin_deserialize_key_sample(
    PRESTypePluginEndpointData endpoint_data,
    laser_Scan_msg_c *sample, 
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos)
{

    char * position = NULL;

    if (endpoint_data) {} /* To avoid warnings */
    if (endpoint_plugin_qos) {} /* To avoid warnings */

    if(deserialize_encapsulation) {

        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);
    }
    if (deserialize_key) {

        if (!laser_Scan_msg_cPlugin_deserialize_sample(
            endpoint_data, sample, stream, 
            RTI_FALSE, RTI_TRUE, 
            endpoint_plugin_qos)) {
            return RTI_FALSE;
        }
    }

    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;

}

RTIBool laser_Scan_msg_cPlugin_deserialize_key(
    PRESTypePluginEndpointData endpoint_data,
    laser_Scan_msg_c **sample, 
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos)
{
    RTIBool result;
    if (drop_sample) {} /* To avoid warnings */
    stream->_xTypesState.unassignable = RTI_FALSE;
    result= laser_Scan_msg_cPlugin_deserialize_key_sample(
        endpoint_data, (sample != NULL)?*sample:NULL, stream,
        deserialize_encapsulation, deserialize_key, endpoint_plugin_qos);
    if (result) {
        if (stream->_xTypesState.unassignable) {
            result = RTI_FALSE;
        }
    }

    return result;    

}

unsigned int
laser_Scan_msg_cPlugin_get_serialized_key_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;

    if (endpoint_data) {} /* To avoid warnings */
    if (overflow) {} /* To avoid warnings */

    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }
        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;
    }

    current_alignment += laser_Scan_msg_cPlugin_get_serialized_sample_max_size_ex(
        endpoint_data, overflow,RTI_FALSE, encapsulation_id, current_alignment);

    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }
    return current_alignment - initial_alignment;
}

unsigned int
laser_Scan_msg_cPlugin_get_serialized_key_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{
    unsigned int size;
    RTIBool overflow = RTI_FALSE;

    size = laser_Scan_msg_cPlugin_get_serialized_key_max_size_ex(
        endpoint_data,&overflow,include_encapsulation,encapsulation_id,current_alignment);

    if (overflow) {
        size = RTI_CDR_MAX_SERIALIZED_SIZE;
    }

    return size;
}

RTIBool 
laser_Scan_msg_cPlugin_serialized_sample_to_key(
    PRESTypePluginEndpointData endpoint_data,
    laser_Scan_msg_c *sample,
    struct RTICdrStream *stream, 
    RTIBool deserialize_encapsulation,  
    RTIBool deserialize_key, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;

    RTIBool done = RTI_FALSE;
    RTIBool error = RTI_FALSE;

    if (stream == NULL) {
        error = RTI_TRUE;
        goto fin;
    }
    if(deserialize_encapsulation) {
        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }
        position = RTICdrStream_resetAlignment(stream);
    }

    if (deserialize_key) {

        if (!laser_Scan_msg_cPlugin_deserialize_sample(
            endpoint_data, sample, stream, RTI_FALSE, 
            RTI_TRUE, endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

    }

    done = RTI_TRUE;
  fin:
    if(!error) {
        if (done != RTI_TRUE && 
        RTICdrStream_getRemainder(stream) >=
        RTI_CDR_PARAMETER_HEADER_ALIGNMENT) {
            return RTI_FALSE;   
        }
    } else {
        return RTI_FALSE;
    }       

    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }

    return RTI_TRUE;
}

/* ------------------------------------------------------------------------
* Plug-in Installation Methods
* ------------------------------------------------------------------------ */
struct PRESTypePlugin *laser_Scan_msg_cPlugin_new(void) 
{ 
    struct PRESTypePlugin *plugin = NULL;
    const struct PRESTypePluginVersion PLUGIN_VERSION = 
    PRES_TYPE_PLUGIN_VERSION_2_0;

    RTIOsapiHeap_allocateStructure(
        &plugin, struct PRESTypePlugin);

    if (plugin == NULL) {
        return NULL;
    }

    plugin->version = PLUGIN_VERSION;

    /* set up parent's function pointers */
    plugin->onParticipantAttached =
    (PRESTypePluginOnParticipantAttachedCallback)
    laser_Scan_msg_cPlugin_on_participant_attached;
    plugin->onParticipantDetached =
    (PRESTypePluginOnParticipantDetachedCallback)
    laser_Scan_msg_cPlugin_on_participant_detached;
    plugin->onEndpointAttached =
    (PRESTypePluginOnEndpointAttachedCallback)
    laser_Scan_msg_cPlugin_on_endpoint_attached;
    plugin->onEndpointDetached =
    (PRESTypePluginOnEndpointDetachedCallback)
    laser_Scan_msg_cPlugin_on_endpoint_detached;

    plugin->copySampleFnc =
    (PRESTypePluginCopySampleFunction)
    laser_Scan_msg_cPlugin_copy_sample;
    plugin->createSampleFnc =
    (PRESTypePluginCreateSampleFunction)
    laser_Scan_msg_cPlugin_create_sample;
    plugin->destroySampleFnc =
    (PRESTypePluginDestroySampleFunction)
    laser_Scan_msg_cPlugin_destroy_sample;
    plugin->finalizeOptionalMembersFnc =
    (PRESTypePluginFinalizeOptionalMembersFunction)
    laser_Scan_msg_c_finalize_optional_members;

    plugin->serializeFnc =
    (PRESTypePluginSerializeFunction)
    laser_Scan_msg_cPlugin_serialize;
    plugin->deserializeFnc =
    (PRESTypePluginDeserializeFunction)
    laser_Scan_msg_cPlugin_deserialize;
    plugin->getSerializedSampleMaxSizeFnc =
    (PRESTypePluginGetSerializedSampleMaxSizeFunction)
    laser_Scan_msg_cPlugin_get_serialized_sample_max_size;
    plugin->getSerializedSampleMinSizeFnc =
    (PRESTypePluginGetSerializedSampleMinSizeFunction)
    laser_Scan_msg_cPlugin_get_serialized_sample_min_size;

    plugin->getSampleFnc =
    (PRESTypePluginGetSampleFunction)
    laser_Scan_msg_cPlugin_get_sample;
    plugin->returnSampleFnc =
    (PRESTypePluginReturnSampleFunction)
    laser_Scan_msg_cPlugin_return_sample;

    plugin->getKeyKindFnc =
    (PRESTypePluginGetKeyKindFunction)
    laser_Scan_msg_cPlugin_get_key_kind;

    /* These functions are only used for keyed types. As this is not a keyed
    type they are all set to NULL
    */
    plugin->serializeKeyFnc = NULL ;    
    plugin->deserializeKeyFnc = NULL;  
    plugin->getKeyFnc = NULL;
    plugin->returnKeyFnc = NULL;
    plugin->instanceToKeyFnc = NULL;
    plugin->keyToInstanceFnc = NULL;
    plugin->getSerializedKeyMaxSizeFnc = NULL;
    plugin->instanceToKeyHashFnc = NULL;
    plugin->serializedSampleToKeyHashFnc = NULL;
    plugin->serializedKeyToKeyHashFnc = NULL;    
    plugin->typeCode =  (struct RTICdrTypeCode *)laser_Scan_msg_c_get_typecode();

    plugin->languageKind = PRES_TYPEPLUGIN_CPP_LANG;

    /* Serialized buffer */
    plugin->getBuffer = 
    (PRESTypePluginGetBufferFunction)
    laser_Scan_msg_cPlugin_get_buffer;
    plugin->returnBuffer = 
    (PRESTypePluginReturnBufferFunction)
    laser_Scan_msg_cPlugin_return_buffer;
    plugin->getSerializedSampleSizeFnc =
    (PRESTypePluginGetSerializedSampleSizeFunction)
    laser_Scan_msg_cPlugin_get_serialized_sample_size;

    plugin->endpointTypeName = laser_Scan_msg_cTYPENAME;

    return plugin;
}

void
laser_Scan_msg_cPlugin_delete(struct PRESTypePlugin *plugin)
{
    RTIOsapiHeap_freeStructure(plugin);
} 
#undef RTI_CDR_CURRENT_SUBMODULE 
