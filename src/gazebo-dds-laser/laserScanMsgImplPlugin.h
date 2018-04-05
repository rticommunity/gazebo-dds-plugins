

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from laserScanMsgImpl.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef laserScanMsgImplPlugin_1972817541_h
#define laserScanMsgImplPlugin_1972817541_h

#include "laserScanMsgImpl.h"

struct RTICdrStream;

#ifndef pres_typePlugin_h
#include "pres/pres_typePlugin.h"
#endif

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif

#define Time_cPlugin_get_sample PRESTypePluginDefaultEndpointData_getSample 
#define Time_cPlugin_get_buffer PRESTypePluginDefaultEndpointData_getBuffer 
#define Time_cPlugin_return_buffer PRESTypePluginDefaultEndpointData_returnBuffer 

#define Time_cPlugin_create_sample PRESTypePluginDefaultEndpointData_createSample 
#define Time_cPlugin_destroy_sample PRESTypePluginDefaultEndpointData_deleteSample 

/* --------------------------------------------------------------------------------------
Support functions:
* -------------------------------------------------------------------------------------- */

NDDSUSERDllExport extern Time_c*
Time_cPluginSupport_create_data_w_params(
    const struct DDS_TypeAllocationParams_t * alloc_params);

NDDSUSERDllExport extern Time_c*
Time_cPluginSupport_create_data_ex(RTIBool allocate_pointers);

NDDSUSERDllExport extern Time_c*
Time_cPluginSupport_create_data(void);

NDDSUSERDllExport extern RTIBool 
Time_cPluginSupport_copy_data(
    Time_c *out,
    const Time_c *in);

NDDSUSERDllExport extern void 
Time_cPluginSupport_destroy_data_w_params(
    Time_c *sample,
    const struct DDS_TypeDeallocationParams_t * dealloc_params);

NDDSUSERDllExport extern void 
Time_cPluginSupport_destroy_data_ex(
    Time_c *sample,RTIBool deallocate_pointers);

NDDSUSERDllExport extern void 
Time_cPluginSupport_destroy_data(
    Time_c *sample);

NDDSUSERDllExport extern void 
Time_cPluginSupport_print_data(
    const Time_c *sample,
    const char *desc,
    unsigned int indent);

/* ----------------------------------------------------------------------------
Callback functions:
* ---------------------------------------------------------------------------- */

NDDSUSERDllExport extern PRESTypePluginParticipantData 
Time_cPlugin_on_participant_attached(
    void *registration_data, 
    const struct PRESTypePluginParticipantInfo *participant_info,
    RTIBool top_level_registration, 
    void *container_plugin_context,
    RTICdrTypeCode *typeCode);

NDDSUSERDllExport extern void 
Time_cPlugin_on_participant_detached(
    PRESTypePluginParticipantData participant_data);

NDDSUSERDllExport extern PRESTypePluginEndpointData 
Time_cPlugin_on_endpoint_attached(
    PRESTypePluginParticipantData participant_data,
    const struct PRESTypePluginEndpointInfo *endpoint_info,
    RTIBool top_level_registration, 
    void *container_plugin_context);

NDDSUSERDllExport extern void 
Time_cPlugin_on_endpoint_detached(
    PRESTypePluginEndpointData endpoint_data);

NDDSUSERDllExport extern void    
Time_cPlugin_return_sample(
    PRESTypePluginEndpointData endpoint_data,
    Time_c *sample,
    void *handle);    

NDDSUSERDllExport extern RTIBool 
Time_cPlugin_copy_sample(
    PRESTypePluginEndpointData endpoint_data,
    Time_c *out,
    const Time_c *in);

/* ----------------------------------------------------------------------------
(De)Serialize functions:
* ------------------------------------------------------------------------- */

NDDSUSERDllExport extern RTIBool 
Time_cPlugin_serialize(
    PRESTypePluginEndpointData endpoint_data,
    const Time_c *sample,
    struct RTICdrStream *stream, 
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
Time_cPlugin_deserialize_sample(
    PRESTypePluginEndpointData endpoint_data,
    Time_c *sample, 
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool
Time_cPlugin_serialize_to_cdr_buffer(
    char * buffer,
    unsigned int * length,
    const Time_c *sample); 

NDDSUSERDllExport extern RTIBool 
Time_cPlugin_deserialize(
    PRESTypePluginEndpointData endpoint_data,
    Time_c **sample, 
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool
Time_cPlugin_deserialize_from_cdr_buffer(
    Time_c *sample,
    const char * buffer,
    unsigned int length);    
NDDSUSERDllExport extern DDS_ReturnCode_t
Time_cPlugin_data_to_string(
    const Time_c *sample,
    char *str,
    DDS_UnsignedLong *str_size, 
    const struct DDS_PrintFormatProperty *property);    

NDDSUSERDllExport extern RTIBool
Time_cPlugin_skip(
    PRESTypePluginEndpointData endpoint_data,
    struct RTICdrStream *stream, 
    RTIBool skip_encapsulation,  
    RTIBool skip_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern unsigned int 
Time_cPlugin_get_serialized_sample_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);    

NDDSUSERDllExport extern unsigned int 
Time_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int 
Time_cPlugin_get_serialized_sample_min_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int
Time_cPlugin_get_serialized_sample_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment,
    const Time_c * sample);

/* --------------------------------------------------------------------------------------
Key Management functions:
* -------------------------------------------------------------------------------------- */
NDDSUSERDllExport extern PRESTypePluginKeyKind 
Time_cPlugin_get_key_kind(void);

NDDSUSERDllExport extern unsigned int 
Time_cPlugin_get_serialized_key_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int 
Time_cPlugin_get_serialized_key_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern RTIBool 
Time_cPlugin_serialize_key(
    PRESTypePluginEndpointData endpoint_data,
    const Time_c *sample,
    struct RTICdrStream *stream,
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
Time_cPlugin_deserialize_key_sample(
    PRESTypePluginEndpointData endpoint_data,
    Time_c * sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
Time_cPlugin_deserialize_key(
    PRESTypePluginEndpointData endpoint_data,
    Time_c ** sample,
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool
Time_cPlugin_serialized_sample_to_key(
    PRESTypePluginEndpointData endpoint_data,
    Time_c *sample,
    struct RTICdrStream *stream, 
    RTIBool deserialize_encapsulation,  
    RTIBool deserialize_key, 
    void *endpoint_plugin_qos);

/* Plugin Functions */
NDDSUSERDllExport extern struct PRESTypePlugin*
Time_cPlugin_new(void);

NDDSUSERDllExport extern void
Time_cPlugin_delete(struct PRESTypePlugin *);

#define Header_cPlugin_get_sample PRESTypePluginDefaultEndpointData_getSample 
#define Header_cPlugin_get_buffer PRESTypePluginDefaultEndpointData_getBuffer 
#define Header_cPlugin_return_buffer PRESTypePluginDefaultEndpointData_returnBuffer 

#define Header_cPlugin_create_sample PRESTypePluginDefaultEndpointData_createSample 
#define Header_cPlugin_destroy_sample PRESTypePluginDefaultEndpointData_deleteSample 

/* --------------------------------------------------------------------------------------
Support functions:
* -------------------------------------------------------------------------------------- */

NDDSUSERDllExport extern Header_c*
Header_cPluginSupport_create_data_w_params(
    const struct DDS_TypeAllocationParams_t * alloc_params);

NDDSUSERDllExport extern Header_c*
Header_cPluginSupport_create_data_ex(RTIBool allocate_pointers);

NDDSUSERDllExport extern Header_c*
Header_cPluginSupport_create_data(void);

NDDSUSERDllExport extern RTIBool 
Header_cPluginSupport_copy_data(
    Header_c *out,
    const Header_c *in);

NDDSUSERDllExport extern void 
Header_cPluginSupport_destroy_data_w_params(
    Header_c *sample,
    const struct DDS_TypeDeallocationParams_t * dealloc_params);

NDDSUSERDllExport extern void 
Header_cPluginSupport_destroy_data_ex(
    Header_c *sample,RTIBool deallocate_pointers);

NDDSUSERDllExport extern void 
Header_cPluginSupport_destroy_data(
    Header_c *sample);

NDDSUSERDllExport extern void 
Header_cPluginSupport_print_data(
    const Header_c *sample,
    const char *desc,
    unsigned int indent);

/* ----------------------------------------------------------------------------
Callback functions:
* ---------------------------------------------------------------------------- */

NDDSUSERDllExport extern PRESTypePluginParticipantData 
Header_cPlugin_on_participant_attached(
    void *registration_data, 
    const struct PRESTypePluginParticipantInfo *participant_info,
    RTIBool top_level_registration, 
    void *container_plugin_context,
    RTICdrTypeCode *typeCode);

NDDSUSERDllExport extern void 
Header_cPlugin_on_participant_detached(
    PRESTypePluginParticipantData participant_data);

NDDSUSERDllExport extern PRESTypePluginEndpointData 
Header_cPlugin_on_endpoint_attached(
    PRESTypePluginParticipantData participant_data,
    const struct PRESTypePluginEndpointInfo *endpoint_info,
    RTIBool top_level_registration, 
    void *container_plugin_context);

NDDSUSERDllExport extern void 
Header_cPlugin_on_endpoint_detached(
    PRESTypePluginEndpointData endpoint_data);

NDDSUSERDllExport extern void    
Header_cPlugin_return_sample(
    PRESTypePluginEndpointData endpoint_data,
    Header_c *sample,
    void *handle);    

NDDSUSERDllExport extern RTIBool 
Header_cPlugin_copy_sample(
    PRESTypePluginEndpointData endpoint_data,
    Header_c *out,
    const Header_c *in);

/* ----------------------------------------------------------------------------
(De)Serialize functions:
* ------------------------------------------------------------------------- */

NDDSUSERDllExport extern RTIBool 
Header_cPlugin_serialize(
    PRESTypePluginEndpointData endpoint_data,
    const Header_c *sample,
    struct RTICdrStream *stream, 
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
Header_cPlugin_deserialize_sample(
    PRESTypePluginEndpointData endpoint_data,
    Header_c *sample, 
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool
Header_cPlugin_serialize_to_cdr_buffer(
    char * buffer,
    unsigned int * length,
    const Header_c *sample); 

NDDSUSERDllExport extern RTIBool 
Header_cPlugin_deserialize(
    PRESTypePluginEndpointData endpoint_data,
    Header_c **sample, 
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool
Header_cPlugin_deserialize_from_cdr_buffer(
    Header_c *sample,
    const char * buffer,
    unsigned int length);    
NDDSUSERDllExport extern DDS_ReturnCode_t
Header_cPlugin_data_to_string(
    const Header_c *sample,
    char *str,
    DDS_UnsignedLong *str_size, 
    const struct DDS_PrintFormatProperty *property);    

NDDSUSERDllExport extern RTIBool
Header_cPlugin_skip(
    PRESTypePluginEndpointData endpoint_data,
    struct RTICdrStream *stream, 
    RTIBool skip_encapsulation,  
    RTIBool skip_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern unsigned int 
Header_cPlugin_get_serialized_sample_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);    

NDDSUSERDllExport extern unsigned int 
Header_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int 
Header_cPlugin_get_serialized_sample_min_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int
Header_cPlugin_get_serialized_sample_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment,
    const Header_c * sample);

/* --------------------------------------------------------------------------------------
Key Management functions:
* -------------------------------------------------------------------------------------- */
NDDSUSERDllExport extern PRESTypePluginKeyKind 
Header_cPlugin_get_key_kind(void);

NDDSUSERDllExport extern unsigned int 
Header_cPlugin_get_serialized_key_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int 
Header_cPlugin_get_serialized_key_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern RTIBool 
Header_cPlugin_serialize_key(
    PRESTypePluginEndpointData endpoint_data,
    const Header_c *sample,
    struct RTICdrStream *stream,
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
Header_cPlugin_deserialize_key_sample(
    PRESTypePluginEndpointData endpoint_data,
    Header_c * sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
Header_cPlugin_deserialize_key(
    PRESTypePluginEndpointData endpoint_data,
    Header_c ** sample,
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool
Header_cPlugin_serialized_sample_to_key(
    PRESTypePluginEndpointData endpoint_data,
    Header_c *sample,
    struct RTICdrStream *stream, 
    RTIBool deserialize_encapsulation,  
    RTIBool deserialize_key, 
    void *endpoint_plugin_qos);

/* Plugin Functions */
NDDSUSERDllExport extern struct PRESTypePlugin*
Header_cPlugin_new(void);

NDDSUSERDllExport extern void
Header_cPlugin_delete(struct PRESTypePlugin *);

#define laser_Scan_msg_cPlugin_get_sample PRESTypePluginDefaultEndpointData_getSample 
#define laser_Scan_msg_cPlugin_get_buffer PRESTypePluginDefaultEndpointData_getBuffer 
#define laser_Scan_msg_cPlugin_return_buffer PRESTypePluginDefaultEndpointData_returnBuffer 

#define laser_Scan_msg_cPlugin_create_sample PRESTypePluginDefaultEndpointData_createSample 
#define laser_Scan_msg_cPlugin_destroy_sample PRESTypePluginDefaultEndpointData_deleteSample 

/* --------------------------------------------------------------------------------------
Support functions:
* -------------------------------------------------------------------------------------- */

NDDSUSERDllExport extern laser_Scan_msg_c*
laser_Scan_msg_cPluginSupport_create_data_w_params(
    const struct DDS_TypeAllocationParams_t * alloc_params);

NDDSUSERDllExport extern laser_Scan_msg_c*
laser_Scan_msg_cPluginSupport_create_data_ex(RTIBool allocate_pointers);

NDDSUSERDllExport extern laser_Scan_msg_c*
laser_Scan_msg_cPluginSupport_create_data(void);

NDDSUSERDllExport extern RTIBool 
laser_Scan_msg_cPluginSupport_copy_data(
    laser_Scan_msg_c *out,
    const laser_Scan_msg_c *in);

NDDSUSERDllExport extern void 
laser_Scan_msg_cPluginSupport_destroy_data_w_params(
    laser_Scan_msg_c *sample,
    const struct DDS_TypeDeallocationParams_t * dealloc_params);

NDDSUSERDllExport extern void 
laser_Scan_msg_cPluginSupport_destroy_data_ex(
    laser_Scan_msg_c *sample,RTIBool deallocate_pointers);

NDDSUSERDllExport extern void 
laser_Scan_msg_cPluginSupport_destroy_data(
    laser_Scan_msg_c *sample);

NDDSUSERDllExport extern void 
laser_Scan_msg_cPluginSupport_print_data(
    const laser_Scan_msg_c *sample,
    const char *desc,
    unsigned int indent);

/* ----------------------------------------------------------------------------
Callback functions:
* ---------------------------------------------------------------------------- */

NDDSUSERDllExport extern PRESTypePluginParticipantData 
laser_Scan_msg_cPlugin_on_participant_attached(
    void *registration_data, 
    const struct PRESTypePluginParticipantInfo *participant_info,
    RTIBool top_level_registration, 
    void *container_plugin_context,
    RTICdrTypeCode *typeCode);

NDDSUSERDllExport extern void 
laser_Scan_msg_cPlugin_on_participant_detached(
    PRESTypePluginParticipantData participant_data);

NDDSUSERDllExport extern PRESTypePluginEndpointData 
laser_Scan_msg_cPlugin_on_endpoint_attached(
    PRESTypePluginParticipantData participant_data,
    const struct PRESTypePluginEndpointInfo *endpoint_info,
    RTIBool top_level_registration, 
    void *container_plugin_context);

NDDSUSERDllExport extern void 
laser_Scan_msg_cPlugin_on_endpoint_detached(
    PRESTypePluginEndpointData endpoint_data);

NDDSUSERDllExport extern void    
laser_Scan_msg_cPlugin_return_sample(
    PRESTypePluginEndpointData endpoint_data,
    laser_Scan_msg_c *sample,
    void *handle);    

NDDSUSERDllExport extern RTIBool 
laser_Scan_msg_cPlugin_copy_sample(
    PRESTypePluginEndpointData endpoint_data,
    laser_Scan_msg_c *out,
    const laser_Scan_msg_c *in);

/* ----------------------------------------------------------------------------
(De)Serialize functions:
* ------------------------------------------------------------------------- */

NDDSUSERDllExport extern RTIBool 
laser_Scan_msg_cPlugin_serialize(
    PRESTypePluginEndpointData endpoint_data,
    const laser_Scan_msg_c *sample,
    struct RTICdrStream *stream, 
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
laser_Scan_msg_cPlugin_deserialize_sample(
    PRESTypePluginEndpointData endpoint_data,
    laser_Scan_msg_c *sample, 
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool
laser_Scan_msg_cPlugin_serialize_to_cdr_buffer(
    char * buffer,
    unsigned int * length,
    const laser_Scan_msg_c *sample); 

NDDSUSERDllExport extern RTIBool 
laser_Scan_msg_cPlugin_deserialize(
    PRESTypePluginEndpointData endpoint_data,
    laser_Scan_msg_c **sample, 
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool
laser_Scan_msg_cPlugin_deserialize_from_cdr_buffer(
    laser_Scan_msg_c *sample,
    const char * buffer,
    unsigned int length);    
NDDSUSERDllExport extern DDS_ReturnCode_t
laser_Scan_msg_cPlugin_data_to_string(
    const laser_Scan_msg_c *sample,
    char *str,
    DDS_UnsignedLong *str_size, 
    const struct DDS_PrintFormatProperty *property);    

NDDSUSERDllExport extern RTIBool
laser_Scan_msg_cPlugin_skip(
    PRESTypePluginEndpointData endpoint_data,
    struct RTICdrStream *stream, 
    RTIBool skip_encapsulation,  
    RTIBool skip_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern unsigned int 
laser_Scan_msg_cPlugin_get_serialized_sample_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);    

NDDSUSERDllExport extern unsigned int 
laser_Scan_msg_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int 
laser_Scan_msg_cPlugin_get_serialized_sample_min_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int
laser_Scan_msg_cPlugin_get_serialized_sample_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment,
    const laser_Scan_msg_c * sample);

/* --------------------------------------------------------------------------------------
Key Management functions:
* -------------------------------------------------------------------------------------- */
NDDSUSERDllExport extern PRESTypePluginKeyKind 
laser_Scan_msg_cPlugin_get_key_kind(void);

NDDSUSERDllExport extern unsigned int 
laser_Scan_msg_cPlugin_get_serialized_key_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int 
laser_Scan_msg_cPlugin_get_serialized_key_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern RTIBool 
laser_Scan_msg_cPlugin_serialize_key(
    PRESTypePluginEndpointData endpoint_data,
    const laser_Scan_msg_c *sample,
    struct RTICdrStream *stream,
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
laser_Scan_msg_cPlugin_deserialize_key_sample(
    PRESTypePluginEndpointData endpoint_data,
    laser_Scan_msg_c * sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
laser_Scan_msg_cPlugin_deserialize_key(
    PRESTypePluginEndpointData endpoint_data,
    laser_Scan_msg_c ** sample,
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool
laser_Scan_msg_cPlugin_serialized_sample_to_key(
    PRESTypePluginEndpointData endpoint_data,
    laser_Scan_msg_c *sample,
    struct RTICdrStream *stream, 
    RTIBool deserialize_encapsulation,  
    RTIBool deserialize_key, 
    void *endpoint_plugin_qos);

/* Plugin Functions */
NDDSUSERDllExport extern struct PRESTypePlugin*
laser_Scan_msg_cPlugin_new(void);

NDDSUSERDllExport extern void
laser_Scan_msg_cPlugin_delete(struct PRESTypePlugin *);

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

#endif /* laserScanMsgImplPlugin_1972817541_h */

