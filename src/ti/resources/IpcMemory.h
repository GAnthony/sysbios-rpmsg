/*
 *  Do not modify this file; it is automatically 
 *  generated and any modifications will be overwritten.
 *
 * @(#) xdc-x20
 */

/*
 * ======== GENERATED SECTIONS ========
 *     
 *     PROLOGUE
 *     INCLUDES
 *     
 *     INTERNAL DEFINITIONS
 *     MODULE-WIDE CONFIGS
 *     FUNCTION DECLARATIONS
 *     SYSTEM FUNCTIONS
 *     
 *     EPILOGUE
 *     STATE STRUCTURES
 *     PREFIX ALIASES
 */


/*
 * ======== PROLOGUE ========
 */

#ifndef ti_resources_IpcMemory__include
#define ti_resources_IpcMemory__include

#ifndef __nested__
#define __nested__
#define ti_resources_IpcMemory__top__
#endif

#ifdef __cplusplus
#define __extern extern "C"
#else
#define __extern extern
#endif

#define ti_resources_IpcMemory___VERS 150


/*
 * ======== INCLUDES ========
 */

#include <xdc/std.h>

#include <xdc/runtime/xdc.h>
#include <xdc/runtime/Types.h>
#include <ti/resources/package/package.defs.h>

#include <xdc/runtime/IModule.h>


/*
 * ======== AUXILIARY DEFINITIONS ========
 */

/* S_SUCCESS */
#define ti_resources_IpcMemory_S_SUCCESS (0)

/* E_NOTFOUND */
#define ti_resources_IpcMemory_E_NOTFOUND (-1)

/* Resource */
typedef xdc_Char __T1_ti_resources_IpcMemory_Resource__name;
typedef xdc_Char __ARRAY1_ti_resources_IpcMemory_Resource__name[48];
typedef __ARRAY1_ti_resources_IpcMemory_Resource__name __TA_ti_resources_IpcMemory_Resource__name;
struct ti_resources_IpcMemory_Resource {
    xdc_UInt32 type;
    xdc_UInt32 da_low;
    xdc_UInt32 da_high;
    xdc_UInt32 pa_low;
    xdc_UInt32 pa_high;
    xdc_UInt32 size;
    xdc_UInt32 reserved;
    __TA_ti_resources_IpcMemory_Resource__name name;
};


/*
 * ======== INTERNAL DEFINITIONS ========
 */


/*
 * ======== MODULE-WIDE CONFIGS ========
 */

/* Module__diagsEnabled */
typedef xdc_Bits32 CT__ti_resources_IpcMemory_Module__diagsEnabled;
__extern __FAR__ const CT__ti_resources_IpcMemory_Module__diagsEnabled ti_resources_IpcMemory_Module__diagsEnabled__C;

/* Module__diagsIncluded */
typedef xdc_Bits32 CT__ti_resources_IpcMemory_Module__diagsIncluded;
__extern __FAR__ const CT__ti_resources_IpcMemory_Module__diagsIncluded ti_resources_IpcMemory_Module__diagsIncluded__C;

/* Module__diagsMask */
typedef xdc_Bits16* CT__ti_resources_IpcMemory_Module__diagsMask;
__extern __FAR__ const CT__ti_resources_IpcMemory_Module__diagsMask ti_resources_IpcMemory_Module__diagsMask__C;

/* Module__gateObj */
typedef xdc_Ptr CT__ti_resources_IpcMemory_Module__gateObj;
__extern __FAR__ const CT__ti_resources_IpcMemory_Module__gateObj ti_resources_IpcMemory_Module__gateObj__C;

/* Module__gatePrms */
typedef xdc_Ptr CT__ti_resources_IpcMemory_Module__gatePrms;
__extern __FAR__ const CT__ti_resources_IpcMemory_Module__gatePrms ti_resources_IpcMemory_Module__gatePrms__C;

/* Module__id */
typedef xdc_runtime_Types_ModuleId CT__ti_resources_IpcMemory_Module__id;
__extern __FAR__ const CT__ti_resources_IpcMemory_Module__id ti_resources_IpcMemory_Module__id__C;

/* Module__loggerDefined */
typedef xdc_Bool CT__ti_resources_IpcMemory_Module__loggerDefined;
__extern __FAR__ const CT__ti_resources_IpcMemory_Module__loggerDefined ti_resources_IpcMemory_Module__loggerDefined__C;

/* Module__loggerObj */
typedef xdc_Ptr CT__ti_resources_IpcMemory_Module__loggerObj;
__extern __FAR__ const CT__ti_resources_IpcMemory_Module__loggerObj ti_resources_IpcMemory_Module__loggerObj__C;

/* Module__loggerFxn0 */
typedef xdc_runtime_Types_LoggerFxn0 CT__ti_resources_IpcMemory_Module__loggerFxn0;
__extern __FAR__ const CT__ti_resources_IpcMemory_Module__loggerFxn0 ti_resources_IpcMemory_Module__loggerFxn0__C;

/* Module__loggerFxn1 */
typedef xdc_runtime_Types_LoggerFxn1 CT__ti_resources_IpcMemory_Module__loggerFxn1;
__extern __FAR__ const CT__ti_resources_IpcMemory_Module__loggerFxn1 ti_resources_IpcMemory_Module__loggerFxn1__C;

/* Module__loggerFxn2 */
typedef xdc_runtime_Types_LoggerFxn2 CT__ti_resources_IpcMemory_Module__loggerFxn2;
__extern __FAR__ const CT__ti_resources_IpcMemory_Module__loggerFxn2 ti_resources_IpcMemory_Module__loggerFxn2__C;

/* Module__loggerFxn4 */
typedef xdc_runtime_Types_LoggerFxn4 CT__ti_resources_IpcMemory_Module__loggerFxn4;
__extern __FAR__ const CT__ti_resources_IpcMemory_Module__loggerFxn4 ti_resources_IpcMemory_Module__loggerFxn4__C;

/* Module__loggerFxn8 */
typedef xdc_runtime_Types_LoggerFxn8 CT__ti_resources_IpcMemory_Module__loggerFxn8;
__extern __FAR__ const CT__ti_resources_IpcMemory_Module__loggerFxn8 ti_resources_IpcMemory_Module__loggerFxn8__C;

/* Module__startupDoneFxn */
typedef xdc_Bool (*CT__ti_resources_IpcMemory_Module__startupDoneFxn)(void);
__extern __FAR__ const CT__ti_resources_IpcMemory_Module__startupDoneFxn ti_resources_IpcMemory_Module__startupDoneFxn__C;

/* Object__count */
typedef xdc_Int CT__ti_resources_IpcMemory_Object__count;
__extern __FAR__ const CT__ti_resources_IpcMemory_Object__count ti_resources_IpcMemory_Object__count__C;

/* Object__heap */
typedef xdc_runtime_IHeap_Handle CT__ti_resources_IpcMemory_Object__heap;
__extern __FAR__ const CT__ti_resources_IpcMemory_Object__heap ti_resources_IpcMemory_Object__heap__C;

/* Object__sizeof */
typedef xdc_SizeT CT__ti_resources_IpcMemory_Object__sizeof;
__extern __FAR__ const CT__ti_resources_IpcMemory_Object__sizeof ti_resources_IpcMemory_Object__sizeof__C;

/* Object__table */
typedef xdc_Ptr CT__ti_resources_IpcMemory_Object__table;
__extern __FAR__ const CT__ti_resources_IpcMemory_Object__table ti_resources_IpcMemory_Object__table__C;


/*
 * ======== FUNCTION DECLARATIONS ========
 */

/* Module_startup */
#define ti_resources_IpcMemory_Module_startup ti_resources_IpcMemory_Module_startup__E
xdc__CODESECT(ti_resources_IpcMemory_Module_startup__E, "ti_resources_IpcMemory_Module_startup")
__extern xdc_Int ti_resources_IpcMemory_Module_startup__E( xdc_Int state );
xdc__CODESECT(ti_resources_IpcMemory_Module_startup__F, "ti_resources_IpcMemory_Module_startup")
__extern xdc_Int ti_resources_IpcMemory_Module_startup__F( xdc_Int state );
xdc__CODESECT(ti_resources_IpcMemory_Module_startup__R, "ti_resources_IpcMemory_Module_startup")
__extern xdc_Int ti_resources_IpcMemory_Module_startup__R( xdc_Int state );

/* Module__startupDone__S */
xdc__CODESECT(ti_resources_IpcMemory_Module__startupDone__S, "ti_resources_IpcMemory_Module__startupDone")
__extern xdc_Bool ti_resources_IpcMemory_Module__startupDone__S( void );

/* virtToPhys__E */
#define ti_resources_IpcMemory_virtToPhys ti_resources_IpcMemory_virtToPhys__E
xdc__CODESECT(ti_resources_IpcMemory_virtToPhys__E, "ti_resources_IpcMemory_virtToPhys")
__extern xdc_Int ti_resources_IpcMemory_virtToPhys__E( xdc_UInt32 da, xdc_UInt32* pa );
xdc__CODESECT(ti_resources_IpcMemory_virtToPhys__F, "ti_resources_IpcMemory_virtToPhys")
__extern xdc_Int ti_resources_IpcMemory_virtToPhys__F( xdc_UInt32 da, xdc_UInt32* pa );
__extern xdc_Int ti_resources_IpcMemory_virtToPhys__R( xdc_UInt32 da, xdc_UInt32* pa );

/* physToVirt__E */
#define ti_resources_IpcMemory_physToVirt ti_resources_IpcMemory_physToVirt__E
xdc__CODESECT(ti_resources_IpcMemory_physToVirt__E, "ti_resources_IpcMemory_physToVirt")
__extern xdc_Int ti_resources_IpcMemory_physToVirt__E( xdc_UInt32 pa, xdc_UInt32* da );
xdc__CODESECT(ti_resources_IpcMemory_physToVirt__F, "ti_resources_IpcMemory_physToVirt")
__extern xdc_Int ti_resources_IpcMemory_physToVirt__F( xdc_UInt32 pa, xdc_UInt32* da );
__extern xdc_Int ti_resources_IpcMemory_physToVirt__R( xdc_UInt32 pa, xdc_UInt32* da );

/* init__I */
#define ti_resources_IpcMemory_init ti_resources_IpcMemory_init__I
xdc__CODESECT(ti_resources_IpcMemory_init__I, "ti_resources_IpcMemory_init")
__extern xdc_Void ti_resources_IpcMemory_init__I( void );

/* getEntry__I */
#define ti_resources_IpcMemory_getEntry ti_resources_IpcMemory_getEntry__I
xdc__CODESECT(ti_resources_IpcMemory_getEntry__I, "ti_resources_IpcMemory_getEntry")
__extern ti_resources_IpcMemory_Resource* ti_resources_IpcMemory_getEntry__I( xdc_UInt entry );


/*
 * ======== SYSTEM FUNCTIONS ========
 */

/* Module_startupDone */
#define ti_resources_IpcMemory_Module_startupDone() ti_resources_IpcMemory_Module__startupDone__S()

/* Object_heap */
#define ti_resources_IpcMemory_Object_heap() ti_resources_IpcMemory_Object__heap__C

/* Module_heap */
#define ti_resources_IpcMemory_Module_heap() ti_resources_IpcMemory_Object__heap__C

/* Module_id */
static inline CT__ti_resources_IpcMemory_Module__id ti_resources_IpcMemory_Module_id( void ) 
{
    return ti_resources_IpcMemory_Module__id__C;
}

/* Module_hasMask */
static inline xdc_Bool ti_resources_IpcMemory_Module_hasMask( void ) 
{
    return ti_resources_IpcMemory_Module__diagsMask__C != NULL;
}

/* Module_getMask */
static inline xdc_Bits16 ti_resources_IpcMemory_Module_getMask( void ) 
{
    return ti_resources_IpcMemory_Module__diagsMask__C != NULL ? *ti_resources_IpcMemory_Module__diagsMask__C : 0;
}

/* Module_setMask */
static inline xdc_Void ti_resources_IpcMemory_Module_setMask( xdc_Bits16 mask ) 
{
    if (ti_resources_IpcMemory_Module__diagsMask__C != NULL) *ti_resources_IpcMemory_Module__diagsMask__C = mask;
}


/*
 * ======== EPILOGUE ========
 */

#ifdef ti_resources_IpcMemory__top__
#undef __nested__
#endif

#endif /* ti_resources_IpcMemory__include */


/*
 * ======== STATE STRUCTURES ========
 */

#if defined(__config__) || (!defined(__nested__) && defined(ti_resources_IpcMemory__internalaccess))

#ifndef ti_resources_IpcMemory__include_state
#define ti_resources_IpcMemory__include_state

/* Module_State */
struct ti_resources_IpcMemory_Module_State {
    xdc_UInt32* pSize;
    ti_resources_IpcMemory_Resource* pTable;
};

/* Module__state__V */
extern struct ti_resources_IpcMemory_Module_State__ ti_resources_IpcMemory_Module__state__V;

#endif /* ti_resources_IpcMemory__include_state */

#endif


/*
 * ======== PREFIX ALIASES ========
 */

#if !defined(__nested__) && !defined(ti_resources_IpcMemory__nolocalnames)

#ifndef ti_resources_IpcMemory__localnames__done
#define ti_resources_IpcMemory__localnames__done

/* module prefix */
#define IpcMemory_S_SUCCESS ti_resources_IpcMemory_S_SUCCESS
#define IpcMemory_E_NOTFOUND ti_resources_IpcMemory_E_NOTFOUND
#define IpcMemory_Resource ti_resources_IpcMemory_Resource
#define IpcMemory_Module_State ti_resources_IpcMemory_Module_State
#define IpcMemory_virtToPhys ti_resources_IpcMemory_virtToPhys
#define IpcMemory_physToVirt ti_resources_IpcMemory_physToVirt
#define IpcMemory_Module_name ti_resources_IpcMemory_Module_name
#define IpcMemory_Module_id ti_resources_IpcMemory_Module_id
#define IpcMemory_Module_startup ti_resources_IpcMemory_Module_startup
#define IpcMemory_Module_startupDone ti_resources_IpcMemory_Module_startupDone
#define IpcMemory_Module_hasMask ti_resources_IpcMemory_Module_hasMask
#define IpcMemory_Module_getMask ti_resources_IpcMemory_Module_getMask
#define IpcMemory_Module_setMask ti_resources_IpcMemory_Module_setMask
#define IpcMemory_Object_heap ti_resources_IpcMemory_Object_heap
#define IpcMemory_Module_heap ti_resources_IpcMemory_Module_heap

#endif /* ti_resources_IpcMemory__localnames__done */
#endif
