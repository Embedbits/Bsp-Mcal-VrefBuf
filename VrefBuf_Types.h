/**
 * \author Mr.Nobody
 * \file VrefBuf_Types.h
 * \ingroup VrefBuf
 * \brief VrefBuf module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through Port file. The lists (modes, voltage
 * levels, states) contain exactly the items the hardware of the STM32H5 family
 * offers.
 *
 */

/**
 * \defgroup VrefBuf VrefBuf
 * \brief VrefBuf module - voltage reference buffer (VREFBUF)
 */

#ifndef VREFBUF_VREFBUF_TYPES_H
#define VREFBUF_VREFBUF_TYPES_H
/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define VREFBUF_NULL_PTR                     ( ( void* ) 0u )

/** Trimming value used to signal that the factory trimming is kept (user trimming is not written) */
#define VREFBUF_TRIMMING_UNUSED              ( 0xFFu )

/** Maximal value of the trimming code (6-bit unsigned code of the TRIM field) */
#define VREFBUF_TRIMMING_MAX                 ( 63u )

/* ========================== EXPORTED MACROS =============================== */

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t vrefBuf_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t vrefBuf_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t vrefBuf_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    vrefBuf_MajorVersion_t Major; /**< Major version */
    vrefBuf_MinorVersion_t Minor; /**< Minor version */
    vrefBuf_PatchVersion_t Patch; /**< Patch version */
}   vrefBuf_ModuleVersion_t;


/** Function status enumeration */
typedef enum
{
    VREFBUF_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    VREFBUF_FUNCTION_ACTIVE         /**< Function status is active   */
}   vrefBuf_FunctionState_t;


/** Enumeration used to signal request processing state */
typedef enum
{
    VREFBUF_REQUEST_ERROR = 0u, /**< Processing request failed  */
    VREFBUF_REQUEST_OK          /**< Processing request succeed */
}   vrefBuf_RequestState_t;


/** Flag states enumeration */
typedef enum
{
    VREFBUF_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    VREFBUF_FLAG_ACTIVE         /**< Active flag state   */
}   vrefBuf_FlagState_t;


/** \brief Electrical mode of the voltage reference buffer (pin VREF+) */
typedef enum
{
    VREFBUF_MODE_OFF = 0u,         /**< Buffer off, the pin VREF+ is pulled down to VSSA (no reference is generated) */
    VREFBUF_MODE_BUFFER,           /**< Buffer on, the pin VREF+ is driven by the buffer (internal reference)        */
    VREFBUF_MODE_HIGH_IMPEDANCE,   /**< Buffer off, the pin VREF+ is an input (external reference, reset state)  */
    VREFBUF_MODE_CNT               /**< Number of modes                                                          */
}   vrefBuf_Mode_t;


/** \brief Output voltage levels of the voltage reference buffer (levels of the device) */
typedef enum
{
    VREFBUF_VOLTAGE_2_5V = 0u,     /**< Output voltage 2.5 V   (VREFBUF0, reset level) */
    VREFBUF_VOLTAGE_2_048V,        /**< Output voltage 2.048 V (VREFBUF1)              */
    VREFBUF_VOLTAGE_1_8V,          /**< Output voltage 1.8 V   (VREFBUF2)              */
    VREFBUF_VOLTAGE_CNT            /**< Number of voltage levels                       */
}   vrefBuf_Voltage_t;


/** \brief State of the voltage reference buffer, consumers (Adc, Dac) convert only in the state READY */
typedef enum
{
    VREFBUF_STATE_OFF = 0u,        /**< The buffer is not driving the pin VREF+ (modes OFF and HIGH_IMPEDANCE) */
    VREFBUF_STATE_STARTING,        /**< The buffer is enabled, the output did not reach the requested level    */
    VREFBUF_STATE_READY,           /**< The buffer output reached the requested level                          */
    VREFBUF_STATE_CNT              /**< Number of states                                                       */
}   vrefBuf_State_t;


/** \brief Trimming code of the reference (6-bit unsigned code, \ref VREFBUF_TRIMMING_MAX) */
typedef uint8_t vrefBuf_Trimming_t;


/** \brief Count of the read-backs of the ready flag after which the start-up is reported as failed */
typedef uint32_t vrefBuf_TimeoutCnt_t;


/** \brief Type used to store VREFBUF configuration */
typedef struct
{
    /** Electrical mode of the buffer */
    vrefBuf_Mode_t       Mode;

    /** Output voltage level */
    vrefBuf_Voltage_t    Voltage;

    /** User trimming code (0 ... \ref VREFBUF_TRIMMING_MAX), \ref VREFBUF_TRIMMING_UNUSED keeps the factory
     *  trimming loaded by the hardware for the selected voltage level */
    vrefBuf_Trimming_t   Trimming;

    /** Start-up timeout - count of the read-backs of the ready flag (must not be zero), used by
     *  \ref VrefBuf_Init and by the later changes of the mode and the voltage */
    vrefBuf_TimeoutCnt_t StartTimeout;
}   vrefBuf_Config_t;

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */


#endif /* VREFBUF_VREFBUF_TYPES_H */
