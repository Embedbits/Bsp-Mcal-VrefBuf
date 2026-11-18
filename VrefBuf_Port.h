/**
 * \author Mr.Nobody
 * \file VrefBuf_Port.h
 * \ingroup VrefBuf
 * \brief VrefBuf module public functionality
 *
 * This file contains all available public functionality, any other files shall
 * not used outside of the module.
 *
 */

#ifndef VREFBUF_VREFBUF_PORT_H
#define VREFBUF_VREFBUF_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "VrefBuf_Types.h"                  /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

vrefBuf_ModuleVersion_t     VrefBuf_Get_ModuleVersion       ( void );

vrefBuf_RequestState_t      VrefBuf_Init                    ( const vrefBuf_Config_t * const vrefBufConfig );
vrefBuf_RequestState_t      VrefBuf_Deinit                  ( void );
void                        VrefBuf_Task                    ( void );

vrefBuf_RequestState_t      VrefBuf_Get_DefaultConfig       ( vrefBuf_Config_t * const vrefBufConfig );

/* --------------------------------- Control -------------------------------- */

vrefBuf_RequestState_t      VrefBuf_Set_Mode                ( vrefBuf_Mode_t mode );
vrefBuf_RequestState_t      VrefBuf_Get_Mode                ( vrefBuf_Mode_t * const vrefBufMode );

vrefBuf_RequestState_t      VrefBuf_Set_Voltage             ( vrefBuf_Voltage_t voltage );
vrefBuf_RequestState_t      VrefBuf_Get_Voltage             ( vrefBuf_Voltage_t * const vrefBufVoltage );

vrefBuf_RequestState_t      VrefBuf_Get_State               ( vrefBuf_State_t * const vrefBufState );

/* -------------------------------- Trimming -------------------------------- */

vrefBuf_RequestState_t      VrefBuf_Set_Trimming            ( vrefBuf_Trimming_t trimming );
vrefBuf_RequestState_t      VrefBuf_Get_Trimming            ( vrefBuf_Trimming_t * const vrefBufTrimming );

#ifdef __cplusplus
}
#endif

#endif /* VREFBUF_VREFBUF_PORT_H */
