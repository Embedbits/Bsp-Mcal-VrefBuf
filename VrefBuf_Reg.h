/**
 * \author Mr.Nobody
 * \file VrefBuf_Reg.h
 * \ingroup VrefBuf
 * \brief VrefBuf module Reg component header file.
 *
 * This component contains functionality for register operations. It is private
 * to the module (the register access is not shared with other modules).
 *
 */

#ifndef VREFBUF_REG_VREFBUF_REG_H
#define VREFBUF_REG_VREFBUF_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "VrefBuf_Types.h"                  /* Module types definition        */
#include "Stm32.h"                          /* VREFBUF registers definition   */
/* ============================= TYPEDEFS =================================== */

/** \brief List of VREFBUF registers (empty list on the lines without VREFBUF) */
typedef enum
{
#if defined(VREFBUF)
    VREFBUF_REG_CSR = 0u, /**< VREFBUF control and status register     */
    VREFBUF_REG_CCR     , /**< VREFBUF calibration control register    */
#endif
    VREFBUF_REG_CNT       /**< Count of available VREFBUF registers    */
}   vrefBuf_RegId_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

#if defined(VREFBUF)

/** Mask of the bits which select the electrical mode (ENVR and HIZ) */
#define VREFBUF_REG_MODE_MASK               ( VREFBUF_CSR_ENVR | VREFBUF_CSR_HIZ )

/** Mode bits: buffer off, the pin VREF+ is pulled down to VSSA (ENVR = 0, HIZ = 0) */
#define VREFBUF_REG_MODE_OFF                ( 0u )

/** Mode bits: buffer on, the pin VREF+ is connected to the buffer output (ENVR = 1, HIZ = 0) */
#define VREFBUF_REG_MODE_BUFFER             ( VREFBUF_CSR_ENVR )

/** Mode bits: buffer off, the pin VREF+ is an input - external voltage reference mode (ENVR = 0, HIZ = 1) */
#define VREFBUF_REG_MODE_HIGH_IMPEDANCE     ( VREFBUF_CSR_HIZ )

/** Voltage field bits: 2.5 V (VRS = 000) */
#define VREFBUF_REG_VOLTAGE_2_5V            ( VREFBUF_CSR_VRS_OUT1 )

/** Voltage field bits: 2.048 V (VRS = 001) */
#define VREFBUF_REG_VOLTAGE_2_048V          ( VREFBUF_CSR_VRS_OUT2 )

/** Voltage field bits: 1.8 V (VRS = 010) */
#define VREFBUF_REG_VOLTAGE_1_8V            ( VREFBUF_CSR_VRS_OUT3 )

#if !defined(STM32H7RS)
/** Voltage field bits: 1.5 V (VRS = 011, STM32H7 lines; reserved on STM32H7R / STM32H7S) */
#define VREFBUF_REG_VOLTAGE_1_5V            ( VREFBUF_CSR_VRS_OUT4 )

/** Highest value of the voltage field which is a level of the line, the higher values are reserved */
#define VREFBUF_REG_VOLTAGE_LAST            ( VREFBUF_REG_VOLTAGE_1_5V )
#else
/** Highest value of the voltage field which is a level of the line, the higher values are reserved */
#define VREFBUF_REG_VOLTAGE_LAST            ( VREFBUF_REG_VOLTAGE_1_8V )
#endif /* STM32H7RS */

#endif /* VREFBUF */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* ---------------------------- Generic accessors --------------------------- */

void                        VrefBuf_Set_RegBit          ( vrefBuf_RegId_t regId, uint32_t bitMask );
void                        VrefBuf_Reset_RegBit        ( vrefBuf_RegId_t regId, uint32_t bitMask );
uint32_t                    VrefBuf_Get_RegBit          ( vrefBuf_RegId_t regId, uint32_t bitMask );

void                        VrefBuf_Set_RegVal          ( vrefBuf_RegId_t regId, uint32_t regMask, uint32_t regValue );
uint32_t                    VrefBuf_Get_RegVal          ( vrefBuf_RegId_t regId, uint32_t regMask );

/* ----------------------------- Register protocol -------------------------- */

vrefBuf_RequestState_t      VrefBuf_Reg_Set_Mode        ( uint32_t modeValue );
vrefBuf_RequestState_t      VrefBuf_Reg_Set_Voltage     ( uint32_t voltageValue, vrefBuf_TimeoutCnt_t timeout );
vrefBuf_RequestState_t      VrefBuf_Reg_Set_Trimming    ( uint32_t trimmingValue );

vrefBuf_FlagState_t         VrefBuf_Reg_Get_ReadyFlag   ( void );
vrefBuf_RequestState_t      VrefBuf_Reg_Wait_Ready      ( vrefBuf_TimeoutCnt_t timeout );

#ifdef __cplusplus
}
#endif

#endif /* VREFBUF_REG_VREFBUF_REG_H */
