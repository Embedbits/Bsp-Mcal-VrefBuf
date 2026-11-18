/**
 * \author Mr.Nobody
 * \file VrefBuf_Reg.c
 * \ingroup VrefBuf
 * \brief VrefBuf module Reg component functionality.
 *
 * Register access of the module: generic accessors (bounds checked register ID)
 * and the register protocols of the voltage reference buffer. The registers are
 * written only here, the masks come from the CMSIS device header. The peripheral
 * exists on every STM32H7 and STM32H7R / STM32H7S line, the component is empty on
 * a line without the CMSIS definition VREFBUF.
 *
 * Properties of the registers, which the protocols respect (RM0433, RM0399, RM0455,
 * RM0468, RM0477 - VREFBUF chapter):
 * - VREFBUF_CSR.VRS can be programmed only while the buffer is disabled (ENVR = 0).
 *   The values 000 ... 011 are levels on the STM32H7 lines, 000 ... 010 on
 *   STM32H7R / STM32H7S, the other values are reserved.
 * - ENVR and HIZ select four modes: buffer off with VREF+ pulled down to VSSA
 *   (0, 0), external voltage reference mode - VREF+ is an input (0, 1, reset
 *   state), internal voltage reference mode - buffer on (1, 0) and hold mode (1, 1).
 * - After enabling the buffer (ENVR = 1, HIZ = 0) the ready flag VRR is set when the
 *   output reached the requested level. In the hold mode the flag keeps its last state.
 * - VREFBUF_CCR.TRIM is loaded with the factory value at reset and at every change of
 *   VRS. A write of TRIM disables the loading until the next reset.
 *
 */
/* ============================== INCLUDES ================================== */
#include "VrefBuf_Reg.h"                    /* Self include                   */
#include "VrefBuf_Types.h"                  /* Module types definitions       */
#include "Stm32.h"                          /* VREFBUF registers definition   */
/* ============================== TYPEDEFS ================================== */

#if defined(VREFBUF)

/** \brief Structure type used to store physical address of VREFBUF registers.
 * This array is used to reduce size of configuration array. With this style
 * the registers are referenced through enumeration.
 *
 */
typedef struct
{
    vrefBuf_RegId_t     RegId;   /**< Peripheral register ID      */
    volatile uint32_t * RegAddr; /**< Peripheral register address */
}   vrefBuf_RegList_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static vrefBuf_RequestState_t   VrefBuf_Reg_Wait_RegVal ( vrefBuf_RegId_t regId, uint32_t regMask, uint32_t regValue );
static vrefBuf_RequestState_t   VrefBuf_Reg_Set_Verified( vrefBuf_RegId_t regId, uint32_t regMask, uint32_t regValue );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Count of read-back iterations of a written register value */
#define VREFBUF_REG_READBACK_TIMEOUT_RAW    ( 0x1000u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief VREFBUF registers configuration structure
 *
 * This structure is used to store addresses of VREFBUF registers.
 * It is used to reduce code size while referring to VREFBUF registers.
 */
static const vrefBuf_RegList_t          vrefBuf_RegList[] =
{
    { .RegId = VREFBUF_REG_CSR , .RegAddr = &(VREFBUF->CSR) }, /**< VREFBUF control and status register  */
    { .RegId = VREFBUF_REG_CCR , .RegAddr = &(VREFBUF->CCR) }, /**< VREFBUF calibration control register */
};

_Static_assert( (sizeof(vrefBuf_RegList) / sizeof(vrefBuf_RegList_t)) == VREFBUF_REG_CNT,
                "VrefBuf_Reg: vrefBuf_RegList has incorrect size." );

_Static_assert( VREFBUF_TRIMMING_MAX == ( VREFBUF_CCR_TRIM >> VREFBUF_CCR_TRIM_Pos ),
                "VrefBuf_Reg: VREFBUF_TRIMMING_MAX does not match the TRIM field." );

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Sets the register bit with given mask.
 *
 * \param regId   [in]: VREFBUF register ID
 * \param bitMask [in]: Mask to apply to the register
 */
void VrefBuf_Set_RegBit( vrefBuf_RegId_t regId, uint32_t bitMask )
{
    if( VREFBUF_REG_CNT > regId )
    {
        *vrefBuf_RegList[regId].RegAddr |= bitMask;
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Resets the register bit with given mask.
 *
 * \param regId   [in]: VREFBUF register ID
 * \param bitMask [in]: Mask to apply to the register
 */
void VrefBuf_Reset_RegBit( vrefBuf_RegId_t regId, uint32_t bitMask )
{
    if( VREFBUF_REG_CNT > regId )
    {
        *vrefBuf_RegList[regId].RegAddr &= ~bitMask;
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Gets the register bit value with given mask.
 *
 * \param regId   [in]: VREFBUF register ID
 * \param bitMask [in]: Mask to apply to the register
 * \return Register bit value with applied mask, 0 for invalid register ID
 */
uint32_t VrefBuf_Get_RegBit( vrefBuf_RegId_t regId, uint32_t bitMask )
{
    uint32_t retValue = 0u;

    if( VREFBUF_REG_CNT > regId )
    {
        retValue = ( *vrefBuf_RegList[regId].RegAddr & bitMask );
    }
    else
    {
        /* Invalid register ID, zero is returned */
        retValue = 0u;
    }

    return ( retValue );
}


/**
 * \brief Updates the register value with given mask and value.
 *
 * \param regId    [in]: VREFBUF register ID
 * \param regMask  [in]: Mask to apply to the register
 * \param regValue [in]: Value to set in the register
 */
void VrefBuf_Set_RegVal( vrefBuf_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    if( VREFBUF_REG_CNT > regId )
    {
        /* Set the register value with mask */
        *vrefBuf_RegList[regId].RegAddr = ( ( *vrefBuf_RegList[regId].RegAddr & ~regMask ) |
                                            ( regValue & regMask ) );
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Gets the register value with given mask.
 *
 * \param regId   [in]: VREFBUF register ID.
 * \param regMask [in]: Mask to apply to the register
 * \return Register value with applied mask, 0 for invalid register ID
 */
uint32_t VrefBuf_Get_RegVal( vrefBuf_RegId_t regId, uint32_t regMask )
{
    uint32_t retValue = 0u;

    if( VREFBUF_REG_CNT > regId )
    {
        /* Get the register value with mask */
        retValue = ( *vrefBuf_RegList[regId].RegAddr & regMask );
    }
    else
    {
        /* Invalid register ID, zero is returned */
        retValue = 0u;
    }

    return ( retValue );
}


/**
 * \brief Writes the electrical mode (bits ENVR and HIZ) and verifies it by read-back.
 *
 * The mode is written with one store, so the pin VREF+ never passes through
 * a mode which is not requested. The caller waits for the ready flag after the
 * buffer is enabled (\ref VrefBuf_Reg_Wait_Ready).
 *
 * \param modeValue [in]: Value of the bits ENVR and HIZ (positioned), see
 *                        \ref VREFBUF_REG_MODE_OFF, \ref VREFBUF_REG_MODE_BUFFER and
 *                        \ref VREFBUF_REG_MODE_HIGH_IMPEDANCE
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if the value read back within the timeout.
 *         Returns \ref VREFBUF_REQUEST_ERROR for a value out of the bits ENVR and HIZ
 *         (nothing is written) or when the read-back does not match.
 */
vrefBuf_RequestState_t VrefBuf_Reg_Set_Mode( uint32_t modeValue )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

    if( 0u == ( modeValue & ~VREFBUF_REG_MODE_MASK ) )
    {
        retState = VrefBuf_Reg_Set_Verified( VREFBUF_REG_CSR, VREFBUF_REG_MODE_MASK, modeValue );
    }
    else
    {
        /* Value is out of the mode bits, nothing is written */
        retState = VREFBUF_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the voltage level (field VRS) and verifies it by read-back.
 *
 * The field VRS can be programmed only while the buffer is disabled (ENVR = 0).
 * When the buffer is enabled (running or hold mode), the protocol writes the high
 * impedance mode first (the external capacitor holds the voltage of the pin VREF+, the
 * pin is never pulled down), writes the level, restores the previous mode and, if it was
 * the buffer mode, waits for the ready flag. The factory trimming of the new level is
 * loaded by the hardware at the change of VRS (unless a user trimming was written).
 *
 * \param voltageValue [in]: Value of the field VRS (positioned), see \ref VREFBUF_REG_VOLTAGE_2_5V,
 *                           \ref VREFBUF_REG_VOLTAGE_2_048V, \ref VREFBUF_REG_VOLTAGE_1_8V and (STM32H7
 *                           lines) VREFBUF_REG_VOLTAGE_1_5V; the values up to \ref VREFBUF_REG_VOLTAGE_LAST
 * \param timeout      [in]: Count of the read-backs of the ready flag (used when the buffer is running)
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if the level was written, verified and, for the running
 *         buffer, the output is ready. Returns \ref VREFBUF_REQUEST_ERROR for a reserved value of
 *         the field (nothing is written), for a mismatch of a read-back and for the ready flag not
 *         set within the timeout (the buffer is left in the mode reached by the protocol).
 */
vrefBuf_RequestState_t VrefBuf_Reg_Set_Voltage( uint32_t voltageValue, vrefBuf_TimeoutCnt_t timeout )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

    if( ( 0u                       == ( voltageValue & ~VREFBUF_CSR_VRS ) ) &&
        ( VREFBUF_REG_VOLTAGE_LAST >= voltageValue                        )    )
    {
        const uint32_t modeValue = VrefBuf_Get_RegVal( VREFBUF_REG_CSR, VREFBUF_REG_MODE_MASK );

        if( 0u != ( modeValue & VREFBUF_CSR_ENVR ) )
        {
            /* Buffer enabled - disabled for the write of VRS, the pin is held by the capacitor */
            retState = VrefBuf_Reg_Set_Mode( VREFBUF_REG_MODE_HIGH_IMPEDANCE );

            if( VREFBUF_REQUEST_OK == retState )
            {
                retState = VrefBuf_Reg_Set_Verified( VREFBUF_REG_CSR, VREFBUF_CSR_VRS, voltageValue );
            }
            else
            {
                /* Previous step failed */
            }

            if( VREFBUF_REQUEST_OK == retState )
            {
                retState = VrefBuf_Reg_Set_Mode( modeValue );
            }
            else
            {
                /* Previous step failed */
            }

            if( ( VREFBUF_REQUEST_OK      == retState  ) &&
                ( VREFBUF_REG_MODE_BUFFER == modeValue )    )
            {
                retState = VrefBuf_Reg_Wait_Ready( timeout );
            }
            else
            {
                /* Previous step failed or the output is not driven, ready flag is not awaited */
            }
        }
        else
        {
            /* Buffer disabled - VRS can be written directly */
            retState = VrefBuf_Reg_Set_Verified( VREFBUF_REG_CSR, VREFBUF_CSR_VRS, voltageValue );
        }
    }
    else
    {
        /* Reserved value of the field, nothing is written */
        retState = VREFBUF_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the trimming code (field TRIM) and verifies it by read-back.
 *
 * The write of TRIM disables the loading of the factory trimming at the change of
 * VRS until the next reset of the peripheral. The reference manual states that an
 * application trimming procedure shall run through the codes in ascending order
 * (000000 to 111111).
 *
 * \param trimmingValue [in]: Value of the field TRIM (positioned)
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if the value read back within the timeout.
 *         Returns \ref VREFBUF_REQUEST_ERROR for a value out of the field (nothing is
 *         written) or when the read-back does not match.
 */
vrefBuf_RequestState_t VrefBuf_Reg_Set_Trimming( uint32_t trimmingValue )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

    if( 0u == ( trimmingValue & ~VREFBUF_CCR_TRIM ) )
    {
        retState = VrefBuf_Reg_Set_Verified( VREFBUF_REG_CCR, VREFBUF_CCR_TRIM, trimmingValue );
    }
    else
    {
        /* Value is out of the field, nothing is written */
        retState = VREFBUF_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the ready flag of the buffer (bit VRR).
 *
 * \return \ref VREFBUF_FLAG_ACTIVE if the buffer output reached the requested level,
 *         otherwise \ref VREFBUF_FLAG_INACTIVE.
 */
vrefBuf_FlagState_t VrefBuf_Reg_Get_ReadyFlag( void )
{
    vrefBuf_FlagState_t retState = VREFBUF_FLAG_INACTIVE;
    const uint32_t      vrrValue = VrefBuf_Get_RegBit( VREFBUF_REG_CSR, VREFBUF_CSR_VRR );

    if( 0u != vrrValue )
    {
        retState = VREFBUF_FLAG_ACTIVE;
    }
    else
    {
        retState = VREFBUF_FLAG_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Waits until the ready flag of the buffer is set.
 *
 * \param timeout [in]: Count of the read-backs of the flag
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if the flag was set within the timeout.
 *         Otherwise (also for the timeout zero) returns \ref VREFBUF_REQUEST_ERROR.
 */
vrefBuf_RequestState_t VrefBuf_Reg_Wait_Ready( vrefBuf_TimeoutCnt_t timeout )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

    for( vrefBuf_TimeoutCnt_t iterationCnt = 0u; timeout > iterationCnt; iterationCnt ++ )
    {
        const vrefBuf_FlagState_t readyFlag = VrefBuf_Reg_Get_ReadyFlag();

        if( VREFBUF_FLAG_ACTIVE == readyFlag )
        {
            retState = VREFBUF_REQUEST_OK;
            break;
        }
        else
        {
            /* Output has not reached the requested level yet */
            retState = VREFBUF_REQUEST_ERROR;
        }
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Waits until the register (bit field) reads back the required value.
 *
 * \param regId    [in]: VREFBUF register ID
 * \param regMask  [in]: Mask of the checked bits
 * \param regValue [in]: Required value of the masked bits
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if the value was read back within the
 *         timeout. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
static vrefBuf_RequestState_t VrefBuf_Reg_Wait_RegVal( vrefBuf_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

    for( uint32_t iterationCnt = 0u; VREFBUF_REG_READBACK_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        const uint32_t readValue = VrefBuf_Get_RegVal( regId, regMask );

        if( ( regValue & regMask ) == readValue )
        {
            retState = VREFBUF_REQUEST_OK;
            break;
        }
        else
        {
            /* Register has not been updated yet */
            retState = VREFBUF_REQUEST_ERROR;
        }
    }

    return ( retState );
}


/**
 * \brief Writes the bit field of the register and verifies it by read-back.
 *
 * \param regId    [in]: VREFBUF register ID
 * \param regMask  [in]: Mask of the written bits
 * \param regValue [in]: Value of the masked bits
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if the value read back within the
 *         timeout. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
static vrefBuf_RequestState_t VrefBuf_Reg_Set_Verified( vrefBuf_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    VrefBuf_Set_RegVal( regId, regMask, regValue );

    return ( VrefBuf_Reg_Wait_RegVal( regId, regMask, regValue ) );
}

#endif /* VREFBUF */

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
