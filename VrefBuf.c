/**
 * \author Mr.Nobody
 * \file VrefBuf.c
 * \ingroup VrefBuf
 * \brief VrefBuf module common functionality
 *
 * # Introduction #
 * The voltage reference buffer (VREFBUF) generates the reference voltage of the ADCs
 * and DACs from the internal reference VREFINT and drives it to the pin VREF+, where it
 * can supply external circuits. The peripheral exists on all STM32H5 lines except
 * STM32H503 (no CMSIS definition VREFBUF). Every public function checks the peripheral
 * (\c defined(VREFBUF)): on the line without it the function keeps its signature, suppresses its
 * unused parameters and returns \ref VREFBUF_REQUEST_ERROR (\ref VrefBuf_Task does nothing,
 * \ref VrefBuf_Get_ModuleVersion returns the version).
 *
 * # Modes #
 * The bits ENVR and HIZ select the electrical mode of the pin VREF+ (see \ref vrefBuf_Mode_t):
 * - \ref VREFBUF_MODE_BUFFER (ENVR = 1, HIZ = 0): the buffer drives the pin with the selected
 *   voltage, the output is ready when the flag VRR is set,
 * - \ref VREFBUF_MODE_HIGH_IMPEDANCE (ENVR = 0, HIZ = 1): external voltage reference mode, the
 *   pin is an input (reset state),
 * - \ref VREFBUF_MODE_OFF (ENVR = 0, HIZ = 0): the buffer is off and the pin VREF+ is pulled
 *   down to VSSA. The mode shall not be used on a board where VREF+ is connected to VDDA.
 * The hold mode of the hardware (ENVR = 1, HIZ = 1) is not a mode of the module and the module
 * does not use it (the change of the voltage level of the running buffer uses the high impedance mode).
 *
 * # Voltage and trimming #
 * The levels 2.5 V, 2.048 V and 1.8 V are selected by the field VRS, which the hardware lets
 * program only while the buffer is disabled - the change of the level of the running buffer is
 * done by the register protocol (\ref VrefBuf_Reg_Set_Voltage). The factory trimming of every
 * level is loaded by the hardware at reset and at every change of VRS. A written user trimming
 * disables the loading until the reset of the peripheral (\ref VrefBuf_Init and \ref VrefBuf_Deinit
 * reset the peripheral).
 *
 * # Start-up #
 * Enabling the buffer, the change of the mode to the buffer mode and the change of the level of
 * the running buffer wait for the ready flag with a bounded loop (\ref vrefBuf_Config_t::StartTimeout).
 * The state \ref VREFBUF_STATE_STARTING is reported while the flag is not set.
 *
 * \note  The module does not know the supply voltage VDDA: the application selects the level
 *        so that VDDA is at least the level plus the dropout of the buffer (datasheet). The buffer
 *        is not available (shall be kept disabled) on the packages where VREF+ is bonded to VDDA.
 *
 * # Device erratum: slow discharge of the capacitor on VREF+ #
 * ES0565 Rev 7 2.8.1 "Slow discharge of the capacitor connected to VREF+ pin" (bug AB#1996), STM32H562,
 * STM32H563 and STM32H573: the pull-down resistor on the VREFBUF output is missing, so the capacitor on
 * the pin VREF+ discharges slowly when the output is switched down. The workaround of the erratum is to
 * wait a sufficient time for the discharge when the output is switched down. The module does not wait for
 * it: the time depends on the capacitor and on the load of the board, no register reports it and a blind
 * delay is not allowed, so the application waits. The functions which switch the output down carry a note
 * (\ref VrefBuf_Init, \ref VrefBuf_Deinit, \ref VrefBuf_Set_Mode, \ref VrefBuf_Set_Voltage). The errata
 * sheets of STM32H503 (ES0561) and of STM32H523 / STM32H533 (ES0621) list no VREFBUF erratum; the errata
 * sheets of the other lines were not available, the erratum is not verified for them.
 *
 */
/* ============================== INCLUDES ================================== */
#include "VrefBuf.h"                        /* Self include                   */
#include "VrefBuf_Port.h"                   /* Own port file include          */
#include "VrefBuf_Types.h"                  /* Module types definitions       */
#include "VrefBuf_Reg.h"                    /* Register access                */
#include "Rcc_Port.h"                       /* RCC handler functionality      */
#include "Stm32.h"                          /* VREFBUF registers definition   */
/* ============================== TYPEDEFS ================================== */

#if defined(VREFBUF)

/** \brief Runtime data of the module */
typedef struct
{
    vrefBuf_FunctionState_t InitState;    /**< Module is initialized (set by VrefBuf_Init)          */
    vrefBuf_TimeoutCnt_t    StartTimeout; /**< Start-up timeout of the later changes of the buffer  */
}   vrefBuf_RuntimeData_t;

#endif /* VREFBUF */

/* ======================== FORWARD DECLARATIONS ============================ */

#if defined(VREFBUF)

static vrefBuf_RequestState_t   VrefBuf_Check_Config    ( const vrefBuf_Config_t * const vrefBufConfig );

static vrefBuf_RequestState_t   VrefBuf_Set_Clock       ( void );
static vrefBuf_RequestState_t   VrefBuf_Set_Peripheral  ( const vrefBuf_Config_t * const vrefBufConfig );

static vrefBuf_RequestState_t   VrefBuf_Write_Mode      ( vrefBuf_Mode_t mode, vrefBuf_TimeoutCnt_t timeout );
static vrefBuf_RequestState_t   VrefBuf_Write_Voltage   ( vrefBuf_Voltage_t voltage, vrefBuf_TimeoutCnt_t timeout );
static vrefBuf_RequestState_t   VrefBuf_Write_Trimming  ( vrefBuf_Trimming_t trimming );

static vrefBuf_RequestState_t   VrefBuf_Read_Mode       ( vrefBuf_Mode_t * const vrefBufMode );
static vrefBuf_RequestState_t   VrefBuf_Read_Voltage    ( vrefBuf_Voltage_t * const vrefBufVoltage );
static vrefBuf_RequestState_t   VrefBuf_Read_State      ( vrefBuf_State_t * const vrefBufState );
static vrefBuf_Trimming_t       VrefBuf_Read_Trimming   ( void );

static vrefBuf_RequestState_t   VrefBuf_Verify_Config   ( const vrefBuf_Config_t * const vrefBufConfig );
static vrefBuf_RequestState_t   VrefBuf_Verify_Output   ( const vrefBuf_Config_t * const vrefBufConfig );

#endif /* VREFBUF */

/* ========================== SYMBOLIC CONSTANTS ============================ */

#if defined(VREFBUF)

/** Default mode used by \ref VrefBuf_Get_DefaultConfig */
#define VREFBUF_DEFAULT_MODE            ( VREFBUF_MODE_BUFFER )

/** Default voltage level used by \ref VrefBuf_Get_DefaultConfig (reset level of the peripheral) */
#define VREFBUF_DEFAULT_VOLTAGE         ( VREFBUF_VOLTAGE_2_5V )

/** Default start-up timeout used by \ref VrefBuf_Get_DefaultConfig - count of the read-backs of the ready flag.
 *  The start-up time of the buffer is below 1 ms (datasheet, load capacitor up to 1.5 uF), the count
 *  gives several milliseconds even with the fastest core clock. */
#define VREFBUF_DEFAULT_TIMEOUT_RAW     ( 100000u )

#endif /* VREFBUF */

/* =============================== MACROS =================================== */

#if defined(VREFBUF)

/** Converts the trimming code to the value of the TRIM field of CCR register */
#define VREFBUF_TRIMMING_TO_REG( trimming )     \
    ( ( (uint32_t)( trimming ) << VREFBUF_CCR_TRIM_Pos ) & VREFBUF_CCR_TRIM )

/** Converts the value of the TRIM field of CCR register to the trimming code */
#define VREFBUF_REG_TO_TRIMMING( regVal )       \
    ( (vrefBuf_Trimming_t)( ( ( regVal ) & VREFBUF_CCR_TRIM ) >> VREFBUF_CCR_TRIM_Pos ) )

#endif /* VREFBUF */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

#if defined(VREFBUF)

/** \brief Mode configuration, indexed by \ref vrefBuf_Mode_t */
static const vrefBuf_ModeConfig_t vrefBuf_ModeLut[] =
{
    [ VREFBUF_MODE_OFF            ] = { .Mode   = VREFBUF_MODE_OFF,
                                        .RegVal = VREFBUF_REG_MODE_OFF            },
    [ VREFBUF_MODE_BUFFER         ] = { .Mode   = VREFBUF_MODE_BUFFER,
                                        .RegVal = VREFBUF_REG_MODE_BUFFER         },
    [ VREFBUF_MODE_HIGH_IMPEDANCE ] = { .Mode   = VREFBUF_MODE_HIGH_IMPEDANCE,
                                        .RegVal = VREFBUF_REG_MODE_HIGH_IMPEDANCE },
};

_Static_assert( VREFBUF_MODE_CNT == ( sizeof( vrefBuf_ModeLut ) / sizeof( vrefBuf_ModeConfig_t ) ),
                "VREFBUF mode LUT size mismatch" );


/** \brief Voltage level configuration, indexed by \ref vrefBuf_Voltage_t */
static const vrefBuf_VoltageConfig_t vrefBuf_VoltageLut[] =
{
    [ VREFBUF_VOLTAGE_2_5V   ] = { .Voltage = VREFBUF_VOLTAGE_2_5V  , .RegVal = VREFBUF_REG_VOLTAGE_2_5V   },
    [ VREFBUF_VOLTAGE_2_048V ] = { .Voltage = VREFBUF_VOLTAGE_2_048V, .RegVal = VREFBUF_REG_VOLTAGE_2_048V },
    [ VREFBUF_VOLTAGE_1_8V   ] = { .Voltage = VREFBUF_VOLTAGE_1_8V  , .RegVal = VREFBUF_REG_VOLTAGE_1_8V   },
};

_Static_assert( VREFBUF_VOLTAGE_CNT == ( sizeof( vrefBuf_VoltageLut ) / sizeof( vrefBuf_VoltageConfig_t ) ),
                "VREFBUF voltage LUT size mismatch" );


/** \brief Runtime data of the module */
static vrefBuf_RuntimeData_t vrefBuf_RuntimeData =
{
    .InitState    = VREFBUF_FUNCTION_INACTIVE,
    .StartTimeout = 0u,
};

#endif /* VREFBUF */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
vrefBuf_ModuleVersion_t VrefBuf_Get_ModuleVersion( void )
{
    vrefBuf_ModuleVersion_t retVersion;

    retVersion.Major = VREFBUF_MAJOR_VERSION;
    retVersion.Minor = VREFBUF_MINOR_VERSION;
    retVersion.Patch = VREFBUF_PATCH_VERSION;

    return (retVersion);
}


/**
 * \brief Main task of module VrefBuf
 *
 * The start-up of the buffer is awaited inside of the functions which change it,
 * so the module has no periodic work. The function is kept in the interface for
 * the common call of the tasks of all modules.
 */
void VrefBuf_Task( void )
{
    return;
}

/**
 * \brief Initializes the voltage reference buffer.
 *
 * Sequence: activation of the peripheral clock and reset of the peripheral (reset state: external
 * voltage reference mode, level 2.5 V, factory trimming), the voltage level, the user trimming (only
 * when requested), the mode (the buffer mode waits for the ready flag) and the verification of
 * the result by read-back. The peripheral reset makes the result independent of the state left by
 * an earlier use, a running buffer is stopped for a moment.
 *
 * \note  The module does not know the supply voltage VDDA: the level shall be chosen so that
 *        VDDA is at least the level plus the dropout of the buffer (datasheet).
 * \note  The buffer is not available (and shall be kept disabled) on the packages where VREF+ is
 *        bonded to VDDA. The mode \ref VREFBUF_MODE_OFF pulls the pin VREF+ down to VSSA.
 * \note  Device errata bug AB#1996 (ES0565 Rev 7 2.8.1 "Slow discharge of the capacitor connected to VREF+ pin"):
 *        the pull-down resistor on the output is missing (STM32H562, STM32H563, STM32H573), so the capacitor
 *        on VREF+ discharges slowly when the output is switched down. The peripheral reset at the start
 *        stops a running buffer, and a lower level or the mode \ref VREFBUF_MODE_OFF /
 *        \ref VREFBUF_MODE_HIGH_IMPEDANCE than the previous use is a switch down: the module does not wait for
 *        the discharge, the application shall wait a sufficient time (it depends on the capacitor and the load
 *        of the board) before it relies on the lower voltage on VREF+.
 *
 * \pre   Configuration shall be valid: mode and voltage level of the lists, trimming within
 *        the field or \ref VREFBUF_TRIMMING_UNUSED, non-zero timeout; otherwise
 *        \ref VREFBUF_REQUEST_ERROR is returned and nothing is changed. Failure of any later
 *        step (clock, reset, read-back, ready flag not set within the timeout) deinitializes
 *        the module (\ref VrefBuf_Deinit) and returns \ref VREFBUF_REQUEST_ERROR.
 *
 * \param vrefBufConfig [in]: Pointer to VREFBUF configuration, see \ref vrefBuf_Config_t. Must not be NULL.
 *
 * \return Function processing state. Returns \ref VREFBUF_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
vrefBuf_RequestState_t VrefBuf_Init( const vrefBuf_Config_t * const vrefBufConfig )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

#if defined(VREFBUF)
    if( VREFBUF_NULL_PTR != vrefBufConfig )
    {
        /*------------------------ Configuration validation ------------------*/
        retState = VrefBuf_Check_Config( vrefBufConfig );

        /*------------------------ Peripheral configuration ------------------*/
        if( VREFBUF_REQUEST_OK == retState )
        {
            retState = VrefBuf_Set_Peripheral( vrefBufConfig );

            if( VREFBUF_REQUEST_OK == retState )
            {
                vrefBuf_RuntimeData.StartTimeout = vrefBufConfig->StartTimeout;
                vrefBuf_RuntimeData.InitState    = VREFBUF_FUNCTION_ACTIVE;
            }
            else
            {
                /* Self-verification failed - the peripheral is returned to the reset state */
                (void)VrefBuf_Deinit();
            }
        }
        else
        {
            /* Invalid configuration, nothing is written */
        }
    }
    else
    {
        retState = VREFBUF_REQUEST_ERROR;
    }
#else
    (void)vrefBufConfig;
#endif /* VREFBUF */

    return ( retState );
}


/**
 * \brief Deinitializes module VrefBuf
 *
 * Resets the peripheral (external voltage reference mode, level 2.5 V, factory trimming) and
 * disables its clock. The reference of the converters is then the voltage connected to the pin
 * VREF+ (VDDA on the boards where the pin is connected to it). The function can be called in any
 * state of the module.
 *
 * \note  The converters which use the buffer shall be stopped before the call.
 * \note  Device errata bug AB#1996 (ES0565 Rev 7 2.8.1 "Slow discharge of the capacitor connected to VREF+ pin"):
 *        the pull-down resistor on the output is missing (STM32H562, STM32H563, STM32H573), so the capacitor
 *        on VREF+ discharges slowly when the output is switched down. The reset of the peripheral stops the
 *        buffer (switch down): the module does not wait for the discharge, the application shall wait a
 *        sufficient time (it depends on the capacitor and the load of the board) before it relies on the
 *        lower voltage on VREF+.
 *
 * \return Function processing state. Returns \ref VREFBUF_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
vrefBuf_RequestState_t VrefBuf_Deinit( void )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

#if defined(VREFBUF)
    const rcc_RequestState_t rstActState   = Rcc_Set_ResetActive( RCC_PERIPH_VREF );
    const rcc_RequestState_t rstInactState = Rcc_Set_ResetInactive( RCC_PERIPH_VREF );
    const rcc_RequestState_t clkState      = Rcc_Set_PeriphInactive( RCC_PERIPH_VREF );

    vrefBuf_RuntimeData.InitState    = VREFBUF_FUNCTION_INACTIVE;
    vrefBuf_RuntimeData.StartTimeout = 0u;

    if( ( RCC_REQUEST_OK == rstActState   ) &&
        ( RCC_REQUEST_OK == rstInactState ) &&
        ( RCC_REQUEST_OK == clkState      )    )
    {
        retState = VREFBUF_REQUEST_OK;
    }
    else
    {
        retState = VREFBUF_REQUEST_ERROR;
    }
#endif /* VREFBUF */

    return ( retState );
}


/**
 * \brief Fills configuration structure with default values.
 *
 * Default configuration: buffer mode, voltage level 2.5 V (reset level), factory trimming,
 * start-up timeout of 100000 read-backs of the ready flag.
 *
 * \param vrefBufConfig [out]: Pointer to store default configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref VREFBUF_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
vrefBuf_RequestState_t VrefBuf_Get_DefaultConfig( vrefBuf_Config_t * const vrefBufConfig )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

#if defined(VREFBUF)
    if( VREFBUF_NULL_PTR != vrefBufConfig )
    {
        vrefBufConfig->Mode         = VREFBUF_DEFAULT_MODE;
        vrefBufConfig->Voltage      = VREFBUF_DEFAULT_VOLTAGE;
        vrefBufConfig->Trimming     = VREFBUF_TRIMMING_UNUSED;
        vrefBufConfig->StartTimeout = VREFBUF_DEFAULT_TIMEOUT_RAW;

        retState = VREFBUF_REQUEST_OK;
    }
    else
    {
        retState = VREFBUF_REQUEST_ERROR;
    }
#else
    (void)vrefBufConfig;
#endif /* VREFBUF */

    return ( retState );
}


/**
 * \brief Sets the electrical mode of the buffer.
 *
 * The mode is written with one store and verified by read-back. The change to the buffer
 * mode waits for the ready flag (start-up timeout of \ref VrefBuf_Init), the state is
 * \ref VREFBUF_STATE_STARTING until then.
 *
 * \note  The mode \ref VREFBUF_MODE_OFF pulls the pin VREF+ down to VSSA, the converters
 *        which use the buffer shall be stopped before the call.
 * \note  When the ready flag is not set within the timeout, the buffer stays enabled (the
 *        state is \ref VREFBUF_STATE_STARTING) and the error is returned.
 * \note  Device errata bug AB#1996 (ES0565 Rev 7 2.8.1 "Slow discharge of the capacitor connected to VREF+ pin"):
 *        the pull-down resistor on the output is missing (STM32H562, STM32H563, STM32H573), so the capacitor
 *        on VREF+ discharges slowly when the output is switched down. The change from the buffer mode to
 *        \ref VREFBUF_MODE_OFF or \ref VREFBUF_MODE_HIGH_IMPEDANCE is a switch down: the module does not wait
 *        for the discharge, the application shall wait a sufficient time (it depends on the capacitor and the
 *        load of the board) before it relies on the lower voltage on VREF+.
 *
 * \pre   The module shall be initialized by \ref VrefBuf_Init and the mode shall be a
 *        mode of the list; otherwise \ref VREFBUF_REQUEST_ERROR is returned and no register is written.
 *
 * \param mode [in]: Required mode, see \ref vrefBuf_Mode_t
 *
 * \return Function processing state. Returns \ref VREFBUF_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
vrefBuf_RequestState_t VrefBuf_Set_Mode( vrefBuf_Mode_t mode )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

#if defined(VREFBUF)
    if( ( VREFBUF_FUNCTION_ACTIVE == vrefBuf_RuntimeData.InitState ) &&
        ( VREFBUF_MODE_CNT        >  mode                          )    )
    {
        retState = VrefBuf_Write_Mode( mode, vrefBuf_RuntimeData.StartTimeout );
    }
    else
    {
        retState = VREFBUF_REQUEST_ERROR;
    }
#else
    (void)mode;
#endif /* VREFBUF */

    return ( retState );
}


/**
 * \brief Returns the electrical mode of the buffer.
 *
 * The mode is read from the registers.
 *
 * \note  The hold mode of the hardware (ENVR = 1, HIZ = 1) is not a mode of the module,
 *        the function returns \ref VREFBUF_REQUEST_ERROR for it and does not touch the output.
 *
 * \pre   The module shall be initialized by \ref VrefBuf_Init; otherwise \ref VREFBUF_REQUEST_ERROR.
 *
 * \param vrefBufMode [out]: Pointer to store the mode. Must not be NULL.
 *
 * \return Function processing state. Returns \ref VREFBUF_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
vrefBuf_RequestState_t VrefBuf_Get_Mode( vrefBuf_Mode_t * const vrefBufMode )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

#if defined(VREFBUF)
    if( ( VREFBUF_NULL_PTR        != vrefBufMode                   ) &&
        ( VREFBUF_FUNCTION_ACTIVE == vrefBuf_RuntimeData.InitState )    )
    {
        retState = VrefBuf_Read_Mode( vrefBufMode );
    }
    else
    {
        retState = VREFBUF_REQUEST_ERROR;
    }
#else
    (void)vrefBufMode;
#endif /* VREFBUF */

    return ( retState );
}


/**
 * \brief Sets the output voltage level of the buffer.
 *
 * The level can be programmed by the hardware only while the buffer is disabled. When
 * the buffer is running, the register protocol puts it to the high impedance mode (the
 * pin is held by the external capacitor), writes the level, restores the buffer mode and
 * waits for the ready flag (start-up timeout of \ref VrefBuf_Init), the state is
 * \ref VREFBUF_STATE_STARTING until then. The hardware loads the factory trimming of the new
 * level, unless a user trimming was written (\ref VrefBuf_Set_Trimming).
 *
 * \note  The level shall be chosen so that VDDA is at least the level plus the dropout of the buffer.
 * \note  Failure of the protocol (read-back mismatch, ready flag not set within the timeout)
 *        returns the error, the buffer is left in the mode reached by the protocol, see
 *        \ref VrefBuf_Get_State and \ref VrefBuf_Get_Mode.
 * \note  Device errata bug AB#1996 (ES0565 Rev 7 2.8.1 "Slow discharge of the capacitor connected to VREF+ pin"):
 *        the pull-down resistor on the output is missing (STM32H562, STM32H563, STM32H573), so the capacitor
 *        on VREF+ discharges slowly when the output is switched down. The change to a lower level is a switch
 *        down: the module does not wait for the discharge (the wait for the ready flag stays bounded by
 *        \ref vrefBuf_Config_t::StartTimeout, the erratum does not describe the ready flag during the slow
 *        discharge), the application shall wait a sufficient time (it depends on the capacitor and the load of
 *        the board) before it relies on the lower voltage on VREF+.
 *
 * \pre   The module shall be initialized by \ref VrefBuf_Init and the level shall be a
 *        level of the list; otherwise \ref VREFBUF_REQUEST_ERROR is returned and no register is written.
 *
 * \param voltage [in]: Required voltage level, see \ref vrefBuf_Voltage_t
 *
 * \return Function processing state. Returns \ref VREFBUF_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
vrefBuf_RequestState_t VrefBuf_Set_Voltage( vrefBuf_Voltage_t voltage )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

#if defined(VREFBUF)
    if( ( VREFBUF_FUNCTION_ACTIVE == vrefBuf_RuntimeData.InitState ) &&
        ( VREFBUF_VOLTAGE_CNT     >  voltage                       )    )
    {
        retState = VrefBuf_Write_Voltage( voltage, vrefBuf_RuntimeData.StartTimeout );
    }
    else
    {
        retState = VREFBUF_REQUEST_ERROR;
    }
#else
    (void)voltage;
#endif /* VREFBUF */

    return ( retState );
}


/**
 * \brief Returns the output voltage level of the buffer.
 *
 * The level is read from the registers (it is the selected level also when the buffer is off).
 *
 * \pre   The module shall be initialized by \ref VrefBuf_Init; otherwise \ref VREFBUF_REQUEST_ERROR.
 *        A reserved value of the field VRS returns \ref VREFBUF_REQUEST_ERROR.
 *
 * \param vrefBufVoltage [out]: Pointer to store the voltage level. Must not be NULL.
 *
 * \return Function processing state. Returns \ref VREFBUF_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
vrefBuf_RequestState_t VrefBuf_Get_Voltage( vrefBuf_Voltage_t * const vrefBufVoltage )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

#if defined(VREFBUF)
    if( ( VREFBUF_NULL_PTR        != vrefBufVoltage                ) &&
        ( VREFBUF_FUNCTION_ACTIVE == vrefBuf_RuntimeData.InitState )    )
    {
        retState = VrefBuf_Read_Voltage( vrefBufVoltage );
    }
    else
    {
        retState = VREFBUF_REQUEST_ERROR;
    }
#else
    (void)vrefBufVoltage;
#endif /* VREFBUF */

    return ( retState );
}


/**
 * \brief Returns the state of the buffer.
 *
 * The state is read from the registers: \ref VREFBUF_STATE_READY in the buffer mode with the ready
 * flag set, \ref VREFBUF_STATE_STARTING in the buffer mode without it (start-up, or the output fell
 * below the requested level, e.g. the supply VDDA is too low), \ref VREFBUF_STATE_OFF in the other
 * modes. The consumers (Adc, Dac) convert only in the state \ref VREFBUF_STATE_READY.
 *
 * \pre   The module shall be initialized by \ref VrefBuf_Init; otherwise \ref VREFBUF_REQUEST_ERROR.
 *
 * \param vrefBufState [out]: Pointer to store the state. Must not be NULL.
 *
 * \return Function processing state. Returns \ref VREFBUF_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
vrefBuf_RequestState_t VrefBuf_Get_State( vrefBuf_State_t * const vrefBufState )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

#if defined(VREFBUF)
    if( ( VREFBUF_NULL_PTR        != vrefBufState                  ) &&
        ( VREFBUF_FUNCTION_ACTIVE == vrefBuf_RuntimeData.InitState )    )
    {
        retState = VrefBuf_Read_State( vrefBufState );
    }
    else
    {
        retState = VREFBUF_REQUEST_ERROR;
    }
#else
    (void)vrefBufState;
#endif /* VREFBUF */

    return ( retState );
}


/**
 * \brief Sets the user trimming code of the reference.
 *
 * The code is written and verified by read-back. The write disables the loading of the factory
 * trimming at the change of the voltage level until the peripheral is reset (\ref VrefBuf_Init
 * or \ref VrefBuf_Deinit). The reference manual states that an application trimming procedure
 * shall run through the codes in ascending order (000000 to 111111).
 *
 * \pre   The module shall be initialized by \ref VrefBuf_Init and the code shall not be above
 *        \ref VREFBUF_TRIMMING_MAX (\ref VREFBUF_TRIMMING_UNUSED is not a code); otherwise
 *        \ref VREFBUF_REQUEST_ERROR is returned and no register is written.
 *
 * \param trimming [in]: Trimming code (0 ... \ref VREFBUF_TRIMMING_MAX)
 *
 * \return Function processing state. Returns \ref VREFBUF_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
vrefBuf_RequestState_t VrefBuf_Set_Trimming( vrefBuf_Trimming_t trimming )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

#if defined(VREFBUF)
    if( VREFBUF_FUNCTION_ACTIVE == vrefBuf_RuntimeData.InitState )
    {
        retState = VrefBuf_Write_Trimming( trimming );
    }
    else
    {
        retState = VREFBUF_REQUEST_ERROR;
    }
#else
    (void)trimming;
#endif /* VREFBUF */

    return ( retState );
}


/**
 * \brief Returns the current trimming code of the reference.
 *
 * The code is read from the register: the factory trimming of the selected level loaded by
 * the hardware, or the user trimming written by \ref VrefBuf_Set_Trimming.
 *
 * \pre   The module shall be initialized by \ref VrefBuf_Init; otherwise \ref VREFBUF_REQUEST_ERROR.
 *
 * \param vrefBufTrimming [out]: Pointer to store the trimming code. Must not be NULL.
 *
 * \return Function processing state. Returns \ref VREFBUF_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
vrefBuf_RequestState_t VrefBuf_Get_Trimming( vrefBuf_Trimming_t * const vrefBufTrimming )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

#if defined(VREFBUF)
    if( ( VREFBUF_NULL_PTR        != vrefBufTrimming               ) &&
        ( VREFBUF_FUNCTION_ACTIVE == vrefBuf_RuntimeData.InitState )    )
    {
        *vrefBufTrimming = VrefBuf_Read_Trimming();

        retState = VREFBUF_REQUEST_OK;
    }
    else
    {
        retState = VREFBUF_REQUEST_ERROR;
    }
#else
    (void)vrefBufTrimming;
#endif /* VREFBUF */

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

#if defined(VREFBUF)

/**
 * \brief Checks the configuration before any register is written.
 *
 * \param vrefBufConfig [in]: Pointer to VREFBUF configuration
 *
 * \return Returns \ref VREFBUF_REQUEST_OK for a valid configuration (mode and voltage level of the
 *         lists, trimming within the field or unused, non-zero timeout). Otherwise returns
 *         \ref VREFBUF_REQUEST_ERROR.
 */
static vrefBuf_RequestState_t VrefBuf_Check_Config( const vrefBuf_Config_t * const vrefBufConfig )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

    if( ( VREFBUF_MODE_CNT    >  vrefBufConfig->Mode         ) &&
        ( VREFBUF_VOLTAGE_CNT >  vrefBufConfig->Voltage      ) &&
        ( 0u                  != vrefBufConfig->StartTimeout )    )
    {
        if( VREFBUF_TRIMMING_UNUSED == vrefBufConfig->Trimming )
        {
            retState = VREFBUF_REQUEST_OK;
        }
        else if( VREFBUF_TRIMMING_MAX >= vrefBufConfig->Trimming )
        {
            retState = VREFBUF_REQUEST_OK;
        }
        else
        {
            /* Trimming code is out of the field */
            retState = VREFBUF_REQUEST_ERROR;
        }
    }
    else
    {
        /* Mode or voltage level is not in the list, or the timeout is zero */
        retState = VREFBUF_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Activates the peripheral clock and resets the peripheral.
 *
 * The reset gives the reset state (external voltage reference mode, level 2.5 V, factory
 * trimming) independent of an earlier use of the peripheral.
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if the clock was activated and the reset was
 *         activated and released. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
static vrefBuf_RequestState_t VrefBuf_Set_Clock( void )
{
    vrefBuf_RequestState_t   retState = VREFBUF_REQUEST_ERROR;
    const rcc_RequestState_t clkState = Rcc_Set_PeriphActive( RCC_PERIPH_VREF );

    if( RCC_REQUEST_OK == clkState )
    {
        const rcc_RequestState_t rstActState   = Rcc_Set_ResetActive( RCC_PERIPH_VREF );
        const rcc_RequestState_t rstInactState = Rcc_Set_ResetInactive( RCC_PERIPH_VREF );

        if( ( RCC_REQUEST_OK == rstActState   ) &&
            ( RCC_REQUEST_OK == rstInactState )    )
        {
            retState = VREFBUF_REQUEST_OK;
        }
        else
        {
            /* Reset of the peripheral failed */
            retState = VREFBUF_REQUEST_ERROR;
        }
    }
    else
    {
        /* Clock of the peripheral is not available */
        retState = VREFBUF_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures the peripheral and verifies the result.
 *
 * The voltage level is written first (the buffer is disabled after the reset), then the user
 * trimming (a later change of the level would load the factory trimming otherwise) and the
 * mode as the last step, so the buffer starts with the final level and trimming.
 *
 * \param vrefBufConfig [in]: Pointer to VREFBUF configuration (valid)
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if every step succeeded and the result read back as required.
 *         Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
static vrefBuf_RequestState_t VrefBuf_Set_Peripheral( const vrefBuf_Config_t * const vrefBufConfig )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

    /* -1- Peripheral clock and reset */
    retState = VrefBuf_Set_Clock();

    /* -2- Voltage level */
    if( VREFBUF_REQUEST_OK == retState )
    {
        retState = VrefBuf_Write_Voltage( vrefBufConfig->Voltage, vrefBufConfig->StartTimeout );
    }
    else
    {
        /* Previous step failed */
    }

    /* -3- User trimming (optional) */
    if( VREFBUF_REQUEST_OK == retState )
    {
        if( VREFBUF_TRIMMING_UNUSED != vrefBufConfig->Trimming )
        {
            retState = VrefBuf_Write_Trimming( vrefBufConfig->Trimming );
        }
        else
        {
            /* Factory trimming is kept */
        }
    }
    else
    {
        /* Previous step failed */
    }

    /* -4- Mode (the buffer mode waits for the ready flag) */
    if( VREFBUF_REQUEST_OK == retState )
    {
        retState = VrefBuf_Write_Mode( vrefBufConfig->Mode, vrefBufConfig->StartTimeout );
    }
    else
    {
        /* Previous step failed */
    }

    /* -5- Verification of the result */
    if( VREFBUF_REQUEST_OK == retState )
    {
        retState = VrefBuf_Verify_Config( vrefBufConfig );
    }
    else
    {
        /* Previous step failed */
    }

    return ( retState );
}


/**
 * \brief Writes the mode, the buffer mode waits for the ready flag.
 *
 * \param mode    [in]: Required mode
 * \param timeout [in]: Count of the read-backs of the ready flag
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if the mode was written and verified and, for the
 *         buffer mode, the output is ready. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
static vrefBuf_RequestState_t VrefBuf_Write_Mode( vrefBuf_Mode_t mode, vrefBuf_TimeoutCnt_t timeout )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

    if( VREFBUF_MODE_CNT > mode )
    {
        retState = VrefBuf_Reg_Set_Mode( vrefBuf_ModeLut[ mode ].RegVal );

        if( ( VREFBUF_REQUEST_OK  == retState ) &&
            ( VREFBUF_MODE_BUFFER == mode     )    )
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
        retState = VREFBUF_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the voltage level by the register protocol (disabled buffer, or the running buffer
 *        with the wait for the ready flag).
 *
 * \param voltage [in]: Required voltage level
 * \param timeout [in]: Count of the read-backs of the ready flag
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if the level was written and verified (and the running output is
 *         ready). Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
static vrefBuf_RequestState_t VrefBuf_Write_Voltage( vrefBuf_Voltage_t voltage, vrefBuf_TimeoutCnt_t timeout )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

    if( VREFBUF_VOLTAGE_CNT > voltage )
    {
        retState = VrefBuf_Reg_Set_Voltage( vrefBuf_VoltageLut[ voltage ].RegVal, timeout );
    }
    else
    {
        retState = VREFBUF_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the trimming code.
 *
 * \param trimming [in]: Trimming code (0 ... \ref VREFBUF_TRIMMING_MAX)
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if the code was written and verified. Returns
 *         \ref VREFBUF_REQUEST_ERROR for a code above the maximum (nothing is written)
 *         or when the read-back does not match.
 */
static vrefBuf_RequestState_t VrefBuf_Write_Trimming( vrefBuf_Trimming_t trimming )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_ERROR;

    if( VREFBUF_TRIMMING_MAX >= trimming )
    {
        retState = VrefBuf_Reg_Set_Trimming( VREFBUF_TRIMMING_TO_REG( trimming ) );
    }
    else
    {
        retState = VREFBUF_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the mode from the registers.
 *
 * \param vrefBufMode [out]: Pointer to store the mode, not changed for a combination of the bits
 *                           which is not a mode of the module (hold mode)
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if the bits are a mode of the module. Otherwise
 *         returns \ref VREFBUF_REQUEST_ERROR.
 */
static vrefBuf_RequestState_t VrefBuf_Read_Mode( vrefBuf_Mode_t * const vrefBufMode )
{
    vrefBuf_RequestState_t retState  = VREFBUF_REQUEST_ERROR;
    const uint32_t         modeValue = VrefBuf_Get_RegVal( VREFBUF_REG_CSR, VREFBUF_REG_MODE_MASK );

    for( uint32_t modeIdx = 0u; VREFBUF_MODE_CNT > modeIdx; modeIdx ++ )
    {
        if( vrefBuf_ModeLut[ modeIdx ].RegVal == modeValue )
        {
            *vrefBufMode = vrefBuf_ModeLut[ modeIdx ].Mode;
            retState     = VREFBUF_REQUEST_OK;
            break;
        }
        else
        {
            /* Bits do not match this mode */
        }
    }

    return ( retState );
}


/**
 * \brief Reads the voltage level from the registers.
 *
 * \param vrefBufVoltage [out]: Pointer to store the voltage level, not changed for a reserved value of VRS
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if the field is a level of the list. Otherwise
 *         returns \ref VREFBUF_REQUEST_ERROR.
 */
static vrefBuf_RequestState_t VrefBuf_Read_Voltage( vrefBuf_Voltage_t * const vrefBufVoltage )
{
    vrefBuf_RequestState_t retState     = VREFBUF_REQUEST_ERROR;
    const uint32_t         voltageValue = VrefBuf_Get_RegVal( VREFBUF_REG_CSR, VREFBUF_CSR_VRS );

    for( uint32_t voltageIdx = 0u; VREFBUF_VOLTAGE_CNT > voltageIdx; voltageIdx ++ )
    {
        if( vrefBuf_VoltageLut[ voltageIdx ].RegVal == voltageValue )
        {
            *vrefBufVoltage = vrefBuf_VoltageLut[ voltageIdx ].Voltage;
            retState        = VREFBUF_REQUEST_OK;
            break;
        }
        else
        {
            /* Field does not match this level */
        }
    }

    return ( retState );
}


/**
 * \brief Reads the state of the buffer from the registers.
 *
 * \param vrefBufState [out]: Pointer to store the state
 *
 * \return Always \ref VREFBUF_REQUEST_OK, a combination of the bits which is not a mode of the module
 *         (hold mode) is the state \ref VREFBUF_STATE_OFF.
 */
static vrefBuf_RequestState_t VrefBuf_Read_State( vrefBuf_State_t * const vrefBufState )
{
    const uint32_t            modeValue = VrefBuf_Get_RegVal( VREFBUF_REG_CSR, VREFBUF_REG_MODE_MASK );
    const vrefBuf_FlagState_t readyFlag = VrefBuf_Reg_Get_ReadyFlag();

    if( VREFBUF_REG_MODE_BUFFER == modeValue )
    {
        if( VREFBUF_FLAG_ACTIVE == readyFlag )
        {
            *vrefBufState = VREFBUF_STATE_READY;
        }
        else
        {
            *vrefBufState = VREFBUF_STATE_STARTING;
        }
    }
    else
    {
        *vrefBufState = VREFBUF_STATE_OFF;
    }

    return ( VREFBUF_REQUEST_OK );
}


/**
 * \brief Reads the trimming code from the register.
 *
 * \return Trimming code
 */
static vrefBuf_Trimming_t VrefBuf_Read_Trimming( void )
{
    const uint32_t trimmingValue = VrefBuf_Get_RegVal( VREFBUF_REG_CCR, VREFBUF_CCR_TRIM );

    return ( VREFBUF_REG_TO_TRIMMING( trimmingValue ) );
}


/**
 * \brief Verifies by read-back that the peripheral is configured as required.
 *
 * \param vrefBufConfig [in]: Pointer to VREFBUF configuration (valid)
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if mode, voltage level, trimming (when requested) and the
 *         ready flag (for the buffer mode) read back as required. Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
static vrefBuf_RequestState_t VrefBuf_Verify_Config( const vrefBuf_Config_t * const vrefBufConfig )
{
    vrefBuf_RequestState_t       retState  = VREFBUF_REQUEST_ERROR;
    vrefBuf_Mode_t               mode      = VREFBUF_MODE_CNT;
    vrefBuf_Voltage_t            voltage   = VREFBUF_VOLTAGE_CNT;
    const vrefBuf_RequestState_t modeState = VrefBuf_Read_Mode( &mode );
    const vrefBuf_RequestState_t voltState = VrefBuf_Read_Voltage( &voltage );

    if( ( VREFBUF_REQUEST_OK     == modeState ) &&
        ( VREFBUF_REQUEST_OK     == voltState ) &&
        ( vrefBufConfig->Mode    == mode      ) &&
        ( vrefBufConfig->Voltage == voltage   )    )
    {
        retState = VrefBuf_Verify_Output( vrefBufConfig );
    }
    else
    {
        /* Mode or voltage level does not read back as written */
        retState = VREFBUF_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Verifies by read-back the trimming (when requested) and the ready flag (for the buffer mode).
 *
 * \param vrefBufConfig [in]: Pointer to VREFBUF configuration (valid)
 *
 * \return Returns \ref VREFBUF_REQUEST_OK if the trimming and the ready flag read back as required.
 *         Otherwise returns \ref VREFBUF_REQUEST_ERROR.
 */
static vrefBuf_RequestState_t VrefBuf_Verify_Output( const vrefBuf_Config_t * const vrefBufConfig )
{
    vrefBuf_RequestState_t retState = VREFBUF_REQUEST_OK;
    vrefBuf_State_t        state    = VREFBUF_STATE_CNT;

    if( VREFBUF_TRIMMING_UNUSED != vrefBufConfig->Trimming )
    {
        const vrefBuf_Trimming_t trimming = VrefBuf_Read_Trimming();

        if( vrefBufConfig->Trimming != trimming )
        {
            retState = VREFBUF_REQUEST_ERROR;
        }
        else
        {
            /* Trimming reads back as written */
        }
    }
    else
    {
        /* Factory trimming is kept, any value is valid */
    }

    if( ( VREFBUF_REQUEST_OK  == retState            ) &&
        ( VREFBUF_MODE_BUFFER == vrefBufConfig->Mode )    )
    {
        (void)VrefBuf_Read_State( &state );

        if( VREFBUF_STATE_READY != state )
        {
            retState = VREFBUF_REQUEST_ERROR;
        }
        else
        {
            /* Output is ready */
        }
    }
    else
    {
        /* Previous step failed or the output is not driven */
    }

    return ( retState );
}

#endif /* VREFBUF */

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
