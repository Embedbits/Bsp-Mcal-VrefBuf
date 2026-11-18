/**
 * \author Mr.Nobody
 * \file ItTest_VrefBuf.c
 * \ingroup VrefBuf
 * \brief Integration tests of Voltage Reference Buffer (VREFBUF) module on target.
 *
 * VrefBuf module runs on the MCU together with the real RCC module and the hardware. Tests verify
 * behavior which cannot be verified by unit tests (emulated registers): the start-up of the buffer
 * to the ready state at every voltage level, the change of the level of the running buffer, the
 * switching of the mode, the trimming (including the loading of the factory trimming by the reset of
 * the peripheral and its suppression by a written trimming). The test cases use only the public
 * interface of the module.
 *
 * Lines with VREFBUF: every STM32H7 and STM32H7R / STM32H7S line. On a line without the peripheral the
 * test cases of the buffer are ignored and one test case verifies that the interface returns errors (it
 * is ignored on the lines of the family). The level 1.5 V is tested on the STM32H7 lines only.
 *
 * Used resources:
 * - VREFBUF, the external capacitor on VREF+ (datasheet: 0.5 ... 1.5 uF)
 * - VDDA of at least 2.8 V (the level 2.5 V needs the highest supply, datasheet VREFBUF characteristics)
 *
 * \note Wiring: the pin VREF+ must not be connected to VDDA on the board (the buffer would drive the
 *       supply rail and its ready flag is never set, because the output cannot reach a level below the
 *       supply), the connection (jumper or solder bridge, see the user manual of the board) shall be
 *       open. The NUCLEO boards of the inventory have it closed (the ADC measures against VDDA without
 *       the buffer), so the test cases which start the buffer need the fixture feature VREFBUF_OUTPUT
 *       of the board (IT_FEATURE_VREFBUF_OUTPUT) and are ignored on a board of the inventory without it.
 *       The mode VREFBUF_MODE_OFF is not used by the tests, it pulls the pin VREF+
 *       down to VSSA. The reference level is verified only by the ready flag of the hardware (the
 *       measurement of VREF+ by the ADC against VREFINT and by an external meter is not part of
 *       these test cases).
 * \note The buffer test cases were not run on a board with a free pin VREF+. The test case of the high
 *       impedance mode and the test case of the line without VREFBUF run on every board.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "VrefBuf_Port.h"                   /* Module under test              */
#include "Stm32.h"                          /* Presence of VREFBUF            */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

#if defined(VREFBUF)

static void                 ItVrefBuf_Require_BufferOutput( void );
static void                 ItVrefBuf_Init_Module       ( vrefBuf_Mode_t mode, vrefBuf_Voltage_t voltage );
static void                 ItVrefBuf_Check_State       ( vrefBuf_State_t expectedState );
static void                 ItVrefBuf_Check_Voltage     ( vrefBuf_Voltage_t expectedVoltage );
static vrefBuf_Trimming_t   ItVrefBuf_Get_OtherTrimming ( vrefBuf_Trimming_t trimming );

#endif /* VREFBUF */

/* ========================= SYMBOLIC CONSTANTS ============================= */

#if defined(VREFBUF)

/** Count of the voltage levels (the levels of the line) */
#define IT_VREFBUF_LEVEL_CNT                ( VREFBUF_VOLTAGE_CNT )

/** Start-up timeout of the buffer - count of the read-backs of the ready flag (the start-up takes
 *  below 1 ms, datasheet), long with a large margin for the default system clock */
#define IT_VREFBUF_TIMEOUT_CNT              ( 1000000u )

/** User trimming code used by the test of the initialization with a user trimming */
#define IT_VREFBUF_TRIM_USER                ( 0x20u )

#endif /* VREFBUF */

/* ========================== LOCAL VARIABLES =============================== */

#if defined(VREFBUF)

/** Voltage levels tested one after another */
static const vrefBuf_Voltage_t itVrefBuf_LevelList[ IT_VREFBUF_LEVEL_CNT ] =
{
    VREFBUF_VOLTAGE_2_5V,
    VREFBUF_VOLTAGE_2_048V,
    VREFBUF_VOLTAGE_1_8V,
#if !defined(STM32H7RS)
    VREFBUF_VOLTAGE_1_5V
#endif /* STM32H7RS */
};

#endif /* VREFBUF */

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    /* Every test case starts after the system reset, the peripheral is in the reset state */
}


void tearDown( void )
{
    /* Peripheral state is cleared by system reset after the test case */
}

/* =============================== TESTS ==================================== */

/**
 * \brief   VrefBuf_Init() with default configuration starts the buffer at 2.5 V.
 *
 * \details Initializes the module with the default configuration (buffer mode, 2.5 V, factory
 *          trimming). The initialization returns after the ready flag of the hardware is set.
 *
 * \par Expected results
 * - VrefBuf_Init() returns VREFBUF_REQUEST_OK.
 * - Mode buffer, level 2.5 V, state ready, the trimming reads (factory code).
 */
void It_VrefBuf_Init_DefaultConfig_BufferReady( void )
{
#if defined(VREFBUF)
    ItVrefBuf_Require_BufferOutput();

    vrefBuf_Config_t   config   = { VREFBUF_MODE_CNT, VREFBUF_VOLTAGE_CNT, VREFBUF_TRIMMING_UNUSED, 0u };
    vrefBuf_Mode_t     mode     = VREFBUF_MODE_CNT;
    vrefBuf_Trimming_t trimming = VREFBUF_TRIMMING_UNUSED;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Init( &config ) );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_BUFFER, mode );
    ItVrefBuf_Check_Voltage( VREFBUF_VOLTAGE_2_5V );
    ItVrefBuf_Check_State( VREFBUF_STATE_READY );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Trimming( &trimming ) );
    TEST_ASSERT_TRUE( VREFBUF_TRIMMING_MAX >= trimming );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   Every voltage level of the buffer reaches the ready state after the initialization.
 *
 * \details Initializes the buffer mode with 2.5 V, 2.048 V, 1.8 V and 1.5 V (STM32H7 lines) one after another
 *          (every initialization resets the peripheral and starts the buffer again).
 *
 * \note    Device errata bug AB#2305 (ES0392 Rev 15 2.11.1, ES0445 Rev 6 2.12.1, ES0478 Rev 12 2.9.1, ES0491 Rev 9
 *          2.12.1 "Overshoot on VREFBUF output", STM32H7 lines): the output might overshoot when the buffer is enabled
 *          while VREF+ has a residual voltage; the module does not wait for the discharge, the application does. The
 *          test case enables the buffer again with the capacitor charged and verifies the ready state only; the
 *          overshoot itself is not measured.
 *
 * \par Expected results
 * - VrefBuf_Init() returns VREFBUF_REQUEST_OK for every level.
 * - The level reads back and the state is ready.
 */
void It_VrefBuf_Init_EveryVoltage_ReachesReady( void )
{
#if defined(VREFBUF)
    ItVrefBuf_Require_BufferOutput();

    for( uint32_t levelIdx = 0u; IT_VREFBUF_LEVEL_CNT > levelIdx; levelIdx ++ )
    {
        ItVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, itVrefBuf_LevelList[ levelIdx ] );

        ItVrefBuf_Check_Voltage( itVrefBuf_LevelList[ levelIdx ] );
        ItVrefBuf_Check_State( VREFBUF_STATE_READY );
    }
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() in the high impedance mode leaves the buffer off.
 *
 * \details Initializes the high impedance mode (external reference mode, VREF+ is an input).
 *
 * \par Expected results
 * - VrefBuf_Init() returns VREFBUF_REQUEST_OK without waiting for the ready flag.
 * - Mode high impedance, state off.
 */
void It_VrefBuf_Init_ModeHighImpedance_StateOff( void )
{
#if defined(VREFBUF)
    vrefBuf_Mode_t mode = VREFBUF_MODE_CNT;

    ItVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_HIGH_IMPEDANCE, mode );
    ItVrefBuf_Check_State( VREFBUF_STATE_OFF );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   The level of the running buffer can be changed and the buffer returns to the ready state.
 *
 * \details Initializes the buffer mode with 2.5 V, then sets 1.5 V (STM32H7 lines), 1.8 V, 2.048 V and 2.5 V
 *          one after another
 *          (the module disables the buffer for the write of the level, the hardware allows it only
 *          with the buffer disabled).
 *
 * \note    Device errata bug AB#2305 (ES0392 Rev 15 2.11.1, ES0445 Rev 6 2.12.1, ES0478 Rev 12 2.9.1, ES0491 Rev 9
 *          2.12.1 "Overshoot on VREFBUF output", STM32H7 lines): the output might overshoot when the buffer is enabled
 *          while VREF+ has a residual voltage; the module does not wait for the discharge, the application does. The
 *          test case enables the buffer again with the capacitor charged and verifies the ready state only; the
 *          overshoot itself is not measured.
 *
 * \par Expected results
 * - VrefBuf_Set_Voltage() returns VREFBUF_REQUEST_OK for every level (it returns after the ready flag).
 * - The level reads back, the mode is buffer and the state is ready after every change.
 */
void It_VrefBuf_Set_Voltage_WhileRunning_ReturnsToReady( void )
{
#if defined(VREFBUF)
    ItVrefBuf_Require_BufferOutput();

    static const vrefBuf_Voltage_t changeList[ IT_VREFBUF_LEVEL_CNT ] =
    {
#if !defined(STM32H7RS)
        VREFBUF_VOLTAGE_1_5V,
#endif /* STM32H7RS */
        VREFBUF_VOLTAGE_1_8V,
        VREFBUF_VOLTAGE_2_048V,
        VREFBUF_VOLTAGE_2_5V
    };

    ItVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );

    for( uint32_t levelIdx = 0u; IT_VREFBUF_LEVEL_CNT > levelIdx; levelIdx ++ )
    {
        vrefBuf_Mode_t mode = VREFBUF_MODE_CNT;

        TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Voltage( changeList[ levelIdx ] ) );

        ItVrefBuf_Check_Voltage( changeList[ levelIdx ] );
        TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
        TEST_ASSERT_EQUAL( VREFBUF_MODE_BUFFER, mode );
        ItVrefBuf_Check_State( VREFBUF_STATE_READY );
    }
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   The mode of the buffer can be switched between the buffer and the high impedance mode.
 *
 * \details Initializes the buffer mode, switches to the high impedance mode (external reference) and back.
 *
 * \note    Device errata bug AB#2305 (ES0392 Rev 15 2.11.1, ES0445 Rev 6 2.12.1, ES0478 Rev 12 2.9.1, ES0491 Rev 9
 *          2.12.1 "Overshoot on VREFBUF output", STM32H7 lines): the output might overshoot when the buffer is enabled
 *          while VREF+ has a residual voltage; the module does not wait for the discharge, the application does. The
 *          test case enables the buffer again with the capacitor charged and verifies the ready state only; the
 *          overshoot itself is not measured.
 *
 * \par Expected results
 * - VrefBuf_Set_Mode() returns VREFBUF_REQUEST_OK both times.
 * - High impedance mode: state off. Buffer mode: state ready (the call returns after the ready flag).
 */
void It_VrefBuf_Set_Mode_HighImpedanceAndBack_StateFollows( void )
{
#if defined(VREFBUF)
    ItVrefBuf_Require_BufferOutput();

    vrefBuf_Mode_t mode = VREFBUF_MODE_CNT;

    ItVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_048V );
    ItVrefBuf_Check_State( VREFBUF_STATE_READY );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Mode( VREFBUF_MODE_HIGH_IMPEDANCE ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_HIGH_IMPEDANCE, mode );
    ItVrefBuf_Check_State( VREFBUF_STATE_OFF );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Mode( VREFBUF_MODE_BUFFER ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_BUFFER, mode );
    ItVrefBuf_Check_State( VREFBUF_STATE_READY );
    ItVrefBuf_Check_Voltage( VREFBUF_VOLTAGE_2_048V );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() writes the user trimming and the buffer starts with it.
 *
 * \details Initializes the buffer mode with a user trimming code.
 *
 * \par Expected results
 * - VrefBuf_Init() returns VREFBUF_REQUEST_OK, the trimming reads back as the written code.
 * - State ready (the buffer reached the requested level with the user trimming).
 */
void It_VrefBuf_Init_UserTrimming_ReadsBackAndReady( void )
{
#if defined(VREFBUF)
    ItVrefBuf_Require_BufferOutput();

    vrefBuf_Config_t   config   = { VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_048V,
                                    IT_VREFBUF_TRIM_USER, IT_VREFBUF_TIMEOUT_CNT };
    vrefBuf_Trimming_t trimming = VREFBUF_TRIMMING_UNUSED;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Init( &config ) );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Trimming( &trimming ) );
    TEST_ASSERT_EQUAL_UINT8( IT_VREFBUF_TRIM_USER, trimming );
    ItVrefBuf_Check_State( VREFBUF_STATE_READY );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   A written trimming keeps the buffer ready and survives the change of the level.
 *
 * \details Initializes the buffer mode, reads the factory trimming, writes another code, then changes the
 *          level of the running buffer. The hardware loads the factory trimming at a change of the level
 *          only until the first write of the trimming.
 *
 * \par Expected results
 * - The written code reads back, the buffer stays ready.
 * - After the change of the level the trimming is still the written code (the factory trimming of
 *   the new level is not loaded), the buffer is ready.
 */
void It_VrefBuf_Set_Trimming_UserCode_SurvivesLevelChange( void )
{
#if defined(VREFBUF)
    ItVrefBuf_Require_BufferOutput();

    vrefBuf_Trimming_t factoryTrimming = VREFBUF_TRIMMING_UNUSED;
    vrefBuf_Trimming_t userTrimming    = VREFBUF_TRIMMING_UNUSED;
    vrefBuf_Trimming_t trimming        = VREFBUF_TRIMMING_UNUSED;

    ItVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Trimming( &factoryTrimming ) );
    userTrimming = ItVrefBuf_Get_OtherTrimming( factoryTrimming );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Trimming( userTrimming ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Trimming( &trimming ) );
    TEST_ASSERT_EQUAL_UINT8( userTrimming, trimming );
    ItVrefBuf_Check_State( VREFBUF_STATE_READY );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Voltage( VREFBUF_VOLTAGE_1_8V ) );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Trimming( &trimming ) );
    TEST_ASSERT_EQUAL_UINT8( userTrimming, trimming );
    ItVrefBuf_Check_State( VREFBUF_STATE_READY );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() restores the factory trimming (reset of the peripheral).
 *
 * \details Initializes the buffer mode, writes a user trimming and initializes the module again; the
 *          initialization resets the peripheral, the hardware loads the factory trimming at the reset.
 *
 * \par Expected results
 * - After the second initialization the trimming equals the factory trimming read after the first one.
 * - The buffer is ready.
 */
void It_VrefBuf_Init_AfterUserTrimming_RestoresFactoryTrimming( void )
{
#if defined(VREFBUF)
    ItVrefBuf_Require_BufferOutput();

    vrefBuf_Trimming_t factoryTrimming = VREFBUF_TRIMMING_UNUSED;
    vrefBuf_Trimming_t trimming        = VREFBUF_TRIMMING_UNUSED;

    ItVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Trimming( &factoryTrimming ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Trimming( ItVrefBuf_Get_OtherTrimming( factoryTrimming ) ) );

    ItVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Trimming( &trimming ) );
    TEST_ASSERT_EQUAL_UINT8( factoryTrimming, trimming );
    ItVrefBuf_Check_State( VREFBUF_STATE_READY );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Deinit() stops the module and the buffer can be started again.
 *
 * \details Initializes the buffer mode, deinitializes the module (reset of the peripheral, clock disabled),
 *          then initializes it again.
 *
 * \note    Device errata bug AB#2305 (ES0392 Rev 15 2.11.1, ES0445 Rev 6 2.12.1, ES0478 Rev 12 2.9.1, ES0491 Rev 9
 *          2.12.1 "Overshoot on VREFBUF output", STM32H7 lines): the output might overshoot when the buffer is enabled
 *          while VREF+ has a residual voltage; the module does not wait for the discharge, the application does. The
 *          test case enables the buffer again with the capacitor charged and verifies the ready state only; the
 *          overshoot itself is not measured.
 *
 * \par Expected results
 * - VrefBuf_Deinit() returns VREFBUF_REQUEST_OK, afterwards the state function returns VREFBUF_REQUEST_ERROR
 *   (module not initialized).
 * - The second initialization returns VREFBUF_REQUEST_OK and the buffer is ready.
 */
void It_VrefBuf_Deinit_Initialized_ModuleStopsAndRestarts( void )
{
#if defined(VREFBUF)
    ItVrefBuf_Require_BufferOutput();

    vrefBuf_State_t state = VREFBUF_STATE_CNT;

    ItVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Deinit() );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_State( &state ) );

    ItVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );
    ItVrefBuf_Check_State( VREFBUF_STATE_READY );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   The interface returns errors on the line without the VREFBUF peripheral.
 *
 * \details A line without the VREFBUF peripheral (none in the family). Calls the functions of the module (the
 *          lines with the peripheral ignore the test case).
 *
 * \par Expected results
 * - Init, Deinit, default configuration, setters and getters return VREFBUF_REQUEST_ERROR, the output
 *   variables are not changed.
 */
void It_VrefBuf_NoPeripheral_AllFunctions_ReturnError( void )
{
#if !defined(VREFBUF)
    vrefBuf_Config_t   config   = { VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V, VREFBUF_TRIMMING_UNUSED, 1u };
    vrefBuf_Mode_t     mode     = VREFBUF_MODE_CNT;
    vrefBuf_Voltage_t  voltage  = VREFBUF_VOLTAGE_CNT;
    vrefBuf_State_t    state    = VREFBUF_STATE_CNT;
    vrefBuf_Trimming_t trimming = VREFBUF_TRIMMING_UNUSED;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Init( &config ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Deinit() );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Mode( VREFBUF_MODE_BUFFER ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Voltage( VREFBUF_VOLTAGE_2_5V ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Trimming( 0u ) );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Voltage( &voltage ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_State( &state ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Trimming( &trimming ) );

    TEST_ASSERT_EQUAL( VREFBUF_MODE_CNT, mode );
    TEST_ASSERT_EQUAL( VREFBUF_VOLTAGE_CNT, voltage );
    TEST_ASSERT_EQUAL( VREFBUF_STATE_CNT, state );
    TEST_ASSERT_EQUAL_UINT8( VREFBUF_TRIMMING_UNUSED, trimming );
#else
    TEST_IGNORE_MESSAGE( "MCU with VREFBUF" );
#endif
}

/* =========================== LOCAL FUNCTIONS ============================== */

#if defined(VREFBUF)

/** Ignores the test on a board whose pin VREF+ is not free for the buffer output. The ready flag of the buffer is set
 *  only when the output reaches the requested level, which cannot happen when the pin VREF+ is connected to VDDA
 *  (NUCLEO boards: the connection to the supply is closed by default). A board of the inventory without the
 *  fixture feature VREFBUF_OUTPUT (IT_FIXTURE defined, the feature not listed) ignores the test, a board without
 *  any inventory (IT_FIXTURE not defined) runs it. */
static void ItVrefBuf_Require_BufferOutput( void )
{
#if defined(IT_FEATURE_VREFBUF_OUTPUT) || \
    !defined(IT_FIXTURE)
    /* The pin VREF+ is free (or the board is not in the inventory), the test runs */
#else
    TEST_IGNORE_MESSAGE( "Board without the fixture VREFBUF_OUTPUT (the pin VREF+ is connected to VDDA)" );
#endif
}


/** Initializes the module (factory trimming kept, start-up timeout of the tests) and expects success */
static void ItVrefBuf_Init_Module( vrefBuf_Mode_t mode, vrefBuf_Voltage_t voltage )
{
    vrefBuf_Config_t config = { VREFBUF_MODE_CNT, VREFBUF_VOLTAGE_CNT,
                                VREFBUF_TRIMMING_UNUSED, IT_VREFBUF_TIMEOUT_CNT };

    config.Mode    = mode;
    config.Voltage = voltage;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Init( &config ) );
}


/** Checks the state of the buffer */
static void ItVrefBuf_Check_State( vrefBuf_State_t expectedState )
{
    vrefBuf_State_t state = VREFBUF_STATE_CNT;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_State( &state ) );
    TEST_ASSERT_EQUAL( expectedState, state );
}


/** Checks the voltage level of the buffer */
static void ItVrefBuf_Check_Voltage( vrefBuf_Voltage_t expectedVoltage )
{
    vrefBuf_Voltage_t voltage = VREFBUF_VOLTAGE_CNT;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Voltage( &voltage ) );
    TEST_ASSERT_EQUAL( expectedVoltage, voltage );
}


/** Returns a trimming code which differs from the given one by one step (the next code, the previous
 *  one for the highest code) */
static vrefBuf_Trimming_t ItVrefBuf_Get_OtherTrimming( vrefBuf_Trimming_t trimming )
{
    vrefBuf_Trimming_t otherTrimming = 0u;

    if( VREFBUF_TRIMMING_MAX > trimming )
    {
        otherTrimming = (vrefBuf_Trimming_t)( trimming + 1u );
    }
    else
    {
        otherTrimming = (vrefBuf_Trimming_t)( trimming - 1u );
    }

    return ( otherTrimming );
}

#endif /* VREFBUF */
