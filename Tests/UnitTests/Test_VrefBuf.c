/**
 * \author Mr.Nobody
 * \file Test_VrefBuf.c
 * \ingroup VrefBuf
 * \brief Unit tests of Voltage Reference Buffer (VREFBUF) module.
 *
 * VrefBuf.c and VrefBuf_Reg.c are compiled unchanged, VREFBUF registers
 * are emulated by RegMem, the RCC module (peripheral clock and reset) is mocked by CMock.
 * The code of the lines with the peripheral (all STM32H5 lines except STM32H503) is tested on
 * those lines, the code of the line without it (STM32H503, no CMSIS definition VREFBUF: every
 * public function only returns the error value) is tested there by the peripheral check test
 * cases. Every test case exists on every line, the case of the other kind is reported
 * as ignored (the runner of the test cases ignores the preprocessor).
 *
 * \note Limitations of the register emulation (RegMem registers are plain memory):
 *       - The ready flag VRR and the factory loading of the trimming (TRIM is loaded at the change of
 *         VRS, until the first write of TRIM) are not emulated by the plain memory. The ready flag
 *         is emulated by the HW model of RegMem (UtVrefBuf_HwModel_Ready: set while ENVR = 1 and
 *         HIZ = 0, cleared while the buffer is off, kept in the hold mode). The model is evaluated
 *         after every store, not while the module waits - a delayed ready flag cannot be emulated,
 *         the timeout is tested with a flag which is never set.
 *       - A register value which HW would not accept (read-back mismatch) is created by the HW
 *         models of RegMem (UtVrefBuf_HwModel_xxxNotKept), which run after every store.
 *       - Reset state of the peripheral (ENVR = 0, HIZ = 1) and the factory trimming are not
 *         present after RegMem_Reset(), the registers are zero (mode OFF); the reset of the
 *         peripheral by RCC is a mocked call, tests preset the register values they need.
 *       - Every test case is executed in its own process (CTest), the runtime data
 *         of the module start from the reset state in every test.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "VrefBuf_Port.h"                   /* Module under test              */
#include "VrefBuf_Reg.h"                    /* Register access of the module  */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "Stm32.h"                          /* VREFBUF registers definition   */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

#if defined(VREFBUF)

static void                 UtVrefBuf_Expect_InitClock      ( void );
static void                 UtVrefBuf_Expect_Deinit         ( rcc_RequestState_t resetActState,
                                                              rcc_RequestState_t resetInactState,
                                                              rcc_RequestState_t clkState );
static void                 UtVrefBuf_Expect_DeinitOk       ( void );
static void                 UtVrefBuf_Set_Model             ( regMem_ModelCallback_t model );
static vrefBuf_Config_t     UtVrefBuf_Get_Config            ( vrefBuf_Mode_t mode,
                                                              vrefBuf_Voltage_t voltage,
                                                              vrefBuf_Trimming_t trimming );
static void                 UtVrefBuf_Init_Module           ( vrefBuf_Mode_t mode, vrefBuf_Voltage_t voltage );
static void                 UtVrefBuf_Check_InitRejected    ( const vrefBuf_Config_t * const config );
static void                 UtVrefBuf_Check_InitFailed      ( const vrefBuf_Config_t * const config );
static void                 UtVrefBuf_Check_DeinitError     ( rcc_RequestState_t resetActState,
                                                              rcc_RequestState_t resetInactState,
                                                              rcc_RequestState_t clkState );
static void                 UtVrefBuf_Check_State           ( vrefBuf_State_t expectedState );
static void                 UtVrefBuf_Reset_Record          ( void );

static void                 UtVrefBuf_HwModel_Ready         ( void );
static void                 UtVrefBuf_HwModel_Sequence      ( void );
static void                 UtVrefBuf_HwModel_NotReady      ( void );
static void                 UtVrefBuf_HwModel_EnvrNotKept   ( void );
static void                 UtVrefBuf_HwModel_EnvrStuck     ( void );
static void                 UtVrefBuf_HwModel_VrsNotKept    ( void );
static void                 UtVrefBuf_HwModel_TrimNotKept   ( void );

#endif /* VREFBUF */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Trimming code stored in a configuration which a function of the line without VREFBUF must not change */
#define UT_VREFBUF_SENTINEL_TRIM            ( 0x11u )

/** Timeout stored in a configuration which a function of the line without VREFBUF must not change */
#define UT_VREFBUF_SENTINEL_TIMEOUT         ( 0x22u )

/** Trimming code written by the setter test of the line without VREFBUF */
#define UT_VREFBUF_NO_PERIPH_TRIM         ( 0x15u )

#if defined(VREFBUF)

/** Start-up timeout used by tests - count of the read-backs of the ready flag (short, the flag is either
 *  set at the first read or never) */
#define UT_VREFBUF_TIMEOUT_CNT              ( 20u )

/** Default start-up timeout of VrefBuf_Get_DefaultConfig */
#define UT_VREFBUF_DEFAULT_TIMEOUT_CNT      ( 100000u )

/** Mask of the bits which select the electrical mode */
#define UT_VREFBUF_MODE_MASK                ( VREFBUF_CSR_ENVR | VREFBUF_CSR_HIZ )

/** Mode bits of the buffer mode (ENVR = 1, HIZ = 0) */
#define UT_VREFBUF_MODE_BITS_BUFFER         ( VREFBUF_CSR_ENVR )

/** Mode bits of the high impedance mode (ENVR = 0, HIZ = 1) */
#define UT_VREFBUF_MODE_BITS_HIZ            ( VREFBUF_CSR_HIZ )

/** Mode bits of the hold mode of the hardware (ENVR = 1, HIZ = 1), not a mode of the module */
#define UT_VREFBUF_MODE_BITS_HOLD           ( VREFBUF_CSR_ENVR | VREFBUF_CSR_HIZ )

/** Factory trimming code preset by tests (the hardware loads it, RegMem does not) */
#define UT_VREFBUF_TRIM_FACTORY             ( 0x2Au )

/** User trimming code used by tests */
#define UT_VREFBUF_TRIM_USER                ( 0x15u )

/** Highest trimming code (6-bit field) */
#define UT_VREFBUF_TRIM_MAX                 ( 63u )

/** Bits of CCR register above the TRIM field (reserved, never written by the module) */
#define UT_VREFBUF_CCR_RESERVED             ( 0xA5A5A5C0u )

/** Bits of CSR register preset by tests which must stay untouched where the module rejects a request
 *  (reserved bits and the ready flag) */
#define UT_VREFBUF_CSR_PATTERN              ( 0xA5A5A5A5u )

/** First reserved value of the field VRS */
#define UT_VREFBUF_VRS_RESERVED_FIRST       ( 3u )

/** Last reserved value of the field VRS */
#define UT_VREFBUF_VRS_RESERVED_LAST        ( 7u )

/** Count of the voltage levels tested one after another */
#define UT_VREFBUF_LEVEL_CNT                ( 3u )

/** All bits of a 32-bit register value (mask of the accessor tests) */
#define UT_VREFBUF_ALL_BITS                 ( 0xFFFFFFFFu )

/** Timeout of the wait for the ready flag in the register protocol test */
#define UT_VREFBUF_WAIT_CNT                 ( 3u )

/** Count of the stores which change the mode bits while the level of the running buffer is changed
 *  (buffer to high impedance mode and back) */
#define UT_VREFBUF_MODE_CHANGES_PER_LEVEL   ( 2u )

#endif /* VREFBUF */

/* ========================== LOCAL VARIABLES =============================== */

#if defined(VREFBUF)

/** Voltage levels in the order of the test of all levels (the last one is the reset level) */
static const vrefBuf_Voltage_t utVrefBuf_LevelList[ UT_VREFBUF_LEVEL_CNT ] =
{
    VREFBUF_VOLTAGE_1_8V,
    VREFBUF_VOLTAGE_2_048V,
    VREFBUF_VOLTAGE_2_5V
};

/** Register values of the voltage levels, indexed as \ref utVrefBuf_LevelList */
static const uint32_t utVrefBuf_LevelRegList[ UT_VREFBUF_LEVEL_CNT ] =
{
    VREFBUF_CSR_VRS_1,
    VREFBUF_CSR_VRS_0,
    0u
};

/** Value of CSR register seen by the previous run of the recording HW model */
static volatile uint32_t            utVrefBuf_PrevCsr;

/** Value of the TRIM field seen by the previous run of the recording HW model */
static volatile uint32_t            utVrefBuf_PrevTrim;

/** Count of the stores which changed VRS while the buffer was enabled (hardware forbids it) */
static volatile uint32_t            utVrefBuf_VrsWhileEnabledCnt;

/** Count of the stores after which the pin VREF+ was pulled down (mode OFF) */
static volatile uint32_t            utVrefBuf_PulledDownCnt;

/** Count of the stores after which the buffer was in the high impedance mode */
static volatile uint32_t            utVrefBuf_HighImpedanceCnt;

/** Count of the stores which changed the mode bits */
static volatile uint32_t            utVrefBuf_ModeChangeCnt;

/** The buffer was seen enabled for the first time (\ref VREFBUF_FLAG_ACTIVE) */
static volatile vrefBuf_FlagState_t utVrefBuf_EnableSeen;

/** Value of the TRIM field when the buffer was enabled for the first time */
static volatile uint32_t            utVrefBuf_TrimAtEnable;

/** The TRIM field was seen changed for the first time (\ref VREFBUF_FLAG_ACTIVE) */
static volatile vrefBuf_FlagState_t utVrefBuf_TrimSeen;

/** Value of the field VRS when the TRIM field was changed for the first time */
static volatile uint32_t            utVrefBuf_VrsAtTrim;

#endif /* VREFBUF */

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

#if defined(VREFBUF)
    UtVrefBuf_Reset_Record();
#endif
}


void tearDown( void )
{
    /* Mocks are verified by generated runner */
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   VrefBuf_Get_ModuleVersion() returns version of the module.
 *
 * \details Reads the module version structure (common for the lines with and without VREFBUF).
 *
 * \par Expected results
 * - Version is 0.1.0 (Major 0, Minor 1, Patch 0).
 */
void Ut_VrefBuf_Get_ModuleVersion_ReturnsVersion( void )
{
    vrefBuf_ModuleVersion_t version = VrefBuf_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 0u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 1u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}


/**
 * \brief   VrefBuf_Task() has no periodic work.
 *
 * \details Presets the registers (lines with VREFBUF), calls the task twice.
 *
 * \par Expected results
 * - The registers are untouched, no RCC function is called.
 */
void Ut_VrefBuf_Task_NoPeriodicWork_DoesNothing( void )
{
#if defined(VREFBUF)
    VREFBUF->CSR = UT_VREFBUF_CSR_PATTERN;
    VREFBUF->CCR = UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY;
#endif

    VrefBuf_Task();
    VrefBuf_Task();

#if defined(VREFBUF)
    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CSR_PATTERN, VREFBUF->CSR );
    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY, VREFBUF->CCR );
#endif
}

/* =========================== DEFAULT CONFIG =============================== */

/**
 * \brief   VrefBuf_Get_DefaultConfig() fills default configuration.
 *
 * \details Reads default configuration, then calls the function with NULL.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, buffer mode, 2.5 V, factory trimming kept (unused), timeout 100000 read-backs.
 * - NULL pointer: VREFBUF_REQUEST_ERROR.
 */
void Ut_VrefBuf_Get_DefaultConfig_ReturnsDefaults( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t config;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( VREFBUF_MODE_BUFFER, config.Mode );
    TEST_ASSERT_EQUAL( VREFBUF_VOLTAGE_2_5V, config.Voltage );
    TEST_ASSERT_EQUAL_UINT8( VREFBUF_TRIMMING_UNUSED, config.Trimming );
    TEST_ASSERT_EQUAL_UINT32( UT_VREFBUF_DEFAULT_TIMEOUT_CNT, config.StartTimeout );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_DefaultConfig( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}

/* ============================ INITIALIZATION ============================== */

/**
 * \brief   VrefBuf_Init() with default configuration starts the buffer at 2.5 V.
 *
 * \details Initializes the module with the default configuration (buffer mode, 2.5 V, factory
 *          trimming); the HW model sets the ready flag while the buffer is enabled.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, peripheral clock activated and peripheral reset pulsed by RCC (in this order).
 * - CSR: ENVR set, HIZ clear, VRS = 000, ready flag set. CCR untouched.
 * - Mode buffer, level 2.5 V, state ready.
 */
void Ut_VrefBuf_Init_DefaultConfig_BufferReadyAt2_5V( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t  config;
    vrefBuf_Mode_t    mode    = VREFBUF_MODE_CNT;
    vrefBuf_Voltage_t voltage = VREFBUF_VOLTAGE_CNT;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_DefaultConfig( &config ) );

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Expect_InitClock();

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_ENVR | VREFBUF_CSR_VRR, VREFBUF->CSR );
    TEST_ASSERT_EQUAL_HEX32( 0u, VREFBUF->CCR );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_BUFFER, mode );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Voltage( &voltage ) );
    TEST_ASSERT_EQUAL( VREFBUF_VOLTAGE_2_5V, voltage );
    UtVrefBuf_Check_State( VREFBUF_STATE_READY );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() writes the level 2.048 V.
 *
 * \details Initializes the buffer mode with the level 2.048 V.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, VRS = 001, ENVR set, HIZ clear, ready flag set.
 * - Level 2.048 V, state ready.
 */
void Ut_VrefBuf_Init_Voltage2_048V_WritesVrs( void )
{
#if defined(VREFBUF)
    vrefBuf_Voltage_t voltage = VREFBUF_VOLTAGE_CNT;

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_048V );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_0 | VREFBUF_CSR_ENVR | VREFBUF_CSR_VRR, VREFBUF->CSR );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Voltage( &voltage ) );
    TEST_ASSERT_EQUAL( VREFBUF_VOLTAGE_2_048V, voltage );
    UtVrefBuf_Check_State( VREFBUF_STATE_READY );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() writes the level 1.8 V.
 *
 * \details Initializes the buffer mode with the level 1.8 V.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, VRS = 010, ENVR set, HIZ clear, ready flag set.
 * - Level 1.8 V, state ready.
 */
void Ut_VrefBuf_Init_Voltage1_8V_WritesVrs( void )
{
#if defined(VREFBUF)
    vrefBuf_Voltage_t voltage = VREFBUF_VOLTAGE_CNT;

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_1_8V );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_1 | VREFBUF_CSR_ENVR | VREFBUF_CSR_VRR, VREFBUF->CSR );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Voltage( &voltage ) );
    TEST_ASSERT_EQUAL( VREFBUF_VOLTAGE_1_8V, voltage );
    UtVrefBuf_Check_State( VREFBUF_STATE_READY );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() in the high impedance mode does not wait for the ready flag.
 *
 * \details Initializes the high impedance mode without a HW model (the ready flag is never set).
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK (no wait), CSR: ENVR clear, HIZ set, VRS = 000.
 * - Mode high impedance, state off.
 */
void Ut_VrefBuf_Init_ModeHighImpedance_SetsHizWithoutWaiting( void )
{
#if defined(VREFBUF)
    vrefBuf_Mode_t mode = VREFBUF_MODE_CNT;

    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_HIZ, VREFBUF->CSR );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_HIGH_IMPEDANCE, mode );
    UtVrefBuf_Check_State( VREFBUF_STATE_OFF );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() in the off mode clears ENVR and HIZ.
 *
 * \details Presets the reset state of the peripheral (HIZ set), initializes the off mode with
 *          the level 1.8 V without a HW model.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK (no wait), CSR: ENVR clear, HIZ clear, VRS = 010.
 * - Mode off, level 1.8 V, state off.
 */
void Ut_VrefBuf_Init_ModeOff_ClearsEnvrAndHiz( void )
{
#if defined(VREFBUF)
    vrefBuf_Mode_t mode = VREFBUF_MODE_CNT;

    VREFBUF->CSR = VREFBUF_CSR_HIZ;     /* Reset state of the peripheral */

    UtVrefBuf_Init_Module( VREFBUF_MODE_OFF, VREFBUF_VOLTAGE_1_8V );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_1, VREFBUF->CSR );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_OFF, mode );
    UtVrefBuf_Check_State( VREFBUF_STATE_OFF );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() writes the user trimming after the level and before the buffer is enabled.
 *
 * \details Initializes the buffer mode with the level 2.048 V and a user trimming; the recording HW model
 *          stores the TRIM field at the moment the buffer is enabled and the field VRS at the moment
 *          the TRIM field changes. A later change of VRS would load the factory trimming, the buffer
 *          shall start with the final trimming.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, CCR.TRIM is the user trimming, reserved bits untouched.
 * - TRIM was already written when the buffer was enabled, VRS was already written when TRIM was written.
 * - The trimming reads back by VrefBuf_Get_Trimming().
 */
void Ut_VrefBuf_Init_UserTrimming_WritesTrimAfterLevelBeforeEnable( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t   config   = UtVrefBuf_Get_Config( VREFBUF_MODE_BUFFER,
                                                        VREFBUF_VOLTAGE_2_048V,
                                                        UT_VREFBUF_TRIM_USER );
    vrefBuf_Trimming_t trimming = VREFBUF_TRIMMING_UNUSED;

    VREFBUF->CCR = UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY;

    UtVrefBuf_Reset_Record();
    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Sequence );
    UtVrefBuf_Expect_InitClock();

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_USER, VREFBUF->CCR );
    TEST_ASSERT_EQUAL( VREFBUF_FLAG_ACTIVE, utVrefBuf_EnableSeen );
    TEST_ASSERT_EQUAL_UINT32( UT_VREFBUF_TRIM_USER, utVrefBuf_TrimAtEnable );
    TEST_ASSERT_EQUAL( VREFBUF_FLAG_ACTIVE, utVrefBuf_TrimSeen );
    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_0, utVrefBuf_VrsAtTrim );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Trimming( &trimming ) );
    TEST_ASSERT_EQUAL_UINT8( UT_VREFBUF_TRIM_USER, trimming );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() keeps the factory trimming when no trimming is requested.
 *
 * \details Presets the factory trimming (loaded by the hardware) and initializes the buffer mode with
 *          the trimming VREFBUF_TRIMMING_UNUSED.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, CCR is not written (factory value and reserved bits stay).
 * - VrefBuf_Get_Trimming() returns the factory trimming.
 */
void Ut_VrefBuf_Init_TrimmingUnused_KeepsFactoryTrimming( void )
{
#if defined(VREFBUF)
    vrefBuf_Trimming_t trimming = VREFBUF_TRIMMING_UNUSED;

    VREFBUF->CCR = UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY;

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY, VREFBUF->CCR );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Trimming( &trimming ) );
    TEST_ASSERT_EQUAL_UINT8( UT_VREFBUF_TRIM_FACTORY, trimming );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() accepts the highest and the lowest trimming code.
 *
 * \details Initializes the high impedance mode with the trimming 0, then (re-initialization) with
 *          the trimming 63.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK both times, the TRIM field reads back 0 and 63.
 */
void Ut_VrefBuf_Init_TrimmingLimits_AreAccepted( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t   lowConfig  = UtVrefBuf_Get_Config( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V, 0u );
    vrefBuf_Config_t   highConfig = UtVrefBuf_Get_Config( VREFBUF_MODE_HIGH_IMPEDANCE,
                                                          VREFBUF_VOLTAGE_2_5V,
                                                          UT_VREFBUF_TRIM_MAX );
    vrefBuf_Trimming_t trimming   = VREFBUF_TRIMMING_UNUSED;

    VREFBUF->CCR = UT_VREFBUF_TRIM_FACTORY;

    UtVrefBuf_Expect_InitClock();
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Init( &lowConfig ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Trimming( &trimming ) );
    TEST_ASSERT_EQUAL_UINT8( 0u, trimming );

    UtVrefBuf_Expect_InitClock();
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Init( &highConfig ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Trimming( &trimming ) );
    TEST_ASSERT_EQUAL_UINT8( UT_VREFBUF_TRIM_MAX, trimming );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() can be repeated with another configuration.
 *
 * \details Initializes the buffer mode with 2.5 V, then the high impedance mode with 1.8 V.
 *
 * \note    Device errata bug AB#1996 (ES0565 Rev 7 2.8.1 "Slow discharge of the capacitor connected to VREF+ pin"):
 *          the pull-down resistor on the VREFBUF output is missing (STM32H562, STM32H563, STM32H573), so the
 *          capacitor on VREF+ discharges slowly after the output is switched down; the module does not wait for the
 *          discharge, the application does. The test proves the sequence of the accesses which switch the output down
 *          only: RegMem does not emulate the capacitor, the discharge is not tested.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK both times, each initialization activates the clock and resets the peripheral.
 * - The second configuration is read back: high impedance mode, 1.8 V, state off.
 */
void Ut_VrefBuf_Init_Repeated_AppliesNewConfiguration( void )
{
#if defined(VREFBUF)
    vrefBuf_Voltage_t voltage = VREFBUF_VOLTAGE_CNT;
    vrefBuf_Mode_t    mode    = VREFBUF_MODE_CNT;

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );
    UtVrefBuf_Check_State( VREFBUF_STATE_READY );

    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_1_8V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_HIGH_IMPEDANCE, mode );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Voltage( &voltage ) );
    TEST_ASSERT_EQUAL( VREFBUF_VOLTAGE_1_8V, voltage );
    UtVrefBuf_Check_State( VREFBUF_STATE_OFF );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() rejects NULL configuration.
 *
 * \details Calls VrefBuf_Init() with NULL pointer.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, RCC not called, registers untouched, module not initialized.
 */
void Ut_VrefBuf_Init_NullConfig_ReturnsErrorWithoutAccess( void )
{
#if defined(VREFBUF)
    VREFBUF->CSR = UT_VREFBUF_CSR_PATTERN;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Init( NULL ) );

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CSR_PATTERN, VREFBUF->CSR );
    UtVrefBuf_Check_State( VREFBUF_STATE_CNT );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() rejects a mode which is not in the list.
 *
 * \details Initializes with the mode VREFBUF_MODE_CNT.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, RCC not called, registers untouched, module not initialized.
 */
void Ut_VrefBuf_Init_InvalidMode_ReturnsErrorWithoutAccess( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t config = UtVrefBuf_Get_Config( VREFBUF_MODE_CNT, VREFBUF_VOLTAGE_2_5V, VREFBUF_TRIMMING_UNUSED );

    UtVrefBuf_Check_InitRejected( &config );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() rejects a voltage level which is not in the list.
 *
 * \details Initializes with the level VREFBUF_VOLTAGE_CNT.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, RCC not called, registers untouched, module not initialized.
 */
void Ut_VrefBuf_Init_InvalidVoltage_ReturnsErrorWithoutAccess( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t config = UtVrefBuf_Get_Config( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_CNT, VREFBUF_TRIMMING_UNUSED );

    UtVrefBuf_Check_InitRejected( &config );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() rejects a trimming code above the field.
 *
 * \details Initializes with the trimming 64 (one above the maximum) and with 254 (not the unused marker).
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR both times, RCC not called, registers untouched, module not initialized.
 */
void Ut_VrefBuf_Init_TrimmingAboveMax_ReturnsErrorWithoutAccess( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t aboveConfig = UtVrefBuf_Get_Config( VREFBUF_MODE_BUFFER,
                                                         VREFBUF_VOLTAGE_2_5V,
                                                         UT_VREFBUF_TRIM_MAX + 1u );
    vrefBuf_Config_t markerConfig = UtVrefBuf_Get_Config( VREFBUF_MODE_BUFFER,
                                                          VREFBUF_VOLTAGE_2_5V,
                                                          VREFBUF_TRIMMING_UNUSED - 1u );

    UtVrefBuf_Check_InitRejected( &aboveConfig );
    UtVrefBuf_Check_InitRejected( &markerConfig );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() rejects the zero start-up timeout.
 *
 * \details Initializes with the timeout 0 (the ready flag could not be read at all).
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, RCC not called, registers untouched, module not initialized.
 */
void Ut_VrefBuf_Init_ZeroTimeout_ReturnsErrorWithoutAccess( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t config = UtVrefBuf_Get_Config( VREFBUF_MODE_BUFFER,
                                                    VREFBUF_VOLTAGE_2_5V,
                                                    VREFBUF_TRIMMING_UNUSED );

    config.StartTimeout = 0u;

    UtVrefBuf_Check_InitRejected( &config );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() reports the ready flag not set within the timeout.
 *
 * \details Initializes the buffer mode without a HW model (the ready flag is never set).
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, the module is deinitialized (peripheral reset and clock disabled again by RCC).
 * - Module not initialized (the state function returns an error).
 */
void Ut_VrefBuf_Init_ReadyNeverSet_ReturnsErrorAndDeinitializes( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t config = UtVrefBuf_Get_Config( VREFBUF_MODE_BUFFER,
                                                    VREFBUF_VOLTAGE_2_5V,
                                                    VREFBUF_TRIMMING_UNUSED );

    UtVrefBuf_Check_InitFailed( &config );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() verifies the written mode by read-back.
 *
 * \details The HW model clears ENVR after every store (the written mode does not stay).
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, module deinitialized.
 */
void Ut_VrefBuf_Init_ModeReadBackMismatch_ReturnsErrorAndDeinitializes( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t config = UtVrefBuf_Get_Config( VREFBUF_MODE_BUFFER,
                                                    VREFBUF_VOLTAGE_2_5V,
                                                    VREFBUF_TRIMMING_UNUSED );

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_EnvrNotKept );

    UtVrefBuf_Check_InitFailed( &config );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() verifies the written voltage level by read-back and enables nothing after a failure.
 *
 * \details The HW model clears the field VRS after every store (the written level does not stay).
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, module deinitialized.
 * - The buffer was not enabled (ENVR clear): the failure of the level stops the sequence.
 */
void Ut_VrefBuf_Init_VoltageReadBackMismatch_ReturnsErrorBufferNotEnabled( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t config = UtVrefBuf_Get_Config( VREFBUF_MODE_BUFFER,
                                                    VREFBUF_VOLTAGE_2_048V,
                                                    VREFBUF_TRIMMING_UNUSED );

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_VrsNotKept );

    UtVrefBuf_Check_InitFailed( &config );

    TEST_ASSERT_BITS_LOW( VREFBUF_CSR_ENVR, VREFBUF->CSR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() verifies the written trimming by read-back and enables nothing after a failure.
 *
 * \details The HW model clears the field TRIM after every store (the written trimming does not stay).
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, module deinitialized.
 * - The buffer was not enabled (ENVR clear): the failure of the trimming stops the sequence.
 */
void Ut_VrefBuf_Init_TrimmingReadBackMismatch_ReturnsErrorBufferNotEnabled( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t config = UtVrefBuf_Get_Config( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V, UT_VREFBUF_TRIM_USER );

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_TrimNotKept );

    UtVrefBuf_Check_InitFailed( &config );

    TEST_ASSERT_BITS_LOW( VREFBUF_CSR_ENVR, VREFBUF->CSR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() reports the peripheral clock not available.
 *
 * \details RCC activation of the peripheral clock returns an error.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, no register written, the module is deinitialized (peripheral reset and
 *   clock disabled by RCC).
 */
void Ut_VrefBuf_Init_RccClockError_ReturnsErrorWithoutWrite( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t config = UtVrefBuf_Get_Config( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V, UT_VREFBUF_TRIM_USER );

    VREFBUF->CSR = UT_VREFBUF_CSR_PATTERN;
    VREFBUF->CCR = UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY;

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_VREF, RCC_REQUEST_ERROR );
    UtVrefBuf_Expect_DeinitOk();

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CSR_PATTERN, VREFBUF->CSR );
    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY, VREFBUF->CCR );
    UtVrefBuf_Check_State( VREFBUF_STATE_CNT );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() reports the failed activation of the peripheral reset.
 *
 * \details RCC activation of the reset returns an error, the release of the reset is still requested.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, no register written, the module is deinitialized.
 */
void Ut_VrefBuf_Init_RccResetActiveError_ReturnsErrorWithoutWrite( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t config = UtVrefBuf_Get_Config( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V, UT_VREFBUF_TRIM_USER );

    VREFBUF->CSR = UT_VREFBUF_CSR_PATTERN;

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_VREF, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( RCC_PERIPH_VREF, RCC_REQUEST_ERROR );
    Rcc_Set_ResetInactive_ExpectAndReturn( RCC_PERIPH_VREF, RCC_REQUEST_OK );
    UtVrefBuf_Expect_DeinitOk();

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CSR_PATTERN, VREFBUF->CSR );
    UtVrefBuf_Check_State( VREFBUF_STATE_CNT );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Init() reports the failed release of the peripheral reset.
 *
 * \details RCC release of the reset returns an error.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, no register written, the module is deinitialized.
 */
void Ut_VrefBuf_Init_RccResetInactiveError_ReturnsErrorWithoutWrite( void )
{
#if defined(VREFBUF)
    vrefBuf_Config_t config = UtVrefBuf_Get_Config( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V, UT_VREFBUF_TRIM_USER );

    VREFBUF->CSR = UT_VREFBUF_CSR_PATTERN;

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_VREF, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( RCC_PERIPH_VREF, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( RCC_PERIPH_VREF, RCC_REQUEST_ERROR );
    UtVrefBuf_Expect_DeinitOk();

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CSR_PATTERN, VREFBUF->CSR );
    UtVrefBuf_Check_State( VREFBUF_STATE_CNT );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}

/* ============================ DEINITIALIZATION ============================ */

/**
 * \brief   VrefBuf_Deinit() resets the peripheral and disables its clock.
 *
 * \details Initializes the buffer mode, then deinitializes the module.
 *
 * \note    Device errata bug AB#1996 (ES0565 Rev 7 2.8.1 "Slow discharge of the capacitor connected to VREF+ pin"):
 *          the pull-down resistor on the VREFBUF output is missing (STM32H562, STM32H563, STM32H573), so the
 *          capacitor on VREF+ discharges slowly after the output is switched down; the module does not wait for the
 *          discharge, the application does. The test proves the sequence of the accesses which switch the output down
 *          only: RegMem does not emulate the capacitor, the discharge is not tested.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, RCC activates the reset, releases it and disables the clock (in this order).
 * - The module is not initialized afterwards (the state, mode, level and trimming functions return an error).
 */
void Ut_VrefBuf_Deinit_Initialized_ResetsPeripheralAndDisablesClock( void )
{
#if defined(VREFBUF)
    vrefBuf_Mode_t     mode     = VREFBUF_MODE_CNT;
    vrefBuf_Voltage_t  voltage  = VREFBUF_VOLTAGE_CNT;
    vrefBuf_Trimming_t trimming = VREFBUF_TRIMMING_UNUSED;

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );

    UtVrefBuf_Expect_DeinitOk();

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Deinit() );

    UtVrefBuf_Check_State( VREFBUF_STATE_CNT );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Voltage( &voltage ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Trimming( &trimming ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Mode( VREFBUF_MODE_BUFFER ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Deinit() works in any state of the module.
 *
 * \details Deinitializes the module which was never initialized.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, RCC activates the reset, releases it and disables the clock, no register is accessed.
 */
void Ut_VrefBuf_Deinit_NotInitialized_StillResetsPeripheral( void )
{
#if defined(VREFBUF)
    VREFBUF->CSR = UT_VREFBUF_CSR_PATTERN;

    UtVrefBuf_Expect_DeinitOk();

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Deinit() );

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CSR_PATTERN, VREFBUF->CSR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Deinit() reports the failed activation of the reset and still releases it.
 *
 * \details RCC activation of the reset returns an error.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, the release of the reset and the clock disable are still requested,
 *   the module is not initialized.
 */
void Ut_VrefBuf_Deinit_RccResetActiveError_ReturnsErrorStillReleasesReset( void )
{
#if defined(VREFBUF)
    UtVrefBuf_Check_DeinitError( RCC_REQUEST_ERROR, RCC_REQUEST_OK, RCC_REQUEST_OK );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Deinit() reports the failed release of the reset.
 *
 * \details RCC release of the reset returns an error.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, the clock disable is still requested, the module is not initialized.
 */
void Ut_VrefBuf_Deinit_RccResetInactiveError_ReturnsError( void )
{
#if defined(VREFBUF)
    UtVrefBuf_Check_DeinitError( RCC_REQUEST_OK, RCC_REQUEST_ERROR, RCC_REQUEST_OK );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Deinit() reports the failed clock disable.
 *
 * \details RCC disabling of the peripheral clock returns an error.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, the module is not initialized.
 */
void Ut_VrefBuf_Deinit_RccClockError_ReturnsError( void )
{
#if defined(VREFBUF)
    UtVrefBuf_Check_DeinitError( RCC_REQUEST_OK, RCC_REQUEST_OK, RCC_REQUEST_ERROR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}

/* ================================== MODE ================================== */

/**
 * \brief   VrefBuf_Set_Mode() switches the running buffer to the high impedance mode.
 *
 * \details Initializes the buffer mode, then sets the high impedance mode.
 *
 * \note    Device errata bug AB#1996 (ES0565 Rev 7 2.8.1 "Slow discharge of the capacitor connected to VREF+ pin"):
 *          the pull-down resistor on the VREFBUF output is missing (STM32H562, STM32H563, STM32H573), so the
 *          capacitor on VREF+ discharges slowly after the output is switched down; the module does not wait for the
 *          discharge, the application does. The test proves the sequence of the accesses which switch the output down
 *          only: RegMem does not emulate the capacitor, the discharge is not tested.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, CSR: ENVR clear, HIZ set, the ready flag cleared by the buffer (HW model).
 * - Mode high impedance, state off, level unchanged.
 */
void Ut_VrefBuf_Set_Mode_BufferToHighImpedance_ClearsEnvrSetsHiz( void )
{
#if defined(VREFBUF)
    vrefBuf_Mode_t mode = VREFBUF_MODE_CNT;

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_048V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Mode( VREFBUF_MODE_HIGH_IMPEDANCE ) );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_0 | VREFBUF_CSR_HIZ, VREFBUF->CSR );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_HIGH_IMPEDANCE, mode );
    UtVrefBuf_Check_State( VREFBUF_STATE_OFF );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Mode() enables the buffer from the high impedance mode and waits for the ready flag.
 *
 * \details Initializes the high impedance mode, then sets the buffer mode.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, CSR: ENVR set, HIZ clear, ready flag set (HW model), level unchanged.
 * - Mode buffer, state ready.
 */
void Ut_VrefBuf_Set_Mode_HighImpedanceToBuffer_WaitsForReady( void )
{
#if defined(VREFBUF)
    vrefBuf_Mode_t mode = VREFBUF_MODE_CNT;

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_1_8V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Mode( VREFBUF_MODE_BUFFER ) );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_1 | VREFBUF_CSR_ENVR | VREFBUF_CSR_VRR, VREFBUF->CSR );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_BUFFER, mode );
    UtVrefBuf_Check_State( VREFBUF_STATE_READY );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Mode() switches the running buffer off.
 *
 * \details Initializes the buffer mode, then sets the off mode.
 *
 * \note    Device errata bug AB#1996 (ES0565 Rev 7 2.8.1 "Slow discharge of the capacitor connected to VREF+ pin"):
 *          the pull-down resistor on the VREFBUF output is missing (STM32H562, STM32H563, STM32H573), so the
 *          capacitor on VREF+ discharges slowly after the output is switched down; the module does not wait for the
 *          discharge, the application does. The test proves the sequence of the accesses which switch the output down
 *          only: RegMem does not emulate the capacitor, the discharge is not tested.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, CSR: ENVR clear, HIZ clear, ready flag cleared (HW model).
 * - Mode off, state off.
 */
void Ut_VrefBuf_Set_Mode_BufferToOff_ClearsEnvrAndHiz( void )
{
#if defined(VREFBUF)
    vrefBuf_Mode_t mode = VREFBUF_MODE_CNT;

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Mode( VREFBUF_MODE_OFF ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, VREFBUF->CSR );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_OFF, mode );
    UtVrefBuf_Check_State( VREFBUF_STATE_OFF );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Mode() with the mode already set keeps the buffer ready.
 *
 * \details Initializes the buffer mode, then sets the buffer mode again.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, the buffer is still in the buffer mode and ready.
 */
void Ut_VrefBuf_Set_Mode_SameMode_KeepsBufferReady( void )
{
#if defined(VREFBUF)
    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Mode( VREFBUF_MODE_BUFFER ) );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_ENVR | VREFBUF_CSR_VRR, VREFBUF->CSR );
    UtVrefBuf_Check_State( VREFBUF_STATE_READY );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Mode() reports the ready flag not set within the timeout.
 *
 * \details Initializes the high impedance mode with a HW model which never sets the ready flag, then sets
 *          the buffer mode.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, the buffer stays enabled (ENVR set, HIZ clear).
 * - State starting (enabled, not ready).
 */
void Ut_VrefBuf_Set_Mode_ReadyNeverSet_ReturnsErrorStateStarting( void )
{
#if defined(VREFBUF)
    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_NotReady );
    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Mode( VREFBUF_MODE_BUFFER ) );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_ENVR, VREFBUF->CSR );
    UtVrefBuf_Check_State( VREFBUF_STATE_STARTING );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Mode() verifies the written mode by read-back.
 *
 * \details Initializes the high impedance mode, then replaces the HW model by a model which clears ENVR
 *          after every store and sets the buffer mode.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR.
 */
void Ut_VrefBuf_Set_Mode_ReadBackMismatch_ReturnsError( void )
{
#if defined(VREFBUF)
    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_EnvrNotKept );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Mode( VREFBUF_MODE_BUFFER ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Mode() rejects a mode which is not in the list.
 *
 * \details Initializes the high impedance mode, then sets the mode VREFBUF_MODE_CNT.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, the registers are not written.
 */
void Ut_VrefBuf_Set_Mode_InvalidMode_ReturnsErrorWithoutWrite( void )
{
#if defined(VREFBUF)
    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Mode( VREFBUF_MODE_CNT ) );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_HIZ, VREFBUF->CSR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Mode() needs the initialized module.
 *
 * \details Sets the buffer mode before VrefBuf_Init().
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, the registers are not written, RCC not called.
 */
void Ut_VrefBuf_Set_Mode_NotInitialized_ReturnsErrorWithoutWrite( void )
{
#if defined(VREFBUF)
    VREFBUF->CSR = UT_VREFBUF_CSR_PATTERN;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Mode( VREFBUF_MODE_BUFFER ) );

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CSR_PATTERN, VREFBUF->CSR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Get_Mode() returns every mode of the module.
 *
 * \details Initializes the module, then presets the mode bits of CSR (ENVR / HIZ combinations) and reads
 *          the mode.
 *
 * \par Expected results
 * - ENVR = 0, HIZ = 0: off; ENVR = 1, HIZ = 0: buffer; ENVR = 0, HIZ = 1: high impedance.
 * - The registers are not written by the getter.
 */
void Ut_VrefBuf_Get_Mode_AllModes_ReturnsMode( void )
{
#if defined(VREFBUF)
    vrefBuf_Mode_t mode = VREFBUF_MODE_CNT;

    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    VREFBUF->CSR = 0u;
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_OFF, mode );

    VREFBUF->CSR = VREFBUF_CSR_ENVR;
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_BUFFER, mode );

    VREFBUF->CSR = VREFBUF_CSR_HIZ;
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_HIGH_IMPEDANCE, mode );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_HIZ, VREFBUF->CSR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Get_Mode() reports the hold mode of the hardware as an error.
 *
 * \details Initializes the module, presets the hold mode (ENVR = 1, HIZ = 1) and reads the mode.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR (the hold mode is not a mode of the module), the output value is not changed.
 */
void Ut_VrefBuf_Get_Mode_HoldMode_ReturnsErrorValueUnchanged( void )
{
#if defined(VREFBUF)
    vrefBuf_Mode_t mode = VREFBUF_MODE_CNT;

    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    VREFBUF->CSR = UT_VREFBUF_MODE_BITS_HOLD;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_CNT, mode );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Get_Mode() rejects NULL and the not initialized module.
 *
 * \details Reads the mode into NULL (initialized module), then into a variable before the initialization.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR both times, the variable is not changed.
 */
void Ut_VrefBuf_Get_Mode_NullOrNotInitialized_ReturnsError( void )
{
#if defined(VREFBUF)
    vrefBuf_Mode_t mode = VREFBUF_MODE_CNT;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_CNT, mode );

    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Mode( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}

/* ================================ VOLTAGE ================================= */

/**
 * \brief   VrefBuf_Set_Voltage() changes the level of the running buffer through the high impedance mode.
 *
 * \details Initializes the buffer mode at 2.5 V, then sets 1.8 V; the recording HW model checks the
 *          sequence of the stores.
 *
 * \note    Device errata bug AB#1996 (ES0565 Rev 7 2.8.1 "Slow discharge of the capacitor connected to VREF+ pin"):
 *          the pull-down resistor on the VREFBUF output is missing (STM32H562, STM32H563, STM32H573), so the
 *          capacitor on VREF+ discharges slowly after the output is switched down; the module does not wait for the
 *          discharge, the application does. The test proves the sequence of the accesses which switch the output down
 *          only: RegMem does not emulate the capacitor, the discharge is not tested.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, VRS = 010, buffer enabled again, ready flag set, state ready.
 * - The field VRS never changed while the buffer was enabled (hardware limit), the pin VREF+ was never
 *   pulled down (mode off not seen), the high impedance mode was seen, the mode bits changed twice
 *   (buffer to high impedance and back).
 */
void Ut_VrefBuf_Set_Voltage_BufferRunning_ChangesLevelThroughHighImpedance( void )
{
#if defined(VREFBUF)
    vrefBuf_Voltage_t voltage = VREFBUF_VOLTAGE_CNT;

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Sequence );
    UtVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );
    UtVrefBuf_Reset_Record();

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Voltage( VREFBUF_VOLTAGE_1_8V ) );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_1 | VREFBUF_CSR_ENVR | VREFBUF_CSR_VRR, VREFBUF->CSR );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Voltage( &voltage ) );
    TEST_ASSERT_EQUAL( VREFBUF_VOLTAGE_1_8V, voltage );
    UtVrefBuf_Check_State( VREFBUF_STATE_READY );

    TEST_ASSERT_EQUAL_UINT32( 0u, utVrefBuf_VrsWhileEnabledCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utVrefBuf_PulledDownCnt );
    TEST_ASSERT_TRUE( 0u < utVrefBuf_HighImpedanceCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_VREFBUF_MODE_CHANGES_PER_LEVEL, utVrefBuf_ModeChangeCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Voltage() writes the level directly while the buffer is disabled.
 *
 * \details Initializes the high impedance mode at 2.5 V, then sets 2.048 V; the recording HW model
 *          counts the changes of the mode bits.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, VRS = 001, the mode bits are not written (still high impedance), state off.
 */
void Ut_VrefBuf_Set_Voltage_BufferDisabled_WritesLevelDirectly( void )
{
#if defined(VREFBUF)
    vrefBuf_Voltage_t voltage = VREFBUF_VOLTAGE_CNT;

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Sequence );
    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );
    UtVrefBuf_Reset_Record();

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Voltage( VREFBUF_VOLTAGE_2_048V ) );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_0 | VREFBUF_CSR_HIZ, VREFBUF->CSR );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Voltage( &voltage ) );
    TEST_ASSERT_EQUAL( VREFBUF_VOLTAGE_2_048V, voltage );
    UtVrefBuf_Check_State( VREFBUF_STATE_OFF );

    TEST_ASSERT_EQUAL_UINT32( 0u, utVrefBuf_ModeChangeCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Voltage() writes the level while the buffer is off.
 *
 * \details Initializes the off mode at 2.5 V, then sets 1.8 V.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, VRS = 010, the mode bits stay clear (pin still pulled down, no mode change).
 */
void Ut_VrefBuf_Set_Voltage_ModeOff_WritesLevelWithoutModeChange( void )
{
#if defined(VREFBUF)
    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Sequence );
    VREFBUF->CSR = VREFBUF_CSR_HIZ;
    UtVrefBuf_Init_Module( VREFBUF_MODE_OFF, VREFBUF_VOLTAGE_2_5V );
    UtVrefBuf_Reset_Record();

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Voltage( VREFBUF_VOLTAGE_1_8V ) );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_1, VREFBUF->CSR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utVrefBuf_ModeChangeCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Voltage() walks through all levels of the running buffer.
 *
 * \details Initializes the buffer mode at 2.5 V, then sets 1.8 V, 2.048 V and 2.5 V one after another and
 *          reads the level back each time.
 *
 * \note    Device errata bug AB#1996 (ES0565 Rev 7 2.8.1 "Slow discharge of the capacitor connected to VREF+ pin"):
 *          the pull-down resistor on the VREFBUF output is missing (STM32H562, STM32H563, STM32H573), so the
 *          capacitor on VREF+ discharges slowly after the output is switched down; the module does not wait for the
 *          discharge, the application does. The test proves the sequence of the accesses which switch the output down
 *          only: RegMem does not emulate the capacitor, the discharge is not tested.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK for every level, the register field equals the level (010, 001, 000), the level
 *   reads back and the buffer is ready after every change.
 */
void Ut_VrefBuf_Set_Voltage_AllLevels_ReadBackAndReady( void )
{
#if defined(VREFBUF)
    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );

    for( uint32_t levelIdx = 0u; UT_VREFBUF_LEVEL_CNT > levelIdx; levelIdx ++ )
    {
        vrefBuf_Voltage_t voltage = VREFBUF_VOLTAGE_CNT;

        TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Voltage( utVrefBuf_LevelList[ levelIdx ] ) );

        TEST_ASSERT_EQUAL_HEX32( utVrefBuf_LevelRegList[ levelIdx ], VREFBUF->CSR & VREFBUF_CSR_VRS );
        TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Voltage( &voltage ) );
        TEST_ASSERT_EQUAL( utVrefBuf_LevelList[ levelIdx ], voltage );
        UtVrefBuf_Check_State( VREFBUF_STATE_READY );
    }
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Voltage() reports the ready flag not set after the change of the level.
 *
 * \details Initializes the buffer mode, replaces the HW model by a model which never sets the ready flag,
 *          then sets 2.048 V.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, the new level is written (VRS = 001) and the buffer is enabled again
 *   (the protocol restored the mode), the state is starting.
 */
void Ut_VrefBuf_Set_Voltage_ReadyNeverSet_ReturnsErrorStateStarting( void )
{
#if defined(VREFBUF)
    vrefBuf_Voltage_t voltage = VREFBUF_VOLTAGE_CNT;

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_NotReady );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Voltage( VREFBUF_VOLTAGE_2_048V ) );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_0 | VREFBUF_CSR_ENVR, VREFBUF->CSR );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Voltage( &voltage ) );
    TEST_ASSERT_EQUAL( VREFBUF_VOLTAGE_2_048V, voltage );
    UtVrefBuf_Check_State( VREFBUF_STATE_STARTING );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Voltage() verifies the written level by read-back.
 *
 * \details Initializes the buffer mode at 2.5 V, replaces the HW model by a model which clears the field
 *          VRS after every store, then sets 1.8 V.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, the buffer is left in the high impedance mode reached by the protocol
 *   (mode high impedance, state off).
 */
void Ut_VrefBuf_Set_Voltage_ReadBackMismatch_ReturnsErrorBufferLeftHighImpedance( void )
{
#if defined(VREFBUF)
    vrefBuf_Mode_t mode = VREFBUF_MODE_CNT;

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_VrsNotKept );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Voltage( VREFBUF_VOLTAGE_1_8V ) );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_MODE_HIGH_IMPEDANCE, mode );
    UtVrefBuf_Check_State( VREFBUF_STATE_OFF );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Voltage() does not write VRS when the buffer can not be disabled.
 *
 * \details Initializes the buffer mode at 2.5 V, replaces the HW model by a model which keeps ENVR set,
 *          then sets 1.8 V.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, the field VRS is not written (still 000): the hardware allows the write only
 *   with ENVR = 0.
 */
void Ut_VrefBuf_Set_Voltage_BufferCannotBeDisabled_ReturnsErrorWithoutVrsWrite( void )
{
#if defined(VREFBUF)
    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_EnvrStuck );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Voltage( VREFBUF_VOLTAGE_1_8V ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, VREFBUF->CSR & VREFBUF_CSR_VRS );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Voltage() rejects a level which is not in the list.
 *
 * \details Initializes the high impedance mode, then sets VREFBUF_VOLTAGE_CNT.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, the registers are not written.
 */
void Ut_VrefBuf_Set_Voltage_InvalidVoltage_ReturnsErrorWithoutWrite( void )
{
#if defined(VREFBUF)
    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_048V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Voltage( VREFBUF_VOLTAGE_CNT ) );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_0 | VREFBUF_CSR_HIZ, VREFBUF->CSR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Voltage() needs the initialized module.
 *
 * \details Sets 1.8 V before VrefBuf_Init().
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, the registers are not written.
 */
void Ut_VrefBuf_Set_Voltage_NotInitialized_ReturnsErrorWithoutWrite( void )
{
#if defined(VREFBUF)
    VREFBUF->CSR = UT_VREFBUF_CSR_PATTERN;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Voltage( VREFBUF_VOLTAGE_1_8V ) );

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CSR_PATTERN, VREFBUF->CSR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Get_Voltage() returns every level and rejects reserved values of the field.
 *
 * \details Initializes the module, presets the field VRS (all levels, then the reserved values 3 ... 7)
 *          and reads the level.
 *
 * \par Expected results
 * - VRS = 000: 2.5 V, 001: 2.048 V, 010: 1.8 V.
 * - Reserved values: VREFBUF_REQUEST_ERROR, the output value is not changed.
 */
void Ut_VrefBuf_Get_Voltage_AllValues_ReturnsLevelOrError( void )
{
#if defined(VREFBUF)
    vrefBuf_Voltage_t voltage = VREFBUF_VOLTAGE_CNT;

    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    for( uint32_t levelIdx = 0u; UT_VREFBUF_LEVEL_CNT > levelIdx; levelIdx ++ )
    {
        VREFBUF->CSR = utVrefBuf_LevelRegList[ levelIdx ];

        TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Voltage( &voltage ) );
        TEST_ASSERT_EQUAL( utVrefBuf_LevelList[ levelIdx ], voltage );
    }

    voltage = VREFBUF_VOLTAGE_CNT;

    for( uint32_t vrsValue = UT_VREFBUF_VRS_RESERVED_FIRST; UT_VREFBUF_VRS_RESERVED_LAST >= vrsValue; vrsValue ++ )
    {
        VREFBUF->CSR = ( vrsValue << VREFBUF_CSR_VRS_Pos );

        TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Voltage( &voltage ) );
        TEST_ASSERT_EQUAL( VREFBUF_VOLTAGE_CNT, voltage );
    }
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Get_Voltage() rejects NULL and the not initialized module.
 *
 * \details Reads the level before the initialization, then into NULL.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR both times, the variable is not changed.
 */
void Ut_VrefBuf_Get_Voltage_NullOrNotInitialized_ReturnsError( void )
{
#if defined(VREFBUF)
    vrefBuf_Voltage_t voltage = VREFBUF_VOLTAGE_CNT;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Voltage( &voltage ) );
    TEST_ASSERT_EQUAL( VREFBUF_VOLTAGE_CNT, voltage );

    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Voltage( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}

/* ================================== STATE ================================= */

/**
 * \brief   VrefBuf_Get_State() reports off, starting and ready from the registers.
 *
 * \details Initializes the module, presets CSR (all modes with and without the ready flag) and reads the
 *          state.
 *
 * \par Expected results
 * - Off mode and high impedance mode: off (also with the ready flag set).
 * - Buffer mode without the ready flag: starting, with the ready flag: ready.
 * - Hold mode of the hardware: off.
 * - The getter does not write the registers.
 */
void Ut_VrefBuf_Get_State_AllModes_ReturnsOffStartingReady( void )
{
#if defined(VREFBUF)
    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    VREFBUF->CSR = 0u;
    UtVrefBuf_Check_State( VREFBUF_STATE_OFF );
    VREFBUF->CSR = VREFBUF_CSR_VRR;
    UtVrefBuf_Check_State( VREFBUF_STATE_OFF );

    VREFBUF->CSR = UT_VREFBUF_MODE_BITS_HIZ;
    UtVrefBuf_Check_State( VREFBUF_STATE_OFF );
    VREFBUF->CSR = UT_VREFBUF_MODE_BITS_HIZ | VREFBUF_CSR_VRR;
    UtVrefBuf_Check_State( VREFBUF_STATE_OFF );

    VREFBUF->CSR = UT_VREFBUF_MODE_BITS_BUFFER;
    UtVrefBuf_Check_State( VREFBUF_STATE_STARTING );
    VREFBUF->CSR = UT_VREFBUF_MODE_BITS_BUFFER | VREFBUF_CSR_VRR;
    UtVrefBuf_Check_State( VREFBUF_STATE_READY );

    VREFBUF->CSR = UT_VREFBUF_MODE_BITS_HOLD | VREFBUF_CSR_VRR;
    UtVrefBuf_Check_State( VREFBUF_STATE_OFF );

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_MODE_BITS_HOLD | VREFBUF_CSR_VRR, VREFBUF->CSR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Get_State() rejects NULL and the not initialized module.
 *
 * \details Reads the state before the initialization, then into NULL.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR both times, the variable is not changed.
 */
void Ut_VrefBuf_Get_State_NullOrNotInitialized_ReturnsError( void )
{
#if defined(VREFBUF)
    vrefBuf_State_t state = VREFBUF_STATE_CNT;

    VREFBUF->CSR = UT_VREFBUF_MODE_BITS_BUFFER | VREFBUF_CSR_VRR;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_State( &state ) );
    TEST_ASSERT_EQUAL( VREFBUF_STATE_CNT, state );

    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_State( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}

/* =============================== TRIMMING ================================= */

/**
 * \brief   VrefBuf_Set_Trimming() writes the trimming code and reads it back.
 *
 * \details Initializes the module (factory trimming and reserved bits of CCR preset), writes the codes
 *          0, 21 and 63 and reads them back.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK, the TRIM field equals the code, the reserved bits of CCR and the register CSR
 *   are not changed.
 * - VrefBuf_Get_Trimming() returns the code.
 */
void Ut_VrefBuf_Set_Trimming_ValidCodes_WritesTrimField( void )
{
#if defined(VREFBUF)
    static const vrefBuf_Trimming_t codeList[] = { 0u, UT_VREFBUF_TRIM_USER, UT_VREFBUF_TRIM_MAX };

    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_048V );

    VREFBUF->CCR = UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY;

    for( uint32_t codeIdx = 0u; ( sizeof( codeList ) / sizeof( codeList[ 0u ] ) ) > codeIdx; codeIdx ++ )
    {
        vrefBuf_Trimming_t trimming = VREFBUF_TRIMMING_UNUSED;

        TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Set_Trimming( codeList[ codeIdx ] ) );

        TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CCR_RESERVED | codeList[ codeIdx ], VREFBUF->CCR );
        TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Trimming( &trimming ) );
        TEST_ASSERT_EQUAL_UINT8( codeList[ codeIdx ], trimming );
    }

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_0 | VREFBUF_CSR_HIZ, VREFBUF->CSR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Trimming() rejects a code above the field.
 *
 * \details Initializes the module, then writes the codes 64 and VREFBUF_TRIMMING_UNUSED.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR both times, CCR is not written.
 */
void Ut_VrefBuf_Set_Trimming_AboveMax_ReturnsErrorWithoutWrite( void )
{
#if defined(VREFBUF)
    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    VREFBUF->CCR = UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Trimming( UT_VREFBUF_TRIM_MAX + 1u ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Trimming( VREFBUF_TRIMMING_UNUSED ) );

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY, VREFBUF->CCR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Trimming() verifies the written code by read-back.
 *
 * \details Initializes the module, replaces the HW model by a model which clears the field TRIM after every
 *          store, then writes a code.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR.
 */
void Ut_VrefBuf_Set_Trimming_ReadBackMismatch_ReturnsError( void )
{
#if defined(VREFBUF)
    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_TrimNotKept );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Trimming( UT_VREFBUF_TRIM_USER ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Set_Trimming() needs the initialized module.
 *
 * \details Writes a code before VrefBuf_Init().
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR, CCR is not written.
 */
void Ut_VrefBuf_Set_Trimming_NotInitialized_ReturnsErrorWithoutWrite( void )
{
#if defined(VREFBUF)
    VREFBUF->CCR = UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Trimming( UT_VREFBUF_TRIM_USER ) );

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY, VREFBUF->CCR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Get_Trimming() returns the code of the register and rejects NULL.
 *
 * \details Reads the trimming before the initialization, then after it (factory code preset), then into NULL.
 *
 * \par Expected results
 * - Before the initialization and for NULL: VREFBUF_REQUEST_ERROR, the variable is not changed.
 * - Initialized: VREFBUF_REQUEST_OK and the factory code (reserved bits of CCR are not part of the code).
 */
void Ut_VrefBuf_Get_Trimming_ReturnsRegisterCode( void )
{
#if defined(VREFBUF)
    vrefBuf_Trimming_t trimming = VREFBUF_TRIMMING_UNUSED;

    VREFBUF->CCR = UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Trimming( &trimming ) );
    TEST_ASSERT_EQUAL_UINT8( VREFBUF_TRIMMING_UNUSED, trimming );

    UtVrefBuf_Init_Module( VREFBUF_MODE_HIGH_IMPEDANCE, VREFBUF_VOLTAGE_2_5V );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_Trimming( &trimming ) );
    TEST_ASSERT_EQUAL_UINT8( UT_VREFBUF_TRIM_FACTORY, trimming );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Trimming( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}

/* ============================ REGISTER ACCESS ============================= */

/**
 * \brief   Register accessors modify and read the registers of the module.
 *
 * \details Sets and resets bits, writes a masked value and reads bits and values of CSR and CCR.
 *
 * \par Expected results
 * - Set / reset change only the given bits, the masked write keeps the bits outside of the mask,
 *   the getters return the masked values.
 */
void Ut_VrefBuf_Reg_Accessors_ValidRegister_ModifyAndRead( void )
{
#if defined(VREFBUF)
    VREFBUF->CSR = 0u;
    VREFBUF->CCR = UT_VREFBUF_CCR_RESERVED;

    VrefBuf_Set_RegBit( VREFBUF_REG_CSR, VREFBUF_CSR_HIZ | VREFBUF_CSR_VRS_1 );
    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_HIZ | VREFBUF_CSR_VRS_1, VREFBUF->CSR );
    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_HIZ, VrefBuf_Get_RegBit( VREFBUF_REG_CSR, VREFBUF_CSR_HIZ ) );

    VrefBuf_Reset_RegBit( VREFBUF_REG_CSR, VREFBUF_CSR_HIZ );
    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_1, VREFBUF->CSR );
    TEST_ASSERT_EQUAL_HEX32( 0u, VrefBuf_Get_RegBit( VREFBUF_REG_CSR, VREFBUF_CSR_HIZ ) );

    VrefBuf_Set_RegVal( VREFBUF_REG_CSR, VREFBUF_CSR_VRS, VREFBUF_CSR_VRS_0 );
    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_0, VREFBUF->CSR );
    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_0, VrefBuf_Get_RegVal( VREFBUF_REG_CSR, VREFBUF_CSR_VRS ) );

    VrefBuf_Set_RegVal( VREFBUF_REG_CCR, VREFBUF_CCR_TRIM, UT_VREFBUF_TRIM_USER );
    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_USER, VREFBUF->CCR );
    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_TRIM_USER, VrefBuf_Get_RegVal( VREFBUF_REG_CCR, VREFBUF_CCR_TRIM ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   Register accessors ignore an invalid register ID.
 *
 * \details Calls every accessor with the register ID VREFBUF_REG_CNT and with an ID above it.
 *
 * \par Expected results
 * - No register is written, the getters return zero.
 */
void Ut_VrefBuf_Reg_Accessors_InvalidRegisterId_NoAccess( void )
{
#if defined(VREFBUF)
    const vrefBuf_RegId_t invalidList[] = { VREFBUF_REG_CNT, (vrefBuf_RegId_t)( VREFBUF_REG_CNT + 1u ) };

    VREFBUF->CSR = UT_VREFBUF_CSR_PATTERN;
    VREFBUF->CCR = UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY;

    for( uint32_t idIdx = 0u; ( sizeof( invalidList ) / sizeof( invalidList[ 0u ] ) ) > idIdx; idIdx ++ )
    {
        VrefBuf_Set_RegBit( invalidList[ idIdx ], UT_VREFBUF_ALL_BITS );
        VrefBuf_Reset_RegBit( invalidList[ idIdx ], UT_VREFBUF_ALL_BITS );
        VrefBuf_Set_RegVal( invalidList[ idIdx ], UT_VREFBUF_ALL_BITS, 0u );

        TEST_ASSERT_EQUAL_HEX32( 0u, VrefBuf_Get_RegBit( invalidList[ idIdx ], UT_VREFBUF_ALL_BITS ) );
        TEST_ASSERT_EQUAL_HEX32( 0u, VrefBuf_Get_RegVal( invalidList[ idIdx ], UT_VREFBUF_ALL_BITS ) );
    }

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CSR_PATTERN, VREFBUF->CSR );
    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY, VREFBUF->CCR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Reg_Set_Mode() rejects a value outside of the mode bits.
 *
 * \details Writes the value of the voltage field and of the ready flag as the mode.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR both times, CSR is not written.
 */
void Ut_VrefBuf_Reg_Set_Mode_ValueOutsideModeBits_ReturnsErrorWithoutWrite( void )
{
#if defined(VREFBUF)
    VREFBUF->CSR = VREFBUF_CSR_VRS_0;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Reg_Set_Mode( VREFBUF_CSR_VRS_1 ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Reg_Set_Mode( VREFBUF_CSR_VRR ) );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_0, VREFBUF->CSR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Reg_Set_Voltage() rejects the reserved values of the field VRS.
 *
 * \details Writes the values 3 ... 7 of the field VRS.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR for every value, CSR is not written.
 */
void Ut_VrefBuf_Reg_Set_Voltage_ReservedValue_ReturnsErrorWithoutWrite( void )
{
#if defined(VREFBUF)
    VREFBUF->CSR = VREFBUF_CSR_HIZ;

    for( uint32_t vrsValue = UT_VREFBUF_VRS_RESERVED_FIRST; UT_VREFBUF_VRS_RESERVED_LAST >= vrsValue; vrsValue ++ )
    {
        TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR,
                           VrefBuf_Reg_Set_Voltage( ( vrsValue << VREFBUF_CSR_VRS_Pos ), UT_VREFBUF_TIMEOUT_CNT ) );
    }

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_HIZ, VREFBUF->CSR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Reg_Set_Voltage() restores the hold mode and does not wait for the ready flag.
 *
 * \details Presets the hold mode (ENVR = 1, HIZ = 1, ready flag clear) and writes the level 1.8 V.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_OK (no wait: the hold mode does not drive the output), VRS = 010, the hold mode
 *   is restored, the field VRS never changed while ENVR was set (recording HW model).
 */
void Ut_VrefBuf_Reg_Set_Voltage_HoldMode_RestoresHoldModeWithoutWaiting( void )
{
#if defined(VREFBUF)
    VREFBUF->CSR = UT_VREFBUF_MODE_BITS_HOLD;

    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Sequence );
    UtVrefBuf_Reset_Record();

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Reg_Set_Voltage( VREFBUF_CSR_VRS_1, UT_VREFBUF_TIMEOUT_CNT ) );

    TEST_ASSERT_EQUAL_HEX32( VREFBUF_CSR_VRS_1 | UT_VREFBUF_MODE_BITS_HOLD, VREFBUF->CSR & ~VREFBUF_CSR_VRR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utVrefBuf_VrsWhileEnabledCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Reg_Set_Trimming() rejects a value outside of the field TRIM.
 *
 * \details Writes the value 64 and a value in the reserved bits of CCR.
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR both times, CCR is not written.
 */
void Ut_VrefBuf_Reg_Set_Trimming_ValueOutsideField_ReturnsErrorWithoutWrite( void )
{
#if defined(VREFBUF)
    VREFBUF->CCR = UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Reg_Set_Trimming( UT_VREFBUF_TRIM_MAX + 1u ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Reg_Set_Trimming( UT_VREFBUF_CCR_RESERVED ) );

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY, VREFBUF->CCR );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}


/**
 * \brief   VrefBuf_Reg_Get_ReadyFlag() and VrefBuf_Reg_Wait_Ready() follow the ready flag.
 *
 * \details Reads the flag clear and set, waits with the flag set (timeout 1), clear (timeout 3) and
 *          with the timeout 0.
 *
 * \par Expected results
 * - The flag reads inactive / active as the bit VRR.
 * - The wait returns VREFBUF_REQUEST_OK with the flag set (also with the timeout 1), otherwise (flag clear,
 *   timeout 0 even with the flag set) VREFBUF_REQUEST_ERROR.
 */
void Ut_VrefBuf_Reg_WaitReady_FlagAndTimeout_ReturnsState( void )
{
#if defined(VREFBUF)
    VREFBUF->CSR = 0u;
    TEST_ASSERT_EQUAL( VREFBUF_FLAG_INACTIVE, VrefBuf_Reg_Get_ReadyFlag() );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Reg_Wait_Ready( UT_VREFBUF_WAIT_CNT ) );

    VREFBUF->CSR = VREFBUF_CSR_VRR;
    TEST_ASSERT_EQUAL( VREFBUF_FLAG_ACTIVE, VrefBuf_Reg_Get_ReadyFlag() );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Reg_Wait_Ready( 1u ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Reg_Wait_Ready( 0u ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without VREFBUF" );
#endif
}

/* ======================= LINE WITHOUT THE PERIPHERAL ====================== */

/**
 * \brief   Initialization, deinitialization and default configuration return an error without VREFBUF.
 *
 * \details STM32H503 has no VREFBUF peripheral. Calls VrefBuf_Init() with a valid configuration,
 *          VrefBuf_Deinit() and VrefBuf_Get_DefaultConfig() (the lines with the peripheral ignore the test).
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR for every function, the configuration is not changed, RCC not called
 *   (an unexpected call fails the test).
 */
void Ut_VrefBuf_NoPeripheral_InitDeinitDefaultConfig_ReturnError( void )
{
#if !defined(VREFBUF)
    vrefBuf_Config_t config;

    config.Mode         = VREFBUF_MODE_BUFFER;
    config.Voltage      = VREFBUF_VOLTAGE_2_048V;
    config.Trimming     = UT_VREFBUF_SENTINEL_TRIM;
    config.StartTimeout = UT_VREFBUF_SENTINEL_TIMEOUT;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Init( &config ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Init( NULL ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Deinit() );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( VREFBUF_MODE_BUFFER, config.Mode );
    TEST_ASSERT_EQUAL( VREFBUF_VOLTAGE_2_048V, config.Voltage );
    TEST_ASSERT_EQUAL_UINT8( UT_VREFBUF_SENTINEL_TRIM, config.Trimming );
    TEST_ASSERT_EQUAL_UINT32( UT_VREFBUF_SENTINEL_TIMEOUT, config.StartTimeout );
#else
    TEST_IGNORE_MESSAGE( "MCU with VREFBUF" );
#endif
}


/**
 * \brief   The setters return an error without VREFBUF.
 *
 * \details Calls VrefBuf_Set_Mode(), VrefBuf_Set_Voltage() and VrefBuf_Set_Trimming() with valid
 *          parameters (the lines with the peripheral ignore the test).
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR for every function, RCC not called.
 */
void Ut_VrefBuf_NoPeripheral_Setters_ReturnError( void )
{
#if !defined(VREFBUF)
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Mode( VREFBUF_MODE_BUFFER ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Mode( VREFBUF_MODE_HIGH_IMPEDANCE ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Voltage( VREFBUF_VOLTAGE_2_5V ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Voltage( VREFBUF_VOLTAGE_1_8V ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Trimming( 0u ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Set_Trimming( UT_VREFBUF_NO_PERIPH_TRIM ) );
#else
    TEST_IGNORE_MESSAGE( "MCU with VREFBUF" );
#endif
}


/**
 * \brief   The getters return an error and keep their output without VREFBUF.
 *
 * \details Calls VrefBuf_Get_Mode(), VrefBuf_Get_Voltage(), VrefBuf_Get_State() and
 *          VrefBuf_Get_Trimming() (the lines with the peripheral ignore the test).
 *
 * \par Expected results
 * - VREFBUF_REQUEST_ERROR for every function, the output variables are not changed, also for NULL.
 */
void Ut_VrefBuf_NoPeripheral_Getters_ReturnErrorValueUnchanged( void )
{
#if !defined(VREFBUF)
    vrefBuf_Mode_t     mode     = VREFBUF_MODE_CNT;
    vrefBuf_Voltage_t  voltage  = VREFBUF_VOLTAGE_CNT;
    vrefBuf_State_t    state    = VREFBUF_STATE_CNT;
    vrefBuf_Trimming_t trimming = VREFBUF_TRIMMING_UNUSED;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Mode( &mode ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Voltage( &voltage ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_State( &state ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Trimming( &trimming ) );

    TEST_ASSERT_EQUAL( VREFBUF_MODE_CNT, mode );
    TEST_ASSERT_EQUAL( VREFBUF_VOLTAGE_CNT, voltage );
    TEST_ASSERT_EQUAL( VREFBUF_STATE_CNT, state );
    TEST_ASSERT_EQUAL_UINT8( VREFBUF_TRIMMING_UNUSED, trimming );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Mode( NULL ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Voltage( NULL ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_State( NULL ) );
    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_Trimming( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU with VREFBUF" );
#endif
}

/* =========================== LOCAL FUNCTIONS ============================== */

#if defined(VREFBUF)

/** Expects the RCC calls of the start of VrefBuf_Init(): clock activation, reset activation, reset release */
static void UtVrefBuf_Expect_InitClock( void )
{
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_VREF, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( RCC_PERIPH_VREF, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( RCC_PERIPH_VREF, RCC_REQUEST_OK );
}


/** Expects the RCC calls of VrefBuf_Deinit() with the given results: reset activation, reset release, clock disable */
static void UtVrefBuf_Expect_Deinit( rcc_RequestState_t resetActState,
                                     rcc_RequestState_t resetInactState,
                                     rcc_RequestState_t clkState )
{
    Rcc_Set_ResetActive_ExpectAndReturn( RCC_PERIPH_VREF, resetActState );
    Rcc_Set_ResetInactive_ExpectAndReturn( RCC_PERIPH_VREF, resetInactState );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_VREF, clkState );
}


/** Expects the RCC calls of successful VrefBuf_Deinit() */
static void UtVrefBuf_Expect_DeinitOk( void )
{
    UtVrefBuf_Expect_Deinit( RCC_REQUEST_OK, RCC_REQUEST_OK, RCC_REQUEST_OK );
}


/** Starts the HW model of the registers (the model replaces the active one) */
static void UtVrefBuf_Set_Model( regMem_ModelCallback_t model )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( model ) );
}


/** Returns the configuration with the start-up timeout of the tests */
static vrefBuf_Config_t UtVrefBuf_Get_Config( vrefBuf_Mode_t mode,
                                              vrefBuf_Voltage_t voltage,
                                              vrefBuf_Trimming_t trimming )
{
    vrefBuf_Config_t config;

    config.Mode         = mode;
    config.Voltage      = voltage;
    config.Trimming     = trimming;
    config.StartTimeout = UT_VREFBUF_TIMEOUT_CNT;

    return ( config );
}


/** Initializes the module (factory trimming kept) and expects success; the HW model is set by the caller */
static void UtVrefBuf_Init_Module( vrefBuf_Mode_t mode, vrefBuf_Voltage_t voltage )
{
    vrefBuf_Config_t config = UtVrefBuf_Get_Config( mode, voltage, VREFBUF_TRIMMING_UNUSED );

    UtVrefBuf_Expect_InitClock();

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Init( &config ) );
}


/** Expects that VrefBuf_Init() rejects the configuration before any access: no RCC call, registers
 *  untouched, module not initialized */
static void UtVrefBuf_Check_InitRejected( const vrefBuf_Config_t * const config )
{
    VREFBUF->CSR = UT_VREFBUF_CSR_PATTERN;
    VREFBUF->CCR = UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY;

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Init( config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CSR_PATTERN, VREFBUF->CSR );
    TEST_ASSERT_EQUAL_HEX32( UT_VREFBUF_CCR_RESERVED | UT_VREFBUF_TRIM_FACTORY, VREFBUF->CCR );
    UtVrefBuf_Check_State( VREFBUF_STATE_CNT );
}


/** Expects that VrefBuf_Init() fails after the start of the configuration: the RCC calls of the initialization
 *  and of the deinitialization, the module is not initialized afterwards */
static void UtVrefBuf_Check_InitFailed( const vrefBuf_Config_t * const config )
{
    UtVrefBuf_Expect_InitClock();
    UtVrefBuf_Expect_DeinitOk();

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Init( config ) );

    UtVrefBuf_Check_State( VREFBUF_STATE_CNT );
}


/** Expects that VrefBuf_Deinit() with the given RCC results calls all three RCC functions, returns an error
 *  and leaves the module not initialized */
static void UtVrefBuf_Check_DeinitError( rcc_RequestState_t resetActState,
                                         rcc_RequestState_t resetInactState,
                                         rcc_RequestState_t clkState )
{
    UtVrefBuf_Set_Model( UtVrefBuf_HwModel_Ready );
    UtVrefBuf_Init_Module( VREFBUF_MODE_BUFFER, VREFBUF_VOLTAGE_2_5V );

    UtVrefBuf_Expect_Deinit( resetActState, resetInactState, clkState );

    TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Deinit() );

    UtVrefBuf_Check_State( VREFBUF_STATE_CNT );
}


/** Checks the state returned by VrefBuf_Get_State(); VREFBUF_STATE_CNT expects the error of the not
 *  initialized module (the output is not changed) */
static void UtVrefBuf_Check_State( vrefBuf_State_t expectedState )
{
    vrefBuf_State_t state = VREFBUF_STATE_CNT;

    if( VREFBUF_STATE_CNT == expectedState )
    {
        TEST_ASSERT_EQUAL( VREFBUF_REQUEST_ERROR, VrefBuf_Get_State( &state ) );
    }
    else
    {
        TEST_ASSERT_EQUAL( VREFBUF_REQUEST_OK, VrefBuf_Get_State( &state ) );
    }

    TEST_ASSERT_EQUAL( expectedState, state );
}


/** Clears the record of the recording HW model and takes over the present register values */
static void UtVrefBuf_Reset_Record( void )
{
    utVrefBuf_PrevCsr            = VREFBUF->CSR;
    utVrefBuf_PrevTrim           = ( VREFBUF->CCR & VREFBUF_CCR_TRIM );
    utVrefBuf_VrsWhileEnabledCnt = 0u;
    utVrefBuf_PulledDownCnt      = 0u;
    utVrefBuf_HighImpedanceCnt   = 0u;
    utVrefBuf_ModeChangeCnt      = 0u;
    utVrefBuf_EnableSeen         = VREFBUF_FLAG_INACTIVE;
    utVrefBuf_TrimAtEnable       = 0u;
    utVrefBuf_TrimSeen           = VREFBUF_FLAG_INACTIVE;
    utVrefBuf_VrsAtTrim          = 0u;
}

/* ============================== HW MODELS ================================= */

/** HW model: the ready flag follows the buffer - set while the buffer drives the pin (ENVR = 1, HIZ = 0),
 *  kept in the hold mode (ENVR = 1, HIZ = 1), cleared while the buffer is off */
static void UtVrefBuf_HwModel_Ready( void )
{
    const uint32_t modeValue = ( VREFBUF->CSR & UT_VREFBUF_MODE_MASK );

    if( UT_VREFBUF_MODE_BITS_BUFFER == modeValue )
    {
        (void)__atomic_or_fetch( &VREFBUF->CSR, (uint32_t)VREFBUF_CSR_VRR, __ATOMIC_SEQ_CST );
    }
    else if( UT_VREFBUF_MODE_BITS_HOLD == modeValue )
    {
        /* Hold mode: the ready flag keeps its last state */
    }
    else
    {
        (void)__atomic_and_fetch( &VREFBUF->CSR, (uint32_t)~VREFBUF_CSR_VRR, __ATOMIC_SEQ_CST );
    }
}


/** HW model: the ready flag as \ref UtVrefBuf_HwModel_Ready, additionally records the sequence of the
 *  stores - changes of the mode bits, writes of VRS with the buffer enabled, the mode off (pin pulled
 *  down), the high impedance mode, and the values of TRIM / VRS at the first enable / first change of TRIM */
static void UtVrefBuf_HwModel_Sequence( void )
{
    UtVrefBuf_HwModel_Ready();

    const uint32_t csrValue  = VREFBUF->CSR;
    const uint32_t modeValue = ( csrValue & UT_VREFBUF_MODE_MASK );
    const uint32_t prevMode  = ( utVrefBuf_PrevCsr & UT_VREFBUF_MODE_MASK );
    const uint32_t vrsValue  = ( csrValue & VREFBUF_CSR_VRS );
    const uint32_t trimValue = ( VREFBUF->CCR & VREFBUF_CCR_TRIM );

    if( modeValue != prevMode )
    {
        utVrefBuf_ModeChangeCnt++;
    }
    else
    {
        /* Mode bits did not change */
    }

    if( ( vrsValue != ( utVrefBuf_PrevCsr & VREFBUF_CSR_VRS ) ) &&
        ( 0u       != ( modeValue & VREFBUF_CSR_ENVR         ) )    )
    {
        utVrefBuf_VrsWhileEnabledCnt++;
    }
    else
    {
        /* VRS did not change or the buffer is disabled */
    }

    if( 0u == modeValue )
    {
        utVrefBuf_PulledDownCnt++;
    }
    else
    {
        /* Pin VREF+ is not pulled down */
    }

    if( UT_VREFBUF_MODE_BITS_HIZ == modeValue )
    {
        utVrefBuf_HighImpedanceCnt++;
    }
    else
    {
        /* Not the high impedance mode */
    }

    if( ( 0u                    != ( modeValue & VREFBUF_CSR_ENVR ) ) &&
        ( VREFBUF_FLAG_INACTIVE == utVrefBuf_EnableSeen             )    )
    {
        utVrefBuf_EnableSeen   = VREFBUF_FLAG_ACTIVE;
        utVrefBuf_TrimAtEnable = trimValue;
    }
    else
    {
        /* Buffer disabled or the first enable was already recorded */
    }

    if( ( trimValue             != utVrefBuf_PrevTrim ) &&
        ( VREFBUF_FLAG_INACTIVE == utVrefBuf_TrimSeen )    )
    {
        utVrefBuf_TrimSeen  = VREFBUF_FLAG_ACTIVE;
        utVrefBuf_VrsAtTrim = vrsValue;
    }
    else
    {
        /* TRIM did not change or the first change was already recorded */
    }

    utVrefBuf_PrevCsr  = csrValue;
    utVrefBuf_PrevTrim = trimValue;
}


/** HW model: the ready flag is never set */
static void UtVrefBuf_HwModel_NotReady( void )
{
    (void)__atomic_and_fetch( &VREFBUF->CSR, (uint32_t)~VREFBUF_CSR_VRR, __ATOMIC_SEQ_CST );
}


/** HW model: ENVR stays clear (the written enable bit does not stay) */
static void UtVrefBuf_HwModel_EnvrNotKept( void )
{
    (void)__atomic_and_fetch( &VREFBUF->CSR, (uint32_t)~VREFBUF_CSR_ENVR, __ATOMIC_SEQ_CST );
}


/** HW model: ENVR stays set (the buffer can not be disabled), the ready flag as \ref UtVrefBuf_HwModel_Ready */
static void UtVrefBuf_HwModel_EnvrStuck( void )
{
    (void)__atomic_or_fetch( &VREFBUF->CSR, (uint32_t)VREFBUF_CSR_ENVR, __ATOMIC_SEQ_CST );

    UtVrefBuf_HwModel_Ready();
}


/** HW model: the field VRS stays zero (the written level does not stay), the ready flag as
 *  \ref UtVrefBuf_HwModel_Ready */
static void UtVrefBuf_HwModel_VrsNotKept( void )
{
    (void)__atomic_and_fetch( &VREFBUF->CSR, (uint32_t)~VREFBUF_CSR_VRS, __ATOMIC_SEQ_CST );

    UtVrefBuf_HwModel_Ready();
}


/** HW model: the field TRIM stays zero (the written trimming does not stay), the ready flag as
 *  \ref UtVrefBuf_HwModel_Ready */
static void UtVrefBuf_HwModel_TrimNotKept( void )
{
    (void)__atomic_and_fetch( &VREFBUF->CCR, (uint32_t)~VREFBUF_CCR_TRIM, __ATOMIC_SEQ_CST );

    UtVrefBuf_HwModel_Ready();
}

#endif /* VREFBUF */
