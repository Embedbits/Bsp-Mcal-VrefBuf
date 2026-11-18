#======================== Unit tests configuration ============================#
# Unit tests of VrefBuf module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(VREFBUF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${VREFBUF_DIR}/.." ABSOLUTE)

# The two sources of the module are compiled for every line: VrefBuf.c checks the peripheral in every
# public function (a line without VREFBUF returns the error value), VrefBuf_Reg.c holds
# the register access of the lines with it. Tests of the read-back verification and of the ready flag
# use the HW model of RegMem (register value which does not stay as written, ready flag following the
# enable bit) - executed serially where the model runs in a thread
UnitTesting_Add_Test(
    NAME            VrefBuf
    TEST_SOURCE     Test_VrefBuf.c
    UUT_SOURCES     ${VREFBUF_DIR}/VrefBuf.c
                    ${VREFBUF_DIR}/VrefBuf_Reg.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
    INCLUDE_DIRS    ${VREFBUF_DIR}
                    ${MCAL_DIR}/Rcc
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
    RUN_SERIAL
)
