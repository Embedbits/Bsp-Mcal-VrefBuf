/**
 * \author Mr.Nobody
 * \file VrefBuf.h
 * \ingroup VrefBuf
 * \brief VrefBuf module common functionality header file.
 *
 * This file contains the common functionality used internally by the module,
 * and shall provide interface between the module and the application.
 *
 */

#ifndef VREFBUF_VREFBUF_H
#define VREFBUF_VREFBUF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "VrefBuf_Types.h"                  /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/** \brief Mode configuration item */
typedef struct
{
    vrefBuf_Mode_t Mode;   /**< Electrical mode                                              */
    uint32_t       RegVal; /**< Value of the bits ENVR and HIZ in CSR register (positioned) */
}   vrefBuf_ModeConfig_t;


/** \brief Voltage level configuration item */
typedef struct
{
    vrefBuf_Voltage_t Voltage; /**< Voltage level                                       */
    uint32_t          RegVal;  /**< Value of the field VRS in CSR register (positioned) */
}   vrefBuf_VoltageConfig_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Value of major version of SW module */
#define VREFBUF_MAJOR_VERSION           ( 0u )

/** Value of minor version of SW module */
#define VREFBUF_MINOR_VERSION           ( 1u )

/** Value of patch version of SW module */
#define VREFBUF_PATCH_VERSION           ( 0u )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

#ifdef __cplusplus
}
#endif

#endif /* VREFBUF_VREFBUF_H */
