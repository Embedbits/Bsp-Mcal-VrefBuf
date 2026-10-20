# VREFBUF Peripheral Driver

This module provides an abstraction layer for the **voltage reference buffer** (`VREFBUF`) of STM32 MCUs: the internal buffer that
generates the reference voltage for the analog-to-digital converters (and the digital-to-analog converters and the comparators of the
families where they use it) and drives it to the `VREF+` pin for the external analog circuits. It offers the selection of the
output voltage, the enable and the high-impedance mode, the readiness check and the trimming. It is a low-level MCAL module: it owns the
VREFBUF hardware, the application and the other modules never touch the registers.

The public interface (`VrefBuf_Port.h`, `VrefBuf_Types.h`) is **identical for all supported families**. The implementation
for a family lives in its own branch (`Dev/<Family>` for development, `Releases/<Family>` for the released versions), so
a module or an application written against the interface does not change when the MCU family changes.

This README is the landing page and the **specification of the interface** (master). The family branches contain their
own README with everything that is specific to the family: the instances of the device, their settings and the status of
the implementation.

> **Status: design.** No implementation exists yet, the interface below is a proposal. Stories and branches follow.

---

## Intent and purpose

- The accuracy of every analog measurement is the accuracy of its **reference**. When the reference is the supply (`VDDA`), the readings move with
  the supply; the internal buffer provides a **stable reference of 2.5 V, 2.048 V or 1.8 V** (the levels of the device) independent of the
  supply, which the ADC and the DAC use directly, and which can supply the external sensors through the `VREF+` pin. The 2.048 V level gives a round scale
  (0.5 mV per LSB of a 12-bit conversion).
- The buffer has a **start-up time** and a ready flag; an ADC conversion started before the reference is stable reads a wrong value. The module
  enables the buffer, waits for the ready flag (with a timeout) and reports the readiness, so that the Adc and the Dac modules start only on a good
  reference.
- The buffer can be put in the **high-impedance mode** (the output is off and `VREF+` is supplied externally by a precise reference) or in the
  **off** state (the reference is `VDDA`).
- The module hides the differences between the families: the voltage levels, the trimming of the buffer and the pin (some devices have `VREF+` bonded to `VDDA`
  and no buffer control) differ, the interface stays the same.

What the module is **not**: it does not measure the reference (the Adc module measures `VREFINT` to calculate the real `VDDA`) and it does not calibrate the converters.

---

## Concept

| Item | Meaning |
|---|---|
| **Mode** (`vrefBuf_Mode_t`) | `VREFBUF_MODE_OFF` (the reference is `VDDA`), `VREFBUF_MODE_BUFFER` (the internal buffer drives `VREF+`), `VREFBUF_MODE_HIGH_IMPEDANCE` (the buffer is disconnected, `VREF+` is external). |
| **Voltage** (`vrefBuf_Voltage_t`) | `VREFBUF_VOLTAGE_2_5V`, `_2_048V`, `_1_8V` - only the levels of the device. |
| **Trimming** | The factory trimming of the reference (read-only default) and the user trimming value. |
| **State** | `VREFBUF_STATE_OFF`, `VREFBUF_STATE_STARTING`, `VREFBUF_STATE_READY`. |

Rules:

1. `VrefBuf_Init` selects the voltage and the mode and enables the buffer; it returns after the buffer is ready, or with an error on the timeout. The voltage
   is chosen so that `VDDA` is at least the reference plus the dropout of the buffer (the module cannot measure `VDDA`; the application takes care).
2. A change of the voltage or the mode is allowed while the buffer is running; the module sets the new value and waits for the ready flag again. The state
   `STARTING` tells the consumers (the Adc, the Dac) not to convert.
3. The user trimming is optional; `VrefBuf_Set_Trimming` writes the value and `VrefBuf_Get_Trimming` reads the current one.
4. `VrefBuf_Deinit` switches the buffer off (the reference falls back to `VDDA`); the converters that depend on it must be stopped first.
5. All parameters are checked before a register is written.

---

## Supported families

Every family gets its own story, its own tasks and its own branches `Dev/<Family>` and `Releases/<Family>` cut from this
master.

| Family | Branch | Buffer | Status |
|---|---|---|---|
| STM32H5 | `Dev/STM32H5` | `VREFBUF` on all lines except STM32H503: 2.5 V, 2.048 V, 1.8 V levels, high-impedance mode, ready flag, trimming | planned (first) |
| STM32U5 | `Dev/STM32U5` | `VREFBUF` | planned |
| STM32G4, STM32L4 | `Dev/STM32G4`, `Dev/STM32L4` | `VREFBUF` (2.5 V, 2.048 V) on the lines that have it | planned |
| STM32H7 | `Dev/STM32H7` | `VREFBUF` | planned |
| STM32U3 | `Dev/STM32U3` | device dependent | planned |
| STM32F4, STM32F7 | `Dev/STM32F4`, `Dev/STM32F7` | no reference buffer (the reference is `VREF+`, `VDDA`) - the module is a stub that returns `VREFBUF_REQUEST_ERROR` | planned |

A function without a meaning in a family stays in the interface, returns `VREFBUF_REQUEST_ERROR` and suppresses the unused
parameters (for example a voltage level the device does not have).

---

## Public interface in short (proposal)

| Group | Functions |
|---|---|
| Module management | `VrefBuf_Get_ModuleVersion`, `VrefBuf_Init`, `VrefBuf_Deinit`, `VrefBuf_Task`, `VrefBuf_Get_DefaultConfig` |
| Control | `VrefBuf_Set_Mode` / `Get`, `VrefBuf_Set_Voltage` / `Get`, `VrefBuf_Get_State` |
| Trimming | `VrefBuf_Set_Trimming`, `VrefBuf_Get_Trimming` |

```c
vrefBuf_ModuleVersion_t VrefBuf_Get_ModuleVersion  ( void );

vrefBuf_RequestState_t  VrefBuf_Init               ( const vrefBuf_Config_t * const vrefBufConfig );
vrefBuf_RequestState_t  VrefBuf_Deinit             ( void );
void                    VrefBuf_Task               ( void );
vrefBuf_RequestState_t  VrefBuf_Get_DefaultConfig  ( vrefBuf_Config_t * const vrefBufConfig );

vrefBuf_RequestState_t  VrefBuf_Set_Mode           ( vrefBuf_Mode_t mode );
vrefBuf_RequestState_t  VrefBuf_Set_Voltage        ( vrefBuf_Voltage_t voltage );
vrefBuf_RequestState_t  VrefBuf_Get_State          ( vrefBuf_State_t * const vrefBufState );
vrefBuf_RequestState_t  VrefBuf_Set_Trimming       ( vrefBuf_Trimming_t trimming );
```

Every function returns `VREFBUF_REQUEST_OK` / `VREFBUF_REQUEST_ERROR` and checks its parameters and the state of the module
before it touches a register.

| Type | Meaning |
|---|---|
| `vrefBuf_Config_t` | Mode, voltage, the trimming (optional), the start-up timeout |
| `vrefBuf_Voltage_t` | The levels of the device |
| `vrefBuf_State_t` | `VREFBUF_STATE_OFF`, `VREFBUF_STATE_STARTING`, `VREFBUF_STATE_READY` |

---

## Usage

### The ADC with a 2.048 V reference

```c
#include "VrefBuf_Port.h"

vrefBuf_Config_t vrefBufConfig;

(void)VrefBuf_Get_DefaultConfig( &vrefBufConfig );
vrefBufConfig.Mode    = VREFBUF_MODE_BUFFER;
vrefBufConfig.Voltage = VREFBUF_VOLTAGE_2_048V;

(void)VrefBuf_Init( &vrefBufConfig );       /* returns after the ready flag, the Adc module may convert now */
```

### An external precision reference

```c
(void)VrefBuf_Set_Mode( VREFBUF_MODE_HIGH_IMPEDANCE );     /* VREF+ is supplied by the external reference */
```

---

## Behaviour and limits

- **Dropout.** The buffer needs `VDDA` above the reference by its dropout (the datasheet); with a lower `VDDA` the reference follows `VDDA`
  and the accuracy is lost. The 2.5 V level needs the highest `VDDA`.
- **Load and decoupling.** The output needs the external capacitor on `VREF+`; the current drive is limited (the datasheet) - the buffer is for the ADC / DAC and
  light external loads.
- **Start-up.** The time to the ready flag depends on the capacitor and the level; the converters wait for it.
- **Low power.** The buffer is switched off in the low-power modes where the family says so; the module does not restart it after a Stop (the application calls
  `VrefBuf_Init` or the mode setter again if the family needs it).
- **Not part of the first version:** the measurement of the real reference, the temperature compensation of the trimming.

---

## Dependencies

- **Rcc** - peripheral clock and reset
- **Adc**, **Dac**, **Comp** - the consumers of the reference (they check `VrefBuf_Get_State`)
- **RAL (Register Abstraction Layer)** - used internally to access the registers

All mandatory RCC configurations are handled internally.

---

## Testing

- **Unit tests** (host, Unity + CMock + RegMem): the mode and the voltage combinations (accepted and rejected), the start-up wait with the
  ready flag and its timeout, the state transitions, the trimming, the rejected levels.
- **Integration tests** (target, probe-rs): the reference level measured by the ADC against `VREFINT` and an external meter, the change of the voltage while
  running, the high-impedance mode with an external source.

---

## Interface conventions

The module follows the common rules of the MCAL layer, so that it behaves like the other modules:

- **Lists instead of free members.** Everything that the hardware restricts (the pins, the kernel clock sources, the DMA
  channels, the triggers and the other selectable sources) is a `typedef enum` list with exactly the items that are valid
  for the device line; a pin is an item `VREFBUF_<SIGNAL>_PIN_<INSTANCE>_P<port><pin>` and there is an `*_UNUSED` item. A choice
  that is not in the list cannot be written.
- **Own peripheral only.** The module calls only the registers / LL functions of its own peripheral. The clocks, the
  interrupts, the pins, the DMA and the EXTI lines go through the API of the Rcc, Nvic, Gpio, Gpdma and Exti modules; a
  missing function is added to that module, it is not bypassed.
- **Init verifies itself.** `VrefBuf_Init` checks the result at the end; when it is not as required, the module is
  deinitialized and an error is returned. The waiting for the hardware is a loop on the register state with a timeout,
  never a blind delay.
- **Transfer modes.** Where the module moves data, the POLL (served by `VrefBuf_Task`), ISR and DMA modes have the same
  configuration structure, the same request and the same callbacks (error, half transfer, transfer complete).
- **Flags and states are enums of the module** (`vrefBuf_FunctionState_t`, `vrefBuf_FlagState_t`), the limits and the constants are
  named, there are no magic numbers in the interface.
- **Unsupported functions stay.** A function without a meaning in a family keeps its signature, suppresses the unused
  parameters and returns `VREFBUF_REQUEST_ERROR`.
- **Documentation.** Every function and type has a Doxygen description, the main header has the `\defgroup`.

---

## Repository layout and branches

The master holds this README and the license. The family branches (`Dev/<Family>`, `Releases/<Family>`) hold:

| Path | Content |
|---|---|
| `README.md` | README of the family: the instances of the device, specifics, errata, status |
| `VrefBuf_Port.h`, `VrefBuf_Types.h` | Public interface (identical in all families) |
| `VrefBuf*.c`, `VrefBuf.h` | Implementation of the family |
| `CMakeLists.txt` | Library `VrefBuf_Lib` of the EmBi platform |
| `Tests/UnitTests` | Unit tests (Unity + CMock + RegMem) |
| `Tests/IntegrationTests` | Integration tests on the target (probe-rs) |

A change of the public interface is made in all families.

---

## CMake integration

```cmake
target_link_libraries( User_Lib PRIVATE VrefBuf_Lib )
```

1. Link `VrefBuf_Lib` to your CMake library.
2. Include `VrefBuf_Port.h` in your project.
3. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** - [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.
