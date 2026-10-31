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

> **Status: STM32H5 implemented (version 0.1.0).** The interface below is implemented for STM32H5 in `VrefBuf_Port.h` /
> `VrefBuf_Types.h` (branch `Dev/STM32H5`), the registers are accessed only by the private `VrefBuf_Reg.c/h`. Verified: unit tests
> (Unity + CMock + RegMem, 68 test cases) on one preset of each of the 12 STM32H5 lines (H503, H523, H533, H543, H553, H562, H563,
> H573, H5E4, H5E5, H5F4, H5F5) and a compilation of the module for all of them. The integration tests (`Tests/IntegrationTests`)
> are only compiled (`STM32H503xB_IntegrationTest`, `STM32H563xI_IntegrationTest`), they have not been run on a board yet. Other
> families: stories and branches follow.
>
> Notes of the STM32H5 implementation (the interface above is unchanged, the points below are the STM32H5 specifics and the places
> where the general text of this README is refined for the hardware):
> - **Line without the peripheral.** STM32H503 has no VREFBUF (the pins VREF+ and VREF- are bonded to VDDA and VSSA, there is no
>   CMSIS definition). The module compiles there, every public function checks the peripheral (`#if defined(VREFBUF)`), keeps
>   its signature and returns `VREFBUF_REQUEST_ERROR` without it; `VrefBuf_Get_ModuleVersion` returns the version and `VrefBuf_Task`
>   does nothing. Unit tests have line-conditional cases for the present and the absent peripheral.
> - **Modes.** The bits ENVR and HIZ of `VREFBUF_CSR` select the hardware modes. `VREFBUF_MODE_BUFFER` is ENVR = 1, HIZ = 0 (internal
>   voltage reference mode), `VREFBUF_MODE_HIGH_IMPEDANCE` is ENVR = 0, HIZ = 1 (external voltage reference mode, the pin VREF+ is an
>   input, reset state). `VREFBUF_MODE_OFF` is ENVR = 0, HIZ = 0: the buffer is off **and the pin VREF+ is pulled down to VSSA**
>   (reference manual, table of the buffer modes), so on STM32H5 the off mode is not "the reference is VDDA" as the general text
>   above says. The mode shall not be used where VREF+ is connected to VDDA. The hold mode of the hardware (ENVR = 1, HIZ = 1) is not
>   a mode of the module: it is used only internally while the level of the running buffer is changed and `VrefBuf_Get_Mode`
>   returns `VREFBUF_REQUEST_ERROR` for it.
> - **Levels.** The field VRS selects 2.5 V (000, the reset level), 2.048 V (001) and 1.8 V (010), other values are reserved. The
>   hardware lets the software write VRS only while the buffer is disabled (ENVR = 0). A change of the level of the running buffer
>   therefore goes through the high impedance mode (the external capacitor holds the voltage, the pin is never pulled down), then
>   the previous mode is restored and the ready flag is awaited. Datasheet (normal mode): the level 2.5 V needs VDDA of at least
>   2.8 V, 2.048 V at least 2.4 V and 1.8 V at least 2.1 V; below it the buffer is in the degraded mode and follows VDDA.
> - **Trimming.** `TRIM[5:0]` (codes 0 ... 63, `VREFBUF_TRIMMING_MAX`) holds the factory trimming of the selected level; the hardware
>   loads it at reset and at every change of VRS, until the first write of TRIM, which stops the loading until the next reset of
>   the peripheral. `VrefBuf_Init` and `VrefBuf_Deinit` reset the peripheral through the Rcc module, so the factory trimming is
>   active again after them. The reference manual states that an application trimming procedure shall run through the codes in
>   ascending order. `VREFBUF_TRIMMING_UNUSED` in the configuration keeps the factory trimming.
> - **Init and Deinit.** `VrefBuf_Init` checks the whole configuration before any register is written, activates the clock, resets the
>   peripheral, writes the level, the optional trimming and the mode (in this order, so the buffer starts with the final level
>   and trimming), waits for the ready flag in the buffer mode and verifies mode, level, trimming and the ready state by read-back;
>   on a failure the module is deinitialized and an error is returned. `VrefBuf_Deinit` resets the peripheral and disables its clock:
>   the pin VREF+ is then an input (external voltage reference mode), the reference of the converters is the voltage connected to
>   it (VDDA on the boards where it is connected). It can be called in any state.
> - **State and getters.** The state is read from the registers: `VREFBUF_STATE_READY` in the buffer mode with the flag VRR set,
>   `VREFBUF_STATE_STARTING` in the buffer mode without it (start-up, or the output fell below the requested level), otherwise
>   `VREFBUF_STATE_OFF`. The getters have no side effects (they never enable a clock) and return `VREFBUF_REQUEST_ERROR` before
>   `VrefBuf_Init`. A failed change of the mode or the level (ready flag not set within the timeout) returns an error and leaves the
>   buffer enabled, the state tells `VREFBUF_STATE_STARTING`.
> - **Start-up timeout.** `vrefBuf_Config_t::StartTimeout` is a count of read-backs of the ready flag (no time base is used); the default is
>   100000, the start-up time of the buffer is below 1 ms (datasheet, load capacitor 0.5 ... 1.5 uF).
> - **Interface details derived from the tables of this README.** The code block of the short interface lists only part of the
>   functions, the table lists also `VrefBuf_Get_Mode`, `VrefBuf_Get_Voltage` and `VrefBuf_Get_Trimming`, which are implemented with
>   an output pointer like `VrefBuf_Get_State`. The configuration members are `Mode`, `Voltage`, `Trimming` and `StartTimeout`. The
>   default configuration is the buffer mode at 2.5 V with the factory trimming. `VrefBuf_Task` has no periodic work (the start-up is
>   awaited inside of the functions).
> - **Device erratum ES0565 2.8.1 (bug AB#1996).** "Slow discharge of the capacitor connected to VREF+ pin" (STM32H562, STM32H563,
>   STM32H573, ES0565 Rev 7): the pull-down resistor on the VREFBUF output is missing, so the capacitor on VREF+ discharges slowly
>   when the output is switched down. The workaround of the erratum is to wait a sufficient time for the discharge when the output
>   is switched down. The module does not apply it: the time depends on the capacitor and the load of the board, no register
>   reports it and a blind delay is not allowed. The application waits before it relies on the lower voltage on VREF+ after
>   `VrefBuf_Set_Mode` to `VREFBUF_MODE_OFF` or `VREFBUF_MODE_HIGH_IMPEDANCE`, after `VrefBuf_Set_Voltage` to a lower level and after
>   `VrefBuf_Deinit` / `VrefBuf_Init` (peripheral reset); the Doxygen of these functions carries the note. The wait of the module for the
>   ready flag stays bounded by `vrefBuf_Config_t::StartTimeout`, the erratum does not describe the ready flag during the slow discharge
>   (not verified, no hardware). The errata sheets of STM32H503 (ES0561) and STM32H523 / STM32H533 (ES0621) list no VREFBUF erratum;
>   the errata sheets of STM32H543, STM32H553, STM32H5E4, STM32H5E5, STM32H5F4 and STM32H5F5 were not available, the erratum is not
>   verified for these lines. The unit and integration test cases which switch the output down carry the errata note.
> - **Packages.** On the packages where VREF+ is bonded to VDDA the buffer is not available and shall be kept disabled; the module
>   cannot detect the package, the application does not initialize the module there. A board where the pin VREF+ is connected to
>   VDDA by a bridge needs the bridge open before the buffer mode is used.

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
| STM32H5 | `Dev/STM32H5` | `VREFBUF` on all lines except STM32H503: 2.5 V, 2.048 V, 1.8 V levels, high-impedance mode, ready flag, trimming | implemented (0.1.0) |
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
- **Every function checks the peripheral.** The functions keep their signature on all families; on the line without the
  peripheral (`VREFBUF` is not defined, STM32H503) a function suppresses the unused parameters and returns
  `VREFBUF_REQUEST_ERROR` (`VrefBuf_Task` does nothing, `VrefBuf_Get_ModuleVersion` returns the version).
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
