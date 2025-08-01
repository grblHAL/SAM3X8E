/*

  driver.c - driver code for Atmel SAM3X8E ARM processor

  Part of grblHAL

  Copyright (c) 2019-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdlib.h>

#include "driver.h"
#include "serial.h"

#include "grbl/machine_limits.h"
#include "grbl/crossbar.h"
#include "grbl/motor_pins.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/pin_bits_masks.h"
#include "grbl/task.h"

#if USB_SERIAL_CDC
#include "usb_serial.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "diskio.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if I2C_ENABLE
#include "i2c.h"
#endif

#ifndef OUTPUT
#define OUTPUT true
#endif
#ifndef INPUT
#define INPUT false
#endif

static bool IOInitDone = false;
static pin_debounce_t debounce;
static delay_t delay_ms = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static input_signal_t *a_signals[10] = {0}, *b_signals[10] = {0}, *c_signals[10] = {0}, *d_signals[10] = {0};
static struct {
    uint32_t length;
    uint32_t delay;
    uint32_t t_min_period; // timer ticks
    axes_signals_t out;
#if STEP_INJECT_ENABLE
    struct {
        axes_signals_t claimed;
        volatile axes_signals_t axes;
        volatile axes_signals_t out;
    } inject;
#endif
} step_pulse = {};
#ifdef SQUARING_ENABLED
static axes_signals_t motors_1 = {AXES_BITMASK}, motors_2 = {AXES_BITMASK};
#endif
#if DRIVER_SPINDLE_ENABLE
static spindle_id_t spindle_id = -1;
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
static spindle_pwm_t spindle_pwm;
#endif

static pin_group_pins_t limit_inputs = {0};
static periph_signal_t *periph_pins = NULL;

static input_signal_t inputpin[] = {
    { .id = Input_LimitX,       .port = X_LIMIT_PORT,      .pin = X_LIMIT_PIN,       .group = PinGroup_Limit },
  #ifdef X2_LIMIT_PIN
    { .id = Input_LimitX_2,     .port = X2_LIMIT_PORT,     .pin = X2_LIMIT_PIN,      .group = PinGroup_Limit },
  #endif
  #ifdef X_LIMIT_PIN_MAX
    { .id = Input_LimitX_Max,   .port = X_LIMIT_PORT_MAX,  .pin = X_LIMIT_PIN_MAX,   .group = PinGroup_LimitMax },
  #endif
    { .id = Input_LimitY,       .port = Y_LIMIT_PORT,      .pin = Y_LIMIT_PIN,       .group = PinGroup_Limit },
  #ifdef Y2_LIMIT_PIN
   { .id = Input_LimitY_2,      .port = Y2_LIMIT_PORT,     .pin = Y2_LIMIT_PIN,      .group = PinGroup_Limit },
  #endif
  #ifdef Y_LIMIT_PIN_MAX
    { .id = Input_LimitY_Max,   .port = Y_LIMIT_PORT_MAX,  .pin = Y_LIMIT_PIN_MAX,   .group = PinGroup_LimitMax },
  #endif
    { .id = Input_LimitZ,       .port = Z_LIMIT_PORT,      .pin = Z_LIMIT_PIN,       .group = PinGroup_Limit },
  #ifdef Z2_LIMIT_PIN
    { .id = Input_LimitZ_2,     .port = Z2_LIMIT_PORT,     .pin = Z2_LIMIT_PIN,      .group = PinGroup_Limit },
  #endif
  #ifdef Z_LIMIT_PIN_MAX
    { .id = Input_LimitZ_Max,   .port = Z_LIMIT_PORT_MAX,  .pin = Z_LIMIT_PIN_MAX,   .group = PinGroup_LimitMax },
  #endif
  #ifdef A_LIMIT_PIN
    { .id = Input_LimitA,       .port = A_LIMIT_PORT,      .pin = A_LIMIT_PIN,       .group = PinGroup_Limit },
  #endif
  #ifdef A_LIMIT_PIN_MAX
    { .id = Input_LimitA_Max,   .port = A_LIMIT_PORT_MAX,  .pin = A_LIMIT_PIN_MAX,   .group = PinGroup_LimitMax },
  #endif
  #ifdef B_LIMIT_PIN
    { .id = Input_LimitB,       .port = B_LIMIT_PORT,      .pin = B_LIMIT_PIN,       .group = PinGroup_Limit },
  #endif
  #ifdef B_LIMIT_PIN_MAX
    { .id = Input_LimitB_Max,   .port = B_LIMIT_PORT_MAX,  .pin = B_LIMIT_PIN_MAX,   .group = PinGroup_LimitMax },
  #endif
  #ifdef C_LIMIT_PIN
    { .id = Input_LimitC,       .port = C_LIMIT_PORT,      .pin = C_LIMIT_PIN,       .group = PinGroup_Limit },
  #endif
  #ifdef C_LIMIT_PIN_MAX
    { .id = Input_LimitC_Max,   .port = C_LIMIT_PORT_MAX,  .pin = C_LIMIT_PIN_MAX,   .group = PinGroup_LimitMax },
  #endif
  #ifdef I2C_STROBE_PORT
    { .id = Input_KeypadStrobe, .port = I2C_STROBE_PORT,   .pin = I2C_STROBE_PIN,    .group = PinGroup_Keypad },
  #endif
    // Aux input pins must be consecutive in this array
#ifdef AUXINPUT0_PIN
    { .id = Input_Aux0,         .port = AUXINPUT0_PORT,    .pin = AUXINPUT0_PIN,     .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT1_PIN
    { .id = Input_Aux1,         .port = AUXINPUT1_PORT,    .pin = AUXINPUT1_PIN,     .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT2_PIN
    { .id = Input_Aux2,         .port = AUXINPUT2_PORT,    .pin = AUXINPUT2_PIN,     .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT3_PIN
    { .id = Input_Aux3,         .port = AUXINPUT3_PORT,    .pin = AUXINPUT3_PIN,     .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT4_PIN
    { .id = Input_Aux4,         .port = AUXINPUT4_PORT,    .pin = AUXINPUT4_PIN,     .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT5_PIN
    { .id = Input_Aux5,         .port = AUXINPUT5_PORT,    .pin = AUXINPUT5_PIN,     .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT6_PIN
    { .id = Input_Aux6,         .port = AUXINPUT6_PORT,    .pin = AUXINPUT6_PIN,     .group = PinGroup_AuxInput },
#endif
#ifdef AUXINPUT7_PIN
    { .id = Input_Aux7,         .port = AUXINPUT7_PORT,    .pin = AUXINPUT7_PIN,     .group = PinGroup_AuxInput },
#endif
#ifdef AUXINTPUT0_ANALOG_PIN
    { .id = Input_Analog_Aux0,  .port = AUXINTPUT0_ANALOG_PORT, .pin = AUXINTPUT0_ANALOG_PIN, .group = PinGroup_AuxInputAnalog },
#endif
#ifdef AUXINTPUT1_ANALOG_PIN
    { .id = Input_Analog_Aux1,  .port = AUXINTPUT1_ANALOG_PORT, .pin = AUXINTPUT1_ANALOG_PIN, .group = PinGroup_AuxInputAnalog }
#endif
};

static output_signal_t outputpin[] = {
    { .id = Output_StepX,           .port = X_STEP_PORT,                .pin = X_STEP_PIN,              .group = PinGroup_StepperStep },
#ifdef X2_STEP_PORT
    { .id = Output_StepX_2,          .port = X2_STEP_PORT,               .pin = X2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
    { .id = Output_StepY,           .port = Y_STEP_PORT,                .pin = Y_STEP_PIN,              .group = PinGroup_StepperStep },
#ifdef Y2_STEP_PORT
    { .id = Output_StepY_2,         .port = Y2_STEP_PORT,               .pin = Y2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
    { .id = Output_StepZ,           .port = Z_STEP_PORT,                .pin = Z_STEP_PIN,              .group = PinGroup_StepperStep },
#ifdef Z2_STEP_PORT
    { .id = Output_StepZ_2,         .port = Z2_STEP_PORT,               .pin = Z2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
#ifdef A_AXIS
    { .id = Output_StepA,           .port = A_STEP_PORT,                .pin = A_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
#ifdef B_AXIS
    { .id = Output_StepB,           .port = B_STEP_PORT,                .pin = B_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
#ifdef C_AXIS
    { .id = Output_StepC,           .port = C_STEP_PORT,                .pin = C_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
    { .id = Output_DirX,            .port = X_DIRECTION_PORT,           .pin = X_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#ifdef X2_DIRECTION_PIN
    { .id = Output_DirX_2,          .port = X2_DIRECTION_PORT,          .pin = X2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
    { .id = Output_DirY,            .port = Y_DIRECTION_PORT,           .pin = Y_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#ifdef Y2_DIRECTION_PIN
    { .id = Output_DirY_2,          .port = Y2_DIRECTION_PORT,          .pin = Y2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
    { .id = Output_DirZ,            .port = Z_DIRECTION_PORT,           .pin = Z_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#ifdef Z2_DIRECTION_PIN
    { .id = Output_DirZ_2,          .port = Z2_DIRECTION_PORT,          .pin = Z2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
#ifdef A_AXIS
    { .id = Output_DirA,            .port = A_DIRECTION_PORT,           .pin = A_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#ifdef B_AXIS
    { .id = Output_DirB,            .port = B_DIRECTION_PORT,           .pin = B_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#ifdef C_AXIS
    { .id = Output_DirC,            .port = C_DIRECTION_PORT,           .pin = C_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#if !TRINAMIC_ENABLE
#ifdef STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnable,   .port = STEPPERS_ENABLE_PORT,       .pin = STEPPERS_ENABLE_PIN,     .group = PinGroup_StepperEnable },
#endif
#ifdef X_ENABLE_PORT
    { .id = Output_StepperEnableX,  .port = X_ENABLE_PORT,              .pin = X_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef X2_ENABLE_PORT
    { .id = Output_StepperEnableX,  .port = X2_ENABLE_PORT,             .pin = X2_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#ifdef Y_ENABLE_PORT
    { .id = Output_StepperEnableY,  .port = Y_ENABLE_PORT,              .pin = Y_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef Y2_ENABLE_PORT
    { .id = Output_StepperEnableY,  .port = Y2_ENABLE_PORT,             .pin = Y2_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#ifdef Z_ENABLE_PORT
    { .id = Output_StepperEnableZ,  .port = Z_ENABLE_PORT,              .pin = Z_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef Z2_ENABLE_PORT
    { .id = Output_StepperEnableZ,  .port = Z2_ENABLE_PORT,             .pin = Z2_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#ifdef A_ENABLE_PORT
    { .id = Output_StepperEnableA,  .port = A_ENABLE_PORT,              .pin = A_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef B_ENABLE_PORT
    { .id = Output_StepperEnableB,  .port = B_ENABLE_PORT,              .pin = B_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef C_ENABLE_PORT
    { .id = Output_StepperEnableC,  .port = C_ENABLE_PORT,              .pin = C_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#endif
#ifdef SD_CS_PORT
    { .id = Output_SdCardCS,        .port = SD_CS_PORT,                 .pin = SD_CS_PIN,               .group = PinGroup_SdCard },
#endif
#ifdef AUXOUTPUT0_PORT
    { .id = Output_Aux0,            .port = AUXOUTPUT0_PORT,            .pin = AUXOUTPUT0_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT1_PORT
    { .id = Output_Aux1,            .port = AUXOUTPUT1_PORT,            .pin = AUXOUTPUT1_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT2_PORT
    { .id = Output_Aux2,            .port = AUXOUTPUT2_PORT,            .pin = AUXOUTPUT2_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT3_PORT
    { .id = Output_Aux3,            .port = AUXOUTPUT3_PORT,            .pin = AUXOUTPUT3_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT4_PORT
    { .id = Output_Aux4,            .port = AUXOUTPUT4_PORT,            .pin = AUXOUTPUT4_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT5_PORT
    { .id = Output_Aux5,            .port = AUXOUTPUT5_PORT,            .pin = AUXOUTPUT5_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT6_PORT
    { .id = Output_Aux6,            .port = AUXOUTPUT6_PORT,            .pin = AUXOUTPUT6_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT7_PORT
    { .id = Output_Aux7,            .port = AUXOUTPUT7_PORT,            .pin = AUXOUTPUT7_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT8_PORT
    { .id = Output_Aux8,            .port = AUXOUTPUT8_PORT,            .pin = AUXOUTPUT8_PIN,          .group = PinGroup_AuxOutput }
#endif
};

static uint32_t vectorTable[sizeof(DeviceVectors) / sizeof(uint32_t)] __attribute__(( aligned (0x100ul) ));

static void SysTick_IRQHandler (void);
static void STEPPER_IRQHandler (void);
static void STEP_IRQHandler (void);
static void STEPDELAY_IRQHandler (void);
static void PIOA_IRQHandler (void);
static void PIOB_IRQHandler (void);
static void PIOC_IRQHandler (void);
static void PIOD_IRQHandler (void);

extern void Dummy_Handler(void);

#ifdef SAFETY_DOOR_PIN
static input_signal_t *door_pin;
#endif
#ifdef MPG_MODE_PIN
static uint8_t mpg_port;
static input_signal_t *mpg_pin;
#endif

static void aux_irq_handler (uint8_t port, bool state);

#if I2C_STROBE_ENABLE

static driver_irq_handler_t i2c_strobe = { .type = IRQ_I2C_Strobe };

static bool irq_claim (irq_type_t irq, uint_fast8_t id, irq_callback_ptr handler)
{
    bool ok;

    if((ok = irq == IRQ_I2C_Strobe && i2c_strobe.callback == NULL))
        i2c_strobe.callback = handler;

    return ok;
}

#endif

inline __attribute__((always_inline)) void IRQRegister(int32_t IRQnum, void (*IRQhandler)(void))
{
    vectorTable[IRQnum + 16] = (uint32_t)IRQhandler;
}

void IRQUnRegister(int32_t IRQnum)
{
    vectorTable[IRQnum + 16] = (uint32_t)Dummy_Handler;
}

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if((delay_ms.ms = ms) > 0) {
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        if(!(delay_ms.callback = callback)) {
            while(delay_ms.ms)
                grbl.on_execute_delay(state_get());
        }
    } else if(callback)
        callback();
}

// Set stepper pulse output pins

#ifdef SQUARING_ENABLED

inline static __attribute__((always_inline)) void set_step_outputs (axes_signals_t step_out1)
{
    axes_signals_t step_out2;

#if STEP_INJECT_ENABLE

    axes_signals_t axes = { .bits = step_pulse.inject.axes.bits };

    if(axes.bits) {

        step_out2.bits = step_out1.bits & motors_2.bits;
        step_out1.bits = step_out1.bits & motors_1.bits;

        uint_fast8_t idx, mask = 1;
        axes_signals_t step1 = { .bits = step_out1.bits },
                       step2 = { .bits = step_out2.bits };

        step_out1.bits ^= settings.steppers.step_invert.bits;
        step_out2.bits ^= settings.steppers.step_invert.bits;

        for(idx = 0; idx < N_AXIS; idx++) {

            if(!(axes.bits & mask)) {

                if(step2.bits & mask) switch(idx) {
#ifdef X2_STEP_PIN
                    case X_AXIS:
                        BITBAND_PERI(X2_STEP_PORT->PIO_ODSR, X2_STEP_PIN) = step_out2.x;
                        break;
#endif
#ifdef Y2_STEP_PIN
                    case Y_AXIS:
                        BITBAND_PERI(Y2_STEP_PORT->PIO_ODSR, Y2_STEP_PIN) = step_out2.y;
                        break;
#endif
#ifdef Z2_STEP_PIN
                    case Z_AXIS:
                        BITBAND_PERI(Z2_STEP_PORT->PIO_ODSR, Z2_STEP_PIN) = step_out2.z;
                        break;
#endif
                }
            }

            if(step1.bits & mask) switch(idx) {

                case X_AXIS:
                    BITBAND_PERI(X_STEP_PORT->PIO_ODSR, X_STEP_PIN) = step_out1.x;
                    break;

                case Y_AXIS:
                    BITBAND_PERI(Y_STEP_PORT->PIO_ODSR, Y_STEP_PIN) = step_out1.y;
                    break;

                case Z_AXIS:
                    BITBAND_PERI(Z_STEP_PORT->PIO_ODSR, Z_STEP_PIN) = step_out1.z;
                    break;
#ifdef A_AXIS
                case A_AXIS:
                    BITBAND_PERI(A_STEP_PORT->PIO_ODSR, A_STEP_PIN) = step_out1.a;
                    break;
#endif
#ifdef B_AXIS
                case B_AXIS:
                    BITBAND_PERI(B_STEP_PORT->PIO_ODSR, B_STEP_PIN) = step_out1.b;
                    break;
#endif
#ifdef C_AXIS
                case C_AXIS:
                    BITBAND_PERI(C_STEP_PORT->PIO_ODSR, C_STEP_PIN) = step_out1.c;
                    break;
#endif
            }
            mask <<= 1;
        }
    } else {

#endif // STEP_INJECT_ENABLE

    step_out2.bits = (step_out1.bits & motors_2.bits) ^ settings.steppers.step_invert.bits;
    step_out1.bits = (step_out1.bits & motors_1.bits) ^ settings.steppers.step_invert.bits;

    BITBAND_PERI(X_STEP_PORT->PIO_ODSR, X_STEP_PIN) = step_out1.x;
  #ifdef X2_STEP_PIN
    BITBAND_PERI(X2_STEP_PORT->PIO_ODSR, X2_STEP_PIN) = step_out2.x;
  #endif

    BITBAND_PERI(Y_STEP_PORT->PIO_ODSR, Y_STEP_PIN) = step_out1.y;
  #ifdef Y2_STEP_PIN
    BITBAND_PERI(Y2_STEP_PORT->PIO_ODSR, Y2_STEP_PIN) = step_out2.y;
  #endif
 
    BITBAND_PERI(Z_STEP_PORT->PIO_ODSR, Z_STEP_PIN) = step_out1.z;
  #ifdef Z2_STEP_PIN
    BITBAND_PERI(Z2_STEP_PORT->PIO_ODSR, Z2_STEP_PIN) = step_out2.z;
  #endif

  #ifdef A_STEP_PIN
    BITBAND_PERI(A_STEP_PORT->PIO_ODSR, A_STEP_PIN) = step_out1.a;
  #endif
  #ifdef B_STEP_PIN
    BITBAND_PERI(B_STEP_PORT->PIO_ODSR, B_STEP_PIN) = step_out1.b;
  #endif
  #ifdef C_STEP_PIN
    BITBAND_PERI(C_STEP_PORT->PIO_ODSR, C_STEP_PIN) = step_out1.c;
  #endif
#if STEP_INJECT_ENABLE
    }
#endif
}

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_1.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
    motors_2.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
}

#else // SQUARING DISABLED

inline static void __attribute__((always_inline)) set_step_outputs (axes_signals_t step_out)
{
#if STEP_INJECT_ENABLE

    axes_signals_t axes = { .bits = step_pulse.inject.axes.bits };

    if(axes.bits) {

        uint_fast8_t idx, mask = 1;

        step_out.bits ^= settings.steppers.step_invert.bits;

        for(idx = 0; idx < N_AXIS; idx++) {

            if(!(axes.bits & mask)) switch(idx) {

                case X_AXIS:
                    BITBAND_PERI(X_STEP_PORT->PIO_ODSR, X_STEP_PIN) = step_out.x;
#ifdef X2_STEP_PIN
                    BITBAND_PERI(X2_STEP_PORT->PIO_ODSR, X2_STEP_PIN) = step_out.x;
#endif
                    break;

                case Y_AXIS:
                    BITBAND_PERI(Y_STEP_PORT->PIO_ODSR, Y_STEP_PIN) = step_out.y;
#ifdef Y2_STEP_PIN
                    BITBAND_PERI(Y2_STEP_PORT->PIO_ODSR, Y2_STEP_PIN) = step_out.y;
#endif
                    break;

                case Z_AXIS:
                    BITBAND_PERI(Z_STEP_PORT->PIO_ODSR, Z_STEP_PIN) = step_out.z;
#ifdef Z2_STEP_PIN
                    BITBAND_PERI(Z2_STEP_PORT->PIO_ODSR, Z2_STEP_PIN) = step_out.z;
#endif
                    break;

#ifdef A_AXIS
                case A_AXIS:
                    BITBAND_PERI(A_STEP_PORT->PIO_ODSR, A_STEP_PIN) = step_out.a;
                    break;

#endif
#ifdef B_AXIS
                case B_AXIS:
                    BITBAND_PERI(B_STEP_PORT->PIO_ODSR, B_STEP_PIN) = step_out.b;
                    break;
#endif
#ifdef C_AXIS
                case C_AXIS:
                    BITBAND_PERI(C_STEP_PORT->PIO_ODSR, C_STEP_PIN) = step_out.c;
                    break;
#endif
            }
            mask <<= 1;
        }
    } else {

#endif // STEP_INJECT_ENABLE

    step_out.value ^= settings.steppers.step_invert.mask;

    BITBAND_PERI(X_STEP_PORT->PIO_ODSR, X_STEP_PIN) = step_out.x;
  #ifdef X2_STEP_PIN
    BITBAND_PERI(X2_STEP_PORT->PIO_ODSR, X2_STEP_PIN) = step_out.x;
  #endif
    
    BITBAND_PERI(Y_STEP_PORT->PIO_ODSR, Y_STEP_PIN) = step_out.y;
  #ifdef Y2_STEP_PIN
    BITBAND_PERI(Y2_STEP_PORT->PIO_ODSR, Y2_STEP_PIN) = step_out.y;
  #endif

    BITBAND_PERI(Z_STEP_PORT->PIO_ODSR, Z_STEP_PIN) = step_out.z;
  #ifdef Z2_STEP_PIN
    BITBAND_PERI(Z2_STEP_PORT->PIO_ODSR, Z2_STEP_PIN) = step_out.z;
  #endif 

  #ifdef A_STEP_PIN
    BITBAND_PERI(A_STEP_PORT->PIO_ODSR, A_STEP_PIN) = step_out.a;
  #endif
  #ifdef B_STEP_PIN
    BITBAND_PERI(B_STEP_PORT->PIO_ODSR, B_STEP_PIN) = step_out.b;
  #endif
  #ifdef C_STEP_PIN
    BITBAND_PERI(C_STEP_PORT->PIO_ODSR, C_STEP_PIN) = step_out.c;
  #endif
#if STEP_INJECT_ENABLE
    }
#endif
}

#endif

#ifdef GANGING_ENABLED

static axes_signals_t getGangedAxes (bool auto_squared)
{
    axes_signals_t ganged = {0};

    if(auto_squared) {
        #if X_AUTO_SQUARE
            ganged.x = On;
        #endif
        #if Y_AUTO_SQUARE
            ganged.y = On;
        #endif
        #if Z_AUTO_SQUARE
            ganged.z = On;
        #endif
    } else {
        #if X_GANGED
            ganged.x = On;
        #endif
        #if Y_GANGED
            ganged.y = On;
        #endif
        #if Z_GANGED
            ganged.z = On;
        #endif
    }

    return ganged;
}

#endif

// Set stepper direction output pins
inline static __attribute__((always_inline)) void set_dir_outputs (axes_signals_t dir_out)
{
#if STEP_INJECT_ENABLE

    axes_signals_t axes = { .bits = step_pulse.inject.axes.bits };

    if(axes.bits) {

        uint_fast8_t idx, mask = 1;

        dir_out.bits ^= settings.steppers.dir_invert.bits;

        for(idx = 0; idx < N_AXIS; idx++) {

            if(!(axes.bits & mask)) switch(idx) {

                case X_AXIS:
                    BITBAND_PERI(X_DIRECTION_PORT->PIO_ODSR, X_DIRECTION_PIN) = dir_out.x;
#ifdef X2_DIRECTION_PIN
                    BITBAND_PERI(X2_DIRECTION_PORT->PIO_ODSR, X2_DIRECTION_PIN) = dir_out.x ^ settings.steppers.ganged_dir_invert.x;
#endif
                    break;

                case Y_AXIS:
                    BITBAND_PERI(Y_DIRECTION_PORT->PIO_ODSR, Y_DIRECTION_PIN) = dir_out.y;
#ifdef Y2_DIRECTION_PIN
                    BITBAND_PERI(Y2_DIRECTION_PORT->PIO_ODSR, Y2_DIRECTION_PIN) = dir_out.y ^ settings.steppers.ganged_dir_invert.y;
#endif
                    break;

                case Z_AXIS:
                    BITBAND_PERI(Z_DIRECTION_PORT->PIO_ODSR, Z_DIRECTION_PIN) = dir_out.z;
#ifdef Z2_DIRECTION_PIN
                    BITBAND_PERI(Z2_DIRECTION_PORT->PIO_ODSR, Z2_DIRECTION_PIN) = dir_out.z ^ settings.steppers.ganged_dir_invert.z;
#endif
                    break;
#ifdef A_AXIS
                case A_AXIS:
                    BITBAND_PERI(A_DIRECTION_PORT->PIO_ODSR, A_DIRECTION_PIN) = dir_out.a;
                    break;
#endif
#ifdef B_AXIS
                case B_AXIS:
                    BITBAND_PERI(B_DIRECTION_PORT->PIO_ODSR, B_DIRECTION_PIN) = dir_out.b;
                    break;
#endif
#ifdef C_AXIS
                case C_AXIS:
                    BITBAND_PERI(C_DIRECTION_PORT->PIO_ODSR, C_DIRECTION_PIN) = dir_out.c;
                    break;
#endif
            }
            mask <<= 1;
        }
    } else {

#endif // STEP_INJECT_ENABLE

    dir_out.mask ^= settings.steppers.dir_invert.mask;

    BITBAND_PERI(X_DIRECTION_PORT->PIO_ODSR, X_DIRECTION_PIN) = dir_out.x;
    BITBAND_PERI(Y_DIRECTION_PORT->PIO_ODSR, Y_DIRECTION_PIN) = dir_out.y;
    BITBAND_PERI(Z_DIRECTION_PORT->PIO_ODSR, Z_DIRECTION_PIN) = dir_out.z;

#ifdef GANGING_ENABLED
    dir_out.mask ^= settings.steppers.ganged_dir_invert.mask;
  #ifdef X2_DIRECTION_PIN
    BITBAND_PERI(X2_DIRECTION_PORT->PIO_ODSR, X2_DIRECTION_PIN) = dir_out.x;
  #endif
  #ifdef Y2_DIRECTION_PIN
    BITBAND_PERI(Y2_DIRECTION_PORT->PIO_ODSR, Y2_DIRECTION_PIN) = dir_out.y;
  #endif
  #ifdef Z2_DIRECTION_PIN
    BITBAND_PERI(Z2_DIRECTION_PORT->PIO_ODSR, Z2_DIRECTION_PIN) = dir_out.z;
  #endif
#endif

  #ifdef A_DIRECTION_PIN
    BITBAND_PERI(A_DIRECTION_PORT->PIO_ODSR, A_DIRECTION_PIN) = dir_out.a;
  #endif
  #ifdef B_DIRECTION_PIN
    BITBAND_PERI(B_DIRECTION_PORT->PIO_ODSR, B_DIRECTION_PIN) = dir_out.b;
  #endif
  #ifdef C_DIRECTION_PIN
    BITBAND_PERI(C_DIRECTION_PORT->PIO_ODSR, C_DIRECTION_PIN) = dir_out.c;
  #endif

#if STEP_INJECT_ENABLE
    }
#endif
}

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable, bool hold)
{
    enable.value ^= settings.steppers.enable_invert.mask;
#if TRINAMIC_ENABLE == 2130 && TRINAMIC_I2C
    trinamic_stepper_enable(enable);
#else
    BITBAND_PERI(X_ENABLE_PORT->PIO_ODSR, X_ENABLE_PIN) = enable.x;
  #ifdef X2_ENABLE_PIN
    BITBAND_PERI(X2_ENABLE_PORT->PIO_ODSR, X2_ENABLE_PIN) = enable.x;
  #endif
  #ifdef Y_ENABLE_PIN
    BITBAND_PERI(Y_ENABLE_PORT->PIO_ODSR, Y_ENABLE_PIN) = enable.y;
  #endif
  #ifdef Y2_ENABLE_PIN
    BITBAND_PERI(Y2_ENABLE_PORT->PIO_ODSR, Y2_ENABLE_PIN) = enable.y;
  #endif
  #ifdef Z_ENABLE_PIN
    BITBAND_PERI(Z_ENABLE_PORT->PIO_ODSR, Z_ENABLE_PIN) = enable.z;
  #endif
  #ifdef Z2_ENABLE_PIN
    BITBAND_PERI(Z2_ENABLE_PORT->PIO_ODSR, Z2_ENABLE_PIN) = enable.z;
  #endif
  #ifdef A_ENABLE_PIN
    BITBAND_PERI(A_ENABLE_PORT->PIO_ODSR, A_ENABLE_PIN) = enable.a;
  #endif
  #ifdef B_ENABLE_PIN
    BITBAND_PERI(B_ENABLE_PORT->PIO_ODSR, B_ENABLE_PIN) = enable.b;
  #endif
  #ifdef C_ENABLE_PIN
    BITBAND_PERI(C_ENABLE_PORT->PIO_ODSR, C_ENABLE_PIN) = enable.c;
  #endif
#endif
}

// Sets up stepper driver interrupt timeout, AMASS version
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    STEPPER_TIMER.TC_CCR = TC_CCR_CLKDIS;
// Limit min steps/s to about 2 (hal.f_step_timer @ 20MHz)
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    STEPPER_TIMER.TC_RC = cycles_per_tick < (1UL << 18) ? max(cycles_per_tick, step_pulse.t_min_period) : (1UL << 18) - 1UL;
#else
    STEPPER_TIMER.TC_RC = cycles_per_tick < (1UL << 23) ? max(cycles_per_tick, step_pulse.t_min_period) : (1UL << 23) - 1UL;
#endif
    STEPPER_TIMER.TC_CCR = TC_CCR_CLKEN|TC_CCR_SWTRG;
}

// Resets and enables stepper driver ISR timer
static void stepperWakeUp (void)
{
    hal.stepper.enable((axes_signals_t){AXES_BITMASK}, false);
    stepperCyclesPerTick(hal.f_step_timer / 500); // ~2ms delay to allow drivers time to wake up.
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    STEPPER_TIMER.TC_CCR = TC_CCR_CLKDIS;

    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets stepper direction and pulse pins and starts a step pulse.
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        set_dir_outputs(stepper->dir_out);
    }

    if(stepper->step_out.bits) {
        set_step_outputs(stepper->step_out);
        STEP_TIMER.TC_CCR = TC_CCR_SWTRG;
    }
}

// Start a stepper pulse, delay version.
// Note: delay is only added when there is a direction change and a pulse to be output.
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {

        stepper->dir_changed.bits = 0;
        set_dir_outputs(stepper->dir_out);

        if(stepper->step_out.bits) {

            step_pulse.out = stepper->step_out; // Store out_bits

            IRQRegister(STEP_TIMER_IRQn, STEPDELAY_IRQHandler);

            STEP_TIMER.TC_RC = step_pulse.delay;
            STEP_TIMER.TC_CCR = TC_CCR_SWTRG;
        }

        return;
    }

    if(stepper->step_out.bits) {
        set_step_outputs(stepper->step_out);
        STEP_TIMER.TC_CCR = TC_CCR_SWTRG;
    }
}

#if STEP_INJECT_ENABLE

static void output_pulse_isr (void);
static void output_pulse_isr_delayed (void);

static inline __attribute__((always_inline)) void inject_step (axes_signals_t step_out, axes_signals_t axes)
{
    uint_fast8_t idx = N_AXIS - 1, mask = 1 << (N_AXIS - 1);

    step_out.bits ^= settings.steppers.step_invert.bits;

    do {
        if(axes.bits & mask) {

            axes.bits ^= mask;

            switch(idx) {

                case X_AXIS:
                    BITBAND_PERI(X_STEP_PORT->PIO_ODSR, X_STEP_PIN) = step_out.x;
#ifdef X2_STEP_PIN
                    BITBAND_PERI(X2_STEP_PORT->PIO_ODSR, X2_STEP_PIN) = step_out.x;
#endif
                    break;

                case Y_AXIS:
                    BITBAND_PERI(Y_STEP_PORT->PIO_ODSR, Y_STEP_PIN) = step_out.y;
#ifdef Y2_STEP_PIN
                    BITBAND_PERI(Y2_STEP_PORT->PIO_ODSR, Y2_STEP_PIN) = step_out.y;
#endif
                    break;

                case Z_AXIS:
                    BITBAND_PERI(Z_STEP_PORT->PIO_ODSR, Z_STEP_PIN) = step_out.z;
#ifdef Z2_STEP_PIN
                    BITBAND_PERI(Z2_STEP_PORT->PIO_ODSR, Z2_STEP_PIN) = step_out.z;
#endif
                    break;

#ifdef A_AXIS
                case A_AXIS:
                    BITBAND_PERI(A_STEP_PORT->PIO_ODSR, A_STEP_PIN) = step_out.a;
                    break;

#endif
#ifdef B_AXIS
                case B_AXIS:
                    BITBAND_PERI(B_STEP_PORT->PIO_ODSR, B_STEP_PIN) = step_out.b;
                    break;
#endif
#ifdef C_AXIS
                case C_AXIS:
                    BITBAND_PERI(C_STEP_PORT->PIO_ODSR, C_STEP_PIN) = step_out.c;
                    break;
#endif
            }
        }
        idx--;
        mask >>= 1;
    } while(axes.bits);
}

static void stepperClaimMotor (uint_fast8_t axis_id, bool claim)
{
    if(claim)
        step_pulse.inject.claimed.mask |= ((1 << axis_id) & AXES_BITMASK);
    else
        step_pulse.inject.claimed.mask &= ~(1 << axis_id);
}

void stepperOutputStep (axes_signals_t step_out, axes_signals_t dir_out)
{
    if(step_out.bits) {

        uint_fast8_t idx = N_AXIS - 1, mask = 1 << (N_AXIS - 1);
        axes_signals_t axes = { .bits = step_out.bits };

        step_pulse.inject.out = step_out;
        step_pulse.inject.axes.bits = step_pulse.inject.claimed.bits | step_out.bits;
        dir_out.bits ^= settings.steppers.dir_invert.bits;

        do {
            if(axes.bits & mask) {

                axes.bits ^= mask;

                switch(idx) {

                    case X_AXIS:
                        BITBAND_PERI(X_DIRECTION_PORT->PIO_ODSR, X_DIRECTION_PIN) = dir_out.x;
#ifdef X2_DIRECTION_PIN
                        BITBAND_PERI(X2_DIRECTION_PORT->PIO_ODSR, X2_DIRECTION_PIN) = dir_out.x ^ settings.steppers.ganged_dir_invert.x;
#endif
                        break;

                    case Y_AXIS:
                        BITBAND_PERI(Y_DIRECTION_PORT->PIO_ODSR, Y_DIRECTION_PIN) = dir_out.y;
#ifdef Y2_DIRECTION_PIN
                        BITBAND_PERI(Y2_DIRECTION_PORT->PIO_ODSR, Y2_DIRECTION_PIN) = dir_out.y ^ settings.steppers.ganged_dir_invert.y;
#endif
                        break;

                    case Z_AXIS:
                        BITBAND_PERI(Z_DIRECTION_PORT->PIO_ODSR, Z_DIRECTION_PIN) = dir_out.z;
#ifdef Z2_DIRECTION_PIN
                        BITBAND_PERI(Z2_DIRECTION_PORT->PIO_ODSR, Z2_DIRECTION_PIN) = dir_out.z ^ settings.steppers.ganged_dir_invert.z;
#endif
                        break;

#ifdef A_AXIS
                    case A_AXIS:
                        BITBAND_PERI(A_DIRECTION_PORT->PIO_ODSR, A_DIRECTION_PIN) = dir_out.a;
                        break;
#endif
#ifdef B_AXIS
                    case B_AXIS:
                        BITBAND_PERI(B_DIRECTION_PORT->PIO_ODSR, B_DIRECTION_PIN) = dir_out.b;
                        break;
#endif
#ifdef C_AXIS
                    case C_AXIS:
                        BITBAND_PERI(C_DIRECTION_PORT->PIO_ODSR, C_DIRECTION_PIN) = dir_out.c;
                        break;
#endif
                }
            }
            idx--;
            mask >>= 1;
        } while(axes.bits);

        if(step_pulse.delay) {
            IRQRegister(STEP2_TIMER_IRQ, output_pulse_isr_delayed);
            STEP2_TIMER.TC_RC = step_pulse.delay;
            STEP2_TIMER.TC_CCR = TC_CCR_SWTRG;
        } else {
            inject_step(step_out, step_out);
            STEP2_TIMER.TC_CCR = TC_CCR_SWTRG;
        }
    }
}

#endif // STEP_INJECT_ENABLE

// Returns limit state as an limit_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState (void)
{
    limit_signals_t signals = {0};

    signals.min.mask = settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
    signals.min2.mask = settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
    signals.max.mask = settings.limits.invert.mask;
#endif
    
    signals.min.x = BITBAND_PERI(X_LIMIT_PORT->PIO_PDSR, X_LIMIT_PIN);
    signals.min.y = BITBAND_PERI(Y_LIMIT_PORT->PIO_PDSR, Y_LIMIT_PIN);
    signals.min.z = BITBAND_PERI(Z_LIMIT_PORT->PIO_PDSR, Z_LIMIT_PIN);
#ifdef A_LIMIT_PIN
    signals.min.a = BITBAND_PERI(A_LIMIT_PORT->PIO_PDSR, A_LIMIT_PIN);
#endif
#ifdef B_LIMIT_PIN
    signals.min.b = BITBAND_PERI(B_LIMIT_PORT->PIO_PDSR, B_LIMIT_PIN);
#endif
#ifdef C_LIMIT_PIN
    signals.min.c = BITBAND_PERI(C_LIMIT_PORT->PIO_PDSR, C_LIMIT_PIN);
#endif

#ifdef X2_LIMIT_PIN
    signals.min2.x = BITBAND_PERI(X2_LIMIT_PORT->PIO_PDSR, X2_LIMIT_PIN);
#endif
#ifdef Y2_LIMIT_PIN
    signals.min2.y = BITBAND_PERI(Y2_LIMIT_PORT->PIO_PDSR, Y2_LIMIT_PIN);
#endif
#ifdef Z2_LIMIT_PIN
    signals.min2.z = BITBAND_PERI(Z2_LIMIT_PORT->PIO_PDSR, Z2_LIMIT_PIN);
#endif

#ifdef X_LIMIT_PIN_MAX
    signals.max.x = BITBAND_PERI(X_LIMIT_PORT_MAX->PIO_PDSR, X_LIMIT_PIN_MAX);
#endif
#ifdef Y_LIMIT_PIN_MAX
    signals.max.y = BITBAND_PERI(Y_LIMIT_PORT_MAX->PIO_PDSR, Y_LIMIT_PIN_MAX);
#endif
#ifdef Z_LIMIT_PIN_MAX
    signals.max.z = BITBAND_PERI(Z_LIMIT_PORT_MAX->PIO_PDSR, Z_LIMIT_PIN_MAX);
#endif
#ifdef A_LIMIT_PIN_MAX
    signals.max.a = BITBAND_PERI(A_LIMIT_PORT_MAX->PIO_PDSR, A_LIMIT_PIN_MAX);
#endif
#ifdef B_LIMIT_PIN_MAX
    signals.max.b = BITBAND_PERI(B_LIMIT_PORT_MAX->PIO_PDSR, B_LIMIT_PIN_MAX);
#endif
#ifdef C_LIMIT_PIN_MAX
    signals.max.c = BITBAND_PERI(C_LIMIT_PORT_MAX->PIO_PDSR, C_LIMIT_PIN_MAX);
#endif

    if (settings.limits.invert.mask) {
        signals.min.value ^= settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
        signals.min2.mask ^= settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
        signals.max.value ^= settings.limits.invert.mask;
#endif
    }

    return signals;
}

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    bool disable = !on;
    uint32_t i = limit_inputs.n_pins;
    axes_signals_t pin;
    limit_signals_t homing_source = xbar_get_homing_source_from_cycle(homing_cycle);

    do {
        i--;
        if(on && homing_cycle.mask) {
            pin = xbar_fn_to_axismask(limit_inputs.pins.inputs[i].id);
            disable = limit_inputs.pins.inputs[i].group == PinGroup_Limit ? (pin.mask & homing_source.min.mask) : (pin.mask & homing_source.max.mask);
        }
        if(disable)
            limit_inputs.pins.inputs[i].port->PIO_IDR = limit_inputs.pins.inputs[i].bit;
        else
            limit_inputs.pins.inputs[i].port->PIO_IER = limit_inputs.pins.inputs[i].bit;
    } while(i);
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t systemGetState (void)
{
    control_signals_t signals = { settings.control_invert.mask };

#if defined(RESET_PIN) && !ESTOP_ENABLE
    signals.reset = DIGITAL_IN(RESET_PORT, RESET_PIN);
#endif
#if defined(RESET_PIN) && ESTOP_ENABLE
    signals.e_stop = DIGITAL_IN(RESET_PORT, RESET_PIN);
#endif
#ifdef FEED_HOLD_PIN
    signals.feed_hold = DIGITAL_IN(FEED_HOLD_PORT, FEED_HOLD_PIN);
#endif
#ifdef CYCLE_START_PIN
    signals.cycle_start = DIGITAL_IN(CYCLE_START_PORT, CYCLE_START_PIN);
#endif

#ifdef SAFETY_DOOR_PIN
    if(debounce.safety_door)
        signals.safety_door_ajar = !settings.control_invert.safety_door_ajar;
    else
        signals.safety_door_ajar = DIGITAL_IN(SAFETY_DOOR_PORT, SAFETY_DOOR_PIN);
#endif
#ifdef MOTOR_FAULT_PIN
    signals.motor_fault = DIGITAL_IN(MOTOR_FAULT_PORT, MOTOR_FAULT_PIN);
#endif
#ifdef MOTOR_WARNING_PIN
    signals.motor_warning = DIGITAL_IN(MOTOR_WARNING_PORT, MOTOR_WARNING_PIN);
#endif

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

    return aux_ctrl_scan_status(signals);
}

#if DRIVER_PROBES

static probe_state_t probe_state = { .connected = On };
static probe_t probes[DRIVER_PROBES], *probe = &probes[0];

// Toggle probe connected status. Used when no input pin is available.
static void probeConnectedToggle (void)
{
    probe->flags.connected = !probe_state.connected;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    bool invert;

    switch((probe_id_t)probe->probe_id) {
#if TOOLSETTER_ENABLE
        case Probe_Toolsetter:
            invert = settings.probe.invert_toolsetter_input;
            break;
#endif
#if PROBE2_ENABLE
        case Probe_2:
            invert = settings.probe.invert_probe2_input;
            break;
#endif
        default: // Probe_Default
            invert = settings.probe.invert_probe_pin;
            break;
    }

    probe_state.inverted = is_probe_away ? !invert : invert;

    if(probe->flags.latchable) {
        probe_state.is_probing = Off;
        probe_state.triggered = hal.probe.get_state().triggered;
        pin_irq_mode_t irq_mode = probing && !probe_state.triggered ? (probe_state.inverted ? IRQ_Mode_Falling : IRQ_Mode_Rising) : IRQ_Mode_None;
        probe_state.irq_enabled = ioport_enable_irq(probe->port, irq_mode, aux_irq_handler) && irq_mode != IRQ_Mode_None;
    }

    if(!probe_state.irq_enabled)
        probe_state.triggered = Off;

    probe_state.is_probing = probing;
}


// Returns the probe connected and triggered pin states.
static probe_state_t probeGetState (void)
{
    probe_state_t state = {};

    state.probe_id  = probe->probe_id;
    state.connected = probe->flags.connected;

    if(probe_state.is_probing && probe_state.irq_enabled)
        state.triggered = probe_state.triggered;
    else
        state.triggered = DIGITAL_IN(((input_signal_t *)probe->input)->port, ((input_signal_t *)probe->input)->pin) ^ probe_state.inverted;

    return state;
}

static bool probeSelect (probe_id_t probe_id)
{
    bool ok = false;
    uint_fast8_t i = sizeof(probes) / sizeof(probe_t);

    if(!probe_state.is_probing) do {
        i--;
        if((ok = probes[i].probe_id == probe_id && probes[i].input)) {
            probe = &probes[i];
            hal.probe.configure(false, false);
            break;
        }
    } while(i);

    return ok;
}

static bool probe_add (probe_id_t probe_id, uint8_t port, pin_irq_mode_t irq_mode, void *input)
{
    static uint_fast8_t i = 0;

    if(i >= sizeof(probes) / sizeof(probe_t))
        return false;

    bool can_latch;

    if(!(can_latch = (irq_mode & IRQ_Mode_RisingFalling) == IRQ_Mode_RisingFalling))
        hal.signals_cap.probe_triggered = Off;
    else if(i == 0)
        hal.signals_cap.probe_triggered = On;

    probes[i].probe_id = probe_id;
    probes[i].port = port;
    probes[i].flags.connected = probe_state.connected;
    probes[i].flags.latchable = can_latch;
    probes[i].flags.watchable = !!(irq_mode & IRQ_Mode_Change);
    probes[i++].input = input;

    hal.driver_cap.probe_pull_up = On;
    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;
    hal.probe.connected_toggle = probeConnectedToggle;

    if(i == 1)
        hal.probe.select = probeSelect;

    return true;
}

#endif // DRIVER_PROBES

#if MPG_ENABLE == 1

static void mpg_select (void *data)
{
    stream_mpg_enable(DIGITAL_IN(MPG_MODE_PORT, MPG_MODE_PIN) == 0);

    hal.port.register_interrupt_handler(mpg_port, mpg_pin->mode.irq_mode = sys.mpg_mode ? IRQ_Mode_Rising : IRQ_Mode_Falling, aux_irq_handler);

//    PIO_EnableInterrupt(mpg_pin, (mpg_pin->mode.irq_mode = sys.mpg_mode ? IRQ_Mode_Rising : IRQ_Mode_Falling));
}

static void mpg_enable (void *data)
{
    if(sys.mpg_mode == DIGITAL_IN(MPG_MODE_PORT, MPG_MODE_PIN))
        stream_mpg_enable(true);

    hal.port.register_interrupt_handler(mpg_port, mpg_pin->mode.irq_mode = sys.mpg_mode ? IRQ_Mode_Rising : IRQ_Mode_Falling, aux_irq_handler);

//    PIO_EnableInterrupt(mpg_pin, (mpg_pin->mode.irq_mode = sys.mpg_mode ? IRQ_Mode_Rising : IRQ_Mode_Falling));
}

#endif // MPG_ENABLE == 1

static void aux_irq_handler (uint8_t port, bool state)
{
    aux_ctrl_t *pin;
    control_signals_t signals = {0};

    if((pin = aux_ctrl_get_pin(port))) {
        switch(pin->function) {
#if DRIVER_PROBES
  #if PROBE_ENABLE
            case Input_Probe:
  #endif
  #if PROBE2_ENABLE
            case Input_Probe2:
  #endif
  #if TOOLSETTER_ENABLE
            case Input_Toolsetter:
  #endif
                if(probe_state.is_probing) {
                    probe_state.triggered = On;
                    return;
                } else
                    signals.probe_triggered = On;
                break;
#endif
#ifdef I2C_STROBE_PIN
            case Input_I2CStrobe:
                if(i2c_strobe.callback)
                    i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, (1 << I2C_STROBE_PIN)) == 0);
                break;
#endif
#ifdef MPG_MODE_PIN
            case Input_MPGSelect:
                task_add_immediate(mpg_select, NULL);
                break;
#endif
            default:
                break;
        }
        signals.mask |= pin->cap.mask;
        if(!signals.probe_triggered && pin->irq_mode == IRQ_Mode_Change)
            signals.deasserted = hal.port.wait_on_input(Port_Digital, pin->aux_port, WaitMode_Immediate, 0.0f) == 0;
    }

    if(signals.mask) {
        if(!signals.deasserted)
            signals.mask |= systemGetState().mask;
        hal.control.interrupt_callback(signals);
    }
}

static bool aux_claim_explicit (aux_ctrl_t *aux_ctrl)
{
    xbar_t *pin;

    if(aux_ctrl->input == NULL) {

        uint_fast8_t i = sizeof(inputpin) / sizeof(input_signal_t);

        do {
            --i;
            if(inputpin[i].group == PinGroup_AuxInput && inputpin[i].user_port == aux_ctrl->aux_port)
                aux_ctrl->input = &inputpin[i];
        } while(i && aux_ctrl->input == NULL);
    }

    if(aux_ctrl->input && (pin = ioport_claim(Port_Digital, Port_Input, &aux_ctrl->aux_port, NULL))) {

        ioport_set_function(pin, aux_ctrl->function, &aux_ctrl->cap);

        switch(aux_ctrl->function) {
#if PROBE_ENABLE
            case Input_Probe:
                hal.driver_cap.probe = probe_add(Probe_Default, aux_ctrl->aux_port, pin->cap.irq_mode, aux_ctrl->input);
                break;
#endif
#if PROBE2_ENABLE
            case Input_Probe2:
                hal.driver_cap.probe2 = probe_add(Probe_2, aux_ctrl->aux_port, pin->cap.irq_mode, aux_ctrl->input);
                break;

#endif
#if TOOLSETTER_ENABLE
            case Input_Toolsetter:
                hal.driver_cap.toolsetter = probe_add(Probe_Toolsetter, aux_ctrl->aux_port, pin->cap.irq_mode, aux_ctrl->input);
                break;
#endif
#if defined(RESET_PIN) && !ESTOP_ENABLE
            case Input_Reset:
                ((input_signal_t *)aux_ctrl->input)->mode.debounce = hal.driver_cap.software_debounce;
                break;
#endif
#if SAFETY_DOOR_ENABLE
            case Input_SafetyDoor:
                door_pin = (input_signal_t *)aux_ctrl->input;
                door_pin->mode.debounce = hal.driver_cap.software_debounce;
                break;
#endif
#ifdef MPG_MODE_PIN
            case Input_MPGSelect:
                mpg_port = aux_ctrl->aux_port;
                mpg_pin = (input_signal_t *)aux_ctrl->input;
                break;
        }
#endif
            default: break;
        }
    } else
        aux_ctrl->aux_port = 0xFF;

    return aux_ctrl->aux_port != 0xFF;
}

bool aux_out_claim_explicit (aux_ctrl_out_t *aux_ctrl)
{
    xbar_t *pin;

    if((pin = ioport_claim(Port_Digital, Port_Output, &aux_ctrl->aux_port, NULL)))
        ioport_set_function(pin, aux_ctrl->function, NULL);
    else
        aux_ctrl->aux_port = 0xFF;

    return aux_ctrl->aux_port != 0xFF;
}

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (spindle_ptrs_t *spindle)
{
#ifdef SPINDLE_PWM_PIN
    spindle->context.pwm->flags.enable_out = false;
  #ifdef SPINDLE_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned) {
        BITBAND_PERI(SPINDLE_DIRECTION_PORT->PIO_ODSR, SPINDLE_DIRECTION_PIN) = settings.pwm_spindle.invert.ccw;
    } else {
        BITBAND_PERI(SPINDLE_ENABLE_PORT->PIO_ODSR, SPINDLE_ENABLE_PIN) = settings.pwm_spindle.invert.on;
    }
  #else
    BITBAND_PERI(SPINDLE_ENABLE_PORT->PIO_ODSR, SPINDLE_ENABLE_PIN) = settings.pwm_spindle.invert.on;
  #endif
#else
    BITBAND_PERI(SPINDLE_ENABLE_PORT->PIO_ODSR, SPINDLE_ENABLE_PIN) = settings.pwm_spindle.invert.on;
#endif
}

inline static void spindle_on (spindle_ptrs_t *spindle)
{
#ifdef SPINDLE_PWM_PIN
    spindle->context.pwm->flags.enable_out = true;
  #ifdef SPINDLE_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned) {
        BITBAND_PERI(SPINDLE_DIRECTION_PORT->PIO_ODSR, SPINDLE_DIRECTION_PIN) = !settings.pwm_spindle.invert.ccw;
    } else {
        BITBAND_PERI(SPINDLE_ENABLE_PORT->PIO_ODSR, SPINDLE_ENABLE_PIN) = !settings.pwm_spindle.invert.on;
    }
  #else
    BITBAND_PERI(SPINDLE_ENABLE_PORT->PIO_ODSR, SPINDLE_ENABLE_PIN) = !settings.pwm_spindle.invert.on;
  #endif
#else
    BITBAND_PERI(SPINDLE_ENABLE_PORT->PIO_ODSR, SPINDLE_ENABLE_PIN) = !settings.pwm_spindle.invert.on;
#endif
}

inline static void spindle_dir (bool ccw)
{
#ifdef SPINDLE_DIRECTION_PIN
    BITBAND_PERI(SPINDLE_DIRECTION_PORT->PIO_ODSR, SPINDLE_DIRECTION_PIN) = (ccw ^ settings.pwm_spindle.invert.ccw);
#else
    UNUSED(ccw);
#endif
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(rpm);
    UNUSED(spindle);

    if(!state.on)
        spindle_off(spindle);
    else {
        spindle_dir(state.ccw);
        spindle_on(spindle);
    }
}

// Variable spindle control functions

#ifdef SPINDLE_PWM_PIN

#ifdef SPINDLE_PWM_CHANNEL

static inline void pwm_off (spindle_ptrs_t *spindle)
{
    if(spindle->context.pwm->flags.always_on) {
        if(PWM->PWM_SR & (1 << SPINDLE_PWM_CHANNEL))
            PWM->PWM_CH_NUM[SPINDLE_PWM_CHANNEL].PWM_CDTYUPD = spindle->context.pwm->off_value;
        else {
            PWM->PWM_CH_NUM[SPINDLE_PWM_CHANNEL].PWM_CDTY = spindle->context.pwm->off_value;
            REG_PWM_ENA = (1 << SPINDLE_PWM_CHANNEL);
        }
    } else
        REG_PWM_DIS = (1 << SPINDLE_PWM_CHANNEL);
}

#else

static inline void pwm_off (spindle_ptrs_t *spindle)
{
    if(spindle->context.pwm->flags.always_on) {
        SPINDLE_PWM_TIMER.TC_RA = spindle->context.pwm->period - spindle->context.pwm->off_value;
        SPINDLE_PWM_TIMER.TC_CMR &= ~TC_CMR_CPCSTOP;
        SPINDLE_PWM_TIMER.TC_CCR = TC_CCR_CLKEN|TC_CCR_SWTRG;
    } else
        SPINDLE_PWM_TIMER.TC_CMR |= TC_CMR_CPCSTOP; // Ensure output is low, by setting timer to stop at TCC match
}

#endif

// Sets spindle speed
static void spindleSetSpeed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if(pwm_value == spindle->context.pwm->off_value) {

        if(spindle->context.pwm->flags.rpm_controlled) {
            spindle_off(spindle);
#ifdef SPINDLE_PWM_CHANNEL
            if(spindle->context.pwm->flags.laser_off_overdrive) {
                if(PWM->PWM_SR & (1 << SPINDLE_PWM_CHANNEL))
                    PWM->PWM_CH_NUM[SPINDLE_PWM_CHANNEL].PWM_CDTYUPD = spindle->context.pwm->pwm_overdrive;
                else {
                    PWM->PWM_CH_NUM[SPINDLE_PWM_CHANNEL].PWM_CDTY = spindle->context.pwm->pwm_overdrive;
                    REG_PWM_ENA = (1 << SPINDLE_PWM_CHANNEL);
                }
            }
#else
            if(spindle->context.pwm->flags.laser_off_overdrive)
                SPINDLE_PWM_TIMER.TC_RA = spindle->context.pwm->period == spindle->context.pwm->pwm_overdrive ? 1 : spindle->context.pwm->period - spindle->context.pwm->pwm_overdrive;
#endif
        } else
            pwm_off(spindle);

    } else {

  #ifdef SPINDLE_PWM_CHANNEL
        if(!spindle->context.pwm->flags.enable_out && spindle->context.pwm->flags.rpm_controlled)
            spindle_on(spindle);

        if(PWM->PWM_SR & (1 << SPINDLE_PWM_CHANNEL))
            PWM->PWM_CH_NUM[SPINDLE_PWM_CHANNEL].PWM_CDTYUPD = pwm_value;
        else {
            PWM->PWM_CH_NUM[SPINDLE_PWM_CHANNEL].PWM_CDTY = pwm_value;
            REG_PWM_ENA = (1 << SPINDLE_PWM_CHANNEL);
        }
  #else
        SPINDLE_PWM_TIMER.TC_RA = spindle->context.pwm->period == pwm_value ? 1 : spindle->context.pwm->period - pwm_value;

        if(!spindle->context.pwm->flags.enable_out && spindle->context.pwm->flags.rpm_controlled)
            spindle_on(spindle);

        SPINDLE_PWM_TIMER.TC_CMR &= ~TC_CMR_CPCSTOP;
        SPINDLE_PWM_TIMER.TC_CCR = TC_CCR_CLKEN|TC_CCR_SWTRG;
  #endif
    }
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false);
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!(spindle->context.pwm->flags.cloned ? state.ccw : state.on)) {
        spindle_off(spindle);
        pwm_off(spindle);
    } else {
#ifdef SPINDLE_DIRECTION_PIN
        if(!spindle->context.pwm->flags.cloned)
            spindle_dir(state.ccw);
#endif
        if(rpm == 0.0f && spindle->context.pwm->flags.rpm_controlled)
            spindle_off(spindle);
        else {
            spindle_on(spindle);
            spindleSetSpeed(spindle, spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false));
        }
    }
}

bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    if(spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle, hal.f_step_timer)) {
#ifdef SPINDLE_PWM_CHANNEL
        PWM->PWM_CH_NUM[SPINDLE_PWM_CHANNEL].PWM_CPRD = spindle_pwm.period;
#else
        SPINDLE_PWM_TIMER.TC_RC = spindle_pwm.period;
#endif
        spindle->set_state = spindleSetStateVariable;
    } else {
        if(spindle->context.pwm->flags.enable_out)
            spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
        spindle->set_state = spindleSetState;
    }

    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

    return true;
}

#endif // SPINDLE_PWM_PIN

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    UNUSED(spindle);

    spindle_state_t state = { settings.pwm_spindle.invert.mask };

    state.on = BITBAND_PERI(SPINDLE_ENABLE_PORT->PIO_ODSR, SPINDLE_ENABLE_PIN) != 0;
#ifdef SPINDLE_DIRECTION_PIN
    state.ccw = BITBAND_PERI(SPINDLE_DIRECTION_PORT->PIO_ODSR, SPINDLE_DIRECTION_PIN) != 0;
#endif

    state.value ^= settings.pwm_spindle.invert.mask;

#ifdef SPINDLE_PWM_PIN
    state.on = state.on || spindle->param->rpm != 0.0f;
#endif

    return state;
}

#endif // DRIVER_SPINDLE_ENABLE

#ifdef DEBUGOUT
void debug_out (bool on)
{
#ifdef COOLANT_MIST_PIN
    BITBAND_PERI(COOLANT_MIST_PORT->PIO_ODSR, COOLANT_MIST_PIN) = on;
#endif
}
#endif

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant.invert.mask;

  #ifdef COOLANT_FLOOD_PIN
    BITBAND_PERI(COOLANT_FLOOD_PORT->PIO_ODSR, COOLANT_FLOOD_PIN) = mode.flood;
  #endif
  #ifdef COOLANT_MIST_PIN
    BITBAND_PERI(COOLANT_MIST_PORT->PIO_ODSR, COOLANT_MIST_PIN) = mode.mist;
  #endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

  #ifdef COOLANT_FLOOD_PIN
    state.flood = BITBAND_PERI(COOLANT_FLOOD_PORT->PIO_ODSR, COOLANT_FLOOD_PIN);
  #endif
  #ifdef COOLANT_MIST_PIN
    state.mist  = BITBAND_PERI(COOLANT_MIST_PORT->PIO_ODSR, COOLANT_MIST_PIN);
  #endif

    state.value ^= settings.coolant.invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    *ptr |= bits;
    __enable_irq();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    __enable_irq();

    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    __enable_irq();

    return prev;
}

static void PIO_Mode (Pio *port, uint32_t bit, bool mode)
{
    port->PIO_WPMR = PIO_WPMR_WPKEY(0x50494F);

    port->PIO_ABSR &= ~bit;

    if(mode == OUTPUT)
        port->PIO_OER = bit;
    else
        port->PIO_ODR = bit;

    port->PIO_PER = bit;
    port->PIO_WPMR = PIO_WPMR_WPKEY(0x50494F)|PIO_WPMR_WPEN;
}

void PIO_EnableInterrupt (const input_signal_t *input, pin_irq_mode_t irq_mode)
{
    input->port->PIO_IDR = input->bit;

    switch(irq_mode) {

        case IRQ_Mode_Falling:
            input->port->PIO_AIMER = input->bit;
            input->port->PIO_ESR = input->bit;
            input->port->PIO_FELLSR = input->bit;
            break;

        case IRQ_Mode_Rising:
            input->port->PIO_AIMER = input->bit;
            input->port->PIO_ESR = input->bit;
            input->port->PIO_REHLSR = input->bit;
            break;

        case IRQ_Mode_Change:
            input->port->PIO_AIMDR = input->bit;
            input->port->PIO_ESR = input->bit;
            break;

        default:
            break;
    }

    input->port->PIO_ISR; // Clear interrupts

    if(!(irq_mode == IRQ_Mode_None || input->group == PinGroup_Limit || input->group == PinGroup_LimitMax))
        input->port->PIO_IER = input->bit;
    else
        input->port->PIO_IDR = input->bit;
} 

void PIO_InputMode (Pio *port, uint32_t bit, bool no_pullup)
{
    port->PIO_WPMR = PIO_WPMR_WPKEY(0x50494F);

    port->PIO_PER = bit;
    port->PIO_ODR = bit;

    if(no_pullup)
        port->PIO_PUDR = bit;
    else
        port->PIO_PUER = bit;

    port->PIO_WPMR = PIO_WPMR_WPKEY(0x50494F)|PIO_WPMR_WPEN;
}

// Configures perhipherals when settings are initialized or changed
void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    if(IOInitDone) {

#ifdef SQUARING_ENABLED
        hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
#endif

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
        if(changed.spindle) {
            spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
            if(spindle_id == spindle_get_default())
                spindle_select(spindle_id);
        }
#endif

        float ts = hal.f_step_timer / 1000000.0f;
        step_pulse.t_min_period = (uint32_t)((hal.step_us_min + STEP_PULSE_TOFF_MIN) * ts);
        step_pulse.length = (uint32_t)(ts * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY)) - 1;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            int32_t delay = (uint32_t)(ts * (settings->steppers.pulse_delay_microseconds - 0.6f));
            step_pulse.delay = delay < 2 ? 2 : delay;
            hal.stepper.pulse_start = stepperPulseStartDelayed;
        } else
            hal.stepper.pulse_start = stepperPulseStart;

        IRQRegister(STEP_TIMER_IRQn, STEP_IRQHandler);
        STEP_TIMER.TC_RC = step_pulse.length;
        STEP_TIMER.TC_IER = TC_IER_CPCS; // Enable step end interrupt

#if STEP_INJECT_ENABLE
        STEP2_TIMER.TC_RC = step_pulse.length;
        STEP2_TIMER.TC_IER = TC_IER_CPCS; // Enable step end interrupt
#endif

        /****************************************
         *  Control, limit & probe pins config  *
         ****************************************/

        input_signal_t *input;

        control_signals_t control_fei;
        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        NVIC_DisableIRQ(PIOA_IRQn);
        NVIC_DisableIRQ(PIOB_IRQn);
        NVIC_DisableIRQ(PIOC_IRQn);
        NVIC_DisableIRQ(PIOD_IRQn);

        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t), a = 0, b = 0, c = 0, d = 0;

        do {

            input = &inputpin[--i];

            if(input->group == PinGroup_AuxInputAnalog)
                continue;

            if(!(input->group == PinGroup_AuxInput || input->group == PinGroup_MPG))
                input->mode.irq_mode = IRQ_Mode_None;

            if(input->port != NULL) {

                switch(input->id) {

                    case Input_Reset:
                        input->mode.pull_mode = settings->control_disable_pullup.reset ? PullMode_Down : PullMode_Up;
                        input->mode.irq_mode = control_fei.reset ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_FeedHold:
                        input->mode.pull_mode = settings->control_disable_pullup.feed_hold ? PullMode_Down : PullMode_Up;
                        input->mode.irq_mode = control_fei.feed_hold ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_CycleStart:
                        input->mode.pull_mode = settings->control_disable_pullup.cycle_start ? PullMode_Down : PullMode_Up;
                        input->mode.irq_mode = control_fei.cycle_start ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_LimitX:
                    case Input_LimitX_2:
                    case Input_LimitX_Max:
                        input->mode.pull_mode = settings->limits.disable_pullup.x ? PullMode_Down : PullMode_Up;
                        input->mode.irq_mode = limit_fei.x ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_LimitY:
                    case Input_LimitY_2:
                    case Input_LimitY_Max:
                        input->mode.pull_mode = settings->limits.disable_pullup.y ? PullMode_Down : PullMode_Up;
                        input->mode.irq_mode = limit_fei.y ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_LimitZ:
                    case Input_LimitZ_2:
                    case Input_LimitZ_Max:
                        input->mode.pull_mode = settings->limits.disable_pullup.z ? PullMode_Down : PullMode_Up;
                        input->mode.irq_mode = limit_fei.z ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_LimitA:
                    case Input_LimitA_Max:
                        input->mode.pull_mode = settings->limits.disable_pullup.a ? PullMode_Down : PullMode_Up;
                        input->mode.irq_mode = limit_fei.a ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_LimitB:
                    case Input_LimitB_Max:
                        input->mode.pull_mode = settings->limits.disable_pullup.b ? PullMode_Down : PullMode_Up;
                        input->mode.irq_mode = limit_fei.b ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    case Input_LimitC:
                    case Input_LimitC_Max:
                        input->mode.pull_mode = settings->limits.disable_pullup.c ? PullMode_Down : PullMode_Up;
                        input->mode.irq_mode = limit_fei.c ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                        break;

                    default:
                        break;
                }

                PIO_Mode(input->port, input->bit, INPUT);
                PIO_InputMode(input->port, input->bit, input->mode.pull_mode != PullMode_Up);
                PIO_EnableInterrupt(input, input->mode.irq_mode);

                if(input->port == PIOA)
                    a_signals[a++] = input;
                else if(input->port == PIOB)
                    b_signals[b++] = input;
                else if(input->port == PIOC)
                    c_signals[c++] = input;
                else if(input->port == PIOD)
                    d_signals[d++] = input;
            }
        } while(i);

        // Clear and enable PIO interrupts
        if(a) {
            PIOA->PIO_ISR;
            IRQRegister(PIOA_IRQn, PIOA_IRQHandler);
            NVIC_ClearPendingIRQ(PIOA_IRQn);
            NVIC_EnableIRQ(PIOA_IRQn);
        }

        if(b) {
            PIOB->PIO_ISR;
            IRQRegister(PIOB_IRQn, PIOB_IRQHandler);
            NVIC_ClearPendingIRQ(PIOB_IRQn);
            NVIC_EnableIRQ(PIOB_IRQn);
        }

        if(c) {
            PIOC->PIO_ISR;
            IRQRegister(PIOC_IRQn, PIOC_IRQHandler);
            NVIC_ClearPendingIRQ(PIOC_IRQn);
            NVIC_EnableIRQ(PIOC_IRQn);
        }

        if(d) {
            PIOD->PIO_ISR;
            IRQRegister(PIOD_IRQn, PIOD_IRQHandler);
            NVIC_ClearPendingIRQ(PIOD_IRQn);
            NVIC_EnableIRQ(PIOD_IRQn);
        }

        if(settings->limits.flags.hard_enabled)
            hal.limits.enable(true, (axes_signals_t){0});

        aux_ctrl_irq_enable(settings, aux_irq_handler);
    }
}

static char *port2char(Pio *port)
{
    static char s[2] = "?.";

    s[0] = 'A' + ((uint32_t)port - (uint32_t)PIOA) / ((uint32_t)PIOB - (uint32_t)PIOA);

    return s;
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {0};
    uint32_t i, id = 0;

    pin.mode.input = On;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        pin.id = id++;
        pin.pin = inputpin[i].pin;
        pin.function = inputpin[i].id;
        pin.group = inputpin[i].group;
        pin.port = low_level ? (void *)inputpin[i].port : (void *)port2char(inputpin[i].port);
        pin.description = inputpin[i].description;
        pin.mode.pwm = pin.group == PinGroup_SpindlePWM;

        pin_info(&pin, data);
    };

    pin.mode.mask = 0;
    pin.mode.output = On;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        pin.id = id++;
        pin.pin = outputpin[i].pin;
        pin.function = outputpin[i].id;
        pin.group = outputpin[i].group;
        pin.port = low_level ? (void *)outputpin[i].port : (void *)port2char(outputpin[i].port);
        pin.description = outputpin[i].description;

        pin_info(&pin, data);
    };

    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        pin.id = id++;
        pin.pin = ppin->pin.pin;
        pin.function = ppin->pin.function;
        pin.group = ppin->pin.group;
        pin.port = low_level ? ppin->pin.port : (void *)port2char(ppin->pin.port);
        pin.mode = ppin->pin.mode;
        pin.description = ppin->pin.description;

        pin_info(&pin, data);

        ppin = ppin->next;
    } while(ppin);
}

void registerPeriphPin (const periph_pin_t *pin)
{
    periph_signal_t *add_pin = malloc(sizeof(periph_signal_t));

    if(!add_pin)
        return;

    memcpy(&add_pin->pin, pin, sizeof(periph_pin_t));
    add_pin->next = NULL;

    if(periph_pins == NULL) {
        periph_pins = add_pin;
    } else {
        periph_signal_t *last = periph_pins;
        while(last->next)
            last = last->next;
        last->next = add_pin;
    }
}

void setPeriphPinDescription (const pin_function_t function, const pin_group_t group, const char *description)
{
    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        if(ppin->pin.function == function && ppin->pin.group == group) {
            ppin->pin.description = description;
            ppin = NULL;
        } else
            ppin = ppin->next;
    } while(ppin);
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
    pmc_enable_periph_clk(ID_PIOA);
    pmc_enable_periph_clk(ID_PIOB);
    pmc_enable_periph_clk(ID_PIOC);
    pmc_enable_periph_clk(ID_PIOD);
    pmc_enable_periph_clk(ID_TC0);
    pmc_enable_periph_clk(ID_TC1);
    pmc_enable_periph_clk(ID_TC2);
    pmc_enable_periph_clk(ID_TC3);
    pmc_enable_periph_clk(ID_TC6);
    pmc_enable_periph_clk(ID_TC7);
    pmc_enable_periph_clk(ID_TC8);

    /*************************
     *  Output signals init  *
     *************************/

    uint32_t i;
    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        outputpin[i].bit = 1U << outputpin[i].pin;
        PIO_Mode(outputpin[i].port, outputpin[i].bit, OUTPUT);
    }

 // Stepper init

    TC0->TC_WPMR = TC_WPMR_WPKEY(0x54494D); //|TC_WPMR_WPEN;
    TC2->TC_WPMR = TC_WPMR_WPKEY(0x54494D); //|TC_WPMR_WPEN;

    STEPPER_TIMER.TC_CCR = TC_CCR_CLKDIS;
    STEPPER_TIMER.TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC;
    STEPPER_TIMER.TC_IER = TC_IER_CPCS;
    STEPPER_TIMER.TC_CCR = TC_CCR_CLKEN;

    IRQRegister(STEPPER_TIMER_IRQn, STEPPER_IRQHandler);
    NVIC_EnableIRQ(STEPPER_TIMER_IRQn); // Enable stepper interrupt
    NVIC_SetPriority(STEPPER_TIMER_IRQn, 2);

    STEP_TIMER.TC_CCR = TC_CCR_CLKDIS;
    STEP_TIMER.TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP|TC_CMR_CPCSTOP;
    STEP_TIMER.TC_IER = TC_IER_CPCS;
    STEP_TIMER.TC_CCR = TC_CCR_CLKEN;

    IRQRegister(STEP_TIMER_IRQn, STEP_IRQHandler);
    NVIC_EnableIRQ(STEP_TIMER_IRQn);    // Enable stepper interrupts
    NVIC_SetPriority(STEP_TIMER_IRQn, 0);

#if STEP_INJECT_ENABLE

    TC1->TC_WPMR = TC_WPMR_WPKEY(0x54494D); //|TC_WPMR_WPEN;

    STEP2_TIMER.TC_CCR = TC_CCR_CLKDIS;
    STEP2_TIMER.TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP|TC_CMR_CPCSTOP;
    STEP2_TIMER.TC_IER = TC_IER_CPCS;
    STEP2_TIMER.TC_CCR = TC_CCR_CLKEN;

    IRQRegister(STEP2_TIMER_IRQ, output_pulse_isr);
    NVIC_EnableIRQ(STEP2_TIMER_IRQ);    // Enable stepper interrupts
    NVIC_SetPriority(STEP2_TIMER_IRQ, 0);

#endif

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

 // Spindle init

    SPINDLE_PWM_PORT->PIO_WPMR = PIO_WPMR_WPKEY(0x50494F);
    SPINDLE_PWM_PORT->PIO_ABSR |= (1 << SPINDLE_PWM_PIN);
    SPINDLE_PWM_PORT->PIO_PDR = (1 << SPINDLE_PWM_PIN);

#ifdef SPINDLE_PWM_CHANNEL
    REG_PMC_PCER1 |= PMC_PCER1_PID36;
    REG_PWM_CLK = PWM_CLK_PREA(1) | PWM_CLK_DIVA(1);
    PWM->PWM_CH_NUM[SPINDLE_PWM_CHANNEL].PWM_CMR = PWM_CMR_CPRE_CLKA;
#else
    SPINDLE_PWM_TIMER.TC_CCR = TC_CCR_CLKDIS;
    SPINDLE_PWM_TIMER.TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC|TC_CMR_ASWTRG_CLEAR|TC_CMR_ACPA_SET|TC_CMR_ACPC_CLEAR; //|TC_CMR_EEVT_XC0;
#endif

#endif // DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

#if TRINAMIC_ENABLE == 2130
    trinamic_start(true);
#endif

 // Set defaults

    IOInitDone = settings->version.id == 23;

    hal.settings_changed(settings, (settings_changed_flags_t){0});

    hal.stepper.go_idle(true);

#if SDCARD_ENABLE
    pinMode(SD_CD_PIN, INPUT_PULLUP);

// This does not work, the card detect pin is not interrupt capable(!) and inserting a card causes a hard reset...
// The bootloader needs modifying for it to work? Or perhaps the schematic is plain wrong?
// attachInterrupt(SD_CD_PIN, SD_IRQHandler, CHANGE);

    if(BITBAND_PERI(SD_CD_PIN) == 0)
        power_on();

    sdcard_init();
#endif

    return IOInitDone;
}

// EEPROM emulation - stores settings in flash
// Note: settings will not survive a reflash unless protected

typedef struct {
    void *addr;
    uint16_t page;
    uint16_t pages;
} nvs_storage_t;

static nvs_storage_t grblNVS;

bool nvsRead (uint8_t *dest)
{
    if(grblNVS.addr != NULL)
        memcpy(dest, grblNVS.addr, hal.nvs.size);

    return grblNVS.addr != NULL;
}

bool nvsWrite (uint8_t *source)
{
    uint16_t page = grblNVS.page;
    uint32_t size = grblNVS.pages;
    uint32_t *dest = (uint32_t *)grblNVS.addr, *src = (uint32_t *)source, words;

    while(!(EFC1->EEFC_FSR & EEFC_FSR_FRDY));

    while(size) {

        // Fill page buffer
        words = IFLASH1_PAGE_SIZE / sizeof(uint32_t);
        do {
            *dest++ = *src++;
        } while(--words);

        // Write page buffer to flash
        EFC1->EEFC_FCR = EEFC_FCR_FKEY(0x5A)|EEFC_FCR_FARG(page)|EEFC_FCR_FCMD(0x03);
        while(!(EFC1->EEFC_FSR & EEFC_FSR_FRDY));
        
        if(EFC1->EEFC_FSR & EEFC_FSR_FLOCKE)
            break;

        size--;
        page++;
    }

    return size == 0;
}

bool nvsInit (void)
{
    if(NVS_SIZE & ~(IFLASH1_PAGE_SIZE - 1))
        grblNVS.pages = (NVS_SIZE & ~(IFLASH1_PAGE_SIZE - 1)) / IFLASH1_PAGE_SIZE + 1;
    else
        grblNVS.pages = NVS_SIZE / IFLASH1_PAGE_SIZE;

//  grblNVS.row_size = IFLASH1_LOCK_REGION_SIZE; // 16K
    grblNVS.page = IFLASH1_NB_OF_PAGES - grblNVS.pages;
    grblNVS.addr = (void *)(IFLASH1_ADDR + IFLASH1_PAGE_SIZE * grblNVS.page);

    return true;
}

// End EEPROM emulation

inline static uint64_t get_micros (void)
{
    return (uint64_t)micros();
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
    uint32_t i = 0;

    WDT_Disable (WDT);

    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    // Copy vector table to RAM so we can override the default Arduino IRQ assignments

    __disable_irq();

    memcpy(&vectorTable, (void *)SCB->VTOR, sizeof(vectorTable));

    SCB->VTOR = ((uint32_t)&vectorTable & SCB_VTOR_TBLOFF_Msk) | SCB_VTOR_TBLBASE_Msk;
    __DSB();
    __enable_irq();

    // End vector table copy

    SysTick_Config((SystemCoreClock / 1000));
    IRQRegister(SysTick_IRQn, SysTick_IRQHandler);

    /*
    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk;

//    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
//    NVIC_EnableIRQ(SysTick_IRQn);
*/

#ifdef MPG_MODE_PIN
    // Pull down MPG mode pin until startup is completed.
    while(mpg_pin == NULL) {
        if(inputpin[i].id == Input_ModeSelect) {
            mpg_pin = &inputpin[i];
            mpg_pin->bit = 1U << mpg_pin->pin;
            PIO_Mode(mpg_pin->port, mpg_pin->bit, OUTPUT);
            BITBAND_PERI(MPG_MODE_PORT->PIO_ODSR, MPG_MODE_PIN) = 0;
        }
        i++;
    }
#endif

    hal.info = "SAM3X8E";
    hal.driver_version = "250706";
    hal.driver_url = GRBL_URL "/SAM3X8E";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
#ifdef BOARD_URL
    hal.board_url = BOARD_URL;
#endif

    hal.driver_setup = driver_setup;
    hal.f_step_timer = SystemCoreClock / 2; // 42 MHz
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
#ifdef GANGING_ENABLED
    hal.stepper.get_ganged = getGangedAxes;
#endif
#ifdef SQUARING_ENABLED
    hal.stepper.disable_motors = StepperDisableMotors;
#endif
#if STEP_INJECT_ENABLE
    hal.stepper.output_step = stepperOutputStep;
    hal.stepper.claim_motor = stepperClaimMotor;
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.control.get_state = systemGetState;

#ifdef DEBUGOUT
    hal.debug_out = debug_out;
#endif

    hal.irq_enable = __enable_irq;
    hal.irq_disable = __disable_irq;
#if I2C_STROBE_ENABLE
    hal.irq_claim = irq_claim;
#endif
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_micros = get_micros;
    hal.get_elapsed_ticks = millis;
    hal.enumerate_pins = enumeratePins;
    hal.periph_port.register_pin = registerPeriphPin;
    hal.periph_port.set_pin_description = setPeriphPinDescription;

    serialRegisterStreams();

#if USB_SERIAL_CDC
    stream_connect(usb_serialInit());
#else
    if(!stream_connect_instance(SERIAL_STREAM, BAUD_RATE))
        while(true); // Cannot boot if no communication channel is available!
#endif

#if EEPROM_ENABLE
    i2c_eeprom_init();
#else
    if(nvsInit()) {
        hal.nvs.type = NVS_Flash;
        hal.nvs.memcpy_from_flash = nvsRead;
        hal.nvs.memcpy_to_flash = nvsWrite;
    } else
        hal.nvs.type = NVS_None;
#endif

#if DRIVER_SPINDLE_ENABLE

 #if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
        .ref_id = SPINDLE_PWM0,
#else
        .ref_id = SPINDLE_PWM0_NODIR,
#endif
        .config = spindleConfig,
        .set_state = spindleSetStateVariable,
        .get_state = spindleGetState,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindleSetSpeed,
        .cap = {
            .gpio_controlled = On,
            .variable = On,
            .laser = On,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
            .direction = On
  #endif
        }
    };

 #else

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Basic,
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
        .ref_id = SPINDLE_ONOFF0_DIR,
#else
        .ref_id = SPINDLE_ONOFF0,
#endif
        .set_state = spindleSetState,
        .get_state = spindleGetState,
        .cap = {
            .gpio_controlled = On,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
            .direction = On
  #endif
        }
    };

 #endif

    spindle_id = spindle_register(&spindle, DRIVER_SPINDLE_NAME);

#endif // DRIVER_SPINDLE_ENABLE

 // driver capabilities, used for announcing and negotiating (with the ciorre) driver functionality

    hal.limits_cap = get_limits_cap();
    hal.home_cap = get_home_cap();
    hal.coolant_cap.bits = COOLANT_ENABLE;
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;

    static pin_group_pins_t aux_inputs = {0}, aux_outputs = {0};
#if AUX_ANALOG
    static pin_group_pins_t aux_analog_in = {0}, aux_analog_out = {0};
#endif

    input_signal_t *input;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {

        input = &inputpin[i];
        input->bit = 1 << input->pin;
        input->mode.input = input->cap.input = On;

        if(input->group == PinGroup_AuxInput) {
            if(aux_inputs.pins.inputs == NULL)
                aux_inputs.pins.inputs = input;
            input->user_port = aux_inputs.n_pins++;
            input->id = (pin_function_t)(Input_Aux0 + input->user_port);
            input->cap.debounce = hal.driver_cap.software_debounce;
            input->cap.irq_mode = IRQ_Mode_Edges;
            input->mode.pull_mode = input->cap.pull_mode = PullMode_Up;

            aux_ctrl_t *aux_remap;
            if((aux_remap = aux_ctrl_remap_explicit(input->port, input->pin, input->user_port, input))) {
                if(aux_remap->function == Input_Probe && input->cap.irq_mode == IRQ_Mode_Edges)
                    aux_remap->irq_mode = IRQ_Mode_Change;
            }

        } else if(input->group & (PinGroup_Limit|PinGroup_LimitMax)) {
            if(limit_inputs.pins.inputs == NULL)
                limit_inputs.pins.inputs = input;
            limit_inputs.n_pins++;
        } else if(input->group == PinGroup_Control)
            input->mode.debounce = hal.driver_cap.software_debounce;
#if AUX_ANALOG
        else if(input->group == PinGroup_AuxInputAnalog) {
           if(aux_analog_in.pins.inputs == NULL)
               aux_analog_in.pins.inputs = input;
           input->id = (pin_function_t)(Input_Analog_Aux0 + aux_analog_in.n_pins++);
           input->mode.analog = input->cap.analog = On;
       }
#endif
    }

    output_signal_t *output;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {

        output = &outputpin[i];
        output->mode.output = On;

        if(output->group == PinGroup_AuxOutput) {
            if(aux_outputs.pins.outputs == NULL)
                aux_outputs.pins.outputs = output;
            output->id = (pin_function_t)(Output_Aux0 + aux_outputs.n_pins);
            aux_out_remap_explicit(output->port, output->pin, aux_outputs.n_pins, output);
            aux_outputs.n_pins++;
        }
    }

    ioports_init(&aux_inputs, &aux_outputs);

#if AUX_ANALOG

    if(aux_analog_in.n_pins || aux_analog_out.n_pins)
        ioports_init_analog(&aux_analog_in, &aux_analog_out);
#endif

    io_expanders_init();
    aux_ctrl_claim_ports(aux_claim_explicit, NULL);
    aux_ctrl_claim_out_ports(aux_out_claim_explicit, NULL);

#include "grbl/plugins_init.h"

#if MPG_ENABLE == 1
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, NULL);
    if(hal.driver_cap.mpg_mode)
        task_run_on_startup(mpg_enable, NULL);
#elif MPG_ENABLE == 2
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, stream_mpg_check_enable);
#endif

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 10;
}

/* interrupt handlers */

// Main stepper driver
static void STEPPER_IRQHandler (void)
{
    if(STEPPER_TIMER.TC_SR & STEPPER_TIMER.TC_IMR)
        hal.stepper.interrupt_callback();
}

// Step output off
static void STEP_IRQHandler (void)
{
    if(STEP_TIMER.TC_SR & STEP_TIMER.TC_IMR)
        set_step_outputs((axes_signals_t){0});  
}

// Step output on/off
static void STEPDELAY_IRQHandler (void)
{
    if(STEP_TIMER.TC_SR & STEP_TIMER.TC_IMR) {

        set_step_outputs(step_pulse.out);

        IRQRegister(STEP_TIMER_IRQn, STEP_IRQHandler);

        STEP_TIMER.TC_RC = step_pulse.length;
        STEP_TIMER.TC_CCR = TC_CCR_SWTRG;
    }
}

#if STEP_INJECT_ENABLE

static void output_pulse_isr (void)
{
    if(STEP2_TIMER.TC_SR & STEP2_TIMER.TC_IMR) {

        axes_signals_t axes = { .bits = step_pulse.inject.out.bits };

        step_pulse.inject.out.bits = 0;
        step_pulse.inject.axes.bits = step_pulse.inject.claimed.bits;

        inject_step((axes_signals_t){0}, axes);
    }
}

static void output_pulse_isr_delayed (void)
{
    if(STEP2_TIMER.TC_SR & STEP2_TIMER.TC_IMR) {

        inject_step(step_pulse.inject.out, step_pulse.inject.out);

        IRQRegister(STEP2_TIMER_IRQ, output_pulse_isr);
        STEP2_TIMER.TC_RC = step_pulse.delay;
        STEP2_TIMER.TC_CCR = TC_CCR_SWTRG;
    }
}

#endif // STEP_INJECT_ENABLE

void pin_debounce (void *pin)
{
    input_signal_t *input = (input_signal_t *)pin;

#if SAFETY_DOOR_ENABLE
    if(input->id == Input_SafetyDoor)
        debounce.safety_door = Off;
#endif

    if(input->mode.irq_mode == IRQ_Mode_Change ||
        (input->port->PIO_PDSR & input->bit) == (input->mode.irq_mode == IRQ_Mode_Falling ? 0 : input->bit)) {

        switch(input->group) {

            case PinGroup_Limit:
            case PinGroup_LimitMax:
                {
                    limit_signals_t state = limitsGetState();
                    if(limit_signals_merge(state).value)
                        hal.limits.interrupt_callback(state);
                }
                break;

            case PinGroup_Control:
                hal.control.interrupt_callback(systemGetState());
                break;

            case PinGroup_AuxInput:
                ioports_event(input);
                break;

            default:
                break;
        }
    }

    input->port->PIO_ISR;
    input->port->PIO_IER = input->bit;
}

inline static void PIO_IRQHandler (input_signal_t **signals, uint32_t isr)
{
    uint32_t grp = 0, i = 0;

    while(signals[i] != NULL) {
        if(isr & signals[i]->bit) {
            if(signals[i]->mode.debounce && task_add_delayed(pin_debounce, signals[i], 40)) {
                signals[i]->port->PIO_IDR = signals[i]->bit;
                if(signals[i]->id == Input_SafetyDoor)
                    debounce.safety_door = On;
            } else switch(signals[i]->group) {

                case PinGroup_AuxInput:
                    ioports_event(signals[i]);
                    break;

                default:
                    grp |= signals[i]->group;
                    break;
            }
        }
        i++;
    }

    if(grp & (PinGroup_Limit|PinGroup_LimitMax)) {
        limit_signals_t state = limitsGetState();
        if(limit_signals_merge(state).value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
            hal.limits.interrupt_callback(state);
    }

    if(grp & PinGroup_Control)
        hal.control.interrupt_callback(systemGetState());
}

static void PIOA_IRQHandler (void)
{
    PIO_IRQHandler(a_signals, PIOA->PIO_ISR);
}

static void PIOB_IRQHandler (void)
{
    PIO_IRQHandler(b_signals, PIOB->PIO_ISR);
}

static void PIOC_IRQHandler (void)
{
    PIO_IRQHandler(c_signals, PIOC->PIO_ISR);
}

static void PIOD_IRQHandler (void)
{
    PIO_IRQHandler(d_signals, PIOD->PIO_ISR);
}

// Interrupt handler for 1 ms interval timer
static void SysTick_IRQHandler (void)
{
    SysTick_Handler();

#if SDCARD_ENABLE
    static uint32_t fatfs_ticks = 10;
    if(!(--fatfs_ticks)) {
        disk_timerproc();
        fatfs_ticks = 10;
    }
#endif

    if(delay_ms.ms && !(--delay_ms.ms)) {
        if(delay_ms.callback) {
            delay_ms.callback();
            delay_ms.callback = NULL;
        }
    }
}
