/*
  driver.h - driver code for Atmel SAM3X8E ARM processor

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

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#define _WIRING_CONSTANTS_

#include "Arduino.h"
#include "grbl/hal.h"

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "grbl/driver_opts.h"

/******************************************************************************
* Definitions for bit band access and dynamic IRQ registration                *
******************************************************************************/

/* Bit band SRAM definitions */
#define BITBAND_SRAM_REF   0x20000000
#define BITBAND_SRAM_BASE  0x22000000

#define BITBAND_SRAM(a,b) (*((__IO uint32_t *)((BITBAND_SRAM_BASE + ((((uint32_t)(uint32_t *)&a)-BITBAND_SRAM_REF)<<5) + (b<<2)))))

/* Bit band PERIPHERAL definitions */
#define BITBAND_PERI_REF   0x40000000
#define BITBAND_PERI_BASE  0x42000000

#define BITBAND_PERI(a,b) (*((__IO uint32_t *)((BITBAND_PERI_BASE + ((((uint32_t)(uint32_t *)&a)-BITBAND_PERI_REF)<<5) + (b<<2)))))

#define DIGITAL_IN(port, pin) BITBAND_PERI(port->PIO_PDSR, pin)
#define DIGITAL_OUT(port, pin, on) { BITBAND_PERI(port->PIO_ODSR, pin) = on; }

void IRQRegister(int32_t IRQnum, void (*IRQhandler)(void));
void IRQUnRegister(int32_t IRQnum);

/*****************************************************************************/

// timer definitions

#define STEPPER_TIMER       (TC0->TC_CHANNEL[0])
#define STEPPER_TIMER_IRQn  TC0_IRQn
#define STEP_TIMER          (TC0->TC_CHANNEL[1])
#define STEP_TIMER_IRQn     TC1_IRQn
#if STEP_INJECT_ENABLE
#define STEP2_TIMER         (TC1->TC_CHANNEL[0])
#define STEP2_TIMER_IRQ     TC3_IRQn
#endif

#ifndef CONTROL_ENABLE
#define CONTROL_ENABLE (CONTROL_HALT|CONTROL_FEED_HOLD|CONTROL_CYCLE_START)
#endif

#ifdef BOARD_TINYG2_DUE
    #include "boards/tinyg2_due_map.h"
#elif defined(BOARD_RAMPS_16)
    #include "boards/ramps_1.6_map.h"
#elif defined(BOARD_RAMPS_SMART)
    #include "boards/ramps_smart_map.h"
#elif defined(BOARD_CMCGRATH)
    #include "boards/cmcgrath_rev3_map.h"
#elif defined(BOARD_MEGA256)
    #include "boards/mega_2560_map.h"
#elif defined(BOARD_PROTONEER)
    #include "boards/protoneer_3.xx_map.h"
#elif defined(BOARD_RADDS_16)
    #include "boards/radds_1.6_map.h"
#elif defined(BOARD_MY_MACHINE)
    #include "boards/my_machine_map.h"
#else
    #include "boards/generic_map.h"
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.

// Minimum pulse off time.
#ifndef STEP_PULSE_TOFF_MIN
#define STEP_PULSE_TOFF_MIN 2.0f
#endif
// Time from step out to step reset.
// Adjust for correct step pulse time.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 1.0f // microseconds
#endif

// End configuration

#include "grbl/driver_opts2.h"

#if !defined(SERIAL2_DEVICE) && (MODBUS_ENABLE & MODBUS_RTU_ENABLED)
#define SERIAL2_DEVICE 1 // Select serial device for ModBus communication, default is 1, allowed values are 0, 1 and 2
#endif

#if BLUETOOTH_ENABLE == 2
#define SERIAL2_DEVICE 1
#endif

// Define I2C port/pins

#if I2C_ENABLE == 1
#define I2C_PERIPH  TWI0
#define I2C_ID      ID_TWI0
#define I2C_IRQ     TWI0_IRQn
#define I2C_PORT    PIOA
#define I2C_SDA_PIN 17  // Arduino Due SDA1 pin
#define I2C_SCL_PIN 18  // Arduino Due SCL1 pin
#define I2C_SDA_BIT (1<<I2C_SDA_PIN)
#define I2C_SCL_BIT (1<<I2C_SCL_PIN)
#endif

#if I2C_ENABLE == 2
#define I2C_PERIPH  TWI1
#define I2C_ID      ID_TWI1
#define I2C_IRQ     TWI1_IRQn
#define I2C_PORT    PIOB
#define I2C_SDA_PIN 12  // Arduino Due SDA0-3 pin
#define I2C_SCL_PIN 13  // Arduino Due SCL0-3 pin
#define I2C_SDA_BIT (1<<I2C_SDA_PIN)
#define I2C_SCL_BIT (1<<I2C_SCL_PIN)
#endif

#define I2C_CLOCK 100000

#if defined(AUXOUTPUT0_PWM_PORT) || defined(AUXOUTPUT1_PWM_PORT) ||\
     defined(AUXOUTPUT0_ANALOG_PORT) || defined(AUXOUTPUT1_ANALOG_PORT) ||\
      defined(AUXINTPUT0_ANALOG_PORT) || defined(AUXINTPUT1_ANALOG_PORT) ||\
       defined(MCP3221_ENABLE)
#define AUX_ANALOG 1
#else
#define AUX_ANALOG 0
#endif

typedef struct {
    pin_function_t id;
    pin_cap_t cap;
    pin_mode_t mode;
    uint8_t user_port;
    Pio *port;
    uint_fast8_t pin;
    EAnalogChannel adc_ch;
    uint32_t bit;
    pin_group_t group;
    ioport_interrupt_callback_ptr interrupt_callback;
    const char *description;
} input_signal_t;

typedef struct {
    Pio *port;
    uint_fast8_t pin;
    uint32_t bit;
    pin_function_t id;
    pin_group_t group;
    pin_mode_t mode;
    const char *description;
} output_signal_t;

typedef struct {
    uint8_t n_pins;
    union {
        input_signal_t *inputs;
        output_signal_t *outputs;
    } pins;
} pin_group_pins_t;

void PIO_InputMode (Pio *port, uint32_t bit, bool no_pullup);
void PIO_EnableInterrupt (const input_signal_t *input, pin_irq_mode_t irq_mode);

void ioports_init (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (input_signal_t *input);

#endif
