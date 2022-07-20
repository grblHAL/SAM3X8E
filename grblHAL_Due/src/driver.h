/*
  driver.h - driver code for Atmel SAM3X8E ARM processor

  Part of grblHAL

  Copyright (c) 2019-2022 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

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

#define DEBOUNCE_TIMER      (TC1->TC_CHANNEL[0])
#define DEBOUNCE_TIMER_IRQn TC3_IRQn

#ifdef BOARD_TINYG2_DUE
    #include "tinyg2_due_map.h"
#elif defined(BOARD_RAMPS_16)
    #include "ramps_1.6_map.h"
#elif defined(BOARD_CMCGRATH)
    #include "cmcgrath_rev3_map.h"
#elif defined(BOARD_MEGA256)
    #include "mega_2560_map.h"
#elif defined(BOARD_PROTONEER)
    #include "protoneer_3.xx_map.h"
#elif defined(BOARD_RADDS_16)
    #include "radds_1.6_map.h"
#elif defined(BOARD_MY_MACHINE)
    #include "my_machine_map.h"
#else
    #include "generic_map.h"
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 1.0f // microseconds
#endif

// End configuration

#if BLUETOOTH_ENABLE == 2
#define SERIAL2_DEVICE 1
#endif

#if TRINAMIC_ENABLE == 2130
#include "tmc2130/trinamic.h"
#endif

#if MODBUS_ENABLE
#include "spindle/modbus.h"
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

// Simple sanity check...

#if KEYPAD_ENABLE == 1 && !defined(I2C_STROBE_PORT)
#error Keypad plugin not supported!
#elif I2C_STROBE_ENABLE && !defined(I2C_STROBE_PORT)
#error I2C strobe not supported!
#endif

#if MPG_MODE == 1 && !defined(MPG_MODE_PIN)
#error "MPG_MODE_PIN must be defined!"
#endif

typedef struct {
    Pio *port;
    uint_fast8_t pin;
    uint32_t bit;
    pin_function_t id;
    pin_group_t group;
    bool debounce;
    pin_irq_mode_t irq_mode;
    pin_mode_t cap;
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

void PIO_EnableInterrupt (const input_signal_t *input, pin_irq_mode_t irq_mode);

#ifdef HAS_IOPORTS
void ioports_init (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (input_signal_t *input);
#endif

#endif
