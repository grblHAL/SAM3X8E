/*
  ramps_1.6_map.h - driver code for Atmel SAM3X8E ARM processor, pin mappings compatible with Ramps 1.6 board

  NOTE: board must be modified for 3.3V IO before use!

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

  Mappings according to Re-ARM for NXP LPC1768

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

#if (N_AUTO_SQUARED && N_AUTO_SQUARED < N_ABC_MOTORS) || N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "Ramps 1.6"
#define HAS_IOPORTS

 // Define step pulse output pins.
#define X_STEP_PORT         PIOA
#define X_STEP_PIN          16  // Due Analog Pin 0
#define Y_STEP_PORT         PIOA
#define Y_STEP_PIN          3   // Due Analog Pin 6
#define Z_STEP_PORT         PIOC
#define Z_STEP_PIN          17  // Due Digital Pin 46

// Define step direction output pins.
#define X_DIRECTION_PORT    PIOA
#define X_DIRECTION_PIN     24  // Due Analog Pin 1
#define Y_DIRECTION_PORT    PIOA
#define Y_DIRECTION_PIN     2   // Due Analog Pin 7
#define Z_DIRECTION_PORT    PIOC
#define Z_DIRECTION_PIN     15  // Due Digital Pin 48

// Define stepper driver enable/disable output pin(s).
#define X_ENABLE_PORT       PIOC
#define X_ENABLE_PIN        6   // Due Digital Pin 38
#define Y_ENABLE_PORT       PIOA
#define Y_ENABLE_PIN        23  // Due Analog Pin 2
#define Z_ENABLE_PORT       PIOB
#define Z_ENABLE_PIN        17  // Due Analog Pin 8

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT        PIOC
#define X_LIMIT_PIN         28  // Due Digital Pin 3
#define Y_LIMIT_PORT        PIOD
#define Y_LIMIT_PIN         4   // Due Digital Pin 14
#define Z_LIMIT_PORT        PIOA
#define Z_LIMIT_PIN         11  // Due Digital Pin 18

// Define homing/hard limit switch input pins.
#if X_AUTO_SQUARE
#define M3_LIMIT_PORT       PIOB
#define M3_LIMIT_PIN        25  // Due Digital Pin 2
#else
#define X_LIMIT_PORT_MAX    PIOB
#define X_LIMIT_PIN_MAX     25  // Due Digital Pin 2
#define X_LIMIT_BIT_MAX     (1<<X_LIMIT_PIN_MAX)
#endif
#if Y_AUTO_SQUARE
#define M3_LIMIT_PORT       PIOD
#define M3_LIMIT_PIN        5   // Due Digital Pin 15
#else
#define Y_LIMIT_PORT_MAX    PIOD
#define Y_LIMIT_PIN_MAX     5   // Due Digital Pin 15
#define Y_LIMIT_BIT_MAX     (1<<Y_LIMIT_PIN_MAX)
#endif
#if Z_AUTO_SQUARE
#define M3_LIMIT_PORT       PIOA
#define M3_LIMIT_PIN        10  // Due Digital Pin 19
#else
#define Z_LIMIT_PORT_MAX    PIOA
#define Z_LIMIT_PIN_MAX     10  // Due Digital Pin 19
#endif

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PORT        PIOD
#define M3_STEP_PIN         1   // Due Digital Pin 61?
#define M3_DIRECTION_PORT   PIOD
#define M3_DIRECTION_PIN    3   // Due Digital Pin 60?
#define M3_ENABLE_PORT      PIOA
#define M3_ENABLE_PIN       15  // Due Digital Pin 62?
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_STEP_PORT        PIOC
#define M4_STEP_PIN         4   // Due Digital Pin 64?
#define M4_DIRECTION_PORT   PIOC
#define M4_DIRECTION_PIN    2   // Due Digital Pin 63?
#define M4_ENABLE_PORT      PIOC
#define M4_ENABLE_PIN       9   // Due Digital Pin 65
#endif

#if VFD_SPINDLE == 0

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     PIOC
#define SPINDLE_ENABLE_PIN      26  // Due Digital Pin 4
#define SPINDLE_DIRECTION_PORT  PIOC
#define SPINDLE_DIRECTION_PIN   22  // Due Digital Pin 8

// Start of PWM & Stepper Enabled Spindle
#define SPINDLE_PWM_TIMER   (TC2->TC_CHANNEL[0])
#define SPINDLE_PWM_CCREG   2
#define SPINDLE_PWM_PORT    PIOC
#define SPINDLE_PWM_PIN     25  // Due Digital Pin 5 // TIOA6

#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT  PIOB
#define COOLANT_FLOOD_PIN   18  // Due Analog port 89

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PORT          PIOA
#define RESET_PIN           22  // DUE Analog Pin 3
#define FEED_HOLD_PORT      PIOA
#define FEED_HOLD_PIN       4   // DUE Analog Pin 4
#define CYCLE_START_PORT    PIOA
#define CYCLE_START_PIN     6   // DUE Analog Pin 5

#ifdef HAS_IOPORTS

#define AUXINPUT0_PORT      PIOA
#define AUXINPUT0_PIN       14
#define AUXINPUT1_PORT      PIOD
#define AUXINPUT1_PIN       0
#define AUXINPUT2_PORT      PIOD
#define AUXINPUT2_PIN       2

#define AUXOUTPUT0_PORT     PIOB
#define AUXOUTPUT0_PIN      14
#define AUXOUTPUT1_PORT     PIOC
#define AUXOUTPUT1_PIN      12
#define AUXOUTPUT2_PORT     PIOC
#define AUXOUTPUT2_PIN      14

#endif

/**/
