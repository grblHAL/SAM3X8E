/*
  radds_1.6_map.h - driver code for Atmel SAM3X8E ARM processor, pin mappings compatible with Radds 1.6 board

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

#define BOARD_NAME "Radds 1.6"

 // Define step pulse output pins.
#define X_STEP_PORT         PIOA
#define X_STEP_PIN          15  // Due Analog Pin 24
#define Y_STEP_PORT         PIOA
#define Y_STEP_PIN          12   // Due Analog Pin 17
#define Z_STEP_PORT         PIOB
#define Z_STEP_PIN          25  // Due Digital Pin 2

// Define step direction output pins.
#define X_DIRECTION_PORT    PIOB
#define X_DIRECTION_PIN     26  // Due Analog Pin 23
#define Y_DIRECTION_PORT    PIOA
#define Y_DIRECTION_PIN     13   // Due Analog Pin 16
#define Z_DIRECTION_PORT    PIOC
#define Z_DIRECTION_PIN     28  // Due Digital Pin 3

// Define stepper driver enable/disable output pin(s).
#define X_ENABLE_PORT       PIOD
#define X_ENABLE_PIN        1   // Due Digital Pin 26
#define Y_ENABLE_PORT       PIOB
#define Y_ENABLE_PIN        26  // Due Analog Pin 22
#define Z_ENABLE_PORT       PIOD
#define Z_ENABLE_PIN        5  // Due Analog Pin 15

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT        PIOD
#define X_LIMIT_PIN         3  // Due Digital Pin 28
#define Y_LIMIT_PORT        PIOD
#define Y_LIMIT_PIN         9   // Due Digital Pin 30
#define Z_LIMIT_PORT        PIOD
#define Z_LIMIT_PIN         10  // Due Digital Pin 32

// Define homing/hard limit switch input pins.
#if X_AUTO_SQUARE
#define M3_LIMIT_PORT       PIOC
#define M3_LIMIT_PIN        2  // Due Digital Pin 34
#else
#define X_LIMIT_PORT_MAX    PIOC
#define X_LIMIT_PIN_MAX     2  // Due Digital Pin 34
#define X_LIMIT_BIT_MAX     (1<<X_LIMIT_PIN_MAX)
#endif
#if Y_AUTO_SQUARE
#define M3_LIMIT_PORT       PIOC
#define M3_LIMIT_PIN        4   // Due Digital Pin 36
#else
#define Y_LIMIT_PORT_MAX    PIOC
#define Y_LIMIT_PIN_MAX     4   // Due Digital Pin 36
#define Y_LIMIT_BIT_MAX     (1<<Y_LIMIT_PIN_MAX)
#endif
#if X_AUTO_SQUARE
#define M3_LIMIT_PORT       PIOC
#define M3_LIMIT_PIN        6  // Due Digital Pin 38
#else
#define Z_LIMIT_PORT_MAX    PIOC
#define Z_LIMIT_PIN_MAX     6  // Due Digital Pin 38
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
#define SPINDLE_ENABLE_PORT PIOC
#define SPINDLE_ENABLE_PIN  23  // Due Digital Pin 7
/*
#define SPINDLE_DIRECTION_PORT  PIOC
#define SPINDLE_DIRECTION_PIN   25  // Due Digital Pin 5
*/

// Start of PWM & Stepper Enabled Spindle
#define SPINDLE_PWM_TIMER   (TC2->TC_CHANNEL[0])
#define SPINDLE_PWM_CCREG   2
#define SPINDLE_PWM_PORT    PIOC
#define SPINDLE_PWM_PIN     22  // Due Digital Pin 8 // PWML5 B

#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT  PIOB
#define COOLANT_FLOOD_PIN   18  // Due Analog port 9

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PORT          PIOA
#define RESET_PIN           16  // DUE Analog Pin 0
#define FEED_HOLD_PORT      PIOA
#define FEED_HOLD_PIN       24   // DUE Analog Pin 1
#define CYCLE_START_PORT    PIOA
#define CYCLE_START_PIN     23   // DUE Analog Pin 2

/**/
