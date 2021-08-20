/*
  tinyg2_due_map.h - driver code for Atmel SAM3X8E ARM processor, pin mappings compatible with cmcgrath rev3 g2core board

  Board design files (KiCad): https://github.com/cmcgrath5035/G2core-DUE-External-Interfaces

  Part of grblHAL

  Copyright (c) 2019 Terje Io

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

#if (N_AUTO_SQUARED && N_AUTO_SQUARED < N_ABC_MOTORS) || N_ABC_MOTORS > 3
#error "Axis configuration is not supported!"
#endif

 // Define step pulse output pins.
#define X_STEP_PORT         PIOB
#define X_STEP_PIN          25  // Due Digital Pin 2
#define Y_STEP_PORT         PIOC
#define Y_STEP_PIN          28  // Due Digital Pin 3
#define Z_STEP_PORT         PIOC
#define Z_STEP_PIN          26  // Due Digital Pin 4

// Define step direction output pins.
#define X_DIRECTION_PORT    PIOC
#define X_DIRECTION_PIN     25  // Due Digital Pin 5
#define Y_DIRECTION_PORT    PIOC
#define Y_DIRECTION_PIN     24  // Due Digital Pin 6
#define Z_DIRECTION_PORT    PIOC
#define Z_DIRECTION_PIN     23  // Due Digital Pin 7

// Define stepper driver enable/disable output pin(s).
#define X_ENABLE_PORT       PIOB
#define X_ENABLE_PIN        26  // Due Digital Pin 22
#define Y_ENABLE_PORT       PIOD
#define Y_ENABLE_PIN        0   // Due Digital Pin 25
#define Z_ENABLE_PORT       PIOD
#define Z_ENABLE_PIN        3   // Due Digital Pin 28

// Define homing/hard limit switch min input pins.
#define X_LIMIT_PORT        PIOD
#define X_LIMIT_PIN         4   // Due Digital Pin 14

#define Y_LIMIT_PORT        PIOA
#define Y_LIMIT_PIN         13  // Due Digital Pin 16

#define Z_LIMIT_PORT        PIOA
#define Z_LIMIT_PIN         11  // Due Digital Pin 18


// Define homing/hard limit switch max input pins.
#if X_AUTO_SQUARE
#define M3_LIMIT_PORT       PIOD
#define M3_LIMIT_PIN        5   // Due Digital Pin 15
#else
#define X_LIMIT_PORT_MAX    PIOD
#define X_LIMIT_PIN_MAX     5   // Due Digital Pin 15
#define X_LIMIT_BIT_MAX     (1<<X_LIMIT_PIN_MAX)
#endif
#if Y_AUTO_SQUARE
#define M3_LIMIT_PORT       PIOA
#define M3_LIMIT_PIN        12  // Due Digital Pin 17
#else
#define Y_LIMIT_PORT_MAX    PIOA
#define Y_LIMIT_PIN_MAX     12  // Due Digital Pin 17
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
#define M3_STEP_PORT        PIOA
#define M3_STEP_PIN         7   // Due Digital Pin 31
#define M3_DIRECTION_PORT   PIOD
#define M3_DIRECTION_PIN    10  // Due Digital Pin 32
#define M3_ENABLE_PORT      PIOC
#define M3_ENABLE_PIN       1   // Due Digital Pin 33
#ifndef M3_LIMIT_PORT
#define M3_LIMIT_PORT       PIOA
#define M3_LIMIT_PIN        6   // Due Analog Pin 4
#endif
#define M3_LIMIT_PORT_MAX   PIOA
#define M3_LIMIT_PIN_MAX    4   // Due Analog Pin 5
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 1
#define M4_AVAILABLE
#define M4_STEP_PORT        PIOC
#define M4_STEP_PIN         5   // Due Digital Pin 37
#define M4_DIRECTION_PORT   PIOC
#define M4_DIRECTION_PIN    6   // Due Digital Pin 38
#define M4_ENABLE_PORT      PIOC
#define M4_ENABLE_PIN       7   // Due Digital Pin 39
#define M4_LIMIT_PORT       PIOA
#define M4_LIMIT_PIN        6   // Due Analog Pin 4
#define M4_LIMIT_PORT_MAX   PIOA
#define M4_LIMIT_PIN_MAX    2   // Due Analog Pin 7
#endif

// Define ganged axis or C axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 3
#define M5_AVAILABLE
#define M5_STEP_PORT        PIOA
#define M5_STEP_PIN         19  // Due Digital Pin 42
#define M5_DIRECTION_PORT   PIOA
#define M5_DIRECTION_PIN    20  // Due Digital Pin 43
#define M5_ENABLE_PORT      PIOC
#define M5_ENABLE_PIN       19  // Due Digital Pin 44
#define M5_LIMIT_PORT       PIOA
#define M5_LIMIT_PIN        6   // Due Analog Pin 4
#define M5_LIMIT_PORT_MAX   PIOC
#define M5_LIMIT_PIN_MAX    12  // Due Digital Pin 51
#endif

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT PIOD
#define SPINDLE_ENABLE_PIN  8   // Due Digital Pin 12

// Start of PWM & Stepper Enabled Spindle
#define SPINDLE_PWM_TIMER   (TC2->TC_CHANNEL[2])
#define SPINDLE_PWM_PORT    PIOD
#define SPINDLE_PWM_PIN     7   // Due Digital Pin 11 / TIOA8

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT  PIOA
#define COOLANT_FLOOD_PIN   22

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PORT          PIOA
#define RESET_PIN           16
#define FEED_HOLD_PORT      PIOA
#define FEED_HOLD_PIN       24
#define CYCLE_START_PORT    PIOA
#define CYCLE_START_PIN     23

// Define probe switch input pin.
// NA

/**/
