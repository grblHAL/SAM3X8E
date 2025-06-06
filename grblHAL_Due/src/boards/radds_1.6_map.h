/*
  radds_1.6_map.h - driver code for Atmel SAM3X8E ARM processor, pin mappings compatible with Radds 1.6 board

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  grblHAL with Grbl. If not, see <http://www.gnu.org/licenses/>.
*/

#if (N_AUTO_SQUARED && N_AUTO_SQUARED < N_ABC_MOTORS) || N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "Radds 1.6"

 // Define step pulse output pins.
#define X_STEP_PORT         PIOA
#define X_STEP_PIN          15  // Due Analog Pin 24
#define Y_STEP_PORT         PIOA
#define Y_STEP_PIN          12  // Due Analog Pin 17
#define Z_STEP_PORT         PIOB
#define Z_STEP_PIN          25  // Due Digital Pin 2

// Define step direction output pins.
#define X_DIRECTION_PORT    PIOB
#define X_DIRECTION_PIN     26  // Due Analog Pin 23
#define Y_DIRECTION_PORT    PIOA
#define Y_DIRECTION_PIN     13  // Due Analog Pin 16
#define Z_DIRECTION_PORT    PIOC
#define Z_DIRECTION_PIN     28  // Due Digital Pin 3

// Define stepper driver enable/disable output pin(s).
#define X_ENABLE_PORT       PIOD
#define X_ENABLE_PIN        1   // Due Digital Pin 26
#define Y_ENABLE_PORT       PIOB
#define Y_ENABLE_PIN        26  // Due Analog Pin 22
#define Z_ENABLE_PORT       PIOD
#define Z_ENABLE_PIN        5   // Due Analog Pin 15

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT        PIOD
#define X_LIMIT_PIN         3   // Due Digital Pin 28
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
#define M3_LIMIT_PIN        4  // Due Digital Pin 36
#else
#define Y_LIMIT_PORT_MAX    PIOC
#define Y_LIMIT_PIN_MAX     4  // Due Digital Pin 36
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
#define M3_STEP_PIN         1   // Due Digital Pin 26
#define M3_DIRECTION_PORT   PIOD
#define M3_DIRECTION_PIN    3   // Due Digital Pin 28
#define M3_ENABLE_PORT      PIOA
#define M3_ENABLE_PIN       15  // Due Digital Pin 24
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_STEP_PORT        PIOC
#define M4_STEP_PIN         4   // Due Digital Pin 36
#define M4_DIRECTION_PORT   PIOC
#define M4_DIRECTION_PIN    2   // Due Digital Pin 34
#define M4_ENABLE_PORT      PIOD
#define M4_ENABLE_PIN       9   // Due Digital Pin 30
#endif

// Define auxiliary output pins
#define AUXOUTPUT0_PORT     PIOC // Spindle PWM, Due Digital Pin 8 // PWML5 B
#define AUXOUTPUT0_PIN      22
#define AUXOUTPUT1_PORT     PIOC // Spindle enable, Due Digital Pin 7
#define AUXOUTPUT1_PIN      23
#define AUXOUTPUT2_PORT     PIOD // Coolant flood, Due Analog port 9
#define AUXOUTPUT2_PIN      18

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT1_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT1_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_TIMER       (TC2->TC_CHANNEL[0])
#define SPINDLE_PWM_PORT        AUXOUTPUT0_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT2_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT2_PIN
#if COOLANT_ENABLE & COOLANT_MIST
#undef COOLANT_ENABLE
#define COOLANT_ENABLE COOLANT_FLOOD
#endif
#elif COOLANT_ENABLE & COOLANT_MIST
#undef COOLANT_ENABLE
#define COOLANT_ENABLE 0
#endif

#define AUXINPUT0_PORT          PIOA // Reset/EStop
#define AUXINPUT0_PIN           16
#define AUXINPUT1_PORT          PIOA // Feed hold
#define AUXINPUT1_PIN           24
#define AUXINPUT2_PORT          PIOA // Cycle start
#define AUXINPUT2_PIN           23

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT0_PORT
#define RESET_PIN               AUXINPUT0_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT1_PORT
#define FEED_HOLD_PIN           AUXINPUT1_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT2_PORT
#define CYCLE_START_PIN         AUXINPUT2_PIN
#endif

/**/
