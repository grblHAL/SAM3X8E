/*
  ramps_1.6_map.h - driver code for Atmel SAM3X8E ARM processor, pin mappings compatible with Ramps 1.6 board

  NOTE: board must be modified for 3.3V IO before use!

  Part of grblHAL

  Copyright (c) 2019-2023 Terje Io

  Mappings according to Re-ARM for NXP LPC1768

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

#define BOARD_NAME "Ramps 1.6"
#define BOARD_URL "https://github.com/bigtreetech/ramps-1.6"

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
#endif
#if Y_AUTO_SQUARE
#define M3_LIMIT_PORT       PIOD
#define M3_LIMIT_PIN        5   // Due Digital Pin 15
#else
#define Y_LIMIT_PORT_MAX    PIOD
#define Y_LIMIT_PIN_MAX     5   // Due Digital Pin 15
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
#define AUXOUTPUT0_PORT     PIOB
#define AUXOUTPUT0_PIN      14
#define AUXOUTPUT1_PORT     PIOC
#define AUXOUTPUT1_PIN      12
#define AUXOUTPUT2_PORT     PIOC
#define AUXOUTPUT2_PIN      14
#define AUXOUTPUT3_PORT     PIOC // Spindle PWM, Due Digital Pin 5 // TI
#define AUXOUTPUT3_PIN      25
#define AUXOUTPUT4_PORT     PIOC // Spindle direction, Due Digital Pin 8
#define AUXOUTPUT4_PIN      22
#define AUXOUTPUT5_PORT     PIOC // Spindle enable, Due Digital Pin 4
#define AUXOUTPUT5_PIN      26
#define AUXOUTPUT6_PORT     PIOB // Coolant flood, Due Analog Pin 9
#define AUXOUTPUT6_PIN      18

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT5_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT5_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_TIMER       (TC2->TC_CHANNEL[0])
#define SPINDLE_PWM_CCREG       2
#define SPINDLE_PWM_PORT        AUXOUTPUT3_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT3_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT4_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT4_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT6_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT6_PIN
#if COOLANT_ENABLE & COOLANT_MIST
#undef COOLANT_ENABLE
#define COOLANT_ENABLE COOLANT_FLOOD
#endif
#elif COOLANT_ENABLE & COOLANT_MIST
#undef COOLANT_ENABLE
#define COOLANT_ENABLE 0
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PORT          PIOA
#define RESET_PIN           22  // DUE Analog Pin 3
#define FEED_HOLD_PORT      PIOA
#define FEED_HOLD_PIN       4   // DUE Analog Pin 4
#define CYCLE_START_PORT    PIOA
#define CYCLE_START_PIN     6   // DUE Analog Pin 5

#define AUXINPUT0_PORT      PIOA
#define AUXINPUT0_PIN       14
#define AUXINPUT1_PORT      PIOD
#define AUXINPUT1_PIN       0
#define AUXINPUT2_PORT      PIOD
#define AUXINPUT2_PIN       2
#define AUXINPUT3_PORT      PIOB  // Due Digital Pin 21
#define AUXINPUT3_PIN       13

#if PROBE_ENABLE
#define PROBE_PORT          AUXINPUT3_PORT
#define PROBE_PIN           AUXINPUT3_PIN
#endif

#define AUXOUTPUT0_PORT     PIOB
#define AUXOUTPUT0_PIN      14
#define AUXOUTPUT1_PORT     PIOC
#define AUXOUTPUT1_PIN      12
#define AUXOUTPUT2_PORT     PIOC
#define AUXOUTPUT2_PIN      14

/**/
