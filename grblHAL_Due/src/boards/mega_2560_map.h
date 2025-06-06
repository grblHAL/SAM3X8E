/*
  mega_2560_map.h - driver code for Atmel SAM3X8E ARM processor, pin mappings compatible with Arduino Mega 2560

  Part of grblHAL

  Copyright (c) 2019-2023 Terje Io

  Mappings according to cpu_map.h for Arduino Mega 2560 : Working @EliteEng

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

#if N_ABC_MOTORS
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "Mega 2560"

 // Define step pulse output pins.
#define X_STEP_PORT         PIOA
#define X_STEP_PIN          15  // Due Digital Pin 24
#define Y_STEP_PORT         PIOD
#define Y_STEP_PIN          0   // Due Digital Pin 25
#define Z_STEP_PORT         PIOD
#define Z_STEP_PIN          1   // Due Digital Pin 26

// Define step direction output pins.
#define X_DIRECTION_PORT    PIOD
#define X_DIRECTION_PIN     9   // Due Digital Pin 30
#define Y_DIRECTION_PORT    PIOA
#define Y_DIRECTION_PIN     7   // Due Digital Pin 31
#define Z_DIRECTION_PORT    PIOD
#define Z_DIRECTION_PIN     10  // Due Digital Pin 32

// Define stepper driver enable/disable output pin(s).
#define X_ENABLE_PORT       PIOB
#define X_ENABLE_PIN        27  // Due Digital Pin 13

/*
#define Y_ENABLE_PORT      PIOA
#define Y_ENABLE_PIN       23
#define Z_ENABLE_PORT      PIOB
#define Z_ENABLE_PIN       17

*/
// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT        PIOC
#define X_LIMIT_PIN         29  // Due Digital Pin 10
#define Y_LIMIT_PORT        PIOD
#define Y_LIMIT_PIN         7   // Due Digital Pin 11
#define Z_LIMIT_PORT        PIOD
#define Z_LIMIT_PIN         8   // Due Digital Pin 12

// Define auxiliary output pins
#define AUXOUTPUT0_PORT     PIOC // Spindle PWM, Due Digital Pin 7 / PWML6 B
#define AUXOUTPUT0_PIN      23
#define AUXOUTPUT1_PORT     PIOC // Spindle direction, Due Digital Pin 5
#define AUXOUTPUT1_PIN      25
#define AUXOUTPUT2_PORT     PIOC // Spindle enable, Due Digital Pin 6
#define AUXOUTPUT2_PIN      24
#define AUXOUTPUT3_PORT     PIOC // Coolant flood, Due Digital Pin 8
#define AUXOUTPUT3_PIN      22
#define AUXOUTPUT4_PORT     PIOC // Coolant mist, Due Digital Pin 9
#define AUXOUTPUT4_PIN      21

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_CHANNEL     6
#define SPINDLE_PWM_PORT        AUXOUTPUT0_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT1_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT3_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT4_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT4_PIN
#endif

#define AUXINPUT0_PORT      PIOB
#define AUXINPUT0_PIN       20 // DUE Analog Pin 11
#define AUXINPUT1_PORT      PIOA
#define AUXINPUT1_PIN       0  // DUE Analog Pin CANTX
#define AUXINPUT2_PORT      PIOB // A8 Reset/EStop
#define AUXINPUT2_PIN       17
#define AUXINPUT3_PORT      PIOB // A9 Feed hold
#define AUXINPUT3_PIN       18
#define AUXINPUT4_PORT      PIOB // A10 Cycle start
#define AUXINPUT4_PIN       19

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT2_PORT
#define RESET_PIN               AUXINPUT2_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT3_PORT
#define FEED_HOLD_PIN           AUXINPUT3_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT4_PORT
#define CYCLE_START_PIN         AUXINPUT4_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT          AUXINPUT1_PORT
#define PROBE_PIN           AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT    AUXINPUT0_PORT
#define SAFETY_DOOR_PIN     AUXINPUT0_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT    AUXINPUT0_PORT
#define MOTOR_FAULT_PIN     AUXINPUT0_PIN
#elif MOTOR_WARNING_ENABLE
#define MOTOR_WARNING_PORT  AUXINPUT0_PORT
#define MOTOR_WARNING_PIN   AUXINPUT0_PIN
#endif

/**/
