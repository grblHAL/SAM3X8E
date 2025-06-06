/*
  protoneer_3.xx_map.h - driver code for Atmel SAM3X8E ARM processor, pin mappings compatible with Protoneer 3.xx boards

  Part of grblHAL

  Copyright (c) 2019-2024 Terje Io

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

#if N_ABC_MOTORS
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "Protoneer v3"

// Define step pulse output pins.
#define X_STEP_PORT             PIOB
#define X_STEP_PIN              25
#define Y_STEP_PORT             PIOC
#define Y_STEP_PIN              28
#define Z_STEP_PORT             PIOC
#define Z_STEP_PIN              26

// Define step direction output pins.
#define X_DIRECTION_PORT        PIOC
#define X_DIRECTION_PIN         25
#define Y_DIRECTION_PORT        PIOC
#define Y_DIRECTION_PIN         24
#define Z_DIRECTION_PORT        PIOC
#define Z_DIRECTION_PIN         23

// Define stepper driver enable/disable output pin(s).
#define X_ENABLE_PORT           PIOC
#define X_ENABLE_PIN            22

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT            PIOC // C28
#define X_LIMIT_PIN             21
#define Y_LIMIT_PORT            PIOC // D14
#define Y_LIMIT_PIN             29
#define Z_LIMIT_PORT            PIOD // A11
#define Z_LIMIT_PIN             8

// Define auxiliary output pins
#define AUXOUTPUT0_PORT         PIOD // Spindle PWM, TIOA8
#define AUXOUTPUT0_PIN          7
#define AUXOUTPUT1_PORT         PIOB // Spindle direction, Due Digital Pin 8
#define AUXOUTPUT1_PIN          25
#define AUXOUTPUT2_PORT         PIOA // Spindle enable
#define AUXOUTPUT2_PIN          15
#define AUXOUTPUT3_PORT         PIOA // Coolant flood
#define AUXOUTPUT3_PIN          22
#define AUXOUTPUT4_PORT         PIOA // Coolant mist
#define AUXOUTPUT4_PIN          6

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_TIMER       (TC2->TC_CHANNEL[2])
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

#define AUXINPUT0_PORT          PIOA
#define AUXINPUT0_PIN           4
#define AUXINPUT1_PORT          PIOA // Reset/EStop
#define AUXINPUT1_PIN           16
#define AUXINPUT2_PORT          PIOA // Feed hold
#define AUXINPUT2_PIN           24
#define AUXINPUT3_PORT          PIOA // Cycle start
#define AUXINPUT3_PIN           23

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT1_PORT
#define RESET_PIN               AUXINPUT1_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT2_PORT
#define FEED_HOLD_PIN           AUXINPUT2_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT3_PORT
#define CYCLE_START_PIN         AUXINPUT3_PIN
#endif

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT0_PORT
#define PROBE_PIN               AUXINPUT0_PIN
#endif

/**/
