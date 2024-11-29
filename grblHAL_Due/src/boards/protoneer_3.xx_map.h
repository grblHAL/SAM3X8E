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

// Define driver spindle pins

#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_TIMER       (TC2->TC_CHANNEL[2])
#define SPINDLE_PWM_PORT        PIOD
#define SPINDLE_PWM_PIN         7 // TIOA8
#else
#define AUXOUTPUT0_PORT         PIOD
#define AUXOUTPUT0_PIN          7
#endif

#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PORT  PIOB
#define SPINDLE_DIRECTION_PIN   27  // Due Digital Pin 8
#else
#define AUXOUTPUT1_PORT         PIOB
#define AUXOUTPUT1_PIN          27
#endif

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PORT     PIOA
#define SPINDLE_ENABLE_PIN      15
#else
#define AUXOUTPUT2_PORT         PIOA
#define AUXOUTPUT2_PIN          15
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      PIOA
#define COOLANT_FLOOD_PIN       22
#define COOLANT_MIST_PORT       PIOA
#define COOLANT_MIST_PIN        6

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PORT              PIOA
#define RESET_PIN               16
#define FEED_HOLD_PORT          PIOA
#define FEED_HOLD_PIN           24
#define CYCLE_START_PORT        PIOA
#define CYCLE_START_PIN         23

#define AUXINPUT0_PORT          PIOA
#define AUXINPUT0_PIN           4

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT0_PORT
#define PROBE_PIN               AUXINPUT0_PIN
#endif

/**/
