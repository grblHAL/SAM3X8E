/*
  generic_map.h - river code for Atmel SAM3X8E ARM processor

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

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

#if N_ABC_MOTORS
#error "Axis configuration is not supported!"
#endif

//#define HAS_IOPORTS

// Define step pulse output pins.
#define X_STEP_PORT             PIOA
#define X_STEP_PIN              16
#define Y_STEP_PORT             PIOA
#define Y_STEP_PIN              3
#define Z_STEP_PORT             PIOC
#define Z_STEP_PIN              17

// Define step direction output pins.
#define X_DIRECTION_PORT        PIOA
#define X_DIRECTION_PIN         24
#define Y_DIRECTION_PORT        PIOA
#define Y_DIRECTION_PIN         2
#define Z_DIRECTION_PORT        PIOC
#define Z_DIRECTION_PIN         15

// Define stepper driver enable/disable output pin(s).
#define X_ENABLE_PORT           PIOC
#define X_ENABLE_PIN            6
#define Y_ENABLE_PORT           PIOA
#define Y_ENABLE_PIN            23
#define Z_ENABLE_PORT           PIOB
#define Z_ENABLE_PIN            17

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT            PIOB // C28
#define X_LIMIT_PIN             25

#define Y_LIMIT_PORT            PIOD // D14
#define Y_LIMIT_PIN             25
#define Z_LIMIT_PORT            PIOC // A11
#define Z_LIMIT_PIN             24

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     PIOA
#define SPINDLE_ENABLE_PIN      15
#define SPINDLE_DIRECTION_PORT  PIOD
#define SPINDLE_DIRECTION_PIN   3

// Start of PWM & Stepper Enabled Spindle
#define SPINDLE_PWM_TIMER       (TC2->TC_CHANNEL[0])
#define SPINDLE_PWM_PORT        PIOC
#define SPINDLE_PWM_PIN         25  // TIOA6

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      PIOC
#define COOLANT_FLOOD_PIN       5
#define COOLANT_MIST_PORT       PIOC
#define COOLANT_MIST_PIN        3

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PORT              PIOC
#define RESET_PIN               12
#define FEED_HOLD_PORT          PIOC
#define FEED_HOLD_PIN           14
#define CYCLE_START_PORT        PIOC
#define CYCLE_START_PIN         16
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        PIOC
#define SAFETY_DOOR_PIN         18
#endif

// Define probe switch input pin.
#define PROBE_PORT              PIOC
#define PROBE_PIN               13

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         PIOA
#define I2C_STROBE_PIN          5
#endif

#if SDCARD_ENABLE
// Define SD card detect pin.
#define SD_CD_PORT              PIOA
#define SD_CD_PIN               30
#endif

#ifdef HAS_IOPORTS
#define AUXOUTPUT0_PORT         PIOA
#define AUXOUTPUT0_PIN          14
#define AUXOUTPUT1_PORT         PIOD
#define AUXOUTPUT1_PIN          0
#define AUXOUTPUT2_PORT         PIOD
#define AUXOUTPUT2_PIN          2
#endif

/**/

