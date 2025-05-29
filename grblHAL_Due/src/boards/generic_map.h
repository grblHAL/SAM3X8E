/*
  generic_map.h - driver code for Atmel SAM3X8E ARM processor

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
#define X_LIMIT_PORT            PIOB // 02
#define X_LIMIT_PIN             25

#define Y_LIMIT_PORT            PIOD // ??
#define Y_LIMIT_PIN             25
#define Z_LIMIT_PORT            PIOC // 06
#define Z_LIMIT_PIN             24

// Define auxiliary output pins
#define AUXOUTPUT0_PORT         PIOA
#define AUXOUTPUT0_PIN          14
#define AUXOUTPUT1_PORT         PIOD
#define AUXOUTPUT1_PIN          0
#define AUXOUTPUT2_PORT         PIOD
#define AUXOUTPUT2_PIN          2
#define AUXOUTPUT3_PORT         PIOC // Spindle PWM, TIOA6
#define AUXOUTPUT3_PIN          25
#define AUXOUTPUT4_PORT         PIOD // Spindle direction
#define AUXOUTPUT4_PIN          3
#define AUXOUTPUT5_PORT         PIOA // Spindle enable
#define AUXOUTPUT5_PIN          15
#define AUXOUTPUT6_PORT         PIOC // Coolant flood
#define AUXOUTPUT6_PIN          5
#define AUXOUTPUT7_PORT         PIOC // Coolant mist
#define AUXOUTPUT7_PIN          3

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT5_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT5_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_TIMER       (TC2->TC_CHANNEL[0])
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
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT7_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT7_PIN
#endif

#define AUXINPUT0_PORT          PIOC // 45
#define AUXINPUT0_PIN           18
#define AUXINPUT1_PORT          PIOA // I2C strobe
#define AUXINPUT1_PIN           4
#define AUXINPUT2_PORT          PIOC // Probe 50
#define AUXINPUT2_PIN           13
#define AUXINPUT3_PORT          PIOC // Reset/EStop
#define AUXINPUT3_PIN           12
#define AUXINPUT4_PORT          PIOC // Feed hold
#define AUXINPUT4_PIN           14
#define AUXINPUT5_PORT          PIOC // Cycle start
#define AUXINPUT5_PIN           16

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT3_PORT
#define RESET_PIN               AUXINPUT3_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT4_PORT
#define FEED_HOLD_PIN           AUXINPUT4_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT5_PORT
#define CYCLE_START_PIN         AUXINPUT5_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT2_PORT
#define PROBE_PIN               AUXINPUT2_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         AUXINPUT1_PORT
#define I2C_STROBE_PIN          AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT0_PORT
#define MOTOR_FAULT_PIN         AUXINPUT0_PIN
#elif MOTOR_WARNING_ENABLE
#define MOTOR_WARNING_PORT      AUXINPUT0_PORT
#define MOTOR_WARNING_PIN       AUXINPUT0_PIN
#endif

#if SDCARD_ENABLE
// Define SD card detect pin.
#define SD_CS_PORT              PIOA
#define SD_CS_PIN               30
#endif

/**/
