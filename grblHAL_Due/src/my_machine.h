/*
  my_machine.h - configuration for Atmel SAM3X8E ARM processor

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

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

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used
// NOTE: Only one board may be enabled!
//#define BOARD_TINYG2_DUE
//#define BOARD_RAMPS_16
//#define BOARD_RAMPS_SMART
//#define BOARD_MEGA256
//#define BOARD_PROTONEER
//#define BOARD_CMCGRATH
//#define BOARD_RADDS_16   // NOTE: untested!!
//#define BOARD_MY_MACHINE // Add my_machine_map.h before enabling this!

// Configuration
// Uncomment to enable.

//#define USB_SERIAL_CDC          1 // Use native USB port for communication.
//#define BLUETOOTH_ENABLE        2 // Set to 2 for HC-05 module. Requires and claims one auxiliary input pin.
// Spindle selection:
// Up to four specific spindle drivers can be instantiated at a time
// depending on N_SPINDLE and N_SYS_SPINDLE definitions in grbl/config.h.
// If none are specified the default PWM spindle is instantiated.
// Spindle definitions can be found in grbl/spindle_control.h.
// More here https://github.com/grblHAL/Plugins_spindle
//#define SPINDLE0_ENABLE         SPINDLE_HUANYANG1
//#define SPINDLE1_ENABLE         SPINDLE_PWM0
//#define SPINDLE2_ENABLE         SPINDLE_NONE
//#define SPINDLE2_ENABLE         SPINDLE_NONE
//#define SPINDLE_OFFSET          1 // Uncomment to enable settings for laser spindle XY-offset.
// **********************
//#define MODBUS_ENABLE           1 // Set to 1 for auto direction, 2 for direction signal on auxiliary output pin.
//#define PLASMA_ENABLE           1 // Plasma/THC plugin.
//#define OPENPNP_ENABLE          1 // OpenPNP plugin. To be completed.
//#define MPG_ENABLE              1 // Enable MPG interface. Requires a serial stream and means to switch between normal and MPG mode.
                                    // 1: Mode switching is by handshake pin.
                                    // 2: Mode switching is by the CMD_MPG_MODE_TOGGLE (0x8B) command character.
//#define KEYPAD_ENABLE           1 // 1: uses a I2C keypad for input.
                                    // 2: uses a serial stream for input. If MPG_ENABLE is set > 0 the serial stream is shared with the MPG.

//#define LASER_COOLANT_ENABLE    1 // Laser coolant plugin. To be completed.
//#define LASER_OVD_ENABLE        1 // Enable M-code for overdrive PWM output during spindle off in RPM controlled mode.
//#define LB_CLUSTERS_ENABLE      1 // LaserBurn cluster support.
//#define EEPROM_ENABLE          16 // I2C EEPROM/FRAM support. Set to 16 for 2K, 32 for 4K, 64 for 8K, 128 for 16K and 256 for 16K capacity.
//#define EEPROM_IS_FRAM          1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.
//#define PWM_SERVO_ENABLE        1 // Enable M280 PWM servo support, requires at least one PWM capable auxiliary output.
//#define BLTOUCH_ENABLE          1 // Enable M401/M402 BLTouch support. Requires and claims one auxiliary PWM servo output.
//#define EVENTOUT_ENABLE         1 // Enable binding events (triggers) to control auxiliary outputs.
//#define ESP_AT_ENABLE           1 // Enable support for Telnet communication via UART connected ESP32 running ESP-AT.
//#define FEED_OVERRIDE_ENABLE    1 // Enable M200 feed override control.
//#define HOMING_PULLOFF_ENABLE   1 // Enable per axis homing pulloff distance settings.

// IO expanders:
//
//#define MCP3221_ENABLE          1 // MCP3221 I2C ADC input, default address is 0x9A (MCP3221_ADDRESS).
//#define PCA9654E_ENABLE         1 // PCA9654E I2C digital I/O, default address is 0x40 (PCA9654E_ADDRESS).

// Optional control signals:
// These will be assigned to aux input pins. Use the $pins command to check which pins are assigned.
// NOTE: If not enough pins are available assignment will silently fail.
//#define PROBE_ENABLE            0 // Default enabled, remove comment to disable probe input.
//#define PROBE2_ENABLE           1 // Enable second regular probe input, depending on the board the input assigned may be predefined.
//#define TOOLSETTER_ENABLE       1 // Enable toolsetter input, depending on the board the input assigned may be predefined.
//#define SAFETY_DOOR_ENABLE      1
//#define MOTOR_FAULT_ENABLE      1
//#define MOTOR_WARNING_ENABLE    1
//#define PROBE_DISCONNECT_ENABLE 1
//#define STOP_DISABLE_ENABLE     1
//#define BLOCK_DELETE_ENABLE     1
//#define SINGLE_BLOCK_ENABLE     1
//#define LIMITS_OVERRIDE_ENABLE  1

#ifndef USB_SERIAL_CDC
#define SERIAL_DEVICE        -1 // Select serial device for output if not using native USB, default is -1, max value is 2
#endif

// If the selected board map supports more than three motors ganging and/or auto-squaring
// of axes can be enabled here.
//#define X_GANGED            1
//#define X_AUTO_SQUARE       1
//#define Y_GANGED            1
//#define Y_AUTO_SQUARE       1
//#define Z_GANGED            1
//#define Z_AUTO_SQUARE       1
// For ganged axes the limit switch input (if available) can be configured to act as a max travel limit switch.
// NOTE: If board map already has max limit inputs defined this configuration will be ignored.
//#define X_GANGED_LIM_MAX    1
//#define Y_GANGED_LIM_MAX    1
//#define Z_GANGED_LIM_MAX    1
//

/**/
