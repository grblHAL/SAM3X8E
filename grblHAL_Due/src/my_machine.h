/*
  my_machine.h - configuration for Atmel SAM3X8E ARM processor

  Part of grblHAL

  Copyright (c) 2020-2023 Terje Io

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

//#define USB_SERIAL_CDC     1 // Use native USB port for communication.
//#define SAFETY_DOOR_ENABLE 1 // Enable safety door input.
//#define BLUETOOTH_ENABLE   2 // Set to 2 for HC-05 module. Requires and claims one auxillary input pin.
// Spindle selection:
// Up to four specific spindle drivers can be instantiated at a time
// depending on N_SPINDLE and N_SYS_SPINDLE definitions in grbl/config.h.
// If none are specified the default PWM spindle is instantiated.
// Spindle definitions can be found in grbl/spindle_control.h.
// More here https://github.com/grblHAL/Plugins_spindle
//#define SPINDLE0_ENABLE        SPINDLE_HUANYANG1
//#define SPINDLE1_ENABLE        SPINDLE_PWM0
//#define SPINDLE2_ENABLE        SPINDLE_NONE 
//#define SPINDLE2_ENABLE        SPINDLE_NONE
// **********************
//#define MODBUS_ENABLE      1 // Set to 1 for auto direction, 2 for direction signal on auxillary output pin.
//#define PLASMA_ENABLE      1 // Plasma/THC plugin.
//#define OPENPNP_ENABLE     1 // OpenPNP plugin. To be completed.
//#define MPG_ENABLE         1 // Enable MPG interface. Requires serial port and one handshake pin unless
                               // KEYPAD_ENABLE is set to 2 when mode switching is done by the CMD_MPG_MODE_TOGGLE (0x8B)
                               // command character. Set both MPG_ENABLE and KEYPAD_ENABLE to 2 to use a handshake pin anyway.
//#define KEYPAD_ENABLE      2 // Set to 1 for I2C keypad, 2 for other input such as serial data. If KEYPAD_ENABLE is set to 2
                               // and MPG_ENABLE is uncommented then a serial stream is shared with the MPG.
//#define EEPROM_ENABLE     16 // I2C EEPROM/FRAM support. Set to 16 for 2K, 32 for 4K, 64 for 8K, 128 for 16K and 256 for 16K capacity.
//#define EEPROM_IS_FRAM     1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.
#ifndef USB_SERIAL_CDC
#define SERIAL_DEVICE       -1 // Select serial device for output if not using native USB, default is -1, max value is 2
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
