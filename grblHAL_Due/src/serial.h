/*

  serial.h - low level functions for transmitting bytes via the serial port

  Part of grblHAL

  Copyright (c) 2019-2025 Terje Io

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

// FTDI breakout: TxD -> RxD, RxD -> TxD

#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <stdint.h>
#include <stdbool.h>

#include "driver.h"
#include "grbl/hal.h"
#include "grbl/stream.h"

// Serial ports
// -1 (Default)  0 (PA8)  = RX,   1 (PA9)  = TX (same as USB programming port)
// 0            19 (PA10) = RX,  18 (PA11) = TX
// 1            17 (PA12) = RX,  16 (PA13) = TX
// 2            52 (PB21) = RX, A11 (PB21) = TX

// Set SERIAL_PORT to -1 for communication over the programming port on Arduino Due

#ifdef __cplusplus
extern "C" {
#endif

void serialRegisterStreams (void);

#ifdef __cplusplus
}
#endif

#endif // _SERIAL_H_
