/*

  serial.h - low level functions for transmitting bytes via the serial port

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

// FTDI breakout: TxD -> RxD, RxD -> TxD

#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <stdint.h>
#include <stdbool.h>

#include "driver.h"
#include "grbl/hal.h"
#include "grbl/stream.h"

#define RX_BUFFER_HWM 900
#define RX_BUFFER_LWM 300

#define BUFCOUNT(head, tail, size) ((head >= tail) ? (head - tail) : (size - tail + head))

// Set SERIAL_DEVICE to -1 for communication over the programming port on Arduino Due
#ifndef SERIAL_DEVICE
#define SERIAL_DEVICE -1
#endif

#if SERIAL_DEVICE > 2
#error Illegal SERIAL_DEVICE selected
#endif

#if MODBUS_ENABLE
#ifndef SERIAL2_DEVICE
#define SERIAL2_DEVICE 1
#endif
#if SERIAL2_DEVICE < 0 || SERIAL2_DEVICE > 2
#error Illegal SERIAL2_DEVICE selected for ModBus
#endif
#endif

#if SERIAL_DEVICE < 0
#define SERIAL_PERIPH UART
#define SERIAL_PORT PIOA
#define SERIAL_PORT_ID ID_PIOA
#define SERIAL_ID ID_UART
#define SERIAL_IRQ UART_IRQn
#define SERIAL_RX_PIN 8
#define SERIAL_TX_PIN 9
#endif

#if SERIAL_DEVICE == 0
#define SERIAL_PERIPH USART0
#define SERIAL_PORT PIOA
#define SERIAL_PORT_ID ID_PIOA
#define SERIAL_ID ID_USART0
#define SERIAL_IRQ USART0_IRQn
#define SERIAL_RX_PIN 10
#define SERIAL_TX_PIN 11
#endif

#if SERIAL_DEVICE == 1
#define SERIAL_PERIPH USART1
#define SERIAL_PORT PIOA
#define SERIAL_PORT_ID ID_PIOA
#define SERIAL_ID ID_USART1
#define SERIAL_IRQ USART1_IRQn
#define SERIAL_RX_PIN 12
#define SERIAL_TX_PIN 13
#endif

#if SERIAL_DEVICE == 2
#define SERIAL_PERIPH USART2
#define SERIAL_PORT PIOB
#define SERIAL_PORT_ID ID_PIOB
#define SERIAL_ID ID_USART2
#define SERIAL_IRQ USART2_IRQn
#define SERIAL_RX_PIN 21
#define SERIAL_TX_PIN 20
#endif

#ifdef SERIAL2_DEVICE

#if SERIAL2_DEVICE == 0
#define SERIAL2_PERIPH USART0
#define SERIAL2_PORT PIOA
#define SERIAL2_PORT_ID ID_PIOA
#define SERIAL2_ID ID_USART0
#define SERIAL2_IRQ USART0_IRQn
#define SERIAL2_RX_PIN 10
#define SERIAL2_TX_PIN 11
#endif

#if SERIAL2_DEVICE == 1
#define SERIAL2_PERIPH USART1
#define SERIAL2_PORT PIOA
#define SERIAL2_PORT_ID ID_PIOA
#define SERIAL2_ID ID_USART1
#define SERIAL2_IRQ USART1_IRQn
#define SERIAL2_RX_PIN 12
#define SERIAL2_TX_PIN 13
#endif

#if SERIAL2_DEVICE == 2
#define SERIAL2_PERIPH USART2
#define SERIAL2_PORT PIOB
#define SERIAL2_PORT_ID ID_PIOB
#define SERIAL2_ID ID_USART2
#define SERIAL2_IRQ USART2_IRQn
#define SERIAL2_RX_PIN 21
#define SERIAL2_TX_PIN 20
#endif

#endif // SERIAL2_DEVICE

#ifdef __cplusplus
extern "C" {
#endif

const io_stream_t *serialInit(uint32_t baud_rate);
void serialRegisterStreams (void);
#ifdef SERIAL2_DEVICE
const io_stream_t *serial2Init(uint32_t baud_rate);
#endif

#ifdef __cplusplus
}
#endif

#endif // _SERIAL_H_

