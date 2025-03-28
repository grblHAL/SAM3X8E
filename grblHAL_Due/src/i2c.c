/*
  i2c.c - I2C interface

  Driver code for Atmel SAM3X8E ARM processor

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
  along with Grbl. If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if I2C_ENABLE

#include <Arduino.h>

#include "sam.h"
#include "variant.h"
#include "wiring_private.h"

#include "i2c.h"
#include "serial.h"

#define i2cIsBusy (!(i2c.state == I2CState_Idle || i2c.state == I2CState_Error) || !(I2C_PERIPH->TWI_SR & TWI_SR_TXCOMP))

typedef enum {
    I2CState_Idle = 0,
    I2CState_SendNext,
    I2CState_SendLast,
    I2CState_AwaitACK,
    I2CState_AwaitCompletion,
    I2CState_ReceiveNext,
    I2CState_ReceiveNextToLast,
    I2CState_ReceiveLast,
    I2CState_Error
} i2c_state_t;

typedef struct {
    volatile i2c_state_t state;
    bool nak;
    uint8_t addr;
    uint16_t count;
    uint8_t *data;
    uint8_t regaddr[2];
    keycode_callback_ptr keycode_callback;
    uint8_t buffer[8];
} i2c_trans_t;

static i2c_trans_t i2c = {0};

static void I2C_interrupt_handler (void);

i2c_cap_t i2c_start (void)
{
    static i2c_cap_t cap = {};

    if(!cap.started) {

        pmc_enable_periph_clk(I2C_ID);
        pmc_enable_periph_clk(I2C_PERIPH == TWI0 ? ID_PIOA : ID_PIOB);

        I2C_PORT->PIO_PDR = I2C_SDA_BIT|I2C_SCL_BIT;
        I2C_PORT->PIO_ABSR &= ~(I2C_SDA_BIT|I2C_SCL_BIT);

        I2C_PERIPH->TWI_CR = TWI_CR_SWRST;
        I2C_PERIPH->TWI_RHR;

        hal.delay_ms(10, NULL);        

        I2C_PERIPH->TWI_CR = TWI_CR_SVDIS|TWI_CR_MSEN;

        TWI_SetClock(I2C_PERIPH, I2C_CLOCK, SystemCoreClock);

        IRQRegister(I2C_IRQ, I2C_interrupt_handler);

        NVIC_SetPriority(I2C_IRQ, 0);
        NVIC_EnableIRQ(I2C_IRQ);

        static const periph_pin_t scl = {
            .function = Output_SCK,
            .group = PinGroup_I2C,
            .port = I2C_PORT,
            .pin = I2C_SCL_PIN,
            .mode = { .mask = PINMODE_OD }
        };

        static const periph_pin_t sda = {
            .function = Bidirectional_SDA,
            .group = PinGroup_I2C,
            .port = I2C_PORT,
            .pin = I2C_SDA_PIN,
            .mode = { .mask = PINMODE_OD }
        };

        hal.periph_port.register_pin(&scl);
        hal.periph_port.register_pin(&sda);

        cap.started = cap.tx_non_blocking = On;
    }

    return cap;
}

bool i2c_probe (i2c_address_t i2c_address)
{
    i2c.nak = false;
    i2c_send(i2c_address, NULL, 0, true);

    return !i2c.nak;
}

// get bytes (max 8), waits for result
bool i2c_receive (i2c_address_t i2cAddr, uint8_t *buf, size_t bytes, bool block)
{
    i2c.data  = buf ? buf : i2c.buffer;
    i2c.count = bytes;
    i2c.state = bytes == 1 ? I2CState_ReceiveLast : (bytes == 2 ? I2CState_ReceiveNextToLast : I2CState_ReceiveNext);

    // Send start and address
    I2C_PERIPH->TWI_MMR = TWI_MMR_DADR(i2cAddr)|TWI_MMR_MREAD;
    I2C_PERIPH->TWI_CR = bytes == 1 ? TWI_CR_START : TWI_CR_START|TWI_CR_STOP; // Start condition + stop condition if reading one byte
    I2C_PERIPH->TWI_IER = TWI_IER_TXRDY|TWI_IER_RXRDY;

    if(block)
        while(i2cIsBusy);

    return true;
}

bool i2c_send (i2c_address_t i2cAddr, uint8_t *buf, size_t bytes, bool block)
{
    i2c.count = bytes ? bytes - 1 : 0;
    i2c.data  = buf ? buf : i2c.buffer;
    i2c.state = bytes == 0 ? I2CState_AwaitACK : (i2c.count == 0 ? I2CState_AwaitCompletion : (i2c.count == 1 ? I2CState_SendLast : I2CState_SendNext));

    I2C_PERIPH->TWI_MMR = TWI_MMR_DADR(i2cAddr);
    I2C_PERIPH->TWI_THR = *i2c.data++;
    I2C_PERIPH->TWI_IER = TWI_IER_TXRDY|TWI_IER_NACK;

    if(block)
        while(i2cIsBusy);

    return true;
}

static uint8_t *i2c_readregister (i2c_address_t i2cAddr, uint8_t *buf, uint8_t abytes, uint16_t bytes, bool block)
{
    while(i2cIsBusy);

    i2c.count = bytes;
    i2c.data  = buf ? buf : i2c.buffer;
    i2c.state = bytes == 1 ? I2CState_ReceiveLast : (bytes == 2 ? I2CState_ReceiveNextToLast : I2CState_ReceiveNext);

    if(abytes == 1) {
        I2C_PERIPH->TWI_MMR = TWI_MMR_DADR(i2cAddr)|TWI_MMR_MREAD|TWI_MMR_IADRSZ_1_BYTE;
        I2C_PERIPH->TWI_IADR = i2c.regaddr[0];
    } else {
        I2C_PERIPH->TWI_MMR = TWI_MMR_DADR(i2cAddr)|TWI_MMR_MREAD|TWI_MMR_IADRSZ_2_BYTE;
        I2C_PERIPH->TWI_IADR = (i2c.regaddr[1] << 8) | i2c.regaddr[0];
    }
    I2C_PERIPH->TWI_CR = bytes == 1 ? TWI_CR_START|TWI_CR_STOP : TWI_CR_START; // Start condition + stop condition if reading one byte
    I2C_PERIPH->TWI_IER = TWI_IER_RXRDY;

    if(block)
        while(i2cIsBusy);

    return i2c.buffer;
}

bool i2c_transfer (i2c_transfer_t *transfer, bool read)
{
    static uint8_t txbuf[66];

    bool ok;

    while(i2cIsBusy);

    if((ok = read)) {
        if(transfer->word_addr_bytes == 1)
            i2c.regaddr[0] = transfer->word_addr;
        else {
            i2c.regaddr[0] = transfer->word_addr & 0xFF;
            i2c.regaddr[1] = transfer->word_addr >> 8;
        }
        i2c_readregister(transfer->address, transfer->data, transfer->word_addr_bytes, transfer->count, true);
    } else if((ok = transfer->count <= 64)) {
        memcpy(&txbuf[transfer->word_addr_bytes], transfer->data, transfer->count);
        if(transfer->word_addr_bytes == 1)
            txbuf[0] = transfer->word_addr;
        else {
            txbuf[0] = transfer->word_addr >> 8;
            txbuf[1] = transfer->word_addr & 0xFF;
        }
        i2c_send(transfer->address, txbuf, transfer->count + transfer->word_addr_bytes, !transfer->no_block);
    }

    return ok;
}

bool i2c_get_keycode (i2c_address_t i2cAddr, keycode_callback_ptr callback)
{
    while(i2cIsBusy);

    i2c.keycode_callback = callback;

    return i2c_receive(i2cAddr, NULL, 1, false);
}

#if TRINAMIC_ENABLE == 2130 && TRINAMIC_I2C

static TMC2130_status_t I2C_TMC_ReadRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    uint8_t *res, i2creg;
    TMC2130_status_t status = {0};
;
    if((i2creg = TMCI2C_GetMapAddress((uint8_t)(driver ? (uint32_t)driver->axis : 0), reg->addr).value) == 0xFF)
        return status; // unsupported register

    while(i2cIsBusy);

    i2c.buffer[0] = i2creg;
    i2c.buffer[1] = 0;
    i2c.buffer[2] = 0;
    i2c.buffer[3] = 0;
    i2c.buffer[4] = 0;

    res = i2c_readregister(I2C_ADR_I2CBRIDGE, NULL, 5, true);

    status.value = (uint8_t)*res++;
    reg->payload.value = ((uint8_t)*res++ << 24);
    reg->payload.value |= ((uint8_t)*res++ << 16);
    reg->payload.value |= ((uint8_t)*res++ << 8);
    reg->payload.value |= (uint8_t)*res++;

    return status;
}

static TMC2130_status_t I2C_TMC_WriteRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    TMC2130_status_t status = {0};

    while(i2cIsBusy);

    reg->addr.write = 1;
    i2c.buffer[0] = TMCI2C_GetMapAddress((uint8_t)(driver ? (uint32_t)driver->axis : 0), reg->addr).value;
    reg->addr.write = 0;

    if(i2c.buffer[0] == 0xFF)
        return status; // unsupported register

    i2c.buffer[1] = (reg->payload.value >> 24) & 0xFF;
    i2c.buffer[2] = (reg->payload.value >> 16) & 0xFF;
    i2c.buffer[3] = (reg->payload.value >> 8) & 0xFF;
    i2c.buffer[4] = reg->payload.value & 0xFF;

    i2c_send(I2C_ADR_I2CBRIDGE, NULL, 5, true);

    return status;
}

void I2C_DriverInit (TMC_io_driver_t *driver)
{
    i2c_start();
    driver->WriteRegister = I2C_TMC_WriteRegister;
    driver->ReadRegister = I2C_TMC_ReadRegister;
}

#endif

static void I2C_interrupt_handler (void)
{
    uint8_t ifg = I2C_PERIPH->TWI_SR;

    if(ifg & TWI_SR_ARBLST) {
        I2C_PERIPH->TWI_CR = TWI_CR_STOP; // Stop condition
        i2c.state = I2CState_Error;
    }

    if(ifg & TWI_SR_NACK)
        i2c.state = I2CState_Error;

//    hal.stream.write(uitoa(ifg));

    switch(i2c.state) {

        case I2CState_Idle:
        case I2CState_Error:
            I2C_PERIPH->TWI_IDR = TWI_IDR_TXRDY|TWI_IDR_RXRDY|TWI_IDR_NACK;
            break;

        case I2CState_SendNext:
            I2C_PERIPH->TWI_THR = *i2c.data++;
            if(--i2c.count == 1)
                i2c.state = I2CState_SendLast;
            break;

        case I2CState_SendLast:
            I2C_PERIPH->TWI_THR = *i2c.data;
            I2C_PERIPH->TWI_CR = TWI_CR_STOP; // Stop condition
            i2c.state = I2CState_AwaitCompletion;
            break;

        case I2CState_AwaitACK:
            i2c.nak = !!(ifg & TWI_SR_TXCOMP);
            I2C_PERIPH->TWI_CR = TWI_CR_STOP; // Stop condition
            I2C_PERIPH->TWI_IDR = TWI_IDR_TXRDY|TWI_IDR_RXRDY|TWI_IDR_NACK;
            i2c.state = I2CState_Idle;
            break;

        case I2CState_AwaitCompletion:
            I2C_PERIPH->TWI_IDR = TWI_IDR_TXRDY|TWI_IDR_RXRDY|TWI_IDR_NACK;
            i2c.count = 0;
            i2c.state = I2CState_Idle;
            break;

        case I2CState_ReceiveNext:
            *i2c.data++ = I2C_PERIPH->TWI_RHR;
            if(--i2c.count == 2)
                i2c.state = I2CState_ReceiveNextToLast;
            break;

        case I2CState_ReceiveNextToLast:
            *i2c.data++ = I2C_PERIPH->TWI_RHR;
            I2C_PERIPH->TWI_CR = TWI_CR_STOP;
            i2c.count--;
            i2c.state = I2CState_ReceiveLast;
            break;

        case I2CState_ReceiveLast:
            I2C_PERIPH->TWI_IDR = TWI_IDR_TXRDY|TWI_IDR_RXRDY|TWI_IDR_NACK;
            *i2c.data = I2C_PERIPH->TWI_RHR;
            i2c.count = 0;
            i2c.state = I2CState_Idle;
          #if KEYPAD_ENABLE == 1
            if(i2c.keycode_callback) {
                i2c.keycode_callback(*i2c.data);
                i2c.keycode_callback = NULL;
            }
          #endif
            break;
    }
}

#endif
