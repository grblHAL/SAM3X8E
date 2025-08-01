/*

  serial.c - Atmel SAM3X8E low level functions for transmitting bytes via the serial port

  Part of grblHAL

  Copyright (c) 2019-2025 Terje Io

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "serial.h"
#include "grbl/protocol.h"

#if SERIAL_PORT < 0
#define SERIAL0_PERIPH UART
#define SERIAL0_PORT PIOA
#define SERIAL0_PORT_ID ID_PIOA
#define SERIAL0_ID ID_UART
#define SERIAL0_IRQ UART_IRQn
#define SERIAL0_RX_PIN 8
#define SERIAL0_TX_PIN 9
#elif SERIAL_PORT == 0
#define SERIAL0_PERIPH USART0
#define SERIAL0_PORT PIOA
#define SERIAL0_PORT_ID ID_PIOA
#define SERIAL0_ID ID_USART0
#define SERIAL0_IRQ USART0_IRQn
#define SERIAL0_RX_PIN 10
#define SERIAL0_TX_PIN 11
#elif SERIAL_PORT == 1
#define SERIAL0_PERIPH USART1
#define SERIAL0_PORT PIOA
#define SERIAL0_PORT_ID ID_PIOA
#define SERIAL0_ID ID_USART1
#define SERIAL0_IRQ USART1_IRQn
#define SERIAL0_RX_PIN 12
#define SERIAL0_TX_PIN 13
#elif SERIAL_PORT == 2
#define SERIAL0_PERIPH USART2
#define SERIAL0_PORT PIOB
#define SERIAL0_PORT_ID ID_PIOB
#define SERIAL0_ID ID_USART2
#define SERIAL0_IRQ USART2_IRQn
#define SERIAL0_RX_PIN 21
#define SERIAL0_TX_PIN 20
#else
#error Illegal SERIAL_PORT selected
#endif

#ifdef SERIAL1_PORT

static const io_stream_t *serial2Init(uint32_t baud_rate);

#if SERIAL1_PORT == 0
#define SERIAL2_PERIPH USART0
#define SERIAL2_PORT PIOA
#define SERIAL2_PORT_ID ID_PIOA
#define SERIAL2_ID ID_USART0
#define SERIAL2_IRQ USART0_IRQn
#define SERIAL2_RX_PIN 10
#define SERIAL2_TX_PIN 11
#elif SERIAL1_PORT == 1
#define SERIAL2_PERIPH USART1
#define SERIAL2_PORT PIOA
#define SERIAL2_PORT_ID ID_PIOA
#define SERIAL2_ID ID_USART1
#define SERIAL2_IRQ USART1_IRQn
#define SERIAL2_RX_PIN 12
#define SERIAL2_TX_PIN 13
#elif SERIAL1_PORT == 2
#define SERIAL2_PERIPH USART2
#define SERIAL2_PORT PIOB
#define SERIAL2_PORT_ID ID_PIOB
#define SERIAL2_ID ID_USART2
#define SERIAL2_IRQ USART2_IRQn
#define SERIAL2_RX_PIN 21
#define SERIAL2_TX_PIN 20
#else
#error Illegal SERIAL1_PORT selected
#endif

#endif // SERIAL1_PORT

static stream_tx_buffer_t txbuf = {0};
static stream_rx_buffer_t rxbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

static const io_stream_t *serialInit(uint32_t baud_rate);

static void SERIAL0_IRQHandler (void);

static io_stream_properties_t serial[] = {
    {
      .type = StreamType_Serial,
      .instance = 0,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serialInit
    },
#ifdef SERIAL1_PORT
    {
      .type = StreamType_Serial,
      .instance = 1,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serial2Init
    }
#endif
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

    stream_register_streams(&streams);
}

//
// Returns number of characters in serial output buffer
//
static uint16_t serialTxCount (void)
{
    uint16_t tail = txbuf.tail;

#if SERIAL_PORT == -1
    return BUFCOUNT(txbuf.head, tail, TX_BUFFER_SIZE) + ((SERIAL0_PERIPH->UART_SR & UART_SR_TXEMPTY) ? 0 : 1);
#else
    return BUFCOUNT(txbuf.head, tail, TX_BUFFER_SIZE) + ((SERIAL0_PERIPH->US_CSR & US_CSR_TXEMPTY) ? 0 : 1);
#endif
}

//
// Returns number of characters in serial input buffer
//
static uint16_t serialRxCount (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of free characters in serial input buffer
//
static uint16_t serialRxFree (void)
{
    unsigned int tail = rxbuf.tail, head = rxbuf.head;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serialRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serialRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Attempt to send a character bypassing buffering
//
static inline bool serialPutCNonBlocking (const char c)
{
    bool ok = false;

#if SERIAL_PORT < 0
    if((ok = (SERIAL0_PERIPH->UART_IMR & US_IMR_TXRDY) == 0 && (SERIAL0_PERIPH->UART_SR & UART_SR_TXEMPTY)))
        SERIAL0_PERIPH->UART_THR = c;
#else
    if((ok = (SERIAL0_PERIPH->US_IMR & US_IMR_TXRDY) == 0 && (SERIAL0_PERIPH->US_CSR & US_CSR_TXEMPTY)))
        SERIAL0_PERIPH->US_THR = c;
#endif
    return ok;
}

//
// Writes a character to the serial output stream
//
static bool serialPutC (const char c)
{
    if(txbuf.head != txbuf.tail || !serialPutCNonBlocking(c)) { // Try to send character without buffering...

        uint_fast16_t next_head = BUFNEXT(txbuf.head, txbuf);   // .. if not, get pointer to next free slot in buffer

        while(txbuf.tail == next_head) {                        // While TX buffer full
      //      SERIAL_MODULE->IE |= EUSCI_A_IE_TXIE;             // Enable TX interrupts???
            if(!hal.stream_blocking_callback())                 // check if blocking for space,
                return false;                                   // exit if not (leaves TX buffer in an inconsistent state)
        }

        txbuf.data[txbuf.head] = c;                             // Add data to buffer
        txbuf.head = next_head;                                 // and update head pointer
#if SERIAL_PORT < 0
        SERIAL0_PERIPH->UART_IER = UART_IER_TXRDY;               // Enable TX interrupts
#else
        SERIAL0_PERIPH->US_IER = US_IER_TXRDY;                   // Enable TX interrupts
#endif
    }

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serialWriteS (const char *s)
{
    char c, *ptr = (char *)s;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

//
// Writes a number of characters from string to the serial output stream followed by EOL, blocks if buffer full
//
static void serialWrite (const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serialPutC(*ptr++);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t serialGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;    // Get buffer pointer

    if(tail == rxbuf.head)
        return -1; // no data available

    char data = rxbuf.data[tail];       // Get next character
    rxbuf.tail = BUFNEXT(tail, rxbuf);  // and update pointer

    return (int16_t)data;
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool serialSetBaudRate (uint32_t baud_rate)
{
#if SERIAL_PORT < 0
    SERIAL0_PERIPH->UART_PTCR = UART_PTCR_RXTDIS|UART_PTCR_TXTDIS;
    SERIAL0_PERIPH->UART_CR = UART_CR_RSTRX|UART_CR_RSTTX|UART_CR_RXDIS|UART_CR_TXDIS;
    SERIAL0_PERIPH->UART_BRGR = (SystemCoreClock / baud_rate) >> 4;
    SERIAL0_PERIPH->UART_IER = UART_IER_RXRDY|UART_IER_OVRE|UART_IER_FRAME;
    SERIAL0_PERIPH->UART_CR = UART_CR_RXEN|UART_CR_TXEN;
#else
    SERIAL0_PERIPH->US_PTCR = US_PTCR_RXTDIS|US_PTCR_TXTDIS;
    SERIAL0_PERIPH->US_CR = US_CR_RSTRX|US_CR_RSTTX|US_CR_RXDIS|US_CR_TXDIS;
    SERIAL0_PERIPH->US_BRGR = (SystemCoreClock / baud_rate) >> 4;
    SERIAL0_PERIPH->US_IER = US_IER_RXRDY|US_IER_OVRE|US_IER_FRAME;
    SERIAL0_PERIPH->US_CR = US_CR_RXEN|US_CR_TXEN;
#endif

    return true;
}

static bool serialSetFormat (serial_format_t format)
{
#if SERIAL_PORT < 0
    SERIAL0_PERIPH->UART_MR &= ~UART_MR_PAR_Msk;
    SERIAL0_PERIPH->UART_MR |= (format.parity == Serial_ParityNone
                                 ? UART_MR_PAR_NO
                                 : (format.parity == Serial_ParityEven ? UART_MR_PAR_EVEN : UART_MR_PAR_ODD));
#else
    SERIAL0_PERIPH->US_MR &= ~US_MR_PAR_Msk;
    SERIAL0_PERIPH->US_MR |= (format.parity == Serial_ParityNone
                               ? US_MR_PAR_NO
                               : (format.parity == Serial_ParityEven ? US_MR_PAR_EVEN : US_MR_PAR_ODD));
#endif

    return true;
}

static bool serialDisable (bool disable)
{
#if SERIAL_PORT < 0
    if(disable)
        SERIAL0_PERIPH->UART_IER &= ~(US_IER_RXRDY|US_IER_OVRE|US_IER_FRAME);
    else
        SERIAL0_PERIPH->UART_IER = US_IER_RXRDY|US_IER_OVRE|US_IER_FRAME;
#else
    if(disable)
        SERIAL0_PERIPH->US_IER &= ~(US_IER_RXRDY|US_IER_OVRE|US_IER_FRAME);
    else
        SERIAL0_PERIPH->US_IER = US_IER_RXRDY|US_IER_OVRE|US_IER_FRAME;
#endif

    return true;
}

static bool serialEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

static const io_stream_t *serialInit (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .is_connected = stream_connected,
        .read = serialGetC,
        .write = serialWriteS,
        .write_n =  serialWrite,
        .write_char = serialPutC,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_count = serialRxCount,
        .get_tx_buffer_count = serialTxCount,
        .get_rx_buffer_free = serialRxFree,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .disable_rx = serialDisable,
        .suspend_read = serialSuspendInput,
        .set_baud_rate = serialSetBaudRate,
        .set_format = serialSetFormat,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(serial[0].flags.claimed)
        return NULL;

    serial[0].flags.claimed = On;

    pmc_enable_periph_clk(SERIAL0_ID);
    pmc_enable_periph_clk(SERIAL0_PORT_ID);

#if SERIAL_PORT < 0
    SERIAL0_PERIPH->UART_MR = UART_MR_PAR_NO;
#else
    SERIAL0_PORT->PIO_WPMR = 0x50494F;
    SERIAL0_PORT->PIO_PDR  = (1<<SERIAL0_RX_PIN)|(1<<SERIAL0_TX_PIN);
    SERIAL0_PORT->PIO_OER  = (1<<SERIAL0_TX_PIN);
    SERIAL0_PORT->PIO_ABSR &= ~(1<<SERIAL0_RX_PIN)|(1<<SERIAL0_TX_PIN);
    SERIAL0_PERIPH->US_MR = US_MR_CHRL_8_BIT|US_MR_PAR_NO; // |US_MR_NBSTOP_2
#endif

    serialSetBaudRate(baud_rate);

    IRQRegister(SERIAL0_IRQ, SERIAL0_IRQHandler);

    NVIC_EnableIRQ(SERIAL0_IRQ);
    NVIC_SetPriority(SERIAL0_IRQ, 1);

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART,
        .port = SERIAL0_PORT,
        .pin = SERIAL0_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Primary UART"
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART,
        .port = SERIAL0_PORT,
        .pin = SERIAL0_RX_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Primary UART"
    };

    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);
    return &stream;
}

//
static void SERIAL0_IRQHandler (void)
{
#if SERIAL_PORT < 0
    if(SERIAL0_PERIPH->UART_SR & UART_SR_RXRDY) {
        char data = (char)SERIAL0_PERIPH->UART_RHR;
#else
    if(SERIAL0_PERIPH->US_CSR & US_CSR_RXRDY) {
        char data = (char)SERIAL0_PERIPH->US_RHR;
#endif
        if(!enqueue_realtime_command(data)) {
            uint_fast16_t next_head = BUFNEXT(rxbuf.head, rxbuf);   // Get and increment buffer pointer
            if(next_head == rxbuf.tail)                             // If buffer full
                rxbuf.overflow = 1;                                 // flag overflow,
            else {
                rxbuf.data[rxbuf.head] = data;                      // else add data to buffer
                rxbuf.head = next_head;                             // and update pointer
            }
        }           
    }
#if SERIAL_PORT < 0
    if(SERIAL0_PERIPH->UART_SR & UART_SR_TXRDY) {
#else
    if(SERIAL0_PERIPH->US_CSR & US_CSR_TXRDY) {
#endif
        uint_fast16_t tail = txbuf.tail;                            // Get buffer pointer
        if(tail != txbuf.head) {
#if SERIAL_PORT < 0
            SERIAL0_PERIPH->UART_THR = (uint32_t)txbuf.data[tail];   // Send a byte from the buffer
#else
            SERIAL0_PERIPH->US_THR = (uint32_t)txbuf.data[tail];     // Send a byte from the buffer
#endif
            txbuf.tail = tail = BUFNEXT(tail, txbuf);               // and increment pointer
        }
        if (tail == txbuf.head)                                     // Turn off TX interrupt
#if SERIAL_PORT < 0
            SERIAL0_PERIPH->UART_IDR = UART_IER_TXRDY;               // when buffer empty
#else
            SERIAL0_PERIPH->US_IDR = US_IER_TXRDY;                   // when buffer empty
#endif
    }
}

#ifdef SERIAL1_PORT

static stream_tx_buffer_t tx2buf = {0};
static stream_rx_buffer_t rx2buf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command2 = protocol_enqueue_realtime_command;

static void SERIAL2_IRQHandler (void);

//
// Returns number of characters in serial output buffer
//
static uint16_t serial2TxCount (void)
{
    uint16_t tail = tx2buf.tail;

    return BUFCOUNT(tx2buf.head, tail, TX_BUFFER_SIZE) + ((SERIAL2_PERIPH->US_CSR & US_CSR_TXEMPTY) ? 0 : 1);
}

//
// Returns number of characters in serial input buffer
//
static uint16_t serial2RxCount (void)
{
    uint16_t tail = rx2buf.tail, head = rx2buf.head;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of free characters in serial input buffer
//
static uint16_t serial2RxFree (void)
{
    unsigned int tail = rx2buf.tail, head = rx2buf.head;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serial2RxFlush (void)
{
    rx2buf.tail = rx2buf.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serial2RxCancel (void)
{
    rx2buf.data[rx2buf.head] = ASCII_CAN;
    rx2buf.tail = rx2buf.head;
    rx2buf.head = BUFNEXT(rx2buf.head, rx2buf);
}

//
// Flushes the serial output buffer
//
static void serial2TxFlush (void)
{
    tx2buf.tail = tx2buf.head;
}


//
// Attempt to send a character bypassing buffering
//
static inline bool serial2PutCNonBlocking (const char c)
{
    bool ok = false;

    if((ok = (SERIAL2_PERIPH->US_IMR & US_IMR_TXRDY) == 0 && (SERIAL2_PERIPH->US_CSR & US_CSR_TXEMPTY)))
        SERIAL2_PERIPH->US_THR = c;

    return ok;
}

//
// Writes a character to the serial output stream
//
static bool serial2PutC (const char c)
{
    if(tx2buf.head != tx2buf.tail || !serial2PutCNonBlocking(c)) {  // Try to send character without buffering...

        uint_fast16_t next_head = BUFNEXT(tx2buf.head, tx2buf);     // .. if not, get pointer to next free slot in buffer

        while(tx2buf.tail == next_head) {                           // While TX buffer full
      //      SERIAL2_MODULE->IE |= EUSCI_A_IE_TXIE;                // Enable TX interrupts???
            if(!hal.stream_blocking_callback())                     // check if blocking for space,
                return false;                                       // exit if not (leaves TX buffer in an inconsistent state)
        }

        tx2buf.data[tx2buf.head] = c;                               // Add data to buffer
        tx2buf.head = next_head;                                    // and update head pointer

        SERIAL2_PERIPH->US_IER = US_IER_TXRDY;                      // Enable TX interrupts
    }

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serial2WriteS (const char *s)
{
    char c, *ptr = (char *)s;

    while((c = *ptr++) != '\0')
        serial2PutC(c);
}

//
// Writes a number of characters from string to the serial output stream followed by EOL, blocks if buffer full
//
static void serial2Write (const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serial2PutC(*ptr++);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t serial2GetC (void)
{
    uint_fast16_t tail = rx2buf.tail;       // Get buffer pointer

    if(tail == rx2buf.head)
        return -1; // no data available

    char data = rx2buf.data[tail];          // Get next character
    rx2buf.tail = BUFNEXT(tail, rx2buf);    // and update pointer

    return (int16_t)data;
}

static bool serial2SetBaudRate (uint32_t baud_rate)
{
#if SERIAL1_PORT < 0
    SERIAL2_PERIPH->UART_PTCR = UART_PTCR_RXTDIS|UART_PTCR_TXTDIS;
    SERIAL2_PERIPH->UART_CR = UART_CR_RSTRX|UART_CR_RSTTX|UART_CR_RXDIS|UART_CR_TXDIS;
    SERIAL2_PERIPH->UART_BRGR = (SystemCoreClock / baud_rate) >> 4;
    SERIAL2_PERIPH->UART_IER = UART_IER_RXRDY|UART_IER_OVRE|UART_IER_FRAME;
    SERIAL2_PERIPH->UART_CR = UART_CR_RXEN|UART_CR_TXEN;
#else
    SERIAL2_PERIPH->US_PTCR = US_PTCR_RXTDIS|US_PTCR_TXTDIS;
    SERIAL2_PERIPH->US_CR = US_CR_RSTRX|US_CR_RSTTX|US_CR_RXDIS|US_CR_TXDIS;
    SERIAL2_PERIPH->US_BRGR = (SystemCoreClock / baud_rate) >> 4;
    SERIAL2_PERIPH->US_IER = US_IER_RXRDY|US_IER_OVRE|US_IER_FRAME;
    SERIAL2_PERIPH->US_CR = US_CR_RXEN|US_CR_TXEN;
#endif

    return true;
}

static bool serial2SetFormat (serial_format_t format)
{
#if SERIAL1_PORT < 0
    SERIAL2_PERIPH->UART_MR &= ~UART_MR_PAR_Msk;
    SERIAL2_PERIPH->UART_MR |= (format.parity == Serial_ParityNone
                                 ? UART_MR_PAR_NO
                                 : (format.parity == Serial_ParityEven ? UART_MR_PAR_EVEN : UART_MR_PAR_ODD));
#else
    SERIAL2_PERIPH->US_MR &= ~US_MR_PAR_Msk;
    SERIAL2_PERIPH->US_MR |= (format.parity == Serial_ParityNone
                               ? US_MR_PAR_NO
                               : (format.parity == Serial_ParityEven ? US_MR_PAR_EVEN : US_MR_PAR_ODD));
#endif

    return true;
}

static bool serial2Disable (bool disable)
{
#if SERIAL1_PORT < 0
    if(disable)
        SERIAL2_PERIPH->UART_IER &= ~(US_IER_RXRDY|US_IER_OVRE|US_IER_FRAME);
    else
        SERIA2L_PERIPH->UART_IER = US_IER_RXRDY|US_IER_OVRE|US_IER_FRAME;
#else
    if(disable)
        SERIAL2_PERIPH->US_IER &= ~(US_IER_RXRDY|US_IER_OVRE|US_IER_FRAME);
    else
        SERIAL2_PERIPH->US_IER = US_IER_RXRDY|US_IER_OVRE|US_IER_FRAME;
#endif
    return true;
}

static bool serial2EnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr serial2SetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command2;

    if(handler)
        enqueue_realtime_command2 = handler;

    return prev;
}

static const io_stream_t *serial2Init (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .instance = 1,
        .is_connected = stream_connected,
        .read = serial2GetC,
        .write = serial2WriteS,
        .write_n =  serial2Write,
        .write_char = serial2PutC,
        .enqueue_rt_command = serial2EnqueueRtCommand,
        .get_rx_buffer_free = serial2RxFree,
        .get_rx_buffer_count = serial2RxCount,
        .get_tx_buffer_count = serial2TxCount,
        .reset_write_buffer = serial2TxFlush,
        .reset_read_buffer = serial2RxFlush,
        .cancel_read_buffer = serial2RxCancel,
        .disable_rx = serial2Disable,
    //    .suspend_read = serial2SuspendInput,
        .set_baud_rate = serial2SetBaudRate,
        .set_format = serial2SetFormat,
        .set_enqueue_rt_handler = serial2SetRtHandler
    };

    if(serial[1].flags.claimed)
        return NULL;

    serial[1].flags.claimed = On;

    pmc_enable_periph_clk(SERIAL2_ID);
    pmc_enable_periph_clk(SERIAL2_PORT_ID);

#if SERIAL1_PORT < 0
    SERIAL0_PERIPH->UART_MR = UART_MR_PAR_NO;
#else
    SERIAL2_PORT->PIO_WPMR = 0x50494F;
    SERIAL2_PORT->PIO_PDR  = (1<<SERIAL2_RX_PIN)|(1<<SERIAL2_TX_PIN);
    SERIAL2_PORT->PIO_OER  = (1<<SERIAL2_TX_PIN);
    SERIAL2_PORT->PIO_ABSR &= ~((1<<SERIAL2_RX_PIN)|(1<<SERIAL2_TX_PIN));
    SERIAL2_PERIPH->US_MR = US_MR_CHRL_8_BIT|US_MR_PAR_NO; // |US_MR_NBSTOP_2
#endif

    serial2SetBaudRate(baud_rate);

    IRQRegister(SERIAL2_IRQ, SERIAL2_IRQHandler);

    NVIC_EnableIRQ(SERIAL2_IRQ);
    NVIC_SetPriority(SERIAL2_IRQ, 1);

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART2,
        .port = SERIAL2_PORT,
        .pin = SERIAL2_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Secondary UART"
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART2,
        .port = SERIAL2_PORT,
        .pin = SERIAL2_RX_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Secondary UART"
    };

    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);

    return &stream;
}

//
static void SERIAL2_IRQHandler (void)
{
    if(SERIAL2_PERIPH->US_CSR & US_CSR_RXRDY) {
        char data = (char)SERIAL2_PERIPH->US_RHR;
        if(!enqueue_realtime_command2(data)) {
            uint_fast16_t next_head = BUFNEXT(rx2buf.head, rx2buf); // Get and increment buffer pointer
            if(next_head == rx2buf.tail)                            // If buffer full
                rx2buf.overflow = 1;                                // flag overflow,
            else {
                rx2buf.data[rx2buf.head] = data;                    // else add data to buffer
                rx2buf.head = next_head;                            // and update pointer
            }
        }
    }

    if(SERIAL2_PERIPH->US_CSR & US_CSR_TXRDY) {
        uint_fast16_t tail = tx2buf.tail;                            // Get buffer pointer
        if(tail != tx2buf.head) {
            SERIAL2_PERIPH->US_THR = (uint32_t)tx2buf.data[tail];   // Send a byte from the buffer
            tx2buf.tail = tail = BUFNEXT(tail, tx2buf);             // and increment pointer
        }
        if(tail == tx2buf.head)                                     // Turn off TX interrupt
            SERIAL2_PERIPH->US_IDR = US_IER_TXRDY;                  // when buffer empty
    }
}

#endif // SERIAL1_PORT
