/*
  ioports.c - driver code for Atmel SAM3X8E ARM processor

  Part of grblHAL

  Copyright (c) 2021 Terje Io

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

#include "driver.h"

#include <string.h>

#include "grbl/protocol.h"
#include "grbl/settings.h"

static uint8_t n_out, n_in;
static volatile uint32_t event_bits;
static volatile bool spin_lock = false;
static input_signal_t *aux_in;
static output_signal_t *aux_out;
static ioport_bus_t out = {0};
static char input_ports[56] = "", output_ports[56] = "";

static void aux_settings_load (void);
static status_code_t aux_set_invert_out (setting_id_t id, uint_fast16_t int_value);
static uint32_t aux_get_invert_out (setting_id_t setting);
static bool is_setting_available (const setting_detail_t *setting);

static const setting_group_detail_t aux_groups[] = {
    { Group_Root, Group_AuxPorts, "Aux ports"}
};

static const setting_detail_t aux_settings[] = {
    { Settings_IoPort_InvertIn, Group_AuxPorts, "Invert I/O Port inputs", NULL, Format_Bitfield, input_ports, NULL, NULL, Setting_NonCore, &settings.ioport.invert_in.mask, NULL, is_setting_available },
//    { Settings_IoPort_Pullup_Disable, Group_AuxPorts, "I/O Port inputs pullup disable", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL },
    { Settings_IoPort_InvertOut, Group_AuxPorts, "Invert I/O Port outputs", NULL, Format_Bitfield, output_ports, NULL, NULL, Setting_NonCoreFn, aux_set_invert_out, aux_get_invert_out, is_setting_available },
//    { Settings_IoPort_OD_Enable, Group_AuxPorts, "I/O Port outputs as open drain", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL }
};

static void aux_settings_load (void);

static setting_details_t details = {
    .groups = aux_groups,
    .n_groups = sizeof(aux_groups) / sizeof(setting_group_detail_t),
    .settings = aux_settings,
    .n_settings = sizeof(aux_settings) / sizeof(setting_detail_t),
    .load = aux_settings_load,
    .save = settings_write_global
};

static bool is_setting_available (const setting_detail_t *setting)
{
    bool available = false;

    switch(setting->id) {

        case Settings_IoPort_InvertIn:
        case Settings_IoPort_Pullup_Disable:
            available = n_in > 0;
            break;

        case Settings_IoPort_InvertOut:
        case Settings_IoPort_OD_Enable:
            available = n_out > 0;
            break;

        default:
            break;
    }

    return available;
}

static setting_details_t *onReportSettings (void)
{
    return &details;
}

static status_code_t aux_set_invert_out (setting_id_t id, uint_fast16_t value)
{
    ioport_bus_t invert;
    invert.mask = (uint8_t)value & out.mask;

    if(invert.mask != settings.ioport.invert_out.mask) {
        uint_fast8_t port = n_out;
        do {
            port--;
            if(((settings.ioport.invert_out.mask >> port) & 0x01) != ((invert.mask >> port) & 0x01))
                DIGITAL_OUT(aux_out[port].port, aux_out[port].pin, !DIGITAL_IN(aux_out[port].port, aux_out[port].pin));
        } while(port);

        settings.ioport.invert_out.mask = invert.mask;
    }

    return Status_OK;
}

static uint32_t aux_get_invert_out (setting_id_t setting)
{
    return settings.ioport.invert_out.mask;
}

static void aux_settings_load (void)
{
//    aux_set_pullup();

    uint_fast8_t idx = n_out;
    do {
        idx--;
        DIGITAL_OUT(aux_out[idx].port, aux_out[idx].pin, (settings.ioport.invert_out.mask >> idx) & 0x01);
    } while(idx);
}

/*
static void aux_set_pullup (void)
{
    GPIO_InitTypeDef GPIO_Init = {0};

    GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_Init.Mode = GPIO_MODE_INPUT;

    GPIO_Init.Pin = AUXINPUT0_PIN;
    GPIO_Init.Pull = settings.ioport.pullup_disable_in.bit0 ? GPIO_PULLDOWN : GPIO_PULLUP;
    HAL_GPIO_Init(AUXINPUT0_PORT, &GPIO_Init);

    GPIO_Init.Pin = AUXINPUT1_PIN;
    GPIO_Init.Pull = settings.ioport.pullup_disable_in.bit1 ? GPIO_PULLDOWN : GPIO_PULLUP;
    HAL_GPIO_Init(AUXINPUT1_PORT, &GPIO_Init);
}
*/

static void digital_out (uint8_t port, bool on)
{
    if(port < n_out)
        DIGITAL_OUT(aux_out[port].port, aux_out[port].pin, ((settings.ioport.invert_out.mask >> port) & 0x01) ? !on : on);
}

inline static __attribute__((always_inline)) int32_t get_input (const input_signal_t *input, bool invert, wait_mode_t wait_mode, float timeout)
{
    if(wait_mode == WaitMode_Immediate)
        return DIGITAL_IN(input->port, input->pin) ^ invert;

    int32_t value = -1;
    uint_fast16_t delay = (uint_fast16_t)ceilf((1000.0f / 50.0f) * timeout) + 1;

    if(wait_mode == WaitMode_Rise || wait_mode == WaitMode_Fall) {

        pin_irq_mode_t irq_mode = wait_mode == WaitMode_Rise ? IRQ_Mode_Rising : IRQ_Mode_Falling;

        if(input->cap.irq_mode & irq_mode) {

            event_bits &= ~input->bit;
            PIO_EnableInterrupt(input, irq_mode);

            do {
                if(event_bits & input->bit) {
                    value = DIGITAL_IN(input->port, input->pin) ^ invert;
                    break;
                }
                if(delay) {
                    protocol_execute_realtime();
                    hal.delay_ms(50, NULL);
                } else
                    break;
            } while(--delay && !sys.abort);

            PIO_EnableInterrupt(input, IRQ_Mode_None);    // Restore pin interrupt status
        }

    } else {

        bool wait_for = wait_mode != WaitMode_Low;

        do {
            if((DIGITAL_IN(input->port, input->pin) ^ invert) == wait_for) {
                value = DIGITAL_IN(input->port, input->pin) ^ invert;
                break;
            }
            if(delay) {
                protocol_execute_realtime();
                hal.delay_ms(50, NULL);
            } else
                break;
        } while(--delay && !sys.abort);
    }

    return value;
}

void ioports_event (input_signal_t *input)
{
    spin_lock = true;
    event_bits |= input->bit;

    uint_fast8_t idx = n_in;
    do {
        idx--;
        if(aux_in[idx].port == input->port && aux_in[idx].pin == input->pin && aux_in[idx].interrupt_callback)
            aux_in[idx].interrupt_callback(idx, DIGITAL_IN(input->port, input->pin));
    } while(idx);

    spin_lock = false;
}

static int32_t wait_on_input (bool digital, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    if(digital && port < n_in)
        value = get_input(&aux_in[port], (settings.ioport.invert_in.mask << port) & 0x01, wait_mode, timeout);

    return value;
}

static bool register_interrupt_handler (uint8_t port, pin_irq_mode_t irq_mode, ioport_interrupt_callback_ptr interrupt_callback)
{
    bool ok;

    if((ok = port < n_in && aux_in[port].cap.irq_mode != IRQ_Mode_None)) {

        input_signal_t *input = &aux_in[port];

        if(irq_mode != IRQ_Mode_None && (ok = interrupt_callback != NULL)) {
            input->irq_mode = irq_mode;
            input->interrupt_callback = interrupt_callback;
            PIO_EnableInterrupt(input, irq_mode);
        }

        if(irq_mode == IRQ_Mode_None || !ok) {
            while(spin_lock);
            PIO_EnableInterrupt(input, IRQ_Mode_None);
            input->irq_mode = IRQ_Mode_None;
            input->interrupt_callback = NULL;
        }
    }

    return ok;
}

void ioports_init (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs)
{
    aux_in = aux_inputs->pins.inputs;
    aux_out = aux_outputs->pins.outputs;

    if((hal.port.num_digital_in = n_in = aux_inputs->n_pins)) {
        hal.port.wait_on_input = wait_on_input;
        hal.port.register_interrupt_handler = register_interrupt_handler;
    }

    if((hal.port.num_digital_out = n_out = aux_outputs->n_pins))
        hal.port.digital_out = digital_out;

    details.on_get_settings = grbl.on_get_settings;
    grbl.on_get_settings = onReportSettings;

    uint32_t i;

    for(i = 0; i < min(hal.port.num_digital_in, 8); i++) {
        strcat(input_ports, i == 0 ? "Port " : ",Port ");
        strcat(input_ports, uitoa(i));
    }

    for(i = 0; i < min(hal.port.num_digital_out, 8) ; i++) {
        out.mask = (out.mask << 1) + 1;
        strcat(output_ports, i == 0 ? "Port " : ",Port ");
        strcat(output_ports, uitoa(i));
    }
}
