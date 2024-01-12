/*
  ioports.c - driver code for Atmel SAM3X8E ARM processor

  Part of grblHAL

  Copyright (c) 2021-2024 Terje Io

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
#include <stdlib.h>

#include "grbl/protocol.h"

static io_ports_data_t digital;
static input_signal_t *aux_in;
static output_signal_t *aux_out;
static ioport_bus_t invert_digital_out;
static volatile uint32_t event_bits;
static volatile bool spin_lock = false;

static uint8_t n_analog_in = 1;

static void digital_out (uint8_t port, bool on)
{
    if(port < digital.out.n_ports) {
        port = ioports_map(digital.out, port);
        DIGITAL_OUT(aux_out[port].port, aux_out[port].pin, ((settings.ioport.invert_out.mask >> port) & 0x01) ? !on : on);
    }
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

    uint_fast8_t port = digital.in.n_ports;
    do {
        port--;
        if(aux_in[port].port == input->port && aux_in[port].pin == input->pin && aux_in[port].interrupt_callback)
            aux_in[port].interrupt_callback(ioports_map_reverse(&digital.in, port), DIGITAL_IN(input->port, input->pin));
    } while(port);

    spin_lock = false;
}

static int32_t wait_on_input (io_port_type_t type, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    if(type == Port_Digital && port < digital.in.n_ports) {
        port = ioports_map(digital.in, port);
        value = get_input(&aux_in[port], (settings.ioport.invert_in.mask >> port) & 0x01, wait_mode, timeout);
    } else if(port < n_analog_in)
        value = analogRead(10);

    return value;
}

static bool register_interrupt_handler (uint8_t port, pin_irq_mode_t irq_mode, ioport_interrupt_callback_ptr interrupt_callback)
{
    bool ok;

    port = ioports_map(digital.in, port);

    if((ok = port < digital.in.n_ports && aux_in[port].cap.irq_mode != IRQ_Mode_None)) {

        input_signal_t *input = &aux_in[port];

        if((ok = (irq_mode & aux_in[port].cap.irq_mode) == irq_mode && interrupt_callback != NULL)) {
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

static xbar_t *get_pin_info (io_port_type_t type, io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;
    xbar_t *info = NULL;

    if(type == Port_Digital) {

        memset(&pin, 0, sizeof(xbar_t));

        if(dir == Port_Input && port < digital.in.n_ports) {
            port = ioports_map(digital.in, port);
            pin.mode.input = On;
            pin.mode.irq_mode = aux_in[port].irq_mode;
            pin.mode.can_remap = !aux_in[port].cap.remapped;
            pin.cap = aux_in[port].cap;
            pin.function = aux_in[port].id;
            pin.group = aux_in[port].group;
            pin.pin = aux_in[port].pin;
            pin.bit = aux_in[port].bit;
            pin.port = (void *)aux_in[port].port;
            pin.description = aux_in[port].description;
            info = &pin;
        }

        if(dir == Port_Output && port < digital.out.n_ports) {
            port = ioports_map(digital.out, port);
            pin.mode = aux_out[port].mode;
            pin.mode.output = On;
            pin.function = aux_out[port].id;
            pin.group = aux_out[port].group;
            pin.pin = aux_out[port].pin;
            pin.bit = 1 << aux_out[port].pin;
            pin.port = (void *)aux_out[port].port;
            pin.description = aux_out[port].description;
            info = &pin;
        }
    }

    return info;
}

static void set_pin_description (io_port_type_t type, io_port_direction_t dir, uint8_t port, const char *s)
{
    if(type == Port_Digital) {
        if(dir == Port_Input && port < digital.in.n_ports)
            aux_in[ioports_map(digital.in, port)].description = s;
        if(dir == Port_Output && port < digital.out.n_ports)
            aux_out[ioports_map(digital.out, port)].description = s;
    }
}

static bool claim (io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description)
{
    bool ok = false;

    if(type == Port_Digital) {

        if(dir == Port_Input) {

            if((ok = digital.in.map && *port < digital.in.n_ports && !aux_in[*port].cap.claimed)) {

                uint8_t i;

                hal.port.num_digital_in--;

                for(i = ioports_map_reverse(&digital.in, *port); i < hal.port.num_digital_in ; i++) {
                    digital.in.map[i] = digital.in.map[i + 1];
                    aux_in[digital.in.map[i]].description = iports_get_pnum(digital, i);
                }

                aux_in[*port].cap.claimed = On;
                aux_in[*port].description = description;

                digital.in.map[hal.port.num_digital_in] = *port;
                *port = hal.port.num_digital_in;
            }

        } else if((ok = digital.out.map && *port < digital.out.n_ports && !aux_out[*port].mode.claimed)) {

            uint8_t i;

            hal.port.num_digital_out--;

            for(i = ioports_map_reverse(&digital.out, *port); i < hal.port.num_digital_out; i++) {
                digital.out.map[i] = digital.out.map[i + 1];
                aux_out[digital.out.map[i]].description = iports_get_pnum(digital, i);
            }

            aux_out[*port].mode.claimed = On;
            aux_out[*port].description = description;

            digital.out.map[hal.port.num_digital_out] = *port;
            *port = hal.port.num_digital_out;
        }
    }

    return ok;
}

bool swap_pins (io_port_type_t type, io_port_direction_t dir, uint8_t port_a, uint8_t port_b)
{
    bool ok = port_a == port_b;

    if(!ok && type == Port_Digital) {

        if((ok = dir == Port_Input && port_a < digital.in.n_ports && port_b < digital.in.n_ports &&
                   aux_in[port_a].interrupt_callback == NULL &&
                    aux_in[port_b].interrupt_callback == NULL)) {

            input_signal_t tmp;

            memcpy(&tmp, &aux_in[port_a], sizeof(input_signal_t));
            memcpy(&aux_in[port_a], &aux_in[port_b], sizeof(input_signal_t));
            aux_in[port_a].description = tmp.description;
            tmp.description = aux_in[port_b].description;
            memcpy(&aux_in[port_b], &tmp, sizeof(input_signal_t));
        }

        if((ok = dir == Port_Output && port_a < digital.out.n_ports && port_b < digital.out.n_ports)) {

            output_signal_t tmp;

            memcpy(&tmp, &aux_out[port_a], sizeof(output_signal_t));
            memcpy(&aux_out[port_a], &aux_out[port_b], sizeof(output_signal_t));
            aux_out[port_a].description = tmp.description;
            tmp.description = aux_out[port_b].description;
            memcpy(&aux_out[port_b], &tmp, sizeof(output_signal_t));
        }
    }

    return ok;
}

static void on_setting_changed (setting_id_t id)
{
    bool write = false;
    uint_fast8_t port;

    switch(id) {

        case Settings_IoPort_InvertIn:
            port = digital.in.n_ports;
            do {
                if(aux_in[--port].aux_ctrl) {
                    write = true;
                    if(settings.ioport.invert_in.mask & (1 << port))
                        settings.control_invert.mask |= aux_in[port].aux_ctrl->cap.mask;
                    else
                        settings.control_invert.mask &= ~aux_in[port].aux_ctrl->cap.mask;
                }
            } while(port);
            break;

        case Settings_IoPort_InvertOut:
            if(invert_digital_out.mask != settings.ioport.invert_out.mask) {
                port = digital.out.n_ports;
                do {
                    port--;
                    if(((settings.ioport.invert_out.mask >> port) & 0x01) != ((invert_digital_out.mask >> port) & 0x01))
                        DIGITAL_OUT(aux_out[port].port, aux_out[port].pin, !DIGITAL_IN(aux_out[port].port, aux_out[port].pin));
                } while(port);

                invert_digital_out = settings.ioport.invert_out;
            }
            break;

        case Setting_ControlInvertMask:
            port = digital.in.n_ports;
            do {
                if(aux_in[--port].aux_ctrl) {
                    write = true;
                    if(settings.control_invert.mask & aux_in[port].aux_ctrl->cap.mask)
                        settings.ioport.invert_in.mask |= (1 << port);
                    else
                        settings.ioport.invert_in.mask &= ~(1 << port);
                }
            } while(port);
            break;

        default:
            break;
    }

    if(write)
        settings_write_global();
}

static void on_settings_loaded (void)
{
    bool write = false;
    uint_fast8_t port = digital.out.n_ports;

    invert_digital_out = settings.ioport.invert_out;

    if(digital.out.n_ports) do {
        port--;
        DIGITAL_OUT(aux_out[port].port, aux_out[port].pin, (settings.ioport.invert_out.mask >> port) & 0x01);
    } while(port);

    port = digital.in.n_ports;
    do {
        if(aux_in[--port].aux_ctrl &&
            !!(settings.control_invert.mask & aux_in[port].aux_ctrl->cap.mask) !=
             !!(settings.ioport.invert_in.mask & (1 << port))) {
            write = true;
            if(settings.control_invert.mask & aux_in[port].aux_ctrl->cap.mask)
                settings.ioport.invert_in.mask |= (1 << port);
            else
                settings.ioport.invert_in.mask &= ~(1 << port);
        }
    } while(port);

    if(write)
        settings_write_global();
}

void ioports_init (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs)
{
    aux_in = aux_inputs->pins.inputs;
    aux_out = aux_outputs->pins.outputs;

    if((hal.port.num_analog_in = n_analog_in)) {
        analogReadResolution(12);
        hal.port.wait_on_input = wait_on_input;
    }

    hal.port.set_pin_description = set_pin_description;

    if(ioports_add(&digital, Port_Digital, aux_inputs->n_pins, aux_outputs->n_pins))  {

        if(digital.in.n_ports) {
            hal.port.wait_on_input = wait_on_input;
            hal.port.register_interrupt_handler = register_interrupt_handler;
        }

        if(digital.out.n_ports)
            hal.port.digital_out = digital_out;

        hal.port.claim = claim;
        hal.port.swap_pins = swap_pins;
        hal.port.get_pin_info = get_pin_info;

        ioports_add_settings(on_settings_loaded, on_setting_changed);

    } else
        hal.port.set_pin_description = NULL;
}
