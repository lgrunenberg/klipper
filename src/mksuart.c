// Commands for sending messages to a mks2208 via its single wire UART
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h>     // memcpy
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h"  // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h"    // oid_alloc
#include "command.h"    // DECL_COMMAND
#include "sched.h"      // DECL_SHUTDOWN

struct mksuart_s
{
    struct timer timer;
    struct gpio_out tx_pin;
    struct gpio_in rx_pin;
    uint8_t flags;
    uint8_t pos, read_count, write_count;
    uint32_t cfg_bit_time, bit_time;
    uint8_t data[10];
};

enum
{
    MU_LINE_HIGH = 1 << 0,
    MU_ACTIVE = 1 << 1,
    MU_READ_SYNC = 1 << 2,
    MU_REPORT = 1 << 3,
    MU_PULLUP = 1 << 4,
    MU_SINGLE_WIRE = 1 << 5,
    MU_START = 1 << 6,
    MU_STOP = 1 << 7
};

static struct task_wake mksuart_wake;

// Restore uart line to normal "idle" mode
static void
mksuart_reset_line(struct mksuart_s *t)
{
    if (t->flags & MU_SINGLE_WIRE)
        gpio_out_reset(t->tx_pin, 1);
    else
        gpio_out_write(t->tx_pin, 1);
    t->flags = (t->flags & (MU_PULLUP | MU_SINGLE_WIRE)) | MU_LINE_HIGH;
}

// Helper function to end a transmission and schedule a response
static uint_fast8_t
mksuart_finalize(struct mksuart_s *t)
{
    mksuart_reset_line(t);
    t->flags |= MU_REPORT;
    sched_wake_task(&mksuart_wake);
    return SF_DONE;
}

// Event handler for reading uart bits
static uint_fast8_t
mksuart_read_event(struct timer *timer)
{
    struct mksuart_s *t = container_of(timer, struct mksuart_s, timer);
    uint8_t v = gpio_in_read(t->rx_pin);
    // Read and store bit
    uint8_t pos = t->pos, mask = 1 << (pos & 0x07), data = t->data[pos >> 3];
    if (v)
        data |= mask;
    else
        data &= ~mask;
    t->data[pos >> 3] = data;
    // Skip Stop and next Start bit
    if ((pos & 0x07) == 0x07)
    {
        t->timer.waketime += 2 * t->bit_time;
    }
    pos++;

    if (pos >= t->read_count)
        return mksuart_finalize(t);
    t->pos = pos;
    t->timer.waketime += t->bit_time;
    return SF_RESCHEDULE;
}

// Event handler for detecting start of data reception
static uint_fast8_t
mksuart_read_sync_event(struct timer *timer)
{
    struct mksuart_s *t = container_of(timer, struct mksuart_s, timer);
    uint8_t v = gpio_in_read(t->rx_pin);
    if (v == 0 )
    {
        // Now synchronized - begin reading
        t->pos = 0;
        t->timer.func = mksuart_read_event;
        // return mksuart_read_event(timer);
    }
    if (t->pos++ >= 64)
    {
        // Timeout
        t->read_count = 0;
        return mksuart_finalize(t);
    }
    t->timer.waketime += t->bit_time;
    return SF_RESCHEDULE;
}

// Event handler called at end of uart writing
static uint_fast8_t
mksuart_send_finish_event(struct timer *timer)
{
    struct mksuart_s *t = container_of(timer, struct mksuart_s, timer);
    if (!t->read_count)
        // This is a tx only operation - success
        return mksuart_finalize(t);
    // Prepare for message rx
    if (t->flags & MU_SINGLE_WIRE)
        gpio_in_reset(t->rx_pin, t->flags & MU_PULLUP);
    t->pos = 0;
    t->timer.func = mksuart_read_sync_event;
    // t->timer.waketime += t->bit_time;
    return SF_RESCHEDULE;
}

// Event handler for sending uart bits
static uint_fast8_t
mksuart_send_event(struct timer *timer)
{
    // struct mksuart_s *t = container_of(timer, struct mksuart_s, timer);
    // // Toggle uart output
    // gpio_out_toggle_noirq(t->tx_pin);
    // t->flags ^= MU_LINE_HIGH;
    // // Determine next toggle time
    // uint8_t line_state = t->flags & MU_LINE_HIGH;
    // uint32_t bit_time = t->bit_time, next = bit_time;
    // uint8_t pos = t->pos;
    // for (;;) {
    //     pos++;
    //     if (pos >= t->write_count) {
    //         // No more toggles necessary - schedule finish event
    //         t->timer.func = mksuart_send_finish_event;
    //         t->timer.waketime += next;
    //         return SF_RESCHEDULE;
    //     }
    //     uint8_t data = t->data[pos >> 3], bit = (data >> (pos & 0x07)) & 0x01;
    //     if (bit != line_state)
    //         break;
    //     next += bit_time;
    // }
    // t->pos = pos;
    // t->timer.waketime += next;
    // return SF_RESCHEDULE;

    struct mksuart_s *t = container_of(timer, struct mksuart_s, timer);

    uint8_t pos = t->pos, mask = 1 << (pos & 0x07), data = t->data[pos >> 3];

    // Read and store bit
    if (t->flags & MU_STOP)
    {
        t->flags &= ~(MU_STOP);
        gpio_out_write(t->tx_pin, 1);
        if (pos >= t->write_count)
        {
            t->timer.func = mksuart_send_finish_event;
            t->timer.waketime += t->bit_time;
            return SF_RESCHEDULE;
        }
        t->flags |= MU_START;
        t->timer.waketime += t->bit_time;
        return SF_RESCHEDULE;
    }

    if (t->flags & MU_START)
    {
        t->flags &= ~(MU_START);
        gpio_out_write(t->tx_pin, 0);
        t->timer.waketime += t->bit_time;
        return SF_RESCHEDULE;
    }

    if ((data & mask) == mask)
    {
        gpio_out_write(t->tx_pin, 1);
    }
    else
    {
        gpio_out_write(t->tx_pin, 0);
    }
    if ((pos & 0x07) == 0x07)
    {
        t->flags |= MU_STOP;
    }
    pos++;

    t->pos = pos;
    t->timer.waketime += t->bit_time;
    return SF_RESCHEDULE;
}

// Event handler for sending sync nibble with enhanced baud detection
static uint_fast8_t
mksuart_send_sync_event(struct timer *timer)
{
    struct mksuart_s *t = container_of(timer, struct mksuart_s, timer);
    // Toggle uart output and note toggle time
    gpio_out_toggle_noirq(t->tx_pin);
    uint32_t cur = timer_read_time();
    t->flags ^= MU_LINE_HIGH;
    // Determine next wakeup time
    t->pos++;
    if (t->pos == 1)
    {
        // First bit just sent - record scheduling offset for later use
        uint32_t offset = cur - t->timer.waketime;
        t->bit_time = offset;
    }
    else if (t->pos >= 5)
    {
        // Last bit of sync nibble just sent - calculate actual baud rate used
        uint32_t offset = cur - t->timer.waketime, start_offset = t->bit_time;
        int32_t diff = offset - start_offset;
        t->bit_time = t->cfg_bit_time + DIV_ROUND_CLOSEST(diff, 4);
        t->timer.func = mksuart_send_event;
        t->timer.waketime += diff + t->bit_time;
        return SF_RESCHEDULE;
    }
    t->timer.waketime += t->cfg_bit_time;
    return SF_RESCHEDULE;
}

void command_config_mksuart(uint32_t *args)
{
    struct mksuart_s *t = oid_alloc(args[0], command_config_mksuart, sizeof(*t));
    uint8_t pull_up = args[2];
    uint32_t rx_pin = args[1], tx_pin = args[3];
    t->rx_pin = gpio_in_setup(rx_pin, !!pull_up);
    t->tx_pin = gpio_out_setup(tx_pin, 1);
    t->cfg_bit_time = args[4];
    t->flags = (MU_LINE_HIGH | (pull_up ? MU_PULLUP : 0) | (rx_pin == tx_pin ? MU_SINGLE_WIRE : 0));
}
DECL_COMMAND(command_config_mksuart,
             "config_mksuart oid=%c rx_pin=%u pull_up=%c"
             " tx_pin=%u bit_time=%u");

// Parse and schedule a mks UART transmission request
void command_mksuart_send(uint32_t *args)
{
    struct mksuart_s *t = oid_lookup(args[0], command_config_mksuart);
    if (t->flags & MU_ACTIVE)
        // Uart is busy - silently drop this request (host should retransmit)
        return;
    uint8_t write_len = args[1];
    uint8_t *write = command_decode_ptr(args[2]);
    uint8_t read_len = args[3];
    if (write_len > sizeof(t->data) || read_len > sizeof(t->data))
        shutdown("mksuart data too large");
    memcpy(t->data, write, write_len);
    t->pos = 0;
    t->flags = (t->flags & (MU_LINE_HIGH | MU_PULLUP | MU_SINGLE_WIRE)) | MU_ACTIVE | MU_START;
    t->write_count = write_len * 8;
    t->read_count = read_len * 8;
    if (write_len >= 1 && (t->data[0] & 0x3f) == 0x2a)
    {
        t->timer.func = mksuart_send_sync_event;
    }
    else
    {
        t->bit_time = t->cfg_bit_time;
        t->timer.func = mksuart_send_event;
    }
    gpio_out_write(t->tx_pin, 1);
    irq_disable();
    t->timer.waketime = timer_read_time() + t->bit_time;
    sched_add_timer(&t->timer);
    irq_enable();
}
DECL_COMMAND(command_mksuart_send, "mksuart_send oid=%c write=%*s read=%c");

// Report completed response message back to host
void mksuart_task(void)
{
    if (!sched_check_wake(&mksuart_wake))
        return;
    uint8_t oid;
    struct mksuart_s *t;
    foreach_oid(oid, t, command_config_mksuart)
    {
        if (!(t->flags & MU_REPORT))
            continue;
        irq_disable();
        t->flags &= ~MU_REPORT;
        irq_enable();
        sendf("mksuart_response oid=%c read=%*s", oid, t->read_count / 8, t->data);
    }
}
DECL_TASK(mksuart_task);

void mksuart_shutdown(void)
{
    uint8_t i;
    struct mksuart_s *t;
    foreach_oid(i, t, command_config_mksuart)
    {
        mksuart_reset_line(t);
    }
}
DECL_SHUTDOWN(mksuart_shutdown);
