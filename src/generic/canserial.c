// Generic handling of serial over CAN support
//
// Copyright (C) 2019 Eug Krashtan <eug.krashtan@gmail.com>
// Copyright (C) 2020 Pontus Borg <glpontus@gmail.com>
// Copyright (C) 2021-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_HAVE_BOOTLOADER_REQUEST
#include "board/io.h" // readb
#include "board/irq.h" // irq_save
#include "board/misc.h" // console_sendf
#include "canbus.h" // canbus_send
#include "canserial.h" // canserial_notify_tx
#include "command.h" // DECL_CONSTANT
#include "fasthash.h" // fasthash64
#include "sched.h" // sched_wake_task

#define CANBUS_UUID_LEN 6

// Global storage
static struct canbus_data {
    uint32_t assigned_id;
    uint8_t uuid[CANBUS_UUID_LEN];

    // Tx data
    struct task_wake tx_wake;
    uint8_t transmit_pos, transmit_max;

    // Rx data
    struct task_wake rx_wake;
    uint8_t receive_pos;
    uint32_t admin_pull_pos, admin_push_pos;

    // Transfer buffers
    struct canbus_msg admin_queue[8];
    uint8_t transmit_buf[96];
    uint8_t receive_buf[192];
} CanData;

/* ====================== CONFIG ====================== */

#define ISOTP_MAX_MESSAGE_SIZE   4095    /* ISO-TP maximum */
#define CAN_FRAME_DATA_SIZE      8

/* ====================== TYPES ====================== */

typedef enum
{
    ISOTP_IDLE = 0,
    ISOTP_RECEIVING
} isotp_rx_state_t;

typedef enum
{
    ISOTP_OK = 0,
    ISOTP_IN_PROGRESS,
    ISOTP_ERROR_OVERFLOW,
    ISOTP_ERROR_SEQUENCE,
    ISOTP_ERROR_FORMAT
} isotp_status_t;

typedef struct
{
    isotp_rx_state_t state;

    uint16_t expected_length;
    uint16_t received_length;

    uint8_t next_sequence_number;

    uint8_t buffer[ISOTP_MAX_MESSAGE_SIZE];

} isotp_receiver_t;

/* ====================== INIT ====================== */

void isotp_receiver_init(isotp_receiver_t *ctx)
{
    ctx->state = ISOTP_IDLE;
    ctx->expected_length = 0;
    ctx->received_length = 0;
    ctx->next_sequence_number = 1;
}

isotp_receiver_t crx;

/****************************************************************
 * Data transmission over CAN
 ****************************************************************/

void
canserial_notify_tx(void)
{
    sched_wake_task(&CanData.tx_wake);
}

void
canserial_tx_task(void)
{
    if (!sched_check_wake(&CanData.tx_wake))
        return;
    uint32_t id = CanData.assigned_id;
    if (!id) {
        CanData.transmit_pos = CanData.transmit_max = 0;
        return;
    }
    struct canbus_msg msg;
    msg.id = id + 1;
    uint32_t tpos = CanData.transmit_pos, tmax = CanData.transmit_max;
    for (;;) {
        int avail = tmax - tpos, now = avail > 8 ? 8 : avail;
        if (avail <= 0)
            break;
        msg.dlc = now;
        memcpy(msg.data, &CanData.transmit_buf[tpos], now);
        int ret = canbus_send(&msg);
        if (ret <= 0)
            break;
        tpos += now;
    }
    CanData.transmit_pos = tpos;
}
DECL_TASK(canserial_tx_task);

// Encode and transmit a "response" message
void
console_sendf(const struct command_encoder *ce, va_list args)
{
    // Verify space for message
    uint32_t tpos = CanData.transmit_pos, tmax = CanData.transmit_max;
    if (tpos >= tmax)
        CanData.transmit_pos = CanData.transmit_max = tpos = tmax = 0;
    uint32_t max_size = ce->max_size;
    if (tmax + max_size > sizeof(CanData.transmit_buf)) {
        if (tmax + max_size - tpos > sizeof(CanData.transmit_buf))
            // Not enough space for message
            return;
        // Move buffer
        tmax -= tpos;
        memmove(&CanData.transmit_buf[0], &CanData.transmit_buf[tpos], tmax);
        CanData.transmit_pos = tpos = 0;
        CanData.transmit_max = tmax;
    }

    // Generate message
    uint32_t msglen = command_encode_and_frame(&CanData.transmit_buf[tmax]
                                               , ce, args);

    // Start message transmit
    CanData.transmit_max = tmax + msglen;
    canserial_notify_tx();
}


/****************************************************************
 * CAN "admin" command handling
 ****************************************************************/

// Available commands and responses
#define CANBUS_CMD_QUERY_UNASSIGNED 0x00
#define CANBUS_CMD_SET_KLIPPER_NODEID 0x01
#define CANBUS_CMD_REQUEST_BOOTLOADER 0x02
#define CANBUS_RESP_NEED_NODEID 0x20

// Helper to verify a UUID in a command matches this chip's UUID
static int
can_check_uuid(struct canbus_msg *msg)
{
    return (msg->dlc >= 7
            && memcmp(&msg->data[1], CanData.uuid, sizeof(CanData.uuid)) == 0);
}

// Helpers to encode/decode a CAN identifier to a 1-byte "nodeid"
static int
can_get_nodeid(void)
{
    if (!CanData.assigned_id)
        return 0;
    return (CanData.assigned_id - 0x100) >> 1;
}
static uint32_t
can_decode_nodeid(int nodeid)
{
    return (nodeid << 1) + 0x100;
}

static void
can_process_query_unassigned(struct canbus_msg *msg)
{
    if (CanData.assigned_id)
        return;
    struct canbus_msg send;
    send.id = CANBUS_ID_ADMIN_RESP;
    send.dlc = 8;
    send.data[0] = CANBUS_RESP_NEED_NODEID;
    memcpy(&send.data[1], CanData.uuid, sizeof(CanData.uuid));
    send.data[7] = CANBUS_CMD_SET_KLIPPER_NODEID;
    // Send with retry
    for (;;) {
        int ret = canbus_send(&send);
        if (ret >= 0)
            return;
    }
}

static void
can_id_conflict(void)
{
    CanData.assigned_id = 0;
    canbus_set_filter(CanData.assigned_id);
    shutdown("Another CAN node assigned this ID");
}

static void
can_process_set_klipper_nodeid(struct canbus_msg *msg)
{
    if (msg->dlc < 8)
        return;
    uint32_t newid = can_decode_nodeid(msg->data[7]);
    if (can_check_uuid(msg)) {
        if (newid != CanData.assigned_id) {
            CanData.assigned_id = newid;
            canbus_set_filter(CanData.assigned_id);
        }
    } else if (newid == CanData.assigned_id) {
        can_id_conflict();
    }
}

static void
can_process_request_bootloader(struct canbus_msg *msg)
{
    if (!CONFIG_HAVE_BOOTLOADER_REQUEST || !can_check_uuid(msg))
        return;
    bootloader_request();
}

// Handle an "admin" command
static void
can_process_admin(struct canbus_msg *msg)
{
    if (!msg->dlc)
        return;
    switch (msg->data[0]) {
    case CANBUS_CMD_QUERY_UNASSIGNED:
        can_process_query_unassigned(msg);
        break;
    case CANBUS_CMD_SET_KLIPPER_NODEID:
        can_process_set_klipper_nodeid(msg);
        break;
    case CANBUS_CMD_REQUEST_BOOTLOADER:
        can_process_request_bootloader(msg);
        break;
    }
}


/****************************************************************
 * CAN packet reading
 ****************************************************************/

static void
canserial_notify_rx(void)
{
    sched_wake_task(&CanData.rx_wake);
}

DECL_CONSTANT("RECEIVE_WINDOW", ARRAY_SIZE(CanData.receive_buf));

// Handle incoming data (called from IRQ handler)
void
canserial_process_data(struct canbus_msg *msg)
{
    uint32_t id = msg->id;
    if (CanData.assigned_id && id == CanData.assigned_id) {
        // Add to incoming data buffer
        int rpos = CanData.receive_pos;
        uint32_t len = CANMSG_DATA_LEN(msg);
        if (len > sizeof(CanData.receive_buf) - rpos)
            return;
        memcpy(&CanData.receive_buf[rpos], msg->data, len);
        CanData.receive_pos = rpos + len;
        canserial_notify_rx();
    } else if (id == CANBUS_ID_ADMIN
               || (CanData.assigned_id && id == CanData.assigned_id + 1)) {
        // Add to admin command queue
        uint32_t pushp = CanData.admin_push_pos;
        if (pushp >= CanData.admin_pull_pos + ARRAY_SIZE(CanData.admin_queue))
            // No space - drop message
            return;
        uint32_t pos = pushp % ARRAY_SIZE(CanData.admin_queue);
        memcpy(&CanData.admin_queue[pos], msg, sizeof(*msg));
        CanData.admin_push_pos = pushp + 1;
        canserial_notify_rx();
    }
}

// Remove from the receive buffer the given number of bytes
static void
console_pop_input(int len)
{
    int copied = 0;
    for (;;) {
        int rpos = readb(&CanData.receive_pos);
        int needcopy = rpos - len;
        if (needcopy) {
            memmove(&CanData.receive_buf[copied]
                    , &CanData.receive_buf[copied + len], needcopy - copied);
            copied = needcopy;
            canserial_notify_rx();
        }
        irqstatus_t flag = irq_save();
        if (rpos != readb(&CanData.receive_pos)) {
            // Raced with irq handler - retry
            irq_restore(flag);
            continue;
        }
        CanData.receive_pos = needcopy;
        irq_restore(flag);
        break;
    }
}

isotp_status_t isotp_receiver_process(isotp_receiver_t *ctx,
                                       const uint8_t *data,
                                       uint8_t len)
{
    if (len != CAN_FRAME_DATA_SIZE)
        return ISOTP_ERROR_FORMAT;

    uint8_t pci_type = (data[0] >> 4) & 0x0F;

    switch (pci_type)
    {
        /* ================= SINGLE FRAME ================= */
        case 0x0:
        {
            uint8_t payload_len = data[0] & 0x0F;

            if (payload_len > 7)
                return ISOTP_ERROR_FORMAT;

            memcpy(ctx->buffer, &data[1], payload_len);

            ctx->expected_length = payload_len;
            ctx->received_length = payload_len;

            return ISOTP_OK;
        }

        /* ================= FIRST FRAME ================= */
        case 0x1:
        {
            uint16_t length = ((data[0] & 0x0F) << 8) | data[1];

            if (length > ISOTP_MAX_MESSAGE_SIZE)
                return ISOTP_ERROR_OVERFLOW;

            ctx->expected_length = length;
            ctx->received_length = 6; /* FF carries 6 bytes */
            ctx->next_sequence_number = 1;
            ctx->state = ISOTP_RECEIVING;

            memcpy(ctx->buffer, &data[2], 6);

            return ISOTP_IN_PROGRESS;
        }

        /* ================= CONSECUTIVE FRAME ================= */
        case 0x2:
        {
            if (ctx->state != ISOTP_RECEIVING)
                return ISOTP_ERROR_FORMAT;

            uint8_t seq = data[0] & 0x0F;

            if (seq != ctx->next_sequence_number)
            {
                isotp_receiver_init(ctx);
                return ISOTP_ERROR_SEQUENCE;
            }

            ctx->next_sequence_number++;
            if (ctx->next_sequence_number > 0x0F)
                ctx->next_sequence_number = 0;

            uint16_t remaining = ctx->expected_length - ctx->received_length;
            uint8_t copy_len = (remaining >= 7) ? 7 : remaining;

            memcpy(&ctx->buffer[ctx->received_length], &data[1], copy_len);

            ctx->received_length += copy_len;

            if (ctx->received_length >= ctx->expected_length)
            {
                ctx->state = ISOTP_IDLE;
                return ISOTP_OK;
            }

            return ISOTP_IN_PROGRESS;
        }

        default:
            return ISOTP_ERROR_FORMAT;
    }
}

// Task to process incoming commands and admin messages
void
canserial_rx_task(void)
{
    if (!sched_check_wake(&CanData.rx_wake))
        return;

    // Process pending admin messages
    for (;;) {
        uint32_t pushp = readl(&CanData.admin_push_pos);
        uint32_t pullp = CanData.admin_pull_pos;
        if (pushp == pullp)
            break;
        uint32_t pos = pullp % ARRAY_SIZE(CanData.admin_queue);
        struct canbus_msg *msg = &CanData.admin_queue[pos];
        uint32_t id = msg->id;
        if (CanData.assigned_id && id == CanData.assigned_id + 1)
            can_id_conflict();
        else if (id == CANBUS_ID_ADMIN)
            can_process_admin(msg);
        CanData.admin_pull_pos = pullp + 1;
    }

    // Check for a complete message block and process it
    uint_fast8_t rpos = readb(&CanData.receive_pos), pop_count = 0;
    uint8_t *receive_buf = CanData.receive_buf;
    while(rpos >= 8) {
        isotp_status_t isotp_ret = isotp_receiver_process(&crx, receive_buf, rpos);
        rpos -= 8;
        receive_buf += 8;
        pop_count += 8;
        if (isotp_ret == ISOTP_OK) {
            if(check_seq(crx.buffer[0])) {
                command_dispatch(crx.buffer - 1, crx.received_length + 4);
                command_send_ack();
            }
            isotp_receiver_init(&crx);
        }
    }
    console_pop_input(pop_count);
}
DECL_TASK(canserial_rx_task);


/****************************************************************
 * Setup and shutdown
 ****************************************************************/

DECL_ENUMERATION("canbus_bus_state", "active", CANBUS_STATE_ACTIVE);
DECL_ENUMERATION("canbus_bus_state", "warn", CANBUS_STATE_WARN);
DECL_ENUMERATION("canbus_bus_state", "passive", CANBUS_STATE_PASSIVE);
DECL_ENUMERATION("canbus_bus_state", "off", CANBUS_STATE_OFF);

void
command_get_canbus_status(uint32_t *args)
{
    struct canbus_status status;
    memset(&status, 0, sizeof(status));
    canhw_get_status(&status);
    sendf("canbus_status rx_error=%u tx_error=%u tx_retries=%u"
          " canbus_bus_state=%u"
          , status.rx_error, status.tx_error, status.tx_retries
          , status.bus_state);
}
DECL_COMMAND_FLAGS(command_get_canbus_status, HF_IN_SHUTDOWN
                   , "get_canbus_status");

void
command_get_canbus_id(uint32_t *args)
{
    sendf("canbus_id canbus_uuid=%.*s canbus_nodeid=%u"
          , sizeof(CanData.uuid), CanData.uuid, can_get_nodeid());
}
DECL_COMMAND_FLAGS(command_get_canbus_id, HF_IN_SHUTDOWN, "get_canbus_id");

void
canserial_set_uuid(uint8_t *raw_uuid, uint32_t raw_uuid_len)
{
    uint64_t hash = fasthash64(raw_uuid, raw_uuid_len, 0xA16231A7);
    memcpy(CanData.uuid, &hash, sizeof(CanData.uuid));
    canserial_notify_rx();
}

void
canserial_shutdown(void)
{
    canserial_notify_tx();
    canserial_notify_rx();
}
DECL_SHUTDOWN(canserial_shutdown);
