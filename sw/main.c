// Firmware: answers host requests over the UART (protocol.h).
#include <stdint.h>
#include "crc16_table.h"
#include "kmeans.h"
#include "protocol.h"
#include "soc.h"

extern char __ram_size[];               // from link.ld

static unsigned acc_k, acc_max_points;
static uint32_t rx_timeout;             // cycles to wait for the next byte of a frame
static unsigned n_points;               // points in the point memory
static int      label_src;              // mode of the last run, -1: no labels yet
static unsigned led_state;

// ----------------------------------------------------------------------
// CRC-16/CCITT-FALSE, table driven: a few instructions per byte, which
// keeps the receive loop well ahead of the line rate. The table is a
// constant (crc16_table.h), so the firmware answers immediately after reset.
// ----------------------------------------------------------------------
static inline uint16_t crc_step(uint16_t crc, uint32_t byte)
{
    return (uint16_t)((crc << 8) ^ crc_table[((crc >> 8) ^ byte) & 0xFF]);
}

// ----------------------------------------------------------------------
// Byte I/O
// ----------------------------------------------------------------------
static int rx_byte(uint32_t *b)         // 0, or -1 on timeout
{
    uint32_t t0 = SYS_CYCLE_LO;
    for (;;) {
        uint32_t v = UART_RXDATA;       // one load: empty flag and data
        if (!(v & UART_EMPTY)) {
            *b = v & 0xFF;
            return 0;
        }
        if (SYS_CYCLE_LO - t0 > rx_timeout)
            return -1;
    }
}

static void rx_sync(void)               // wait (forever) for the next request
{
    for (;;) {
        uint32_t v = UART_RXDATA;
        if (!(v & UART_EMPTY) && (v & 0xFF) == SYNC_REQ)
            return;
    }
}

static uint16_t tx_crc;

static void tx_raw(uint32_t b)
{
    while (UART_TXDATA & UART_FULL)
        ;
    UART_TXDATA = b & 0xFF;
}

static void tx_u8(uint32_t b)
{
    tx_crc = crc_step(tx_crc, b & 0xFF);
    tx_raw(b);
}

static void tx_u16(uint32_t v) { tx_u8(v); tx_u8(v >> 8); }
static void tx_u32(uint32_t v) { tx_u16(v); tx_u16(v >> 16); }
static void tx_u64(uint64_t v) { tx_u32((uint32_t)v); tx_u32((uint32_t)(v >> 32)); }

static void resp_begin(unsigned cmd, unsigned status, unsigned len)
{
    tx_raw(SYNC_RESP);
    tx_crc = 0xFFFF;
    tx_u8(cmd);
    tx_u8(status);
    tx_u16(len);
}

static void resp_end(void)
{
    uint16_t c = tx_crc;
    tx_raw(c & 0xFF);
    tx_raw(c >> 8);
}

static void resp_status(unsigned cmd, unsigned status)
{
    resp_begin(cmd, status, 0);
    resp_end();
}

// ----------------------------------------------------------------------
// Request reception
// ----------------------------------------------------------------------
static uint8_t buf[MAX_PAYLOAD];

// Receives the payload of a CMD_LOAD request straight into the point
// memory, four bytes per point, so the data set never has to fit in RAM.
// Points are only stored if the length is valid.
static int rx_points(unsigned len, uint16_t *crc, int store)
{
    uint32_t word = 0;
    for (unsigned i = 0; i < len; i++) {
        uint32_t b;
        if (rx_byte(&b))
            return -1;
        *crc = crc_step(*crc, b);
        word = (word >> 8) | (b << 24);
        if (store && (i & 3) == 3)
            ACC_PMEM[i >> 2] = word;
    }
    return 0;
}

static int rx_into_buf(unsigned len, uint16_t *crc)
{
    for (unsigned i = 0; i < len; i++) {
        uint32_t b;
        if (rx_byte(&b))
            return -1;
        *crc = crc_step(*crc, b);
        if (i < MAX_PAYLOAD)
            buf[i] = (uint8_t)b;
    }
    return 0;
}

static unsigned get_u16(const uint8_t *p) { return p[0] | (p[1] << 8); }

// ----------------------------------------------------------------------
// Commands
// ----------------------------------------------------------------------
static void cmd_info(void)
{
    resp_begin(CMD_INFO, ST_OK, 12);
    tx_u8(PROTO_VERSION);
    tx_u8(acc_k);
    tx_u8((ACC_INFO >> 16) & 0xFF);     // DATA_W
    tx_u8(0);
    tx_u16(acc_max_points);
    tx_u16((uint32_t)__ram_size >> 10);
    tx_u32(SYS_CLK_HZ);
    resp_end();
}

static void cmd_run(unsigned len)
{
    static kmeans_run r;

    if (len != 2 + 4 * acc_k || buf[0] > MODE_SW || buf[1] == 0) {
        resp_status(CMD_RUN, len != 2 + 4 * acc_k ? ST_LEN : ST_PARAM);
        return;
    }
    r.mode     = buf[0];
    r.max_iter = buf[1];
    r.k        = acc_k;
    r.n        = n_points;
    for (unsigned k = 0; k < acc_k; k++) {
        r.cx[k] = (int16_t)get_u16(&buf[2 + 4 * k]);
        r.cy[k] = (int16_t)get_u16(&buf[4 + 4 * k]);
    }

    kmeans(&r);
    label_src = (int)r.mode;
    led_state = (led_state & 0x8000) | (r.mode << 8) | (r.iterations & 0xFF);
    SYS_LED   = led_state;

    resp_begin(CMD_RUN, ST_OK, 4 + 8 * acc_k + 32);
    tx_u8(r.iterations);
    tx_u8(r.converged);
    tx_u8(r.mode);
    tx_u8(r.k);
    for (unsigned k = 0; k < acc_k; k++) {
        tx_u16((uint32_t)r.cx[k]);
        tx_u16((uint32_t)r.cy[k]);
    }
    for (unsigned k = 0; k < acc_k; k++)
        tx_u32(r.count[k]);
    tx_u64(r.sse);
    tx_u64(r.total_cycles);
    tx_u64(r.assign_cycles);
    tx_u64(r.accel_cycles);
    resp_end();
}

static void cmd_labels(unsigned len)
{
    if (len != 4) {
        resp_status(CMD_LABELS, ST_LEN);
        return;
    }
    unsigned off = get_u16(&buf[0]);
    unsigned cnt = get_u16(&buf[2]);
    if (label_src < 0 || off + cnt > n_points) {
        resp_status(CMD_LABELS, ST_PARAM);
        return;
    }
    resp_begin(CMD_LABELS, ST_OK, cnt);
    for (unsigned i = off; i < off + cnt; i++)
        tx_u8(label_src == MODE_HW ? ACC_LMEM[i] : sw_labels[i]);
    resp_end();
}

static void serve_one(void)
{
    uint32_t cmd, lo, hi, c0, c1;
    uint16_t crc = 0xFFFF;

    rx_sync();
    if (rx_byte(&cmd) || rx_byte(&lo) || rx_byte(&hi)) {
        resp_status(0, ST_TIMEOUT);
        return;
    }
    crc = crc_step(crc_step(crc_step(crc, cmd), lo), hi);
    unsigned len = lo | (hi << 8);

    led_state ^= 0x8000;                // activity
    SYS_LED = led_state;

    int err;
    unsigned load_ok = 0;
    if (cmd == CMD_LOAD) {
        load_ok = (len % 4 == 0) && (len / 4 <= acc_max_points);
        if (load_ok) {
            n_points  = 0;              // the old data is being overwritten
            label_src = -1;
        }
        err = rx_points(len, &crc, load_ok);
    } else {
        err = rx_into_buf(len, &crc);
    }
    if (err || rx_byte(&c0) || rx_byte(&c1)) {
        resp_status(cmd, ST_TIMEOUT);
        return;
    }
    if ((c0 | (c1 << 8)) != crc) {
        resp_status(cmd, ST_CRC);
        return;
    }

    switch (cmd) {
    case CMD_INFO:
        cmd_info();
        break;
    case CMD_LOAD:
        if (!load_ok) {
            resp_status(CMD_LOAD, len % 4 ? ST_LEN : ST_PARAM);
            break;
        }
        n_points = len / 4;
        resp_begin(CMD_LOAD, ST_OK, 2);
        tx_u16(n_points);
        resp_end();
        break;
    case CMD_RUN:
        if (len > MAX_PAYLOAD) resp_status(CMD_RUN, ST_LEN);
        else                   cmd_run(len);
        break;
    case CMD_LABELS:
        cmd_labels(len);
        break;
    default:
        resp_status(cmd, ST_CMD);
        break;
    }
}

int main(void)
{
    // No initialised writable data (link.ld checks): the RAM image is only
    // loaded at configuration, so after a reset it could hold stale values.
    label_src      = -1;
    n_points       = 0;
    led_state      = 0;

    uint32_t info  = ACC_INFO;
    acc_k          = (info >> 8) & 0xFF;
    acc_max_points = 1u << ((info >> 24) & 0xFF);
    if (acc_max_points > MAX_POINTS)
        acc_max_points = MAX_POINTS;
    rx_timeout     = SYS_CLK_HZ / 20;   // 50 ms between bytes of one frame
    SYS_LED = 0;

    for (;;)
        serve_one();
}
