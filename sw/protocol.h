// Host <-> SoC protocol over the UART. host/protocol.py is the other end.
//
// Every message is a frame, all fields little-endian:
//   request   0xA5  cmd  len[2]  payload[len]  crc[2]
//   response  0x5A  cmd  status  len[2]  payload[len]  crc[2]
// crc is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over everything
// between the sync byte and the crc. The SoC answers every request with
// exactly one response, so the host never has more than one frame in flight.
#ifndef PROTOCOL_H
#define PROTOCOL_H

#define SYNC_REQ        0xA5
#define SYNC_RESP       0x5A

// Commands
#define CMD_INFO        0x01    // -> version u8, K u8, DATA_W u8, 0 u8, max_points u16,
                                //    ram_kib u16, clk_hz u32
#define CMD_LOAD        0x02    // n x (x i16, y i16) -> n u16
#define CMD_RUN         0x03    // mode u8, max_iter u8, K x (x i16, y i16)
                                // -> iterations u8, converged u8, mode u8, K u8,
                                //    K x (x i16, y i16), K x count u32,
                                //    sse u64, total_cycles u64, assign_cycles u64,
                                //    accel_cycles u64
#define CMD_LABELS      0x04    // offset u16, count u16 -> count x label u8

// Run modes
#define MODE_HW         0       // assignment step on the accelerator
#define MODE_SW         1       // the same algorithm entirely in software

// Status codes
#define ST_OK           0
#define ST_CRC          1       // checksum mismatch: request ignored
#define ST_LEN          2       // payload length wrong for the command
#define ST_CMD          3       // unknown command
#define ST_PARAM        4       // bad parameter (too many points, bad mode, ...)
#define ST_TIMEOUT      5       // the request stopped arriving part way

#define PROTO_VERSION   1
#define MAX_PAYLOAD     256     // except CMD_LOAD, which streams into the accelerator

#endif
