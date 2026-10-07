// Memory map and register definitions of the K-means SoC (see rtl/soc.sv).
#ifndef SOC_H
#define SOC_H

#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))

// K-means accelerator
#define ACC_BASE        0x10000000u
#define ACC_INFO        REG32(ACC_BASE + 0x000)
#define ACC_CTRL        REG32(ACC_BASE + 0x004)
#define ACC_STATUS      REG32(ACC_BASE + 0x008)
#define ACC_NUM_POINTS  REG32(ACC_BASE + 0x00C)
#define ACC_CYCLES      REG32(ACC_BASE + 0x010)
#define ACC_SSE_LO      REG32(ACC_BASE + 0x014)
#define ACC_SSE_HI      REG32(ACC_BASE + 0x018)
#define ACC_CENTROID(k) REG32(ACC_BASE + 0x100 + 4 * (k))
#define ACC_SUM_X(k)    REG32(ACC_BASE + 0x200 + 16 * (k))
#define ACC_SUM_Y(k)    REG32(ACC_BASE + 0x204 + 16 * (k))
#define ACC_COUNT(k)    REG32(ACC_BASE + 0x208 + 16 * (k))
#define ACC_PMEM        ((volatile uint32_t *)(ACC_BASE + 0x10000))
#define ACC_LMEM        ((volatile uint8_t *)(ACC_BASE + 0x20000))

#define ACC_CTRL_START  (1u << 0)
#define ACC_ST_BUSY     (1u << 0)
#define ACC_ST_DONE     (1u << 1)
#define ACC_ST_ERR      (1u << 2)

// UART
#define UART_BASE       0x20000000u
#define UART_TXDATA     REG32(UART_BASE + 0x0)
#define UART_RXDATA     REG32(UART_BASE + 0x4)
#define UART_STATUS     REG32(UART_BASE + 0x8)
#define UART_DIV        REG32(UART_BASE + 0xC)
#define UART_FULL       (1u << 31)
#define UART_EMPTY      (1u << 31)

// System control
#define SYS_BASE        0x30000000u
#define SYS_LED         REG32(SYS_BASE + 0x00)
#define SYS_SW          REG32(SYS_BASE + 0x04)
#define SYS_CYCLE_LO    REG32(SYS_BASE + 0x08)
#define SYS_CYCLE_HI    REG32(SYS_BASE + 0x0C)
#define SYS_CLK_HZ      REG32(SYS_BASE + 0x10)

static inline uint64_t cycles(void)
{
    uint32_t lo = SYS_CYCLE_LO;         // also captures the high word
    return ((uint64_t)SYS_CYCLE_HI << 32) | lo;
}

#endif
