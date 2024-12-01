#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "mini-rv32ima.h"
#include "mmio.h"
#include "spi.h"

#define ntohs(x) ((((x) & 0x00ff) << 8) | (((x) & 0xff00) >> 8))

struct spi_regs_s spi = {
    .SPI_CTLR1 = 0x0,
    .SPI_CTLR2 = 0x0,
    .SPI_STATR = 0x2,
    .SPI_DATAR = 0x0,
    .SPI_CRCR = 0x7,
    .SPI_RCRCR = 0x0,
    .SPI_TCRCR = 0x0,
    .SPI_HSCR = 0x0
};

enum spi_bit_mode_e spi_data_mode()
{
    // DFF (8/16 bit)
    return (spi.SPI_CTLR1 & (0b1 << 11)) ? E_SPI_16BIT : E_SPI_8BIT;
}

static uint32_t spi_write_device(uint32_t val)
{
    uint32_t res;

#ifdef W25Q
    res = w25q_write_device(val);
#endif

    return res;
}

void spi_write(struct memarea_s * area, uint32_t addr, uint32_t val, int res)
{
    fprintf(stderr, "Writing SPI 0x%x = 0x%x (res: %d)\n", addr, val, res);

    // SPI_DATAR
    if (addr == area->addr + 0x0c)
    {
        uint32_t res = spi_write_device((spi_data_mode() == E_SPI_16BIT) ? ntohs(val & 0xffff) : val & 0xff);

        // TXE - Tx buffer empty
        spi.SPI_STATR |= 0b10;

        // Write to buffer what was returned
        spi.SPI_DATAR = res;

        // RXNE - Rx buffer not empty
        spi.SPI_STATR |= 0b1;
    }
    return;
}

uint32_t spi_read(struct memarea_s * area, uint32_t addr, int res)
{
    uint32_t val = *((uint32_t *)((uint8_t *)area->data + (addr - area->addr)));

    // SPI_DATAR
    if (addr == area->addr + 0x0c)
    {
        // RXNE - Rx buffer empty
        spi.SPI_STATR &= ~0b1;
    }

    fprintf(stderr, "Reading SPI 0x%x = 0x%x (res: %d)\n", addr, val, res);

    return val;
}
