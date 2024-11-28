#ifndef __SPI_H__
#define __SPI_H__

#include "w25q.h"

struct spi_regs_s
{
    uint32_t SPI_CTLR1;
    uint32_t SPI_CTLR2;
    uint32_t SPI_STATR;
    uint32_t SPI_DATAR;
    uint32_t SPI_CRCR;
    uint32_t SPI_RCRCR;
    uint32_t SPI_TCRCR;
    uint32_t SPI_HSCR;
};

enum spi_bit_mode_e
{
    E_SPI_8BIT,
    E_SPI_16BIT
};

extern struct spi_regs_s spi;

enum spi_bit_mode_e spi_data_mode();

void spi_write(struct memarea_s * area, uint32_t addr, uint32_t val, int res);

uint32_t spi_read(struct memarea_s * area, uint32_t addr, int res);

#endif // __SPI_H__
