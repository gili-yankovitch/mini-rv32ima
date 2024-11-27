#include <stdint.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "mini-rv32ima.h"
#include "mmio.h"
#include "spi.h"

static uint8_t w25q_buf[128];
static uint32_t w25q_idx = 0;

int w25q_write_device(uint16_t val)
{
    int err = -1;
    static bool data_writing = false;

    if (spi_data_mode() == E_SPI_16BIT)
    {
        *((uint16_t *)((uint8_t *)w25q_buf + w25q_idx)) = val;

        w25q_idx += 2;
    }
    else
    {
        w25q_buf[w25q_idx++] = val;
    }

    // Try to evaluate the command
    switch (w25q_buf[0])
    {
        case 0x9f:
        {
            break;
        }
    }

    err = 0;
error:
    return err;
}
