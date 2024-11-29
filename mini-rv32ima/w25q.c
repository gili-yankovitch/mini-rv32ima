#include <stdint.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "mini-rv32ima.h"
#include "mmio.h"
#include "spi.h"

static uint8_t reqbuf[128];
static uint32_t idx = 0;
static uint8_t rspbuf[128];
static uint32_t rspidx = 0;
static uint32_t rsplen = 0;
static bool rsp = false;

static void resetBuffers()
{
    memset(rspbuf, 0, rspidx + 1);
    memset(reqbuf, 0, idx + 1);
    idx = 0;
    rspidx = 0;
    rsp = false;
}

uint32_t w25q_write_device(uint16_t val)
{
    uint32_t ret = 0;

    if (rsp)
    {
        goto output;
    }

    if (spi_data_mode() == E_SPI_16BIT)
    {
        *((uint16_t *)((uint8_t *)reqbuf + idx)) = val;

        idx += 2;
    }
    else
    {
        reqbuf[idx++] = val;
    }

    // Try to evaluate the command
    switch (reqbuf[0])
    {
        case 0x9f:
        {
            // Initialize response
            rspidx = 0;
            rsplen = 4;
            memset(rspbuf, 0, sizeof(rspbuf));

            // Current command response
            rspbuf[0] = 0;

            //  Manufacturer - Winbond
            rspbuf[1] = 0xef;

            // Memory Type
            rspbuf[2] = 0x40;

            // Capacity
            rspbuf[3] = 0x18;

            // Respond
            rsp = true;

            break;
        }

        default:
        {
            // Reset buffer - nothing we can handle
            resetBuffers();

            goto done;
        }
    }
output:
    if (spi_data_mode() == E_SPI_16BIT)
    {
        ret = *((uint16_t *)((uint8_t *)rspbuf + rspidx));

        rspidx += 2;
    }
    else
    {
        ret = rspbuf[rspidx++];
    }

    // Finished processing he command
    if (rspidx >= rsplen)
    {
        resetBuffers();
    }

done:
    return ret;
}
