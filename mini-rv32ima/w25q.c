#include <stdint.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <malloc.h>

#include "mini-rv32ima.h"
#include "mmio.h"
#include "spi.h"

static uint8_t reqbuf[128];
static uint32_t idx = 0;
static uint8_t rspbuf[128];
static uint32_t rspidx = 0;
static uint32_t rsplen = 0;
static bool rsp = false;
static uint32_t cs = 0;
static uint32_t read_addr = 0;
static uint32_t write_addr = 0;
static bool wel = false;

// All chunks are the same length
struct flash_data_s
{
    uint32_t addr;
    uint8_t * data;
};

#define DATA_CHUNK_SIZE 256
static struct flash_data_s * data = NULL;
static size_t data_chunks = 0;

static struct flash_data_s * new_page(uint32_t addr)
{
    data = realloc(data, sizeof(struct flash_data_s) * (data_chunks + 1));
    data[data_chunks].addr = addr & ~0xff;
    data[data_chunks].data = malloc(DATA_CHUNK_SIZE);

    // Initialize to 0xff
    memset(data[data_chunks].data, 0xff, DATA_CHUNK_SIZE);

    // Add to data chunks
    data_chunks++;

    return &data[data_chunks - 1];
}

static struct flash_data_s * get_page(uint32_t addr)
{
    struct flash_data_s * page = NULL;
    int i = 0;

    for (i = 0; i < data_chunks; ++i)
    {
        if ((data[i].addr <= addr) && (addr < data[i].addr + DATA_CHUNK_SIZE))
        {
            page = &data[i];

            break;
        }
    }

    if (!page)
    {
        page = new_page(addr);
    }

    return page;
}

static uint8_t read_data(uint32_t addr)
{
    struct flash_data_s * page;

    // Limit addresses to 16MB
    if (addr >= (16 * 1024 * 1024))
    {
        return 0xff;
    }

    if (!(page = get_page(addr)))
    {
        fprintf(stderr, "Error getting page for address 0x%x\n", addr);
    }

    return page->data[addr & 0xff];
}

static void erase_sector(uint32_t addr)
{
    int i = 0;

    // Iterage over the sector (4K) in page quants
    for (i = 0; i < (0x1000 >> 8); ++i)
    {
        struct flash_data_s * page = get_page(addr);

        // Reset its data
        memset(page->data, 0xff, DATA_CHUNK_SIZE);
    }
}

static void write_data(uint32_t addr, uint8_t data)
{
    struct flash_data_s * page;

    // Limit addresses to 16MB
    if (addr >= (16 * 1024 * 1024))
    {
        return;
    }

    if (!(page = get_page(addr)))
    {
        fprintf(stderr, "Error getting page for address 0x%x\n", addr);
    }

    // Set data ACCORDING TO HOW THE FLASH WORKS
    page->data[addr & 0xff] &= data;
}

static void resetBuffers()
{
    memset(rspbuf, 0, rspidx + 1);
    memset(reqbuf, 0, idx + 1);
    idx = 0;
    rspidx = 0;
    rsp = false;
    read_addr = 0;
    write_addr = 0;
}

static void prepare_rsp(size_t len)
{
    rspidx = 0;
    rsplen = len;
    memset(rspbuf, 0, sizeof(rspbuf));
}

uint32_t w25q_write_device(uint16_t val)
{
    uint32_t ret = 0;

    // If CS is HIGH, ignore
    if (cs)
    {
        goto done;
    }

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
        // chipID
        case 0x9f:
        {
            // Initialize response
            prepare_rsp(4);

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

        // Status Register 1
        case 0x5f:
        {
            prepare_rsp(2);

            // Current command
            rspbuf[0] = 0;

            // Status register
            // BUSY - 0
            // WEL  - 0
            // BP0  - 0
            // BP1  - 0
            // BP2  - 0
            // TB   - 0
            // SEC  - 0
            // CMP  - 0
            rspbuf[1] = 0 | wel << 1;

            rsp = true;

            break;
        }

        // Read Data
        case 0x03:
        {
            // Wait for addr to be fully populated
            if (idx == 4)
            {
                read_addr = reqbuf[1] << 16 |
                        reqbuf[2] << 8 |
                        reqbuf[3];
            }
            else if (idx > 4)
            {
                // Always keep idx 5
                idx = 5;

                // Return the next data
                ret = read_data(read_addr++);

                if (spi_data_mode() == E_SPI_16BIT)
                {
                    ret <<= 8;
                    ret |= read_data(read_addr++);
                }
            }

            goto done;
        }

        // Write data
        case 0x02:
        {
            if (!wel)
            {
                goto done;
            }

            // Wait for addr to be fully populated
            if (idx == 4)
            {
                write_addr = reqbuf[1] << 16 |
                        reqbuf[2] << 8 |
                        reqbuf[3];
            }
            else if (idx > 4)
            {
                if (spi_data_mode() == E_SPI_16BIT)
                {
                    write_data(write_addr++, reqbuf[idx - 2]);
                }

                write_data(write_addr++, reqbuf[idx - 1]);

                idx = 5;
            }

            goto done;
        }

        // Erase sector
        case 0x20:
        {
            if (!wel)
            {
                goto done;
            }

            // Wait for addr to be fully populated
            if (idx == 4)
            {
                uint32_t erase_addr = reqbuf[1] << 16 |
                        reqbuf[2] << 8 |
                        reqbuf[3];

                erase_sector(erase_addr);

            }

            goto done;
        }

        case 0x06:
        {
            // Enable writing
            wel = true;

            goto done;
        }

        case 0x04:
        {
            // Enable writing
            wel = false;

            goto done;
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

static void toggle_cs(uint32_t val)
{
    // On rising edge, reset buffers
    if ((cs == 0) && (val == 1))
    {
        resetBuffers();
    }

    // Remember previous state, to act only on state changes
    cs = val;
}

void w25q_init()
{
    gpio_register_portc_cb(toggle_cs, 4);
}
