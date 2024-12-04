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
        if ((data[i].addr <= addr) && (addr < (data[i].addr + DATA_CHUNK_SIZE)))
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

    // fprintf(stderr, "Writing to page 0x%x offset 0x%x = 0x%x\n", addr & ~0xff, addr & 0xff, data);

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

#ifdef WASM

void w25q_cmd(uint8_t * buf, size_t len)
{
    int i;
    bool read_data = false;
    enum spi_bit_mode_e prevmode = spi_data_mode();

    if (prevmode == E_SPI_16BIT)
    {
    	spi_set_8bit_mode();
    }

    toggle_cs(0);

    // Is it data read?
    if (buf[0] == 0x03)
    {
        read_data = true;
    }

    for (i = 0; i < len; ++i)
    {
        buf[i] = w25q_write_device(buf[i]);
    }

    toggle_cs(1);

    if (prevmode == E_SPI_16BIT)
    {
    	spi_set_16bit_mode();
    }

    // Print out if this is a data read
    if (read_data)
    {
        for (i = 4; i < len; ++i)
        {
            printf("%02x ", buf[i]);

            if (((i - 4 + 1) % 16) == 0)
            {
                printf("\n");
            }

        }

	    printf("\n");
    }
}

static void setup()
{
	uint8_t first[] = {0x02, 0x01, 0x00, 0x00, 0, 0, 0, 0, 14, 5, 19, 7, 54, 15, 55, 105, 34, 39, 63, 101, 46, 32, 54, 105, 47, 59, 63, 36, 38, 97, 44, 33, 36, 58, 123, 101, 125, 57, 106, 121, 125, 121, 106, 56}; 
	uint8_t second[] = {0x02, 0x02, 0x00, 0x00, 75, 64, 111, 230, 163, 212, 50, 27, 38, 40, 183, 198, 255, 245, 252, 159, 110, 97, 113, 56, 72, 62, 249, 134, 156, 184, 76, 156, 192, 210, 114, 163, 222, 144, 231, 208, 174, 131, 56, 176, 122, 172, 56, 148, 117, 116, 105, 0, 65, 93, 57, 65, 222, 208, 228, 227, 173, 197, 69, 152, 66, 220, 165, 141};
	uint8_t third[] = {0x02, 0x03, 0x00, 0x00, 32, 0, 0, 0, 247, 96, 74, 31, 94, 150, 57, 126, 150, 245, 158, 49, 114, 11, 217, 0, 215, 107, 237, 200, 209, 209, 71, 52, 129, 70, 154, 36, 191, 170, 144, 34, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
	uint8_t fourth[] = {0x02, 0x04, 0x00, 0x00, 64, 0, 0, 0, 142, 60, 90, 245, 86, 130, 212, 12, 90, 38, 219, 29, 113, 243, 207, 17, 112, 218, 203, 157, 48, 112, 255, 94, 126, 249, 123, 170, 70, 178, 4, 7, 241, 45, 43, 70, 140, 134, 86, 219, 212, 85, 202, 85, 68, 213, 100, 164, 32, 79, 86, 197, 163, 240, 58, 229, 84, 2, 213, 241, 221, 40, 153, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
	uint8_t fifth[] = {0x02, 0x05, 0x00, 0x00, 0x0, 0x0, 0x0, 0x0};
	uint8_t wel_cmd[] = {0x06};
	uint8_t nwel_cmd[] = {0x04};

	w25q_cmd(wel_cmd, 1);
	w25q_cmd(first, sizeof(first));
	w25q_cmd(second, sizeof(second));
	w25q_cmd(third, sizeof(third));
	w25q_cmd(fourth, sizeof(fourth));
	w25q_cmd(fifth, sizeof(fifth));
	w25q_cmd(nwel_cmd, 1);
}

#endif

void w25q_init()
{
#ifdef WASM
	setup();
#endif
    gpio_register_portc_cb(toggle_cs, 4);
}
