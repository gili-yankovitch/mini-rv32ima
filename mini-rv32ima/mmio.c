#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "mini-rv32ima.h"
#include "mmio.h"
#include "spi.h"

#ifdef __cplusplus
  #define     __I     volatile                /*!< defines 'read only' permissions      */
#else
  #define     __I     volatile const          /*!< defines 'read only' permissions     */
#endif
#define     __O     volatile                  /*!< defines 'write only' permissions     */
#define     __IO    volatile                  /*!< defines 'read / write' permissions   */

#define DECODER(ir, mask, pos, dest)   (((ir) & ((mask) << (pos))) >> ((pos) - (dest)))
#define DECODEL(ir, mask, pos, dest)   (((ir) & ((mask) << (pos))) << ((dest) - (pos)))

bool hw_break = false;

void bkpt()
{
    hw_break = true;
}

struct gpio_regs_s
{
    uint32_t GPIO_CFGLR;
    uint32_t reserved0;
    uint32_t GPIO_INDR;
    uint32_t GPIO_OUTDR;
    uint32_t GPIO_BSHR;
    uint32_t GPIO_BCR;
    uint32_t GPIO_LCKR;
};

struct gpio_regs_s porta = {
    .GPIO_CFGLR = 0x44444444,
    .reserved0 = 0x0,
    .GPIO_INDR = 0x0,
    .GPIO_OUTDR = 0x0,
    .GPIO_BSHR = 0x0,
    .GPIO_BCR = 0x0,
    .GPIO_LCKR = 0x0
};

struct gpio_regs_s portc = {
    .GPIO_CFGLR = 0x44444444,
    .reserved0 = 0x0,
    .GPIO_INDR = 0x0,
    .GPIO_OUTDR = 0x0,
    .GPIO_BSHR = 0x0,
    .GPIO_BCR = 0x0,
    .GPIO_LCKR = 0x0
};

struct gpio_regs_s portd = {
    .GPIO_CFGLR = 0x44444444,
    .reserved0 = 0x0,
    .GPIO_INDR = 0x0,
    .GPIO_OUTDR = 0x0,
    .GPIO_BSHR = 0x0,
    .GPIO_BCR = 0x0,
    .GPIO_LCKR = 0x0
};

void gpio_write(struct memarea_s * area, uint32_t addr, uint32_t val, int res)
{
    // fprintf(stderr, "Writing GPIO!\n");
    return;
}

struct usart_regs_s
{
    uint32_t USART_STATR;
    uint32_t USART_DATAR;
    uint32_t USART_BRR;
    uint32_t USART_CTLR1;
    uint32_t USART_CTLR2;
    uint32_t USART_CTLR3;
    uint32_t USART_GPR;
} usart = {
    .USART_STATR = 0xc0,
    .USART_DATAR = 0x0,
    .USART_BRR = 0x0,
    .USART_CTLR1 = 0x0,
    .USART_CTLR2 = 0x0,
    .USART_CTLR3 = 0x0,
    .USART_GPR = 0x0
};

void usart_write(struct memarea_s * area, uint32_t addr, uint32_t val, int res)
{
    // Handle USART_DATAR
    if (addr == area->addr + 4)
    {
        printf("%c", val & 0xff);
    }
    else
    {
        // fprintf(stderr, "Writing to usart 0x%x = 0x%x\n", addr, val);
    }
}

uint32_t usart_read(struct memarea_s * area, uint32_t addr, int res)
{
    uint32_t val = *((uint32_t *)((uint8_t *)&usart + (addr - area->addr)));

    // USART_STATR
    if (addr == area->addr + 0)
    {
        fprintf(stderr, "USART STARTR TC: %x\n", val & 64);
    }
    else
    {
        fprintf(stderr, "USART read: 0x%x\n", addr);
    }

    return val;
}

struct rcc_regs_s
{
    uint32_t RCC_CTLR;
    uint32_t RCC_CFGR0;
    uint32_t RCC_INTR;
    uint32_t RCC_APB2PRSTR;
    uint32_t RCC_APB1PRSTR;
    uint32_t RCC_AHBOCENT;
    uint32_t RCC_APB2PCENR;
    uint32_t RCC_APB1PCENR;
    uint32_t RCC_RSTSCKR;
} rcc = {
    .RCC_CTLR = 0x83 /* Reset value */ | 0x2000000 /* PLL Clock-ready lock flag bit */,
    .RCC_CFGR0 = 0x0 /* Reset value */,
    .RCC_INTR = 0x0,
    .RCC_APB2PRSTR = 0x0,
    .RCC_APB1PRSTR = 0x0,
    .RCC_AHBOCENT = 0x14,
    .RCC_APB2PCENR = 0x0,
    .RCC_APB1PCENR = 0x0,
    .RCC_RSTSCKR = 0x0
};

typedef struct
{
    __IO uint32_t CTLR;
    __IO uint32_t SR;
    __IO uint32_t CNT;
    uint32_t RESERVED0;
    __IO uint32_t CMP;
    uint32_t RESERVED1;
} SysTick_Type;

static SysTick_Type systick;

void rcc_write(struct memarea_s * area, uint32_t addr, uint32_t val, int res)
{
    // fprintf(stderr, "RCCCCC!!!! addr = 0x%x\n", addr);

    // RCC_CTLR
    if (addr == area->addr + 0)
    {
        uint32_t writeback = rcc.RCC_CTLR;

        // fprintf(stderr, "Writing 0x%x\n", val);

        // HSION -> HSIRDY
        if (val & 0b1)
        {
            writeback |= 0b10;
        }

        // HSITRIM -> HSICAL
        if (val & 0b11111000)
        {
                            // Default, starts with 24MHz
                            // Default value of HSITRIM is 0b10000 which translates to 24MHz
                            // Adjusted by 60KHz per step.
            writeback |= (((24 * 1024 * 1024) >> 17) +
                            (0b10000 - DECODER(val, 0b11111, 3, 0) * (60 * 1024))) << 8;
        }

        // HSEON -> HSERDY
        if (val & (0b1 << 16))
        {
            writeback |= 0b1 << 17;
        }

        // PLLON - > PLLRDY
        if (val & (0b1 << 24))
        {
            writeback |= 0b1 << 25;
        }

        rcc.RCC_CTLR = writeback;
    }

    // RCC_CFGR0 bits [2:1]
    if (addr == area->addr + 4)
    {
        uint32_t writeback = rcc.RCC_CFGR0;

        if (val & 0b11)
        {
            // fprintf(stderr, "Setting RCC_CFGR0\n");
            writeback |= (val & 0b11) << 2;
        }

        rcc.RCC_CFGR0 = writeback;
    }

    // RCC_INTR
    if (addr == area->addr + 8)
    {
        uint32_t writeback = rcc.RCC_INTR;

        // LSIRDYC -> Clear LSIRDYF
        if (val & (0b1 << 16))
        {
            writeback &= ~0b1;
        }

        //  HSIRDYC -> Clear HSIRDYF
        if (val & (0b1 << 18))
        {
            writeback &= ~0b100;
        }

        // HSERDYC -> Clear HSERDYF
        if (val & (0b1 << 19))
        {
            writeback &= ~0b1000;
        }

        // PLLRDYC -> Clear PLLRDYF
        if (val & (0b1 << 20))
        {
            writeback &= ~0b10000;
        }

        // CSSC -> Clear CSSF
        if (val & (0b1 << 23))
        {
            writeback &= ~0b10000000;
        }

        rcc.RCC_INTR = writeback;
    }
}

void flash_ctlr_write(struct memarea_s * area, uint32_t addr, uint32_t val, int res)
{
}

struct flash_ctlr_s
{
    uint32_t FLASH_ACTLR;
    uint32_t FLASH_KEYR;
    uint32_t FLASH_OBKEYR;
    uint32_t FLASH_STATR;
    uint32_t FLASH_CTLR;
    uint32_t FLASH_ADDR;
    uint32_t FLASH_OBR;
    uint32_t FLASH_WPR;
    uint32_t FLASH_MODEKEYR;
    uint32_t FLASH_BOOT_MODEKEYR;
} flash_ctlr;

struct memarea_s areas[] =
{
    {
        .name = "flash_mirror",
        .addr = 0x0,
        .size = MINIRV32_FLASH_SIZE,
        .data = NULL,
        .write = NULL,
        .read = NULL
    },
    {
        .name = "flash",
        .addr = 0x08000000,
        .size = MINIRV32_FLASH_SIZE,
        .data = NULL,
        .write = NULL,
        .read = NULL
    },
    {
        .name = "RAM",
        .addr = MINIRV32_RAM_IMAGE_OFFSET,
        .size = 16 * 1024 * 1024,
        .data = NULL,
        .write = NULL,
        .read = NULL
    },
    {
        .name = "PORTA",
        .addr = 0x40010800,
        .size = sizeof(struct gpio_regs_s),
        .data = &porta,
        .write = gpio_write,
        .read = NULL
    },
    {
        .name = "PORTC",
        .addr = 0x40011000,
        .size = sizeof(struct gpio_regs_s),
        .data = &portc,
        .write = gpio_write,
        .read = NULL
    },
    {
        .name = "PORTD",
        .addr = 0x40011400,
        .size = sizeof(struct gpio_regs_s),
        .data = &portd,
        .write = gpio_write,
        .read = NULL
    },
    {
        .name = "SPI",
        .addr = 0x40013000,
        .size = sizeof(struct spi_regs_s),
        .data = &spi,
        .write = spi_write,
        .read = spi_read
    },
    {
        .name = "USART",
        .addr = 0x40013800,
        .size = sizeof(struct usart_regs_s),
        .data = &usart,
        .write = usart_write,
        .read = usart_read
    },
    {
        .name = "RCC",
        .addr = 0x40021000,
        .size = sizeof(struct rcc_regs_s),
        .data = &rcc,
        .write = rcc_write,
        .read = NULL
    },
    {
        .name = "FLASH_CTLR",
        .addr = 0x40022000,
        .size = sizeof(struct flash_ctlr_s),
        .data = &flash_ctlr,
        .write = flash_ctlr_write,
        .read = NULL
    },
    {
        .name = "SysTick",
        .addr = 0xe000f000,
        .size = sizeof(SysTick_Type),
        .data = &systick,
        .write = NULL,
        .read = NULL
    }
};

int mmio_is_mapped(uint32_t addr)
{
    int err = -1;
    int i = 0;

    for (i = 0; i < sizeof(areas) / sizeof(struct memarea_s); ++i)
    {
        if ((areas[i].addr <= addr) && (addr < areas[i].addr + areas[i].size))
        {
            err = i;
            break;
        }
    }

    return err;
}

uint32_t mmio_get(uint32_t addr, int res)
{
    int i = mmio_is_mapped(addr);

#if 0
    if ((0x14e8 <= addr) && (addr < 0x1912))
    {
        fprintf(stderr, "READ bkpt on 0x%x\n", addr);
        bkpt();
    }
#endif
    if (i < 0)
    {
        fprintf(stderr, "Unmapped memory address: 0x%x\n", addr);
        goto error;
    }

    if (areas[i].read)
    {
        uint32_t val = areas[i].read(&areas[i], addr, res);

        // Treat resolution appropriately
        switch (res)
        {
            case 2:
                val &= 0xffff;
                break;
            case 1:
                val &= 0xff;
                break;
        }

        return val;
    }

    switch (res)
    {
        case 4:
            return *((uint32_t *)((uint8_t *)areas[i].data + addr - areas[i].addr));
        case 2:
            return *((uint16_t *)((uint8_t *)areas[i].data + addr - areas[i].addr));
        case 1:
            return *((uint8_t *)((uint8_t *)areas[i].data + addr - areas[i].addr));
    }

error:
    return 0;
}


void mmio_set(uint32_t addr, uint32_t val, int res)
{
    int i = mmio_is_mapped(addr);

    // fprintf(stderr, "Setting address: 0x%x\n", addr);

    if (i < 0)
    {
        fprintf(stderr, "mmio is not mapped for address 0x%x!\n", addr);
        goto error;
    }

    switch (res)
    {
        case 4:
            *((uint32_t *)((uint8_t *)areas[i].data + addr - areas[i].addr)) = val;
            break;
        case 2:
            *((uint16_t *)((uint8_t *)areas[i].data + addr - areas[i].addr)) = val;
            break;
        case 1:
            *((uint8_t *)((uint8_t *)areas[i].data + addr - areas[i].addr)) = val;
            break;
    }

    // fprintf(stderr, "mmio_set: %s[0x%x]\n", areas[i].name, addr);

    if (areas[i].write)
    {
        areas[i].write(&areas[i], addr, val, res);
    }
error:
    return;
}
