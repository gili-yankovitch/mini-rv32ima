#include <stdbool.h>

#ifndef __MMIO_H__
#define __MMIO_H__

struct memarea_s {
    char name[16];
    uint32_t addr;
    size_t size;
    void * data;
    void (* write)(struct memarea_s * area, uint32_t addr, uint32_t val, int res);
    uint32_t (* read)(struct memarea_s * area, uint32_t addr, int res);
};

extern bool hw_break;
extern struct memarea_s areas[];

void bkpt();
int mmio_is_mapped(uint32_t addr);
uint32_t mmio_get(uint32_t addr, int res);
void mmio_set(uint32_t addr, uint32_t val, int res);

#endif // __MMIO_H__
