// Copyright 2022 Charles Lohr, you may use this file or any portions herein under any of the BSD, MIT, or CC0 licenses.
#ifndef _MINI_RV32IMAH_H
#define _MINI_RV32IMAH_H

/**
    To use mini-rv32ima.h for the bare minimum, the following:

	#define MINI_RV32_RAM_SIZE ram_amt
	#define MINIRV32_IMPLEMENTATION

	#include "mini-rv32ima.h"

	Though, that's not _that_ interesting. You probably want I/O!


	Notes:
		* There is a dedicated CLNT at 0x10000000.
		* There is free MMIO from there to 0x12000000.
		* You can put things like a UART, or whatever there.
		* Feel free to override any of the functionality with macros.
*/

#ifndef MINIRV32WARN
	#define MINIRV32WARN( x... );
#endif

#ifndef MINIRV32_FLASH_IMAGE_OFFSET
    #define MINIRV32_FLASH_IMAGE_OFFSET 0x0
#endif

#ifndef MINIRV32_FLASH_SIZE
    #define MINIRV32_FLASH_SIZE (16 * 1024 * 1024)
#endif

#ifndef MINIRV32_RAM_IMAGE_OFFSET
	#define MINIRV32_RAM_IMAGE_OFFSET  0x20000000
#endif

#ifndef MINIRV32_POSTEXEC
	#define MINIRV32_POSTEXEC(...);
#endif

#ifndef MINIRV32_HANDLE_MEM_STORE_CONTROL
	#define MINIRV32_HANDLE_MEM_STORE_CONTROL(...);
#endif

#ifndef MINIRV32_HANDLE_MEM_LOAD_CONTROL
	#define MINIRV32_HANDLE_MEM_LOAD_CONTROL(...);
#endif

#ifndef MINIRV32_OTHERCSR_WRITE
	#define MINIRV32_OTHERCSR_WRITE(...);
#endif

#ifndef MINIRV32_OTHERCSR_READ
	#define MINIRV32_OTHERCSR_READ(...);
#endif

// As a note: We quouple-ify these, because in HLSL, we will be operating with
// uint4's.  We are going to uint4 data to/from system RAM.
//
// We're going to try to keep the full processor state to 12 x uint4.
struct MiniRV32IMAState
{
	uint32_t regs[32];

	uint32_t pc;
	uint32_t mstatus;
	uint32_t cyclel;
	uint32_t cycleh;

	uint32_t timerl;
	uint32_t timerh;
	uint32_t timermatchl;
	uint32_t timermatchh;

	uint32_t mscratch;
	uint32_t mtvec;
	uint32_t mie;
	uint32_t mip;

	uint32_t mepc;
	uint32_t mtval;
	uint32_t mcause;

	// Note: only a few bits are used.  (Machine = 3, User = 0)
	// Bits 0..1 = privilege.
	// Bit 2 = WFI (Wait for interrupt)
	// Bit 3+ = Load/Store reservation LSBs.
	uint32_t extraflags;
};

int32_t MiniRV32IMAStep( struct MiniRV32IMAState * state, uint8_t * flash, uint8_t * image, uint32_t vProcAddress, uint32_t elapsedUs, int count );

#endif
