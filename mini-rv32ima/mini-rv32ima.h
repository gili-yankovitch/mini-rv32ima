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

#ifndef MINIRV32_DECORATE
	#define MINIRV32_DECORATE static
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

#ifndef MINIRV32_CUSTOM_MEMORY_BUS
#ifdef NOMMU
	#define MINIRV32_STORE4( ofs, val ) *(uint32_t*)(image + ofs) = val
	#define MINIRV32_STORE2( ofs, val ) *(uint16_t*)(image + ofs) = val
	#define MINIRV32_STORE1( ofs, val ) *(uint8_t*)(image + ofs) = val
	#define MINIRV32_LOAD4( ofs ) *(uint32_t*)(image + ofs)
	#define MINIRV32_LOAD2( ofs ) *(uint16_t*)(image + ofs)
	#define MINIRV32_LOAD1( ofs ) *(uint8_t*)(image + ofs)
	#define MINIRV32_LOAD2_SIGNED( ofs ) *(int16_t*)(image + ofs)
	#define MINIRV32_LOAD1_SIGNED( ofs ) *(int8_t*)(image + ofs)
#else
#define CH32V003
#include "ch32v003fun.h"

static SysTick_Type systick;

    #define SYSTICK (0xE000F000)
    #define ADDR_TO_OFFSET(addr) ((addr) -                                          \
            ((SYSTICK <= (addr)) ? SYSTICK                                      :   \
             (MINIRV32_RAM_IMAGE_OFFSET <= (addr)) ? MINIRV32_RAM_IMAGE_OFFSET  :   \
             (MINIRV32_FLASH_IMAGE_OFFSET)))
    #define ADDR_TO_BASE(addr) \
            ((SYSTICK <= (addr)) ? (uint8_t *)&systick                          :   \
             (MINIRV32_RAM_IMAGE_OFFSET <= (addr)) ? image                      :   \
             (flash))

	#define MINIRV32_STORE4( addr, val ) (*(uint32_t*)(ADDR_TO_BASE(addr) + ADDR_TO_OFFSET(addr))) = val
	#define MINIRV32_STORE2( addr, val ) (*(uint16_t*)(ADDR_TO_BASE(addr) + ADDR_TO_OFFSET(addr))) = val
	#define MINIRV32_STORE1( addr, val ) (*(uint8_t*)(ADDR_TO_BASE(addr) + ADDR_TO_OFFSET(addr))) = val
	#define MINIRV32_LOAD4( addr ) (*(uint32_t*)(ADDR_TO_BASE(addr) + ADDR_TO_OFFSET(addr)))
	#define MINIRV32_LOAD2( addr ) (*(uint16_t*)(ADDR_TO_BASE(addr) + ADDR_TO_OFFSET(addr)))
	#define MINIRV32_LOAD1( addr ) (*(uint8_t*)(ADDR_TO_BASE(addr) + ADDR_TO_OFFSET(addr)))
	#define MINIRV32_LOAD2_SIGNED( addr ) (*(int16_t*)(ADDR_TO_BASE(addr) + ADDR_TO_OFFSET(addr)))
	#define MINIRV32_LOAD1_SIGNED( addr ) (*(int8_t*)(ADDR_TO_BASE(addr) + ADDR_TO_OFFSET(addr)))
#endif
#endif

// As a note: We quouple-ify these, because in HLSL, we will be operating with
// uint4's.  We are going to uint4 data to/from system RAM.
//
// We're going to try to keep the full processor state to 12 x uint4.
#define REGRA 1
#define REGSP 2
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

#ifndef MINIRV32_STEPPROTO
MINIRV32_DECORATE int32_t MiniRV32IMAStep( struct MiniRV32IMAState * state, uint8_t * flash, uint8_t * image, uint32_t vProcAddress, uint32_t elapsedUs, int count );
#endif

#ifdef MINIRV32_IMPLEMENTATION

#ifndef MINIRV32_CUSTOM_INTERNALS
#define CSR( x ) state->x
#define SETCSR( x, val ) { state->x = val; }
#define REG( x ) state->regs[x]
#define REGTAG(x) ((x) + 8)
#define REGSET( x, val ) { state->regs[x] = val; }
#endif

#ifndef MINIRV32_NO_RVC

#define DECODE(ir, mask, pos, dest)   (((ir) & ((mask) << (pos))) >> ((pos) - (dest)))
#define DECODER(ir, mask, pos, dest)   (((ir) & ((mask) << (pos))) << ((dest) - (pos)))

enum rvc_inst_e
{
    RVC_ADDI4SPN    = 0,
    RVC_ADDI,
    RVC_SLLI,
    RVC_FLD         = 4,
    RVC_JAL,
    RVC_FLDSP,
    RVC_LW          = 8,
    RVC_LI,
    RVC_LWSP,
    RVC_FLW         = 12,
    RVC_LUI,
    RVC_FLWSP,
    RVC_RESERVED    = 16,
    RVC_ALU,
    RVC_JALR_MV_ADD,
    RVC_FSD         = 20,
    RVC_J,
    RVC_FSDSP,
    RVC_SW          = 24,
    RVC_BEQZ,
    RVC_SWSP,
    RVC_FSW         = 28,
    RVC_BNEZ,
    RVC_FSWSP
};

enum rvc_alu_e
{
    RVC_ALU_SRLI            = 0,
    RVC_ALU_SRAI            = 4,
    RVC_ALU_ANDI            = 8,
    RVC_ALU_SUB             = 12,
    RVC_ALU_XOR             = 13,
    RVC_ALU_OR              = 14,
    RVC_ALU_AND             = 15
};

#endif

#ifndef MINIRV32_STEPPROTO
MINIRV32_DECORATE int32_t MiniRV32IMAStep( struct MiniRV32IMAState * state, uint8_t * flash, uint8_t * image, uint32_t vProcAddress, uint32_t elapsedUs, int count )
#else
MINIRV32_STEPPROTO
#endif
{
	uint32_t new_timer = CSR( timerl ) + elapsedUs;
	if( new_timer < CSR( timerl ) ) CSR( timerh )++;
	CSR( timerl ) = new_timer;

	// Handle Timer interrupt.
	if( ( CSR( timerh ) > CSR( timermatchh ) || ( CSR( timerh ) == CSR( timermatchh ) && CSR( timerl ) > CSR( timermatchl ) ) ) && ( CSR( timermatchh ) || CSR( timermatchl ) ) )
	{
		CSR( extraflags ) &= ~4; // Clear WFI
		CSR( mip ) |= 1<<7; //MTIP of MIP // https://stackoverflow.com/a/61916199/2926815  Fire interrupt.
	}
	else
		CSR( mip ) &= ~(1<<7);

	// If WFI, don't run processor.
	if( CSR( extraflags ) & 4 )
		return 1;

	uint32_t trap = 0;
	uint32_t rval = 0;
	uint32_t pc = CSR( pc );
	uint32_t cycle = CSR( cyclel );

	if( ( CSR( mip ) & (1<<7) ) && ( CSR( mie ) & (1<<7) /*mtie*/ ) && ( CSR( mstatus ) & 0x8 /*mie*/) )
	{
		// Timer interrupt.
		trap = 0x80000007;
		pc -= 4;
	}
	else // No timer interrupt?  Execute a bunch of instructions.
	for( int icount = 0; icount < count; icount++ )
	{
		uint32_t ir = 0;
		rval = 0;
		cycle++;
		uint32_t ofs_pc = pc; // - MINIRV32_RAM_IMAGE_OFFSET;

		if( ofs_pc >= MINI_RV32_RAM_SIZE )
		{
            fprintf(stderr, "%s:%d ofs_pc = 0x%x\n", __FUNCTION__, __LINE__, ofs_pc);
            exit(1);
			trap = 1 + 1;  // Handle access violation on instruction read.
			break;
		}
		else if( ofs_pc & 1 )
		{
            fprintf(stderr, "%s:%d ofs_pc = 0x%x\n", __FUNCTION__, __LINE__, ofs_pc);
			trap = 1 + 0;  //Handle PC-misaligned access
			break;
		}
		else
		{
		    fprintf(stderr, "Fetching from 0x%x\n", ofs_pc);
			ir = MINIRV32_LOAD4( ofs_pc );
			uint32_t rdid = (ir >> 7) & 0x1f;
#ifndef MINIRV32_NO_RVC
            // Use bits [15:13] | [1:0]
            uint8_t rvc_opcode = (((ir >> 8) & 0b11100000) >> 3) | (ir & 0b11);
            int long_inst = 0;

            fprintf(stderr, "RVC opcode: 0x%x\n", rvc_opcode);

            switch (rvc_opcode)
            {
                case RVC_LI:
                {
                    uint32_t imm = ((ir & (1 << 5)) >> 7) | ((ir & (0b11111 << 2)) >> 2);
                    uint32_t reg = (ir & (0b11111 << 7)) >> 7;

                    REG(reg) = imm;
                    break;
                }

                case RVC_ADDI:
                {
                    uint32_t rs1d = DECODE(ir, 0b11111, 7, 0);
                    uint32_t imm = DECODE(ir, 0b11111, 2, 0) | DECODE(ir, 0b1, 12, 5);

                    fprintf(stderr, "Adding immediate %x to register %x\n", imm ,rs1d);

                    REG(rs1d) += imm;

                    break;
                }

                case RVC_LW:
                {
                    uint32_t rs1 = REGTAG(DECODE(ir, 0b111, 7, 0));
                    uint32_t rd = REGTAG(DECODE(ir, 0b111, 2, 0));
                    uint32_t uimm = DECODE(ir, 0b111, 10, 3) | DECODER(ir, 0b1, 5, 6) | DECODE(ir, 0b1, 6, 2);
                    uint32_t addx = REG(rs1) + uimm; //  - MINIRV32_RAM_IMAGE_OFFSET;

                    fprintf(stderr, "Accessing 0x%x\n", addx);
                    REG(rd) = MINIRV32_LOAD4(addx);

                    break;
                }

                case RVC_LUI:
                {
                    uint32_t rd = DECODE(ir, 0b11111, 7 ,0);
                    uint32_t imm;

                    if (rd == 2)
                    {
                        imm = DECODER(ir, 0b1, 2, 5) |
                            DECODER(ir, 0b11, 3, 7) |
                            DECODER(ir, 0b1, 5, 6) |
                            DECODE(ir, 0b1, 6, 4) |
                            DECODE(ir, 0b1, 12, 9);

                        // Sign-extend?
                        if (DECODE(ir, 0b1, 12, 9))
                        {
                            imm |= 0xffffffff & ~0b111111111;
                        }

                        REG(rd) += imm;
                    }
                    else
                    {
                        imm = DECODER(ir, 0b1, 12, 17) | DECODER(ir, 0b11111, 2, 12);

                        REG(rd) = (REG(rd) & 0x7ff) | imm;
                    }

                    break;
                }

                case RVC_SWSP:
                {
                    uint32_t rs2 = DECODE(ir, 0b11111, 2, 0);
                    uint32_t imm = DECODE(ir, 0b111, 7, 6) | DECODE(ir, 0b1111, 10, 2);
                    uint32_t addy = REG(REGSP) + imm;

                    MINIRV32_STORE4(addy, REG(rs2));
                    break;
                }

                case RVC_JAL:
                {
                    uint32_t imm = DECODER(ir, 0b1, 2, 5)   |
                        DECODE(ir, 0b111, 3, 1)             |
                        DECODER(ir, 0b1, 6, 7)              |
                        DECODE(ir, 0b1, 7, 6)               |
                        DECODER(ir, 0b1, 8, 10)             |
                        DECODE(ir, 0b11, 9, 8)              |
                        DECODE(ir, 0b1, 11, 4)              |
                        DECODE(ir, 0b1, 12, 11);

                    // Sign-extend?
                    if (DECODE(ir, 0b1, 12, 11))
                    {
                        imm |= 0xffffffff & ~0b11111111111;
                    }

                    // Set ra PC + 2
                    REG(REGRA) = pc + 2;

                    fprintf(stderr, "JAL branch to 0x%x\n", pc + imm);

                    // Branch (ignore + instruction at the end)
                    pc += imm - 2;

                    break;
                }

                case RVC_ALU:
                {
                    uint32_t alu_opcode = DECODE(ir, 0b11, 10, 2);
                    uint32_t rs1d = REGTAG(DECODE(ir, 0b111, 7, 0));

                    if (alu_opcode >= RVC_ALU_SUB)
                    {
                        alu_opcode |= (ir & (0b11 << 5)) >> 5;
                    }

                    switch (alu_opcode)
                    {
                        case RVC_ALU_SRLI:
                        {
                            uint32_t imm = ((ir & (0b1 << 12)) >> 7) | ((ir & (0b11111 << 2)) >> 2);
                            REG(rs1d) = REG(rs1d) >> imm;
                            break;
                        }

                        case RVC_ALU_SRAI:
                        {
                            uint32_t imm = ((ir & (0b1 << 12)) >> 7) | ((ir & (0b11111 << 2)) >> 2);
                            REG(rs1d) = (int)REG(rs1d) >> imm;
                            break;
                        }
                        case RVC_ALU_ANDI:
                        {
                            uint32_t imm = ((ir & (0b1 << 12)) >> 7) | ((ir & (0b11111 << 2)) >> 2);

                            REG(rs1d) = REG(rs1d) & imm;

                            break;
                        }
                        case RVC_ALU_SUB:
                        {
                            uint32_t rs2 = REGTAG(DECODE(ir, 0b111, 2, 0));
                            REG(rs1d) = REG(rs1d) - REG(rs2);

                            break;
                        }
                        case RVC_ALU_XOR:
                        {
                            uint32_t rs2 = REGTAG(DECODE(ir, 0b111, 2, 0));
                            REG(rs1d) = REG(rs1d) ^ REG(rs2);

                            break;
                        }
                        case RVC_ALU_OR:
                        {
                            uint32_t rs2 = REGTAG(DECODE(ir, 0b111, 2, 0));
                            REG(rs1d) = REG(rs1d) | REG(rs2);

                            break;
                        }
                        case RVC_ALU_AND:
                        {
                            uint32_t rs2 = REGTAG(DECODE(ir, 0b111, 2, 0));
                            REG(rs1d) = REG(rs1d) & REG(rs2);

                            break;
                        }
                        default:
                        {
                            fprintf(stderr, "Invalid opcode\n");

                            break;
                        }
                    }
                    break;
                }

                case RVC_SW:
                {
                    uint32_t rs1 = REGTAG(DECODE(ir, 0b111, 7, 0));
                    uint32_t rs2 = REGTAG(DECODE(ir, 0b111, 2, 0));
                    uint32_t uimm = DECODE(ir, 0b111, 10, 3) | DECODER(ir, 0b1, 5, 6) | DECODE(ir, 0b1, 6, 2);
                    uint32_t addy = REG(rs1) + uimm; // - MINIRV32_RAM_IMAGE_OFFSET;

                    fprintf(stderr, "rs1 = 0x%x\n", REG(rs1));
                    fprintf(stderr, "sw addy = 0x%x <= rs2 = 0x%x\n", addy, REG(rs2));
                    if (addy <= MINI_RV32_RAM_SIZE - 3)
                    {
                        MINIRV32_STORE4( addy, REG(rs2) );
                    }

                    break;
                }

                default:
                {
                    // 32-bit instruction
                    long_inst = 1;
                }
            }

            // Already handled in the above switch
            if (!long_inst)
            {
                pc += 2;

                continue;
            }
#endif
			switch( ir & 0x7f )
			{
				case 0x37: // LUI (0b0110111)
					rval = ( ir & 0xfffff000 );
					break;
				case 0x17: // AUIPC (0b0010111)
					rval = pc + ( ir & 0xfffff000 );
					fprintf(stderr, "rval = 0x%x pc = 0x%x\n", rval, pc);
					break;
				case 0x6F: // JAL (0b1101111)
				{
					int32_t reladdy = ((ir & 0x80000000)>>11) | ((ir & 0x7fe00000)>>20) | ((ir & 0x00100000)>>9) | ((ir&0x000ff000));
					if( reladdy & 0x00100000 ) reladdy |= 0xffe00000; // Sign extension.
					rval = pc + 4;
					pc = pc + reladdy - 4;
					fprintf(stderr, "Branching to 0x%x\n", pc);
					break;
				}
				case 0x67: // JALR (0b1100111)
				{
					uint32_t imm = ir >> 20;
					int32_t imm_se = imm | (( imm & 0x800 )?0xfffff000:0);
					rval = pc + 4;
					pc = ( (REG( (ir >> 15) & 0x1f ) + imm_se) & ~1) - 4;
					break;
				}
				case 0x63: // Branch (0b1100011)
				{
					uint32_t immm4 = ((ir & 0xf00)>>7) | ((ir & 0x7e000000)>>20) | ((ir & 0x80) << 4) | ((ir >> 31)<<12);
					if( immm4 & 0x1000 ) immm4 |= 0xffffe000;
					int32_t rs1 = REG((ir >> 15) & 0x1f);
					int32_t rs2 = REG((ir >> 20) & 0x1f);
					immm4 = pc + immm4 - 4;
					rdid = 0;
					switch( ( ir >> 12 ) & 0x7 )
					{
						// BEQ, BNE, BLT, BGE, BLTU, BGEU
						case 0: if( rs1 == rs2 ) pc = immm4; break;
						case 1: if( rs1 != rs2 ) pc = immm4; break;
						case 4: if( rs1 < rs2 ) pc = immm4; break;
						case 5: if( rs1 >= rs2 ) pc = immm4; break; //BGE
						case 6: if( (uint32_t)rs1 < (uint32_t)rs2 ) pc = immm4; break;   //BLTU
						case 7: if( (uint32_t)rs1 >= (uint32_t)rs2 ) pc = immm4; break;  //BGEU
						default: trap = (2+1);
					}
					break;
				}
				case 0x03: // Load (0b0000011)
				{
					uint32_t rs1 = REG((ir >> 15) & 0x1f);
					uint32_t imm = ir >> 20;
					int32_t imm_se = imm | (( imm & 0x800 )?0xfffff000:0);
					uint32_t rsval = rs1 + imm_se;

					rsval -= MINIRV32_RAM_IMAGE_OFFSET;
					if( rsval >= MINI_RV32_RAM_SIZE-3 )
					{
						rsval += MINIRV32_RAM_IMAGE_OFFSET;
						if( rsval >= 0x10000000 && rsval < 0x12000000 )  // UART, CLNT
						{
							if( rsval == 0x1100bffc ) // https://chromitem-soc.readthedocs.io/en/latest/clint.html
								rval = CSR( timerh );
							else if( rsval == 0x1100bff8 )
								rval = CSR( timerl );
							else
								MINIRV32_HANDLE_MEM_LOAD_CONTROL( rsval, rval );
						}
						else
						{
							trap = (5+1);
							rval = rsval;
						}
					}
					else
					{
						switch( ( ir >> 12 ) & 0x7 )
						{
							//LB, LH, LW, LBU, LHU
							case 0: rval = MINIRV32_LOAD1_SIGNED( rsval ); break;
							case 1: rval = MINIRV32_LOAD2_SIGNED( rsval ); break;
							case 2: rval = MINIRV32_LOAD4( rsval ); break;
							case 4: rval = MINIRV32_LOAD1( rsval ); break;
							case 5: rval = MINIRV32_LOAD2( rsval ); break;
							default: trap = (2+1);
						}
					}
					break;
				}
				case 0x23: // Store 0b0100011
				{
					uint32_t rs1 = REG((ir >> 15) & 0x1f);
					uint32_t rs2 = REG((ir >> 20) & 0x1f);
					uint32_t addy = ( ( ir >> 7 ) & 0x1f ) | ( ( ir & 0xfe000000 ) >> 20 );
					if( addy & 0x800 ) addy |= 0xfffff000;
					addy += rs1; // - MINIRV32_RAM_IMAGE_OFFSET;
					rdid = 0;

                    fprintf(stderr, "rs1 = 0x%x\n", rs1);
                    fprintf(stderr, "%s:%d addy = 0x%x BASE = 0x%x\n", __FUNCTION__, __LINE__, addy, MINIRV32_RAM_IMAGE_OFFSET);

					if( addy >= MINI_RV32_RAM_SIZE-3 )
					{
                    fprintf(stderr, "%s:%d rs1 - %x %x\n", __FUNCTION__, __LINE__, rs1, addy);
						addy += MINIRV32_RAM_IMAGE_OFFSET;
						if( addy >= 0x10000000 && addy < 0x12000000 )
						{
                    fprintf(stderr, "%s:%d\n", __FUNCTION__, __LINE__);
							// Should be stuff like SYSCON, 8250, CLNT
							if( addy == 0x11004004 ) //CLNT
								CSR( timermatchh ) = rs2;
							else if( addy == 0x11004000 ) //CLNT
								CSR( timermatchl ) = rs2;
							else if( addy == 0x11100000 ) //SYSCON (reboot, poweroff, etc.)
							{
								SETCSR( pc, pc + 4 );
								return rs2; // NOTE: PC will be PC of Syscon.
							}
							else
								MINIRV32_HANDLE_MEM_STORE_CONTROL( addy, rs2 );
						}
						else
						{
                    fprintf(stderr, "%s:%d\n", __FUNCTION__, __LINE__);
							trap = (7+1); // Store access fault.
							rval = addy;
						}
					}
					else
					{
						switch( ( ir >> 12 ) & 0x7 )
						{
							//SB, SH, SW
							case 0: MINIRV32_STORE1( addy, rs2 ); break;
							case 1: MINIRV32_STORE2( addy, rs2 ); break;
							case 2: MINIRV32_STORE4( addy, rs2 ); break;
							default: trap = (2+1);
						}
					}
					break;
				}
				case 0x13: // Op-immediate 0b0010011
				case 0x33: // Op           0b0110011
				{
					uint32_t imm = ir >> 20;
					imm = imm | (( imm & 0x800 )?0xfffff000:0);
					uint32_t rs1 = REG((ir >> 15) & 0x1f);
					uint32_t is_reg = !!( ir & 0x20 );
					uint32_t rs2 = is_reg ? REG(imm & 0x1f) : imm;

					if( is_reg && ( ir & 0x02000000 ) )
					{
						switch( (ir>>12)&7 ) //0x02000000 = RV32M
						{
							case 0: rval = rs1 * rs2; break; // MUL
#ifndef CUSTOM_MULH // If compiling on a system that doesn't natively, or via libgcc support 64-bit math.
							case 1: rval = ((int64_t)((int32_t)rs1) * (int64_t)((int32_t)rs2)) >> 32; break; // MULH
							case 2: rval = ((int64_t)((int32_t)rs1) * (uint64_t)rs2) >> 32; break; // MULHSU
							case 3: rval = ((uint64_t)rs1 * (uint64_t)rs2) >> 32; break; // MULHU
#else
							CUSTOM_MULH
#endif
							case 4: if( rs2 == 0 ) rval = -1; else rval = ((int32_t)rs1 == INT32_MIN && (int32_t)rs2 == -1) ? rs1 : ((int32_t)rs1 / (int32_t)rs2); break; // DIV
							case 5: if( rs2 == 0 ) rval = 0xffffffff; else rval = rs1 / rs2; break; // DIVU
							case 6: if( rs2 == 0 ) rval = rs1; else rval = ((int32_t)rs1 == INT32_MIN && (int32_t)rs2 == -1) ? 0 : ((uint32_t)((int32_t)rs1 % (int32_t)rs2)); break; // REM
							case 7: if( rs2 == 0 ) rval = rs1; else rval = rs1 % rs2; break; // REMU
						}
					}
					else
					{
					    fprintf(stderr, "Arith rs1 = 0x%x rs2 = 0x%x\n", rs1, rs2);
						switch( (ir>>12)&7 ) // These could be either op-immediate or op commands.  Be careful.
						{
							case 0: rval = (is_reg && (ir & 0x40000000) ) ? ( rs1 - rs2 ) : ( rs1 + rs2 ); break; 
							case 1: rval = rs1 << (rs2 & 0x1F); break;
							case 2: rval = (int32_t)rs1 < (int32_t)rs2; break;
							case 3: rval = rs1 < rs2; break;
							case 4: rval = rs1 ^ rs2; break;
							case 5: rval = (ir & 0x40000000 ) ? ( ((int32_t)rs1) >> (rs2 & 0x1F) ) : ( rs1 >> (rs2 & 0x1F) ); break;
							case 6: rval = rs1 | rs2; break;
							case 7: rval = rs1 & rs2; break;
						}
					}
					break;
				}
				case 0x0f: // 0b0001111
					rdid = 0;   // fencetype = (ir >> 12) & 0b111; We ignore fences in this impl.
					break;
				case 0x73: // Zifencei+Zicsr  (0b1110011)
				{
					uint32_t csrno = ir >> 20;
					uint32_t microop = ( ir >> 12 ) & 0x7;
					if( (microop & 3) ) // It's a Zicsr function.
					{
						int rs1imm = (ir >> 15) & 0x1f;
						uint32_t rs1 = REG(rs1imm);
						uint32_t writeval = rs1;

						// https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
						// Generally, support for Zicsr
						switch( csrno )
						{
						case 0x340: rval = CSR( mscratch ); break;
						case 0x305: rval = CSR( mtvec ); break;
						case 0x304: rval = CSR( mie ); break;
						case 0xC00: rval = cycle; break;
						case 0x344: rval = CSR( mip ); break;
						case 0x341: rval = CSR( mepc ); break;
						case 0x300: rval = CSR( mstatus ); break; //mstatus
						case 0x342: rval = CSR( mcause ); break;
						case 0x343: rval = CSR( mtval ); break;
						case 0xf11: rval = 0xff0ff0ff; break; //mvendorid
						case 0x301: rval = 0x40401101; break; //misa (XLEN=32, IMA+X)
						//case 0x3B0: rval = 0; break; //pmpaddr0
						//case 0x3a0: rval = 0; break; //pmpcfg0
						//case 0xf12: rval = 0x00000000; break; //marchid
						//case 0xf13: rval = 0x00000000; break; //mimpid
						//case 0xf14: rval = 0x00000000; break; //mhartid
						default:
							MINIRV32_OTHERCSR_READ( csrno, rval );
							break;
						}

						switch( microop )
						{
							case 1: writeval = rs1; break;  			//CSRRW
							case 2: writeval = rval | rs1; break;		//CSRRS
							case 3: writeval = rval & ~rs1; break;		//CSRRC
							case 5: writeval = rs1imm; break;			//CSRRWI
							case 6: writeval = rval | rs1imm; break;	//CSRRSI
							case 7: writeval = rval & ~rs1imm; break;	//CSRRCI
						}

						switch( csrno )
						{
						case 0x340: SETCSR( mscratch, writeval ); break;
						case 0x305: SETCSR( mtvec, writeval ); break;
						case 0x304: SETCSR( mie, writeval ); break;
						case 0x344: SETCSR( mip, writeval ); break;
						case 0x341: SETCSR( mepc, writeval ); break;
						case 0x300: SETCSR( mstatus, writeval ); break; //mstatus
						case 0x342: SETCSR( mcause, writeval ); break;
						case 0x343: SETCSR( mtval, writeval ); break;
						//case 0x3a0: break; //pmpcfg0
						//case 0x3B0: break; //pmpaddr0
						//case 0xf11: break; //mvendorid
						//case 0xf12: break; //marchid
						//case 0xf13: break; //mimpid
						//case 0xf14: break; //mhartid
						//case 0x301: break; //misa
						default:
							MINIRV32_OTHERCSR_WRITE( csrno, writeval );
							break;
						}
					}
					else if( microop == 0x0 ) // "SYSTEM" 0b000
					{
						rdid = 0;
						if( csrno == 0x105 ) //WFI (Wait for interrupts)
						{
							CSR( mstatus ) |= 8;    //Enable interrupts
							CSR( extraflags ) |= 4; //Infor environment we want to go to sleep.
							SETCSR( pc, pc + 4 );
							return 1;
						}
						else if( ( ( csrno & 0xff ) == 0x02 ) )  // MRET
						{
							//https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
							//Table 7.6. MRET then in mstatus/mstatush sets MPV=0, MPP=0, MIE=MPIE, and MPIE=1. La
							// Should also update mstatus to reflect correct mode.
							uint32_t startmstatus = CSR( mstatus );
							uint32_t startextraflags = CSR( extraflags );
							SETCSR( mstatus , (( startmstatus & 0x80) >> 4) | ((startextraflags&3) << 11) | 0x80 );
							SETCSR( extraflags, (startextraflags & ~3) | ((startmstatus >> 11) & 3) );
							pc = CSR( mepc ) -4;
						}
						else
						{
							switch( csrno )
							{
							case 0: trap = ( CSR( extraflags ) & 3) ? (11+1) : (8+1); break; // ECALL; 8 = "Environment call from U-mode"; 11 = "Environment call from M-mode"
							case 1:	trap = (3+1); break; // EBREAK 3 = "Breakpoint"
							default: trap = (2+1); break; // Illegal opcode.
							}
						}
					}
					else
						trap = (2+1); 				// Note micrrop 0b100 == undefined.
					break;
				}
				case 0x2f: // RV32A (0b00101111)
				{
					uint32_t rs1 = REG((ir >> 15) & 0x1f);
					uint32_t rs2 = REG((ir >> 20) & 0x1f);
					uint32_t irmid = ( ir>>27 ) & 0x1f;

					rs1 -= MINIRV32_RAM_IMAGE_OFFSET;

					// We don't implement load/store from UART or CLNT with RV32A here.

					if( rs1 >= MINI_RV32_RAM_SIZE-3 )
					{
						trap = (7+1); //Store/AMO access fault
						rval = rs1 + MINIRV32_RAM_IMAGE_OFFSET;
					}
					else
					{
						rval = MINIRV32_LOAD4( rs1 );

						// Referenced a little bit of https://github.com/franzflasch/riscv_em/blob/master/src/core/core.c
						uint32_t dowrite = 1;
						switch( irmid )
						{
							case 2: //LR.W (0b00010)
								dowrite = 0;
								CSR( extraflags ) = (CSR( extraflags ) & 0x07) | (rs1<<3);
								break;
							case 3:  //SC.W (0b00011) (Make sure we have a slot, and, it's valid)
								rval = ( CSR( extraflags ) >> 3 != ( rs1 & 0x1fffffff ) );  // Validate that our reservation slot is OK.
								dowrite = !rval; // Only write if slot is valid.
								break;
							case 1: break; //AMOSWAP.W (0b00001)
							case 0: rs2 += rval; break; //AMOADD.W (0b00000)
							case 4: rs2 ^= rval; break; //AMOXOR.W (0b00100)
							case 12: rs2 &= rval; break; //AMOAND.W (0b01100)
							case 8: rs2 |= rval; break; //AMOOR.W (0b01000)
							case 16: rs2 = ((int32_t)rs2<(int32_t)rval)?rs2:rval; break; //AMOMIN.W (0b10000)
							case 20: rs2 = ((int32_t)rs2>(int32_t)rval)?rs2:rval; break; //AMOMAX.W (0b10100)
							case 24: rs2 = (rs2<rval)?rs2:rval; break; //AMOMINU.W (0b11000)
							case 28: rs2 = (rs2>rval)?rs2:rval; break; //AMOMAXU.W (0b11100)
							default: trap = (2+1); dowrite = 0; break; //Not supported.
						}
						if( dowrite ) MINIRV32_STORE4( rs1, rs2 );
					}
					break;
				}
				default: trap = (2+1); // Fault: Invalid opcode.
			}

			// If there was a trap, do NOT allow register writeback.
			if( trap )
				break;

			if( rdid )
			{
				REGSET( rdid, rval ); // Write back register.
			}
		}

		MINIRV32_POSTEXEC( pc, ir, trap );

		pc += 4;
	}

	// Handle traps and interrupts.
	if( trap )
	{
		if( trap & 0x80000000 ) // If prefixed with 1 in MSB, it's an interrupt, not a trap.
		{
			SETCSR( mcause, trap );
			SETCSR( mtval, 0 );
			pc += 4; // PC needs to point to where the PC will return to.
		}
		else
		{
			SETCSR( mcause,  trap - 1 );
			SETCSR( mtval, (trap > 5 && trap <= 8)? rval : pc );
		}
		SETCSR( mepc, pc ); //TRICKY: The kernel advances mepc automatically.
		//CSR( mstatus ) & 8 = MIE, & 0x80 = MPIE
		// On an interrupt, the system moves current MIE into MPIE
		SETCSR( mstatus, (( CSR( mstatus ) & 0x08) << 4) | (( CSR( extraflags ) & 3 ) << 11) );
		pc = (CSR( mtvec ) - 4);

		// If trapping, always enter machine mode.
		CSR( extraflags ) |= 3;

		trap = 0;
		pc += 4;
	}

	if( CSR( cyclel ) > cycle ) CSR( cycleh )++;
	SETCSR( cyclel, cycle );
	SETCSR( pc, pc );
	return 0;
}

#endif

#endif


