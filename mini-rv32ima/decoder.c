#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "mini-rv32ima.h"
#include "mmio.h"

#define DECODER(ir, mask, pos, dest)   (((ir) & ((mask) << (pos))) >> ((pos) - (dest)))
#define DECODEL(ir, mask, pos, dest)   (((ir) & ((mask) << (pos))) << ((dest) - (pos)))

#define MINIRV32_STORE4( addr, val ) mmio_set(addr, val, 4)
#define MINIRV32_STORE2( addr, val ) mmio_set(addr, val, 2)
#define MINIRV32_STORE1( addr, val ) mmio_set(addr, val, 1)
#define MINIRV32_LOAD4( addr ) (uint32_t)mmio_get(addr, 4)
#define MINIRV32_LOAD2( addr ) (uint16_t)mmio_get(addr, 2)
#define MINIRV32_LOAD1( addr ) (uint8_t) mmio_get(addr, 1)
#define MINIRV32_LOAD2_SIGNED( addr ) (int16_t)mmio_get(addr, 2)
#define MINIRV32_LOAD1_SIGNED( addr ) (int16_t)mmio_get(addr, 1)

#define REGRA 1
#define REGSP 2

#ifndef MINIRV32_CUSTOM_INTERNALS
#define CSR( x ) state->x
#define SETCSR( x, val ) { state->x = val; }
#define REG( x ) state->regs[x]
#define REGTAG(x) ((x) + 8)
#define REGSET( x, val ) { state->regs[x] = val; }
#endif

#ifndef MINIRV32_NO_RVC

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

extern uint32_t ram_amt;

int32_t MiniRV32IMAStep( struct MiniRV32IMAState * state, uint8_t * flash, uint8_t * image, uint32_t vProcAddress, uint32_t elapsedUs, int count )
{
	uint32_t new_timer = CSR( timerl ) + elapsedUs;
	if( new_timer < CSR( timerl ) ) CSR( timerh )++;
	CSR( timerl ) = new_timer;

    // Set memory areas
    areas[0].data = flash;
    areas[1].data = flash;
    areas[2].data = image;

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

		if (pc == 0x115c)
		{
		    // bkpt();
		}
#if 0
		if( ofs_pc >= ram_amt )
		{
            // fprintf(stderr, "%s:%d ofs_pc = 0x%x\n", __FUNCTION__, __LINE__, ofs_pc);
            exit(1);
			trap = 1 + 1;  // Handle access violation on instruction read.
			break;
		}
		else
#endif
		if( ofs_pc & 1 )
		{
            fprintf(stderr, "%s:%d ofs_pc = 0x%x\n", __FUNCTION__, __LINE__, ofs_pc);
			trap = 1 + 0;  //Handle PC-misaligned access
			exit(1);
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

            // fprintf(stderr, "RVC opcode: 0x%x (0x%x)\n", rvc_opcode, ir & 0xffff);

            switch (rvc_opcode)
            {
                case RVC_ADDI4SPN:
                {
                    uint32_t rd = REGTAG(DECODER(ir, 0b111, 2, 0));
                    uint32_t imm =
                                DECODER(ir, 0b1, 5, 3) |
                                DECODER(ir, 0b1, 6, 2) |
                                DECODER(ir, 0b1111, 7, 6) |
                                DECODER(ir, 0b11, 11, 4);


                    REG(rd) = REG(REGSP) + imm;

                    break;
                }

                case RVC_LI:
                {
                    uint32_t imm = ((ir & (1 << 5)) >> 7) | ((ir & (0b11111 << 2)) >> 2);
                    uint32_t reg = (ir & (0b11111 << 7)) >> 7;

                    fprintf(stderr, "Loading reg 0x%x with imm 0x%x\n", reg, imm);

                    REG(reg) = imm;
                    break;
                }

                case RVC_ADDI:
                {
                    uint32_t rs1d = DECODER(ir, 0b11111, 7, 0);
                    uint32_t imm = DECODER(ir, 0b11111, 2, 0) | DECODER(ir, 0b1, 12, 5);

                    // Sign-extend
                    if (DECODER(ir, 0b1, 12, 5))
                    {
                        imm |= 0xffffffc0;
                    }

                    fprintf(stderr, "Adding immediate %x to register %x (0x%x)\n", imm , rs1d, REG(rs1d));

                    REG(rs1d) += imm;

                    // fprintf(stderr, " ( = 0x%x)\n", REG(rs1d));

                    break;
                }

                case RVC_SLLI:
                {
                    uint32_t rs1d = DECODER(ir, 0b11111, 7, 0);
                    uint32_t imm = DECODER(ir, 0b11111, 2, 0) | DECODER(ir, 0b1, 12, 5);

                    REG(rs1d) <<= imm;

                    break;
                }

                case RVC_LW:
                {
                    uint32_t rs1 = REGTAG(DECODER(ir, 0b111, 7, 0));
                    uint32_t rd = REGTAG(DECODER(ir, 0b111, 2, 0));
                    uint32_t uimm = DECODER(ir, 0b111, 10, 3) | DECODEL(ir, 0b1, 5, 6) | DECODER(ir, 0b1, 6, 2);
                    uint32_t addx = REG(rs1) + uimm; //  - MINIRV32_RAM_IMAGE_OFFSET;

                    // fprintf(stderr, "Accessing 0x%x\n", addx);
                    REG(rd) = MINIRV32_LOAD4(addx);

                    break;
                }

                case RVC_LUI:
                {
                    uint32_t rd = DECODER(ir, 0b11111, 7 ,0);
                    uint32_t imm;

                    if (rd == 2)
                    {
                        imm = DECODEL(ir, 0b1, 2, 5) |
                            DECODEL(ir, 0b11, 3, 7) |
                            DECODEL(ir, 0b1, 5, 6) |
                            DECODER(ir, 0b1, 6, 4) |
                            DECODER(ir, 0b1, 12, 9);
#if 1
                        // Sign-extend?
                        if (DECODER(ir, 0b1, 12, 9))
                        {
                            imm |= 0xffffffff & ~0b111111111;
                        }
#endif
                        REG(rd) += imm;
                    }
                    else
                    {
                        imm = DECODEL(ir, 0b1, 12, 17) | DECODEL(ir, 0b11111, 2, 12);

                        REG(rd) = imm;
                    }

                    fprintf(stderr, "\t Load upper-immediate: 0x%x\n", imm);
                    break;
                }

                case RVC_SWSP:
                {
                    uint32_t rs2 = DECODER(ir, 0b11111, 2, 0);
                    uint32_t imm = DECODER(ir, 0b11, 7, 6) | DECODER(ir, 0b1111, 9, 2);
                    uint32_t addy = REG(REGSP) + imm;

                    fprintf(stderr, "Storing to SP with offset 0x%x\n", imm);

                    MINIRV32_STORE4(addy, REG(rs2));
                    break;
                }

                case RVC_JAL:
                {
                    uint32_t imm = DECODEL(ir, 0b1, 2, 5)   |
                        DECODER(ir, 0b111, 3, 1)             |
                        DECODEL(ir, 0b1, 6, 7)              |
                        DECODER(ir, 0b1, 7, 6)               |
                        DECODEL(ir, 0b1, 8, 10)             |
                        DECODER(ir, 0b11, 9, 8)              |
                        DECODER(ir, 0b1, 11, 4)              |
                        DECODER(ir, 0b1, 12, 11);

                    // Sign-extend?
                    if (DECODER(ir, 0b1, 12, 11))
                    {
                        imm |= 0xffffffff & ~0b11111111111;
                    }

                    // Set ra PC + 2
                    REG(REGRA) = pc + 2;

                    // fprintf(stderr, "JAL branch to 0x%x\n", pc + imm);

                    // Branch (ignore + instruction at the end)
                    pc += imm - 2;

                    break;
                }

                case RVC_BEQZ:
                {
                    uint32_t rs1 = REGTAG(DECODER(ir, 0b111, 7, 0));
                    uint32_t imm = DECODEL(ir, 0b1, 2, 5)   |
                                DECODER(ir, 0b11, 3, 1)      |
                                DECODEL(ir, 0b11, 5, 6)     |
                                DECODER(ir, 0b11, 10, 3)     |
                                DECODER(ir, 0b1, 12, 8);

                    // Sign-extend
                    if (DECODER(ir, 0b1, 12, 8))
                    {
                        imm |= 0xffffff00;
                    }

                    // fprintf(stderr, "BEQZ from 0x%x to 0x%x\n", pc, pc + imm);
                    // fprintf(stderr, "a5 = 0x%x a3 = 0x%x\n", MINIRV32_LOAD4(0x40021000), REG(rs1 - 2));

                    if (!REG(rs1))
                    {
                        pc += imm - 2;
                    }

                    break;
                }

                case RVC_BNEZ:
                {
                    uint32_t rs1 = REGTAG(DECODER(ir, 0b111, 7, 0));
                    uint32_t imm = DECODEL(ir, 0b1, 2, 5)   |
                                DECODER(ir, 0b11, 3, 1)      |
                                DECODEL(ir, 0b11, 5, 6)     |
                                DECODER(ir, 0b11, 10, 3)     |
                                DECODER(ir, 0b1, 12, 8);

                    // Sign-extend
                    if (DECODER(ir, 0b1, 12, 8))
                    {
                        imm |= 0xffffff00;
                    }

                    // fprintf(stderr, "BNEZ from 0x%x to 0x%x\n", pc, pc + imm);
                    // fprintf(stderr, "a5 = 0x%x a3 = 0x%x\n", MINIRV32_LOAD4(0x40021000), REG(rs1 - 2));

                    if (REG(rs1))
                    {
                        pc += imm - 2;
                    }

                    break;
                }

                case RVC_JALR_MV_ADD:
                {
                    uint32_t rs1d = DECODER(ir, 0b11111, 7, 0);
                    uint32_t rs2 = DECODER(ir, 0b11111, 2, 0);
                    uint32_t bit12 = DECODER(ir, 0b1, 12, 0);

                    if ((!bit12) && (!rs2))
                    {
                        if (rs1d == REGRA)
                        {
                            fprintf(stderr, "Returning to 0x%x\n", REG(rs1d));
                        }

                        // JR
                        pc = REG(rs1d) - 2;
                    }
                    else if (!bit12)
                    {
                        // MV
                        REG(rs1d) = REG(rs2);
                    }
                    else if ((!rs1d) && (!rs2))
                    {
                        // EBREAK - ???
                        // fprintf(stderr, "BREAK at 0x%x\n", pc);
                        exit(1);
                    }
                    else if (!rs2)
                    {
                        // JALR
                        REG(REGRA) = pc + 2;
                        pc = REG(rs1d) - 2;
                    }
                    else
                    {
                        // fprintf(stderr, "Adding reg%d (0x%x) with reg%d (0x%x)\n", rs1d, REG(rs1d), rs2, REG(rs2));
                        // ADD
                        REG(rs1d) += REG(rs2);
                    }
                    break;
                }

                case RVC_J:
                {
                    uint32_t imm =
                        DECODEL(ir, 0b1, 2, 5) |
                        DECODER(ir, 0b111, 3, 1) |
                        DECODEL(ir, 0b1, 6, 7) |
                        DECODER(ir, 0b1, 7, 6) |
                        DECODEL(ir, 0b1, 8, 10) |
                        DECODER(ir, 0b11, 9, 8) |
                        DECODER(ir, 0b1, 11, 4) |
                        DECODER(ir, 0b1, 12, 11);

                    // Sign-extend
                    if (DECODER(ir, 0b1, 12, 11))
                    {
                        imm |= 0xffffffff & ~0b11111111111;
                    }

                    pc += imm - 2;

                    break;
                }

                case RVC_SW:
                {
                    uint32_t rs1 = REGTAG(DECODER(ir, 0b111, 7, 0));
                    uint32_t rs2 = REGTAG(DECODER(ir, 0b111, 2, 0));
                    uint32_t uimm = DECODER(ir, 0b111, 10, 3) | DECODEL(ir, 0b1, 5, 6) | DECODER(ir, 0b1, 6, 2);
                    uint32_t addy = REG(rs1) + uimm; // - MINIRV32_RAM_IMAGE_OFFSET;

                    // fprintf(stderr, "(pc = 0x%x) rs1 = 0x%x\n", pc, REG(rs1));
                    // fprintf(stderr, "sw addy = 0x%x <= rs2 = 0x%x\n", addy, REG(rs2));
                    MINIRV32_STORE4( addy, REG(rs2) );

                    break;
                }

                case RVC_LWSP:
                {
                    uint32_t rd = DECODER(ir, 0b11111, 7, 0);
                    uint32_t imm =
                            DECODEL(ir, 0b11, 2, 6) |
                            DECODER(ir, 0b111, 4, 2) |
                            DECODER(ir, 0b1, 12, 5);

                    fprintf(stderr, "Loading from SP with offset 0x%x\n", imm);

                    REG(rd) = MINIRV32_LOAD4(REG(REGSP) + imm);

                    break;
                }

                case RVC_ALU:
                {
                    uint32_t alu_opcode = DECODER(ir, 0b11, 10, 2);
                    uint32_t rs1d = REGTAG(DECODER(ir, 0b111, 7, 0));

                    if (alu_opcode >= RVC_ALU_SUB)
                    {
                        alu_opcode |= (ir & (0b11 << 5)) >> 5;
                    }

                    switch (alu_opcode)
                    {
                        case RVC_ALU_SRLI:
                        {
                            // uint32_t imm = ((ir & (0b1 << 12)) >> 7) | ((ir & (0b11111 << 2)) >> 2);
                            uint32_t imm = DECODER(ir, 0b11111, 2, 0) | DECODER(ir, 0b1, 12, 5);
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
                            uint32_t rs2 = REGTAG(DECODER(ir, 0b111, 2, 0));
                            REG(rs1d) = REG(rs1d) - REG(rs2);

                            break;
                        }
                        case RVC_ALU_XOR:
                        {
                            uint32_t rs2 = REGTAG(DECODER(ir, 0b111, 2, 0));
                            REG(rs1d) = REG(rs1d) ^ REG(rs2);

                            break;
                        }
                        case RVC_ALU_OR:
                        {
                            uint32_t rs2 = REGTAG(DECODER(ir, 0b111, 2, 0));
                            REG(rs1d) = REG(rs1d) | REG(rs2);

                            break;
                        }
                        case RVC_ALU_AND:
                        {
                            uint32_t rs2 = REGTAG(DECODER(ir, 0b111, 2, 0));
                            REG(rs1d) = REG(rs1d) & REG(rs2);

                            break;
                        }
                        default:
                        {
                            // fprintf(stderr, "Invalid opcode\n");

                            break;
                        }
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
                // fprintf(stderr, "a5 = 0x%x\n", REG(15));
                fprintf(stderr, "t1 = 0x%x\n", REG(6));
                pc += 2;

                goto next;
            }
#endif
			switch( ir & 0x7f )
			{
				case 0x37: // LUI (0b0110111)
					rval = ( ir & 0xfffff000 );
					break;
				case 0x17: // AUIPC (0b0010111)
					rval = pc + ( ir & 0xfffff000 );
					// fprintf(stderr, "rval = 0x%x pc = 0x%x\n", rval, pc);
					break;
				case 0x6F: // JAL (0b1101111)
				{
					int32_t reladdy = ((ir & 0x80000000)>>11) | ((ir & 0x7fe00000)>>20) | ((ir & 0x00100000)>>9) | ((ir&0x000ff000));
					if( reladdy & 0x00100000 ) reladdy |= 0xffe00000; // Sign extension.
					rval = pc + 4;
					pc = pc + reladdy - 4;
					// fprintf(stderr, "Branching to 0x%x\n", pc);
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
					if( rsval >= ram_amt - 3 )
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
                            // fprintf(stderr, "%s:%d rsval = 0x%x\n", __FUNCTION__, __LINE__, rsval);
                            if (!(mmio_is_mapped(rsval) < 0))
                            {
                                goto load;
                            }

							trap = (5+1);
							rval = rsval;
						}
					}
					else
					{
load:
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

					// fprintf(stderr, "LOAD from: 0x%x (reg %d) = 0x%x\n", rs1, (ir >> 15) & 0x1f, rval);
					break;
				}
				case 0x23: // Store 0b0100011
				{
					uint32_t rs1 = REG((ir >> 15) & 0x1f);
					uint32_t rs2 = REG((ir >> 20) & 0x1f);
					uint32_t addy = ( ( ir >> 7 ) & 0x1f ) | ( ( ir & 0xfe000000 ) >> 20 );
					if( addy & 0x800 ) addy |= 0xfffff000;
					addy += rs1; //  - MINIRV32_RAM_IMAGE_OFFSET;
					rdid = 0;

                    // fprintf(stderr, "rs1 = 0x%x\n", rs1);
                    // fprintf(stderr, "%s:%d addy = 0x%x BASE = 0x%x\n", __FUNCTION__, __LINE__, addy, MINIRV32_RAM_IMAGE_OFFSET);

					if( addy >= ram_amt - 3 )
					{
                        // fprintf(stderr, "%s:%d rs1 - %x %x\n", __FUNCTION__, __LINE__, rs1, addy);
	    				if( (addy + MINIRV32_RAM_IMAGE_OFFSET) >= 0x10000000 && (addy + MINIRV32_RAM_IMAGE_OFFSET) < 0x12000000 )
						{
    						addy += MINIRV32_RAM_IMAGE_OFFSET;
                            // fprintf(stderr, "%s:%d\n", __FUNCTION__, __LINE__);
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
                            // fprintf(stderr, "%s:%d addy = 0x%x\n", __FUNCTION__, __LINE__, addy);
                            if (!(mmio_is_mapped(addy) < 0))
                            {
                                goto store;
                            }

							trap = (7+1); // Store access fault.
							rval = addy;
						}
					}
					else
					{
store:
                        fprintf(stderr, "Writing 0x%x to addr 0x%x\n", rs2, addy);
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

					fprintf(stderr, "Op-immediate rs1 = 0x%x rs2 = 0x%x imm = 0x%x\n", rs1, rs2, imm);

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
					    fprintf(stderr, "Arith a5 = 0x%x rs1 = 0x%x subcmd = 0x%x\n", REG(15), (ir >> 15) & 0x1f, (ir>>12)&7);
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

					if( rs1 >= ram_amt - 3 )
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

        fprintf(stderr, "t1 = 0x%x\n", REG(6));
		pc += 4;

next:
        if (hw_break)
        {
            uint32_t * regs = state->regs;

            fprintf(stderr, "Breaking on pc = 0x%x\n", pc);

            fprintf(stderr, "Z:%08x ra:%08x sp:%08x gp:%08x tp:%08x t0:%08x t1:%08x t2:%08x s0:%08x s1:%08x a0:%08x a1:%08x a2:%08x a3:%08x a4:%08x a5:%08x ",
        		regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7],
        		regs[8], regs[9], regs[10], regs[11], regs[12], regs[13], regs[14], regs[15] );
            fprintf(stderr, "a6:%08x a7:%08x s2:%08x s3:%08x s4:%08x s5:%08x s6:%08x s7:%08x s8:%08x s9:%08x s10:%08x s11:%08x t3:%08x t4:%08x t5:%08x t6:%08x\n",
        		regs[16], regs[17], regs[18], regs[19], regs[20], regs[21], regs[22], regs[23],
        		regs[24], regs[25], regs[26], regs[27], regs[28], regs[29], regs[30], regs[31] );

            exit(1);
        }
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
