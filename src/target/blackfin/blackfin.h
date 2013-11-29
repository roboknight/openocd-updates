/***************************************************************************
 *   Copyright (C) 2013 by Brandon Warhurst																 *
 *   roboknight@gmail+openocd.com                                          *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef BLACKFIN_H
#define BLACKFIN_H

#include <jtag/jtag.h>

#define MAX_HARD_BREAKS 2

struct mcu_jtag {
	struct jtag_tap *tap;
	uint8_t dr[48],
			  ir[4];
};

struct blackfin_common {
	struct mcu_jtag jtag_info;
	uint16_t dbgctl,
			  dbgstat;
	uint64_t emuir_a,
			  emuir_b;
	uint32_t emudat_out,
			  emudat_in;
			  
	uint32_t emupc,
			  emupc_orig;
	int target_started;
	int num_hw_bkpts_avail;
};

/* High-Nibble: group code, low nibble: register code.  */
#define T_REG_R                         0x00
#define T_REG_P                         0x10
#define T_REG_I                         0x20
#define T_REG_B                         0x30
#define T_REG_L                         0x34
#define T_REG_M                         0x24
#define T_REG_A                         0x40

enum core_regnum
{
    REG_R0 = T_REG_R, REG_R1, REG_R2, REG_R3, REG_R4, REG_R5, REG_R6, REG_R7,
    REG_P0 = T_REG_P, REG_P1, REG_P2, REG_P3, REG_P4, REG_P5, REG_SP, REG_FP,
    REG_I0 = T_REG_I, REG_I1, REG_I2, REG_I3,
    REG_M0 = T_REG_M, REG_M1, REG_M2, REG_M3,
    REG_B0 = T_REG_B, REG_B1, REG_B2, REG_B3,
    REG_L0 = T_REG_L, REG_L1, REG_L2, REG_L3,
    REG_A0x = T_REG_A, REG_A0w, REG_A1x, REG_A1w,
    REG_ASTAT = 0x46,
    REG_RETS = 0x47,
    REG_LC0 = 0x60, REG_LT0, REG_LB0, REG_LC1, REG_LT1, REG_LB1,
    REG_CYCLES, REG_CYCLES2,
    REG_USP = 0x70, REG_SEQSTAT, REG_SYSCFG,
    REG_RETI, REG_RETX, REG_RETN, REG_RETE, REG_EMUDAT,
};

#define CLASS_MASK                      0xf0
#define GROUP(x)                        (((x) & CLASS_MASK) >> 4)
#define DREG_P(x)                       (((x) & CLASS_MASK) == T_REG_R)
#define PREG_P(x)                       (((x) & CLASS_MASK) == T_REG_P)


#define DTEST_COMMAND                   0xffe00300
#define DTEST_DATA0                     0xffe00400
#define DTEST_DATA1                     0xffe00404

#define ITEST_COMMAND                   0xffe01300
#define ITEST_DATA0                     0xffe01400
#define ITEST_DATA1                     0xffe01404

#define SWRST_ADDR 						0xffc00100
///////////////////////////////////////////////////////
//////////////////// JTAG Reg Defs ////////////////////
///////////////////////////////////////////////////////

enum IR_NAMES {
	EXTEST,
	SAMPLE,
	EMUIR32,
	EMUIR64,
	DBGCTL,
	EMUDAT,
	DBGSTAT,
	IDCODE,
	MEMBIST,
	TMKEY,
	CSKEY,
	EMUPC,
	BYPASS,
	NOSTATE
};

///////////////  DEBUG Control Register ///////////////
enum dbgctl_bitdefs {
	dc_empwr,
	dc_emfen,
	dc_emeen,
	dc_empen,
	dc_emuirsz,
	dc_emuirlpsz = 0x06,
	dc_emudatsz,
	dc_esstep = 0x09,
	dc_sysrst,
	dc_wakeup,
	dc_sram_init,
};
#define REG_MASK(x,mask_bits) (mask_bits<<(x))
#define MSK_DBGCTL_SRAM_INIT	REG_MASK(dc_sram_init,0x01)
#define MSK_DBGCTL_WAKEUP		REG_MASK(dc_wakeup,0x01)
#define MSK_DBGCTL_SYSRST		REG_MASK(dc_sysrst,0x01)
#define MSK_DBGCTL_ESSTEP		REG_MASK(dc_esstep,0x01)
#define MSK_DBGCTL_EMUDATSZ		REG_MASK(dc_emudatsz,0x03)
#define MSK_DBGCTL_EMUIRLPSZ	REG_MASK(dc_emuirlpsz,0x01)
#define MSK_DBGCTL_EMUIRSZ		REG_MASK(dc_emuirsz,0x03)
#define MSK_DBGCTL_EMPEN		REG_MASK(dc_empen,0x01)
#define MSK_DBGCTL_EMEEN		REG_MASK(dc_emeen,0x01)
#define MSK_DBGCTL_EMFEN		REG_MASK(dc_emfen,0x01)
#define MSK_DBGCTL_EMPWR		REG_MASK(dc_empwr,0x01)

////////////////  DEBUG Status Register ///////////////
enum dbgstat_bitdefs {
	ds_emudof,
	ds_emudif,
	ds_emudoovf,
	ds_emudiovf,
	ds_emuready,
	ds_emuack,
	ds_emucause,
	ds_bist_done = 0x0a,
	ds_lpdec0,
	ds_in_reset,
	ds_idle,
	ds_core_fault,
	ds_lpdec1,
};
#define MSK_DBGSTAT_LPDEC1		REG_MASK(ds_lpdec1,0x01)
#define MSK_DBGSTAT_CORE_FAULT	REG_MASK(ds_core_fault,0x01)
#define MSK_DBGSTAT_IDLE		REG_MASK(ds_idle,0x01)
#define MSK_DBGSTAT_IN_RESET	REG_MASK(ds_in_reset,0x01)
#define MSK_DBGSTAT_LPDEC0		REG_MASK(ds_lpdec0,0x01)
#define MSK_DBGSTAT_BIST_DONE	REG_MASK(ds_bist_done,0x01)
#define MSK_DBGSTAT_EMUCAUSE	REG_MASK(ds_emucause,0x0F)
#define MSK_DBGSTAT_EMUACK		REG_MASK(ds_emuack,0x01)
#define MSK_DBGSTAT_EMUREADY	REG_MASK(ds_emuready,0x01)
#define MSK_DBGSTAT_EMUDOF		REG_MASK(ds_emudof,0x01)
#define MSK_DBGSTAT_EMUDIF		REG_MASK(ds_emudif,0x01)
#define MSK_DBGSTAT_EMUDOOVF	REG_MASK(ds_emudoovf,0x01)
#define MSK_DBGSTAT_EMUDIOVF	REG_MASK(ds_emudiovf,0x01)

///////////////////////////////////////////////////////
//////////////////// Prototypes ///////////////////////
///////////////////////////////////////////////////////

extern uint32_t gen_move (enum core_regnum dest, enum core_regnum src);
extern uint32_t gen_ldstidxi (enum core_regnum reg, enum core_regnum ptr, 
								int32_t offset, int w, int sz);
extern uint32_t gen_load32_offset (enum core_regnum dest, enum core_regnum base, int32_t offset);
extern uint32_t gen_store32_offset (enum core_regnum base, int32_t offset, enum core_regnum src);
extern uint32_t gen_load16z_offset (enum core_regnum dest, enum core_regnum base, int32_t offset);
extern uint32_t gen_store16_offset (enum core_regnum base, int32_t offset, enum core_regnum src);
extern uint32_t gen_load8z_offset (enum core_regnum dest, enum core_regnum base, int32_t offset);
extern uint32_t gen_store8_offset (enum core_regnum base, int32_t offset, enum core_regnum src);
extern uint32_t gen_ldst (enum core_regnum reg, enum core_regnum ptr, 
							int post_dec, int w, int sz);
extern uint32_t gen_load32pi (enum core_regnum dest, enum core_regnum base);
extern uint32_t gen_store32pi (enum core_regnum base, enum core_regnum src);
extern uint32_t gen_load16zpi (enum core_regnum dest, enum core_regnum base);
extern uint32_t gen_store16pi (enum core_regnum base, enum core_regnum src);
extern uint32_t gen_load8zpi (enum core_regnum dest, enum core_regnum base);
extern uint32_t gen_store8pi (enum core_regnum base, enum core_regnum src);
extern uint32_t gen_load32 (enum core_regnum dest, enum core_regnum base);
extern uint32_t gen_store32 (enum core_regnum base, enum core_regnum src);
extern uint32_t gen_load16z (enum core_regnum dest, enum core_regnum base);
extern uint32_t gen_store16 (enum core_regnum base, enum core_regnum src);
extern uint32_t gen_load8z (enum core_regnum dest, enum core_regnum base);
extern uint32_t gen_store8 (enum core_regnum base, enum core_regnum src);
extern uint32_t gen_flush_insn (enum core_regnum addr, int op, int post_modify);
extern uint32_t gen_iflush (enum core_regnum addr);
extern uint32_t gen_iflush_pm (enum core_regnum addr);
extern uint32_t gen_flush (enum core_regnum addr);
extern uint32_t gen_flush_pm (enum core_regnum addr);
extern uint32_t gen_flushinv (enum core_regnum addr);
extern uint32_t gen_flushinv_pm (enum core_regnum addr);
extern uint32_t gen_prefetch (enum core_regnum addr);
extern uint32_t gen_prefetch_pm (enum core_regnum addr);
extern uint32_t gen_jump_reg (enum core_regnum addr);

#endif /* BLACKFIN_H */
