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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <unistd.h>
#include <time.h>
#include <target/target.h>
#include <target/target_type.h>
#include <helper/binarybuffer.h>
#include <target/breakpoints.h>

#include "blackfin.h"

#define BLACKFIN_JTAG_INS_LEN	5


#ifdef __GNUC__
#define NOT_USED __attribute__ ((unused))
#else
#define NOT_USED
#endif

struct insn_regs {
	int id, size;
};

static struct insn_regs i_regs[] = {{IDCODE,32},{DBGSTAT,16},{DBGCTL,16},{EMUDAT,32},{EMUPC,32},{EXTEST,197},{MEMBIST,64},{EMUIR32,32},{EMUIR64,64}};

unsigned char ir_codes[] = {
	// The Blackfin might not be 1149.1 compliant.
	// So these values are later flipped.
		0x00, /* EXTEST */
		0x01, /* SAMPLE */
		0x02, /* EMUIR32 */
		0x02, /* EMUIR64 */
		0x04, /* DBGCTL */
		0x05, /* EMUDAT */
		0x06, /* DBGSTAT */
		0x08, /* IDCODE */
		0x0A, /* MEMBIST */
		0x0C, /* TMKEY */
		0x0D, /* CSKEY */
		0x0F, /* SAMPLEPC */
		0x1F  /* BYPASS */
};

/* forward declarations */
static uint64_t flip_u64(uint64_t value);
static int blackfin_target_create(struct target *target, Jim_Interp *interp);
static int blackfin_init_target(struct command_context *cmd_ctx, struct target *target);
static int blackfin_halt(struct target *target);
static int blackfin_arch_state(struct target *target);
static int blackfin_poll(struct target *target);
static int blackfin_resume(struct target *target, int current, uint32_t address, int handle_breakpoints, int debug_execution);
static int blackfin_add_breakpoint(struct target *target, struct breakpoint *breakpoint);
static int blackfin_step(struct target *target, int current, uint32_t address, int handle_breakpoints);
static int blackfin_read_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);
static int blackfin_write_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, const uint8_t *buffer);
static int blackfin_assert_reset(struct target *target);
static int blackfin_deassert_reset(struct target *target);
static int blackfin_soft_reset_halt(struct target *target);

/* IR and DR functions */
static int blackfin_mov_to_idle(void);
static int blackfin_mov_to_drshift(void);
static int blackfin_reset_jtag(void);

/* BLACKFIN Helpers */
static uint32_t blackfin_register_get(struct target *target, enum core_regnum reg);
static void blackfin_register_set(struct target *target, enum core_regnum reg, uint32_t value);
static int blackfin_emuir_set (struct target *target, uint64_t insn1, uint64_t insn2, uint32_t icount, bool, bool);
static int blackfin_wait_in_reset(struct target *target);
static int blackfin_update_dbgctl(struct target *target);
static int blackfin_update_dbgstat(struct target *target);
static int blackfin_update_emudat(struct target *target);
static int blackfin_set_bypass(struct target *target);
static void blackfin_trigger_emulation(struct target *target);

static int get_dr_length(int ir_val);

struct target_type blackfin_target = {
	.name = "blackfin",

	.halt = blackfin_halt,
	.poll = blackfin_poll,
	.arch_state = blackfin_arch_state,

	.target_request_data = NULL,

	.resume = blackfin_resume,
	.step = blackfin_step,

	.assert_reset = blackfin_assert_reset,
	.deassert_reset = blackfin_deassert_reset,

	.read_memory = blackfin_read_memory,
	.write_memory = blackfin_write_memory,
	.soft_reset_halt = blackfin_soft_reset_halt,

	.add_breakpoint = blackfin_add_breakpoint,

/*
	.get_gdb_reg_list = blackfin_get_gdb_reg_list,

	.bulk_write_memory = blackfin_bulk_write_memory,
	.checksum_memory = blackfin_checksum_memory,
	.blank_check_memory = blackfin_blank_check_memory,

	.run_algorithm = blackfin_run_algorithm,

	.remove_breakpoint = blackfin_remove_breakpoint,
	.add_watchpoint = blackfin_add_watchpoint,
	.remove_watchpoint = blackfin_remove_watchpoint,
*/
	.init_target = blackfin_init_target,
	.target_create = blackfin_target_create
};

#define BLACKFIN(x) ((struct blackfin_common *)((struct target *)x->arch_info))->
#define JTAG(x) BLACKFIN(x)jtag_info
#define IR_LEN(x) JTAG(x).tap->ir_length

#define INSN32(x1,x2) (uint64_t)(0x0000000000000000ULL|(((uint32_t)(x1))<<16)|((uint32_t)(x2)))
#define INSN64(x1,x2) (uint64_t)(((uint64_t)(x1)<<32)|((uint64_t)(x2)))

// Basic Blackfin instructions
#define INSN_NOP                        0x0000
#define INSN_RTE                        0x0014
#define INSN_CSYNC                      0x0023
#define INSN_SSYNC                      0x0024
#define INSN_ILLEGAL                    0xffffffff

static const struct timespec bfin_emu_wait_ts = {0, 5000000};

NOT_USED static int blackfin_start_target(enum jtag_event event, void *priv) 
{
	int err = ERROR_OK;
	struct blackfin_common __unused *blackfin = (struct blackfin_common *)priv;

	LOG_DEBUG("Entering");

	LOG_DEBUG("Exiting");
	return err;
}

static int blackfin_target_create(struct target *target, Jim_Interp *interp)
{
	struct blackfin_common *blackfin = calloc(1, sizeof(struct blackfin_common));

	blackfin->jtag_info.tap = target->tap;
	blackfin->target_started = 0;
	blackfin->dbgctl  = 0x0;
	blackfin->dbgstat = 0x0;
	blackfin->emudat_in = 0x0ULL;
	blackfin->emudat_out = 0x0ULL;
	blackfin->emuir_a = 0x0ULL;
	blackfin->emuir_b = 0x0ULL;
	blackfin->emupc   = 0x0;
	blackfin->emupc_orig = 0x0;
	target->arch_info = blackfin;
	BLACKFIN(target)num_hw_bkpts_avail = MAX_HARD_BREAKS;

	jtag_register_event_callback(blackfin_start_target,blackfin);

	return ERROR_OK;
}

static int blackfin_init_target(struct command_context *cmd_ctx, struct target *target)
{
	// Function does not allow any JTAG yet...
	LOG_DEBUG("%s(%d)", __FILE__,__LINE__);

	return ERROR_OK;
}

NOT_USED static int blackfin_check_reg(struct target *target, enum IR_NAMES ir, uint32_t bit, int expected, uint32_t ms_timeout) {
	int retval = ERROR_FAIL;
	
	return retval;
}

NOT_USED static int blackfin_check_status(struct target *target, uint32_t bit, int expected, enum IR_NAMES return_state, uint32_t retries) {
	int retval = ERROR_FAIL;
	
	return retval;
}

NOT_USED static int blackfin_addr_sel(struct target *target, int val1, int val2) {
	return ERROR_FAIL;
}

static int blackfin_poll(struct target *target)
{
	int err = ERROR_FAIL;
	
	if(target->state == TARGET_HALTED) {
		// If we are halted, then we can query
		// the device.
		err = ERROR_OK;
	} else {
		// We aren't halted, so try halting?
		if(blackfin_soft_reset_halt(target) == ERROR_OK) {
			target->state = TARGET_HALTED;
			if(blackfin_soft_reset_halt(target)==ERROR_OK) 
				err = ERROR_OK;
		}
	}

	//LOG_DEBUG("%s", __FILE__,__LINE__);
	
	return err;
}

NOT_USED static int blackfin_arch_read(struct target *target) {
	LOG_DEBUG("%s(%d)", __FILE__,__LINE__);
	return ERROR_OK;
}

static int blackfin_arch_state(struct target *target)
{
	LOG_DEBUG("%s(%d)", __FILE__,__LINE__);
	return ERROR_OK;
}

static int blackfin_resume(struct target *target, int current, uint32_t address,
		int handle_breakpoints, int debug_execution)
{
	int err = ERROR_FAIL;

	LOG_DEBUG("%s(%d) Enter", __FILE__,__LINE__);
	return err;
}

static int blackfin_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int err = ERROR_FAIL;

	return err;
}


static int blackfin_step(struct target *target, int current, uint32_t address, int handle_breakpoints)
{
	LOG_DEBUG("%s(%d)", __FILE__,__LINE__);
	return ERROR_OK;
}

static int blackfin_assert_reset(struct target *target)
{
	int err = ERROR_FAIL;
	
	LOG_DEBUG("%s(%d) Enter", __FILE__,__LINE__);

	LOG_DEBUG("%s(%d) Exit", __FILE__,__LINE__);
	return err;
}

static int blackfin_deassert_reset(struct target *target)
{
	target->state = TARGET_RUNNING;

	LOG_DEBUG("%s", __FILE__);
	return ERROR_OK;
}

static int blackfin_halt(struct target *target)
{
	int err = ERROR_FAIL;

	LOG_DEBUG("%s(%d) Enter", __FILE__,__LINE__);
	LOG_DEBUG("%s(%d) Exit", __FILE__,__LINE__);
	return err;
}

static int blackfin_soft_reset_halt(struct target *target)
{
	int err = ERROR_FAIL;
	uint32_t p0, r0, r1;
	
	LOG_DEBUG("%s(%d) Enter", __FILE__,__LINE__);
	
	BLACKFIN(target)dbgctl |= REG_MASK(dc_empwr,1);
	blackfin_update_dbgctl(target);
	BLACKFIN(target)dbgctl |= REG_MASK(dc_emfen,1) | REG_MASK(dc_emeen,1);
	blackfin_update_dbgctl(target);
	
	// *********** CHAIN SYSTEM RESET BELOW *************
	
	r0 = blackfin_register_get(target, REG_R0);
	p0 = blackfin_register_get(target, REG_P0);
	blackfin_register_set(target, REG_R1, 0xBABABABA);
	r1 = blackfin_register_get(target, REG_R1);
	
	LOG_DEBUG("r0=%08x,p0=%08x,r1=%08x", r0, p0, r1);

	blackfin_register_set(target, REG_P0, SWRST_ADDR);
	p0 = blackfin_register_get(target, REG_P0);
	LOG_DEBUG("************************************* p0=%08x", p0);
	blackfin_register_set(target, REG_R0, 0x07);
	blackfin_emuir_set(target, gen_store16_offset (REG_P0, 0, REG_R0), INSN_NOP,2, false, true);
	blackfin_emuir_set(target, INSN_NOP,INSN_NOP,2, false, true);
	
	usleep (100);
	blackfin_register_set(target, REG_R0, 0x00);
	blackfin_emuir_set(target, gen_store16_offset (REG_P0, 0, REG_R0), INSN_NOP,2, false, true);
	blackfin_emuir_set(target, INSN_NOP,INSN_NOP,2, false, true);

	usleep (100);
	
	blackfin_register_set(target, REG_P0, p0);
	blackfin_register_set(target, REG_R0, r0);
	
	// ************* CORE RESET CODE BELOW **************

	// Set the instructions to NOP
	blackfin_emuir_set(target, INSN_NOP, INSN_NOP,2,false, true);
	blackfin_update_dbgctl(target);
	BLACKFIN(target)dbgctl &= ~REG_MASK(dc_empwr,1);
	BLACKFIN(target)dbgctl |= REG_MASK(dc_sram_init,1) | REG_MASK(dc_sysrst,1);
	blackfin_update_dbgctl(target);

	// Need to wait while in reset here
	if(blackfin_wait_in_reset(target) == ERROR_OK) {

		// Relase the sysrst bit
		BLACKFIN(target)dbgctl &= ~REG_MASK(dc_sysrst,1);
		blackfin_update_dbgctl(target);
		LOG_DEBUG("Wait for reset over 1...%04x\n",BLACKFIN(target)dbgctl);
		
		BLACKFIN(target)dbgctl |= REG_MASK(dc_empwr,1); 
		blackfin_update_dbgctl(target);

		// Need to wait while in reset here
		BLACKFIN(target)dbgctl &= ~(REG_MASK(dc_emuirsz,0x1) | REG_MASK(dc_sram_init,1) | REG_MASK(dc_emudatsz,0x3));
		BLACKFIN(target)dbgctl |= REG_MASK(dc_emfen,1) |
								  REG_MASK(dc_emuirsz,0x2) | REG_MASK(dc_wakeup,1) | REG_MASK(dc_emeen,1);
		blackfin_update_dbgctl(target);
		LOG_DEBUG("Wait for reset over...%04x\n",BLACKFIN(target)dbgctl);
		err = ERROR_OK;
		blackfin_trigger_emulation(target);
	}

	LOG_DEBUG("%s(%d) Exit", __FILE__,__LINE__);

	return err;
}

static int blackfin_write_memory(struct target *target, uint32_t addr, uint32_t size, uint32_t count, const uint8_t *buffer) 
{

	int err = ERROR_FAIL;

	return err;
}

static int blackfin_read_memory(struct target *target, uint32_t addr, uint32_t size, uint32_t count, uint8_t *buffer) 
{
	int err = ERROR_FAIL;
	uint32_t p0,r0,i;
	uint32_t *p32_buff = (uint32_t *)((void *)buffer);
	uint16_t *p16_buff = (uint16_t *)((void *)buffer);


	p0 = blackfin_register_get(target, REG_P0);
	r0 = blackfin_register_get(target, REG_R0);
	blackfin_register_set(target, REG_P0, addr);
	switch(size) {
	case 4:
		blackfin_emuir_set(target, gen_load32pi(REG_R0,REG_P0), gen_move(REG_EMUDAT, REG_R0), 2, false, false);
		for(i = 0; i < count; i++) {
			blackfin_update_emudat(target);
			*p32_buff++ = BLACKFIN(target)emudat_in;
			//blackfin_register_set(target, REG_P0, addr+(i*4));
			//blackfin_emuir_set(target, gen_load32pi(REG_R0,REG_P0), gen_move(REG_EMUDAT, REG_R0), 2, true, true);
		}
		blackfin_register_set(target, REG_P0,p0);
		blackfin_register_set(target, REG_R0,r0);
		err = ERROR_OK;
		break;
	case 2:
		blackfin_emuir_set(target, gen_load16zpi(REG_R0,REG_P0), gen_move(REG_EMUDAT, REG_R0), 2, false, false);
		for(i = 0; i < count; i++) {
			blackfin_update_emudat(target);
			*p16_buff++ = (uint16_t)BLACKFIN(target)emudat_in;
			//blackfin_register_set(target, REG_P0, addr+(i*2));
			//blackfin_emuir_set(target, gen_load16z(REG_R0,REG_P0), gen_move(REG_EMUDAT, REG_R0), 2, true, true);
		}
		blackfin_register_set(target, REG_P0,p0);
		blackfin_register_set(target, REG_R0,r0);
		err = ERROR_OK;
		break;
	case 1:
		blackfin_register_set(target, REG_P0,p0);
		blackfin_register_set(target, REG_R0,r0);
		err = ERROR_OK;
		break;
	default:
		err = ERROR_FAIL;
	}

	return err;
}

//////////////////////////// BLACKFIN HELPERS ////////////////////////////
static int blackfin_wait_in_reset(struct target *target) {
	int err = ERROR_FAIL;
	int in_reset = 0, count=10;

	do {
		// Grab the Data Register
		blackfin_update_dbgstat(target);
		// Wait for the halibut
		nanosleep(&bfin_emu_wait_ts,NULL);
		
		// Check to see if the IN_RESET bit is set.  If not,
		// count down and loop
	} while((in_reset = (BLACKFIN(target)dbgstat & REG_MASK(ds_in_reset,1))) && (--count > 0));

	// If we are IN_RESET, then return ERROR_OK
	if(in_reset) err = ERROR_OK;
	
	// return
	return err;
}

static uint32_t blackfin_register_get(struct target *target, enum core_regnum reg) {
	uint32_t r0, value;

	if (DREG_P(reg) || PREG_P(reg))
		blackfin_emuir_set(target, gen_move(REG_EMUDAT, reg),INSN_NOP,2, false, true);
	else {
		/* First grab register r0 so we can restore it */
		r0 = blackfin_register_get(target, REG_R0);
		
		blackfin_emuir_set(target, gen_move(REG_R0, reg),gen_move(REG_EMUDAT,REG_R0),2, false, true);
	}
	
	blackfin_update_emudat(target);
	value = BLACKFIN(target)emudat_in;
	
	if (!DREG_P(reg) && !PREG_P(reg))
		blackfin_register_set(target, REG_R0, r0);
	
	return value;
}

static void blackfin_register_set(struct target *target, enum core_regnum reg, uint32_t value) {
	uint32_t r0 = 0;
	
	if (!DREG_P(reg) && !PREG_P(reg)) {
		r0 = blackfin_register_get(target, REG_R0);
	}

	BLACKFIN(target)emudat_out = value;
	blackfin_update_emudat(target);
	
	if (DREG_P(reg) || PREG_P (reg))
		blackfin_emuir_set (target, gen_move(reg, REG_EMUDAT), INSN_NOP, 2, false, true);
	else {
		/* First grab register r0 so we can restore it */
		r0 = blackfin_register_get(target, REG_R0);
		
		blackfin_emuir_set(target, gen_move(REG_R0, REG_EMUDAT),gen_move(reg,REG_R0),2, false, true);
		blackfin_register_set(target, REG_R0, r0);
	}
	
}

static int blackfin_emuir_set (struct target *target, uint64_t insn1, uint64_t insn2, uint32_t icount, bool repeat_last, bool execute_once)
{
	bool do_insn32 = false;
	bool do_insn16 = false;
	bool do_emuirb = false;
	uint32_t tmp_a,tmp_b;
	uint64_t tmp64_a, tmp64_b;
	uint8_t out[8];
	uint8_t *nop=NULL;
	LOG_DEBUG("Enter");
	
	do_insn32 = ((insn1 & 0xffffffff00000000ULL) == 0) && ((insn2 & 0xffffffff00000000ULL) == 0);
	do_insn16 = ((insn1 & 0xffffffffffff0000ULL) == 0) && ((insn2 & 0xffffffffffff0000ULL) == 0);
	do_emuirb = (icount > 1);
	
	BLACKFIN(target)dbgctl &= ~(REG_MASK(dc_emuirlpsz,1)|REG_MASK(dc_emuirsz,0x3));
	if(do_emuirb) {
		BLACKFIN(target)emuir_b = insn2 << (do_insn16 ? 16:0);
		BLACKFIN(target)dbgctl |= REG_MASK(dc_emuirlpsz,1);
	} 
	BLACKFIN(target)emuir_a = insn1 << (do_insn16 ? 16:0);

	if(!repeat_last) {
		if(do_insn32) {
			// Run 32 bit instructions here
			BLACKFIN(target)dbgctl |= REG_MASK(dc_emuirsz,0x2);
			blackfin_update_dbgctl(target);
			out[0] = flip_u32(ir_codes[EMUIR32],5);
			jtag_add_plain_ir_scan(BLACKFIN_JTAG_INS_LEN,(const uint8_t *)out,nop, TAP_IRPAUSE);
			if(do_emuirb) {
				tmp_b = flip_u32(BLACKFIN(target)emuir_b,32);
				jtag_add_plain_dr_scan(32, (const uint8_t *)&tmp_b,out, TAP_DRPAUSE);
				blackfin_mov_to_drshift();
//				jtag_execute_queue();
			}
			tmp_a = flip_u32(BLACKFIN(target)emuir_a,32);
			jtag_add_plain_dr_scan(32, (const uint8_t *)&tmp_a,out, TAP_DRPAUSE);
		} else {
			// Run 64 bit instructions here
			blackfin_update_dbgctl(target);
			out[0] = flip_u32(ir_codes[EMUIR64],5);
			jtag_add_plain_ir_scan(BLACKFIN_JTAG_INS_LEN,(const uint8_t *)out,nop, TAP_IRPAUSE);
			if(do_emuirb) {
				tmp64_b = flip_u64(BLACKFIN(target)emuir_b);
				jtag_add_plain_dr_scan(64, (const uint8_t *)&tmp64_b,out, TAP_DRPAUSE);
				blackfin_mov_to_drshift();
//				jtag_execute_queue();
			}
			tmp64_a = flip_u64(BLACKFIN(target)emuir_a);
			jtag_add_plain_dr_scan(64, (const uint8_t *)&tmp64_a,out, TAP_DRPAUSE);
		}
		if(execute_once)
			jtag_add_statemove(TAP_IDLE);
		if(jtag_execute_queue() != ERROR_OK) return ERROR_FAIL;
	} else {
		if(do_insn32)
			BLACKFIN(target)dbgctl |= REG_MASK(dc_emuirsz,0x2);
		BLACKFIN(target)dbgctl |= REG_MASK(dc_wakeup,1) | REG_MASK(dc_emeen,1);
		blackfin_trigger_emulation(target);
	}
	
	if(execute_once) {
		/* Wait for EMUREADY */
		BLACKFIN(target)dbgstat &= ~REG_MASK(ds_emuready,1);
		while(!(BLACKFIN(target)dbgstat & REG_MASK(ds_emuready,1))) {
			blackfin_update_dbgstat(target);
		}
	}
	LOG_DEBUG("Exit");
	return ERROR_OK;
}

///////////////////////// BLACKFIN IR/DR HELPERS //////////////////////////
/* IR and DR functions */
static void blackfin_trigger_emulation(struct target *target) {

	BLACKFIN(target)dbgctl |= REG_MASK(dc_empwr,1);
	blackfin_update_dbgctl(target);
	BLACKFIN(target)dbgctl |= REG_MASK(dc_emfen,1);
	blackfin_update_dbgctl(target);
	
	BLACKFIN(target)dbgctl |= REG_MASK(dc_wakeup,1) | REG_MASK(dc_emeen,1);
	blackfin_update_dbgctl(target);
}

static uint64_t flip_u64(uint64_t value) {
	uint32_t a,b,c;
	
	a = (uint32_t) value;
	b = (uint32_t)(value >> 32);
	c = flip_u32(a,32);
	a = flip_u32(b,32);
	b = c;
	
	return  (uint64_t)((uint64_t)b<<32|(uint64_t)a);
}

NOT_USED static int blackfin_reset_jtag() {

	jtag_add_tlr();
	if (jtag_execute_queue() != ERROR_OK) return ERROR_FAIL;

	return ERROR_OK;
}

static int blackfin_update_emudat(struct target *target) {
	uint8_t out[4];
	uint8_t *nop = NULL;
	uint32_t tmp;
	int drlen;
	
	drlen = get_dr_length(EMUDAT);

	out[0] = flip_u32(ir_codes[EMUDAT],BLACKFIN_JTAG_INS_LEN);
	jtag_add_plain_ir_scan(BLACKFIN_JTAG_INS_LEN, out, nop, TAP_IRPAUSE);
	tmp = flip_u32(BLACKFIN(target)emudat_out,32);
	
	jtag_add_plain_dr_scan(drlen, (const uint8_t *)&tmp, (uint8_t *)&(BLACKFIN(target)emudat_in), TAP_IDLE);
	
	if(jtag_execute_queue() != ERROR_OK) {
		return ERROR_FAIL;
	}
	
	BLACKFIN(target)emudat_in = flip_u32(BLACKFIN(target)emudat_in, 32);
	
	return ERROR_OK;
}

NOT_USED static int blackfin_set_bypass(struct target *target) {
	uint8_t out[2];
	uint8_t *nop = NULL;
	
	out[0] = flip_u32(ir_codes[BYPASS],BLACKFIN_JTAG_INS_LEN);
	jtag_add_plain_ir_scan(IR_LEN(target), &out[0], nop, TAP_IDLE);

	if(jtag_execute_queue() != ERROR_OK) {
		return ERROR_FAIL;
	}
	
	return ERROR_OK;
}

static int blackfin_update_dbgstat(struct target *target) {
	uint8_t out[2];
	uint8_t *nop = NULL;
	int drlen;
	
	drlen = get_dr_length(DBGSTAT);

	out[0] = flip_u32(ir_codes[DBGSTAT],BLACKFIN_JTAG_INS_LEN);
	jtag_add_plain_ir_scan(IR_LEN(target), &out[0], nop, TAP_IRPAUSE);

	memset(out,0,2);
	jtag_add_plain_dr_scan(drlen, out, (uint8_t *)&BLACKFIN(target)dbgstat, TAP_IDLE);
	
	if(jtag_execute_queue() != ERROR_OK) {
		return ERROR_FAIL;
	}
	
	BLACKFIN(target)dbgstat = flip_u32(BLACKFIN(target)dbgstat,16);

	LOG_DEBUG("dbgstat (in) = %04x",BLACKFIN(target)dbgstat);

	return ERROR_OK;
}

static int blackfin_update_dbgctl(struct target *target) {
	uint8_t out[2];
	//~ uint32_t out32;
	//~ uint64_t dummy = 0x12345678BABABABA;
	uint16_t value;
	uint8_t *nop = NULL;
	int drlen;
	
	drlen = get_dr_length(DBGCTL);

	// This *** MUST *** be fliped, otherwise it produces incorrect results
	out[0] = flip_u32(ir_codes[DBGCTL],BLACKFIN_JTAG_INS_LEN);
	// Need to run this twice... don't know why, but it won't switch
	// spots... might be able to "loop" in TAP_IDLE and get same result
	jtag_add_plain_ir_scan(BLACKFIN_JTAG_INS_LEN, out, nop, TAP_IRPAUSE);
	//jtag_add_statemove(TAP_IDLE);
	//jtag_add_plain_ir_scan(BLACKFIN_JTAG_INS_LEN, out, nop, TAP_IDLE);
	//jtag_execute_queue();
	//~ BLACKFIN(target)dbgctl = 0x0020;
	value = flip_u32(BLACKFIN(target)dbgctl, 16);
	jtag_add_plain_dr_scan(drlen, (const uint8_t *)&value, nop, TAP_IDLE);
	
	if(jtag_execute_queue() != ERROR_OK) {
		return ERROR_FAIL;
	}

	LOG_DEBUG("dbgctl (in) = %04x,%04x",BLACKFIN(target)dbgctl,value);
	
	//~ drlen = get_dr_length(EMUPC);
	//~ // This *** MUST *** be fliped, otherwise it produces incorrect results
	//~ out[0] = flip_u32(ir_codes[EMUPC],BLACKFIN_JTAG_INS_LEN);
	//~ // Need to run this twice... don't know why, but it won't switch
	//~ // spots... might be able to "loop" in TAP_IDLE and get same result
	//~ jtag_add_plain_ir_scan(BLACKFIN_JTAG_INS_LEN, out, nop, TAP_IDLE);
	//~ jtag_add_plain_ir_scan(BLACKFIN_JTAG_INS_LEN, out, nop, TAP_IDLE);
	//~ jtag_execute_queue();
	//~ BLACKFIN(target)emuir_a = (uint32_t)dummy;
	//~ BLACKFIN(target)emuir_b = (uint32_t)(dummy>>32);
	//~ jtag_add_plain_dr_scan(drlen, (const uint8_t *)&(BLACKFIN(target)emuir_a), (uint8_t *)&out32, TAP_IDLE);
	//~ LOG_DEBUG("emuir_32 (emupc) = %08x(%08x)",out32,flip_u32(out32,32));
	//~ jtag_add_plain_dr_scan(drlen, (const uint8_t *)&(BLACKFIN(target)emuir_b), (uint8_t *)&out32, TAP_IDLE);
	//~ jtag_execute_queue();
	//~ LOG_DEBUG("emuir_32 (emuir_a) = %08x",out32);
	//~ jtag_add_plain_dr_scan(drlen, (const uint8_t *)&(BLACKFIN(target)emuir_b), (uint8_t *)&out32, TAP_IDLE);
	//~ jtag_execute_queue();
	//~ LOG_DEBUG("emuir_32 (emuir_b) = %08x",out32);

	return ERROR_OK;
}

NOT_USED static int blackfin_mov_to_idle(void) {
	jtag_add_statemove(TAP_IDLE);
	return ERROR_OK;
}

NOT_USED static int blackfin_idle_to_update(struct target *target) {
	tap_state_t path[] = { TAP_DRSELECT, TAP_DRCAPTURE, TAP_DREXIT1, TAP_DRUPDATE };
	jtag_add_pathmove(4, path);
	return ERROR_OK;
}

static int blackfin_mov_to_drshift(void) {
	tap_state_t path[] = { TAP_DREXIT2, TAP_DRUPDATE, TAP_DRSELECT, TAP_DRCAPTURE, TAP_DRSHIFT };
	jtag_add_pathmove(5,path);
	return ERROR_OK;
}

static int get_dr_length(int ir_val) {
	unsigned int i;
	for(i = 0; i < sizeof(i_regs); i++) if(i_regs[i].id == ir_val) return i_regs[i].size;
	return -1;
}

