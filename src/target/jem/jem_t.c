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
#include "target.h"
#include "target_type.h"
#include "binarybuffer.h"
#include "breakpoints.h"
#include "jem_t.h"

#define JEM_JTAG_INS_LEN	4


//////////////
/*************
 * 
 * JStamp Board
 * Connector
 * 
 * ------ ------
 * | N T T T T | *(p1)
 * | C D D M C |
 * |   O I S K |
 * |           |
 * | G G V G G |
 * -------------
 * 
 *************/

/*************
JEM I-Register set
0x00 -
0x01 - IDCODE
0x02 - 164 bits: Target PINS (i.e. Normal JTAG function presumably)
       0               1               2               3               4               
       0.......8.......0.......8.......0.......8.......0.......8.......0.......8.......
       I   I II                IIxIIIIx     I  xIIxxIIx         IIIII           II          
       O   O OO                OO OOOO      O   OO  OO          OOOOO           OO          
       E   E EE                CC CCCC      D   DD  DD          AAAAA           BB          
       7   4 63                01 3456      6   51  40          01234           54          

       5               6               7               8               9               A   A
       0.......8.......0.......8.......0.......8.......0.......8.......0.......8.......0...4
       
       // NOTE: Can seem to check the above pins, but they seem to be only status, NO set feature.
0x03 -
0x04 - 
0x05 -
0x06 -
0x07 -
0x08 - 12 bits: Seems like a cmd/status register
0x09 - 100 bits: Data access register
0x0A - 69 bits: Breakpoint register
       0               1               2               3               4
       0       8       0       8       0       8       0       8       01234
       aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaabbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbXAABB
       a = address of breakpoint A
		 b = address of breakpoint B
		 X = update??
		 A = type for breakpoint A
		 B = type for breakpoint B
0x0B - 21 bits: 
*************/
//////////////

#ifdef __GNUC__
#define NOT_USED __attribute__ ((unused))
#else
#define NOT_USED
#endif

struct insn_regs {
	int id, size;
};

enum IR_NAMES {
	UNK1=0x00,
	IDCODE=0x01,
	TGT_PINS=0x02,
	UNK2=0x03,
	UNK3=0x04,
	UNK4=0x05,
	UNK5=0x06,
	UNK6=0x07,
	CMD=0x08,
	DATA=0x09,
	BKPT=0x0A,
	STATUS=0x0B,
	MICROCODE=0x0C,
	UNK8=0x0D,
	UNK9=0x0E,
	INIT=0x0F, // This looks like it "links" commands
	NOSTATE
};

struct insn_regs i_regs[] = {{IDCODE,32},{TGT_PINS,164},{CMD,12},{DATA,100},{BKPT,69},{STATUS,21},{MICROCODE,249},{INIT,32}};

uint32_t default_timeout_ms = 250;

uint16_t chip_selects[] = { 0xFFC, 0xFFC, 0xFFF, 0xFFF, 0xFFF, 0xFFF, 0xFFF, 0xFFF };

/* forward declarations */
static int jem_target_create(struct target *target, Jim_Interp *interp);
static int jem_init_target(struct command_context *cmd_ctx, struct target *target);
static int jem_halt(struct target *target);
static int jem_arch_state(struct target *target);
static int jem_poll(struct target *target);
static int jem_resume(struct target *target, int current, uint32_t address, int handle_breakpoints, int debug_execution);
static int jem_add_breakpoint(struct target *target, struct breakpoint *breakpoint);
static int jem_step(struct target *target, int current, uint32_t address, int handle_breakpoints);
static int jem_read_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);
static int jem_write_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, const uint8_t *buffer);
static int jem_assert_reset(struct target *target);
static int jem_deassert_reset(struct target *target);
static int jem_soft_reset_halt(struct target *target);

/* IR and DR functions */
static int jem_change_ir(struct target *target, uint32_t ir);
static int jem_get_dr32(struct target *target, enum IR_NAMES ir, uint32_t start, uint32_t length, uint32_t *value);
static int jem_set_dr32(struct target *target, enum IR_NAMES ir, uint32_t start, uint32_t length, uint32_t value);
static int jem_set_dr_bit(struct target *target, enum IR_NAMES ir, uint32_t bit, uint32_t value);
static int jem_get_dr_bit(struct target *target, enum IR_NAMES ir, uint32_t bit);
static int jem_get_drscan_buffer(struct target *target, enum IR_NAMES ir);
static int jem_set_drscan_buffer(struct target *target, enum IR_NAMES ir, bool get_dr);
static int jem_irscan_buffer(struct target *target, enum IR_NAMES ir);

/* JEM Helpers */
static int jem_check_status(struct target *target, uint32_t bit, int expected, enum IR_NAMES return_state, uint32_t retries);
static int jem_reset_jtag(void);

static int create_scan_fields(unsigned char *src, int bit_length, struct scan_field **return_fields, int *num_fields);
static void destroy_scan_fields(struct scan_field *destroy_fields, int field_cnt);
static int get_dr_length(int ir_val);

struct target_type jem_target = {
	.name = "jem",

	.halt = jem_halt,
	.poll = jem_poll,
	.arch_state = jem_arch_state,

	.target_request_data = NULL,

	.resume = jem_resume,
	.step = jem_step,

	.assert_reset = jem_assert_reset,
	.deassert_reset = jem_deassert_reset,

	.read_memory = jem_read_memory,
	.write_memory = jem_write_memory,
	.soft_reset_halt = jem_soft_reset_halt,

	.add_breakpoint = jem_add_breakpoint,

/*
	.get_gdb_reg_list = jem_get_gdb_reg_list,

	.bulk_write_memory = jem_bulk_write_memory,
	.checksum_memory = jem_checksum_memory,
	.blank_check_memory = jem_blank_check_memory,

	.run_algorithm = jem_run_algorithm,

	.remove_breakpoint = jem_remove_breakpoint,
	.add_watchpoint = jem_add_watchpoint,
	.remove_watchpoint = jem_remove_watchpoint,
*/
	.init_target = jem_init_target,
	.target_create = jem_target_create
};

#define JEM(x) ((struct jem_common *)((struct target *)x->arch_info))->
#define JTAG(x) ((struct jem_common *)((struct target *)x->arch_info))->jtag_info
#define IR_LEN(x) ((struct jem_common *)((struct target *)x->arch_info))->jtag_info.tap->ir_length


static int jem_target_create(struct target *target, Jim_Interp *interp)
{
	struct jem_common *jem = calloc(1, sizeof(struct jem_common));

	jem->jtag_info.tap = target->tap;
	jem->target_started = 0;
	target->arch_info = jem;
	JEM(target)num_hw_bkpts_avail = MAX_HARD_BREAKS;

	return ERROR_OK;
}

static int jem_init_target(struct command_context *cmd_ctx, struct target *target)
{
	// Function does not allow any JTAG yet...
	LOG_DEBUG("%s(%d)", __FILE__,__LINE__);

	return ERROR_OK;
}

static int jem_start_target(struct target *target) 
{
	uint32_t check;
	int err = ERROR_FAIL;

	LOG_DEBUG("Entering");

	if(JEM(target)target_started==0) {

		jem_change_ir(target, INIT);

		jem_set_dr32(target, INIT, 0, 32, 0xCAFEBABE);
		jem_set_drscan_buffer(target, INIT, 1);

		if( jem_get_dr32(target, INIT, 0, 32, &check) == ERROR_OK ) {
			LOG_DEBUG("check value = 0x%x", check);
			if ((check & 0xFFFEFFFE) == 0x95FC757C) {
				err = ERROR_OK;
				LOG_DEBUG("%s(%d): target started.",__FILE__,__LINE__);
				jem_change_ir(target, CMD);
				jem_get_drscan_buffer(target, CMD);
				jem_set_dr_bit(target, CMD, 8, 1);
				jem_set_drscan_buffer(target, CMD, 1);

				jem_change_ir(target, BKPT);
				jem_get_drscan_buffer(target, BKPT);
				jem_set_dr_bit(target, BKPT, 64, 1);
				jem_set_drscan_buffer(target, BKPT, 1);

				jem_change_ir(target, CMD);
				jem_get_drscan_buffer(target, CMD);
				jem_set_dr_bit(target, CMD, 11, 0);
				jem_set_drscan_buffer(target, CMD, 1);
				JEM(target)target_started = 1;
			}
		}
	} else err = ERROR_OK;
	LOG_DEBUG("Exiting");
	return err;
}

static int jem_check_reg(struct target *target, enum IR_NAMES ir, uint32_t bit, int expected, uint32_t ms_timeout) {
	int retval = ERROR_FAIL;
	struct timespec start, now;
	long elapsed;
	
	if(jem_change_ir(target, ir) == ERROR_OK) {
		clock_gettime(CLOCK_MONOTONIC, &start);
		do {
			jem_get_drscan_buffer(target, ir);
			if(expected == jem_get_dr_bit(target, ir, bit)) {
				retval = ERROR_OK;
				break;
			}
			clock_gettime(CLOCK_MONOTONIC, &now);
			elapsed = (now.tv_sec * 1000 + now.tv_nsec / 1000000) - (start.tv_sec * 1000 + start.tv_nsec / 1000000);
		} while(elapsed < ms_timeout);
	}
	
	if(retval == ERROR_OK) LOG_DEBUG("!!!!!!!!!!!!!!!!!CHECK WORKED!!!!!!!!!!!!!!!!!!1");
	else LOG_DEBUG("vvvvvvvvvvvvvvvvvvvvvvvvCHECK FAILEDvvvvvvvvvvvvvvvvvvvvvvvvv");
	return retval;
}

static int jem_check_status(struct target *target, uint32_t bit, int expected, enum IR_NAMES return_state, uint32_t retries) {
	int retval = ERROR_FAIL;
	
	do {
		if(jem_check_reg(target, STATUS, bit, expected, default_timeout_ms) == ERROR_OK) { retval = ERROR_OK; break; }
	} while(--retries > 0);
	
	if(return_state != NOSTATE)
		jem_change_ir(target, return_state);
			
	return retval;
}

NOT_USED static int jem_addr_sel(struct target *target, int val1, int val2) {
	if(jem_change_ir(target, CMD) == ERROR_OK) {
		jem_get_drscan_buffer(target, CMD);
		jem_set_dr_bit(target, CMD, 5, val1);
		jem_set_dr_bit(target, CMD, 4, val2);
		if(jem_set_drscan_buffer(target, CMD, 1) == ERROR_OK) return ERROR_OK;
	}
	return ERROR_FAIL;
}

static int jem_poll(struct target *target)
{
	int err = ERROR_FAIL;
	
	if(target->state == TARGET_HALTED) {
		// If we are halted, then we can query
		// the device.
		err = ERROR_OK;
	} else {
		// We aren't halted, so don't query anything.
		err = ERROR_OK;
	}

	//LOG_DEBUG("%s", __FILE__,__LINE__);
	
	return err;
}

NOT_USED static int jem_arch_read(struct target *target) {
	LOG_DEBUG("%s(%d)", __FILE__,__LINE__);
	return ERROR_OK;
}

static int jem_arch_state(struct target *target)
{
	LOG_DEBUG("%s(%d)", __FILE__,__LINE__);
	return ERROR_OK;
}

static int jem_resume(struct target *target, int current, uint32_t address,
		int handle_breakpoints, int debug_execution)
{
	int status;
	int running = 0;
	int err = ERROR_FAIL;

	LOG_DEBUG("%s(%d) Enter", __FILE__,__LINE__);
	if(jem_change_ir(target, STATUS) == ERROR_OK) {
		if(jem_get_drscan_buffer(target, STATUS) == ERROR_OK) {
			if(jem_get_dr_bit(target, STATUS, 14) == 1) {
				if(jem_change_ir(target, CMD) != ERROR_OK) return ERROR_FAIL;
				if(jem_get_drscan_buffer(target, CMD) != ERROR_OK) return ERROR_FAIL;
				jem_set_dr_bit(target, CMD, 7, 1);
				jem_set_dr_bit(target, CMD, 0x0A, 0);
				if(jem_set_drscan_buffer(target, CMD, 0) != ERROR_OK) return ERROR_FAIL;
			}
			if(jem_change_ir(target, STATUS) == ERROR_OK) {
				if(jem_get_drscan_buffer(target, STATUS) == ERROR_OK) {
					status = jem_get_dr_bit(target, STATUS, 13);
					if(jem_change_ir(target, CMD) == ERROR_OK) {
						if(jem_get_drscan_buffer(target, CMD) == ERROR_OK) {
							if(status != 1 && jem_get_dr_bit(target, CMD, 7)==0) running = 1;
							else {
								jem_set_dr_bit(target, CMD, 7, 1);
								jem_set_dr_bit(target, CMD, 9, 0);
								if(jem_set_drscan_buffer(target, CMD, 1) == ERROR_OK) {
									jem_get_drscan_buffer(target, CMD);
									jem_set_dr_bit(target, CMD, 9, 1);
									jem_set_dr_bit(target, CMD, 7, 0);
									if(jem_set_drscan_buffer(target, CMD, 1) == ERROR_OK) {
										running = 1;
									}
								}
							}
						}
					}
				}
			}
		}
	}
	LOG_DEBUG("%s(%d) Exit", __FILE__,__LINE__);
	if(running) {
		target->state = TARGET_RUNNING;
		err = ERROR_OK;
	}
	
	return err;
}

static int jem_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int err = ERROR_FAIL;

	if (breakpoint->type == BKPT_HARD) {
		if (JEM(target)num_hw_bkpts_avail < 1) {
			LOG_INFO("no hardware breakpoint available");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		if(jem_check_status(target, 13, 1, CMD,	1)==ERROR_OK) {
			jem_get_drscan_buffer(target, CMD);
			jem_set_dr_bit(target, CMD, 7, 1);
			if(jem_set_drscan_buffer(target, CMD, 0) != ERROR_OK) return ERROR_FAIL;
		}
		jem_get_drscan_buffer(target, CMD);
		jem_set_dr_bit(target, CMD, 9, 0);
		jem_set_drscan_buffer(target, CMD, 0);
		jem_change_ir(target, BKPT);
		jem_get_drscan_buffer(target, BKPT);
		if(JEM(target)num_hw_bkpts_avail > 1) {
			jem_set_dr32(target, BKPT, 32, 32, breakpoint->address);
			jem_set_dr32(target, BKPT, 67, 2, 3);
		} else {
			jem_set_dr32(target, BKPT, 0, 32, breakpoint->address);
			jem_set_dr32(target, BKPT, 65, 2, 3);
		}
		if(jem_set_drscan_buffer(target, BKPT, 0) == ERROR_OK) {
			JEM(target)num_hw_bkpts_avail--;
			err = ERROR_OK;
		}
	}

	return err;
}


static int jem_step(struct target *target, int current, uint32_t address, int handle_breakpoints)
{
	LOG_DEBUG("%s(%d)", __FILE__,__LINE__);
	return ERROR_OK;
}

static int jem_assert_reset(struct target *target)
{
	int err = ERROR_FAIL;
	
	LOG_DEBUG("%s(%d) Enter", __FILE__,__LINE__);

	target->state = TARGET_UNKNOWN;
	if(jem_reset_jtag() == ERROR_OK) {
		if(jem_start_target(target) == ERROR_OK) {
			jem_change_ir(target, CMD);
			jem_get_drscan_buffer(target, CMD);
			jem_set_dr_bit(target, CMD, 8, 1);
			if(jem_set_drscan_buffer(target, CMD, 0) == ERROR_OK) {
				jem_change_ir(target, BKPT);
				jem_get_drscan_buffer(target, BKPT);
				jem_set_dr_bit(target, BKPT, 64, 1);
				if(jem_set_drscan_buffer(target, BKPT, 0) == ERROR_OK) {
					jem_change_ir(target, CMD);
					jem_get_drscan_buffer(target, CMD);
					jem_set_dr_bit(target, CMD, 11, 0);
					if(jem_set_drscan_buffer(target, CMD, 0) == ERROR_OK) {
						target->state = TARGET_RUNNING;
						err = ERROR_OK;
					}
				}
			}
		}

	}
	LOG_DEBUG("%s(%d) Exit", __FILE__,__LINE__);
	return err;
}

static int jem_deassert_reset(struct target *target)
{
	target->state = TARGET_RUNNING;

	LOG_DEBUG("%s", __FILE__);
	return ERROR_OK;
}

static int jem_halt(struct target *target)
{
	int err = ERROR_FAIL;
	int count = 5;

	LOG_DEBUG("%s(%d) Enter", __FILE__,__LINE__);
	do {
		if(jem_change_ir(target, CMD)==ERROR_OK) {
			jem_get_drscan_buffer(target, CMD);
			jem_set_dr_bit(target, CMD, 7, 1);
			if(jem_set_drscan_buffer(target, CMD, 0) == ERROR_OK) {
				if(jem_check_status(target, 13, 1, CMD, 1) == ERROR_OK) {
					LOG_DEBUG("Do we ever get here???");
					jem_get_drscan_buffer(target, CMD);
					jem_set_dr_bit(target, CMD, 9, 0);
					if(jem_set_drscan_buffer(target, CMD, 1) == ERROR_OK) {
						jem_set_dr_bit(target, CMD, 9, 1);
						jem_set_drscan_buffer(target, CMD, 1);
						target->state = TARGET_HALTED;
						err = ERROR_OK;
						break;
					}
				}
			}
		}
	} while (--count > 0);
	LOG_DEBUG("%s(%d) Exit", __FILE__,__LINE__);
	return err;
}

static int jem_soft_reset_halt(struct target *target)
{
	int err = ERROR_FAIL, i;

	LOG_DEBUG("%s", __FILE__);
	if(jem_change_ir(target,CMD) == ERROR_OK) {
		if(jem_get_drscan_buffer(target,CMD) == ERROR_OK) {
			jem_set_dr_bit(target, CMD, 11, 1);
			jem_set_dr_bit(target, CMD, 10, 0);
			jem_set_dr_bit(target, CMD,  9, 0);
			jem_set_dr_bit(target, CMD,  8, 1);
			//jem_set_dr32(target, CMD, 8, 4, 0x9); // Should be same as above
			if(jem_set_drscan_buffer(target,CMD, 0) == ERROR_OK) {
				if(jem_check_reg(target, TGT_PINS, 128, 0, default_timeout_ms*1) == ERROR_OK) {
					if(jem_change_ir(target, CMD) == ERROR_OK) {
						if(jem_get_drscan_buffer(target, CMD) == ERROR_OK) {
							jem_set_dr_bit(target, CMD, 11, 0); 
							jem_set_dr_bit(target, CMD,  9, 1);
							if(jem_set_drscan_buffer(target, CMD, 0) == ERROR_OK) {
								LOG_DEBUG("%s(%d): ********* Checking status *********.",__FILE__,__LINE__);
								if(jem_check_reg(target, TGT_PINS, 128, 1, default_timeout_ms*5) == ERROR_OK) {
									if(jem_change_ir(target, CMD) == ERROR_OK) {
										if(jem_get_drscan_buffer(target, CMD) == ERROR_OK) {
											err = ERROR_OK;
											target->state = TARGET_RESET;
											if(jem_get_dr_bit(target, CMD, 7)) {
												LOG_DEBUG("%s(%d): reset should succeed.", __FILE__,__LINE__);
												jem_change_ir(target, DATA);
												for (i = 0; i < 8; i++) {
													jem_set_dr32(target, DATA, 32, 32, 0);
													jem_set_dr32(target, DATA, 96, 4, 0x3);
													jem_set_dr32(target, DATA, 64, 32, 0xFFFF0084+(4*i));
													jem_set_dr32(target, DATA, 0,32, chip_selects[i]);
													jem_set_drscan_buffer(target, DATA, 1);
												}
												jem_set_dr32(target, DATA, 32, 32, 0);
												jem_set_dr32(target, DATA, 96, 4, 0x3);
												jem_set_dr32(target, DATA, 64, 32, 0xFFFF00A4);
												jem_set_dr32(target, DATA, 0, 32, 0x100);
												jem_set_drscan_buffer(target, DATA, 1);
												
											} else {
												LOG_DEBUG("%s(%d): reset succeeded.", __FILE__,__LINE__);
												if(jem_start_target(target) == ERROR_OK) {
													LOG_DEBUG("%s(%d): initializing...",__FILE__,__LINE__);
													
												}
											}
											if(jem_halt(target) == ERROR_OK) target->state = TARGET_HALTED;
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	return err;
}

static int jem_write_memory(struct target *target, uint32_t addr, uint32_t size, uint32_t count, const uint8_t *buffer) 
{

	uint32_t local;
	int err = ERROR_FAIL;
	uint32_t address = addr*4;
	uint32_t offset = 0;
	uint32_t count_by_size = count*size;

	if(target->state == TARGET_HALTED) {
		if(jem_change_ir(target, DATA) == ERROR_OK) {
			do {
				local = *(uint32_t *)((uint8_t *)buffer++);
				jem_set_dr32(target, DATA, 0, 32, local);
				jem_set_dr32(target, DATA, 32, 32, 0x0);
				jem_set_dr32(target, DATA, 64, 32, address+(offset++*4));
				jem_set_dr32(target, DATA, 96, 4, 0x5);
				if(jem_set_drscan_buffer(target, DATA, 1) == ERROR_OK) {
					err = ERROR_OK;
				}
			} while(--count_by_size > 0);
		}
	}
	return err;
}

static int jem_read_memory(struct target *target, uint32_t addr, uint32_t size, uint32_t count, uint8_t *buffer) 
{
	uint32_t local;
	int err = ERROR_FAIL;
	uint32_t sizes[] = {0,7,7,0,7};
	uint8_t *lbuff = buffer;
	uint32_t address = addr*4;
	uint32_t offset = 0;
	uint32_t count_by_size = count*size;

	if(target->state == TARGET_HALTED) {
		if(jem_change_ir(target, DATA) == ERROR_OK) {
			do {
				jem_set_dr32(target, DATA, 0, 32, 0x0);
				jem_set_dr32(target, DATA, 32, 32, 0x0);
				jem_set_dr32(target, DATA, 64, 32, address+(offset++*4));
				jem_set_dr32(target, DATA, 96, 4, sizes[size]);
				if(jem_set_drscan_buffer(target, DATA, 1) == ERROR_OK) {
					jem_get_drscan_buffer(target, DATA);
					jem_get_dr32(target, DATA, 0, 32, &local);
					LOG_DEBUG("value 0x%x, size=%d", local,size);
					*lbuff++ = (uint8_t)local;
					err = ERROR_OK;
				}
			} while(--count_by_size > 0);
		}
	}
	
	return err;
}

//////////////////////////// JEM HELPERS ////////////////////////////
/* IR and DR functions */
static int jem_reset_jtag() {

	jtag_add_tlr();
	if (jtag_execute_queue() != ERROR_OK) return ERROR_FAIL;

	return ERROR_OK;
}

static int jem_change_ir(struct target *target, uint32_t ir) 
{
	
	return jem_irscan_buffer(target, ir);
}

static int jem_get_dr32(struct target *target, enum IR_NAMES ir, uint32_t start, uint32_t length, uint32_t *value) 
{
	uint32_t dr_len;
//	uint32_t byte,first;

	*value = 0;

	dr_len = get_dr_length(ir);
	if(start > dr_len) return ERROR_FAIL;
//	byte = start/32;
//	first = start%32;



//	*value = buf_get_u32(&JTAG(target).dr[byte*4],first, length); 	
	buf_set_buf(&JTAG(target).dr, start, value, 0, length);
	
	LOG_DEBUG("################## VALUE = 0x%x", *value);

	return ERROR_OK;
}

static int jem_set_dr32(struct target *target, enum IR_NAMES ir, uint32_t start, uint32_t length, uint32_t value) 
{
	uint32_t dr_len;

	dr_len = get_dr_length(ir);
	if(start > dr_len) return ERROR_FAIL;

	buf_set_buf(&value, 0, &JTAG(target).dr, start, length);

	return ERROR_OK;
}

static int jem_set_dr_bit(struct target *target, enum IR_NAMES ir, uint32_t bit, uint32_t value) 
{
	uint32_t v = (value > 0) ? 1: 0;
	uint32_t dr_len;
	NOT_USED uint32_t byte,shift;
	
	byte = bit/32;
	shift = bit%32;

	dr_len = get_dr_length(ir);
	if(bit > dr_len) return ERROR_FAIL;

	LOG_DEBUG("********* ENTER to SET BIT %d of %d *************", bit, dr_len);
	LOG_DEBUG("cur bits: %s", buf_to_str(JTAG(target).dr, dr_len,16));
	buf_set_buf(&v, 0, &JTAG(target).dr, bit, 1);
	//buf_set_u32(&JTAG(target).dr[byte*4],shift,1,v);
	LOG_DEBUG("new bits: %s", buf_to_str(JTAG(target).dr, dr_len,16));
	LOG_DEBUG("**************EXIT SET BIT*********************");
	return ERROR_OK;
}

static int jem_get_dr_bit(struct target *target, enum IR_NAMES ir, uint32_t bit) 
{
	uint32_t dr_len;
	uint32_t get,v;

	dr_len = get_dr_length(ir);

	if(bit > dr_len) return -1;

	buf_set_buf(&JTAG(target).dr, bit,	&get, 0, 1);
	

	v = get > 0 ? 1 : 0;
	LOG_DEBUG("****************BIT %d = %d*************************", bit, v);

	return v;
}

static int jem_irscan_buffer(struct target *target, enum IR_NAMES ir)
{
	struct scan_field ir_field;
	uint32_t ir_register;

	ir_field.num_bits = IR_LEN(target);
	ir_field.in_value = JTAG(target).ir;
	ir_field.out_value = (uint8_t *)&ir_register;
	buf_set_u32(&ir_register, 0, ir_field.num_bits, ir);
	jtag_add_ir_scan(target->tap, &ir_field, TAP_IDLE);
	if (jtag_execute_queue() != ERROR_OK) return ERROR_FAIL;

	return ERROR_OK;
}

static int jem_get_drscan_buffer(struct target *target, enum IR_NAMES ir) 
{
	struct scan_field *dr_fields=NULL;
	uint32_t dr_len, bit;
	int32_t dr_field_count=0, i;
	uint32_t ms_timeout = 200;
	
	dr_len = get_dr_length(ir);

	memset(JTAG(target).dr, 0x0, 48);

	if(create_scan_fields(JTAG(target).dr, dr_len, &dr_fields, &dr_field_count)) {
		LOG_ERROR("%s(%d): probable memory allocation failure.", __FILE__,__LINE__);
	} else {
		// Do necessary DR stuff here
		jtag_add_dr_scan(JTAG(target).tap, dr_field_count, dr_fields, TAP_DRPAUSE);
		//jtag_add_clocks(jtag_get_speed_khz()/ms_timeout);

		if (jtag_execute_queue() != ERROR_OK) {
			LOG_ERROR("%s(%d): initializing failed.", __FILE__,__LINE__);
			destroy_scan_fields(dr_fields,dr_field_count);
			return ERROR_FAIL;
		}
		bit = 0;
		for(i = 0; i < dr_field_count; i++) {
			buf_set_buf(dr_fields[i].in_value, 0, JTAG(target).dr, bit, dr_fields[i].num_bits);
			bit += dr_fields[i].num_bits;
		}
		LOG_DEBUG("************captured: %s", buf_to_str(JTAG(target).dr, dr_len,16));

		for(i = 0; i < dr_field_count; i++) {
			//*dr_fields[i].in_value = flip_u32(*dr_fields[i].in_value,dr_fields[i].num_bits);
			bit_copy((uint8_t *)dr_fields[i].out_value,0,(uint8_t *)dr_fields[i].in_value,0, dr_fields[i].num_bits);
		}
		
		jtag_add_dr_scan(JTAG(target).tap, dr_field_count, dr_fields, TAP_IDLE);
		jtag_add_runtest(jtag_get_speed_khz()/ms_timeout, TAP_IDLE);

		if (jtag_execute_queue() != ERROR_OK) {
			LOG_ERROR("%s(%d): initializing failed.", __FILE__,__LINE__);
			destroy_scan_fields(dr_fields,dr_field_count);
			return ERROR_FAIL;
		}
	}

	destroy_scan_fields(dr_fields,dr_field_count);
	return ERROR_OK;
}

static int jem_set_drscan_buffer(struct target *target, enum IR_NAMES ir, bool get_dr) 
{
	struct scan_field *dr_fields=NULL;
	NOT_USED uint32_t dr_len, bit;
	NOT_USED int32_t dr_field_count=0, i;
	uint32_t ms_timeout = 200;
	
	dr_len = get_dr_length(ir);

	if(create_scan_fields(JTAG(target).dr, dr_len, &dr_fields, &dr_field_count)) {
		LOG_ERROR("%s(%d): probable memory allocation failure.", __FILE__,__LINE__);
	} else {
		// Do necessary DR stuff here
		jtag_add_dr_scan(JTAG(target).tap, dr_field_count, dr_fields, TAP_IDLE);

		jtag_add_runtest(jtag_get_speed_khz()/ms_timeout,TAP_IDLE);

		if (jtag_execute_queue() != ERROR_OK) {
			LOG_ERROR("%s(%d): initializing failed.", __FILE__,__LINE__);
			destroy_scan_fields(dr_fields,dr_field_count);
			return ERROR_FAIL;
		}

		if(get_dr) {
			bit = 0;
			for(i = 0; i < dr_field_count; i++) {
				buf_set_buf(dr_fields[i].in_value, 0, JTAG(target).dr, bit, dr_fields[i].num_bits);
				bit += dr_fields[i].num_bits;
			}
		}
	}

	destroy_scan_fields(dr_fields, dr_field_count);
	return ERROR_OK;
}

static int get_dr_length(int ir_val) {
	unsigned int i;
	for(i = 0; i < sizeof(i_regs); i++) if(i_regs[i].id == ir_val) return i_regs[i].size;
	return -1;
}

static int create_scan_fields(uint8_t *src, int bit_length, struct scan_field **created_fields, int *num_fields) {
	int field_bits,
		 bits_remaining = bit_length,
		 field_count = 0;
	struct scan_field *return_fields = NULL;
	uint8_t *t, *r;

	LOG_DEBUG("%s Entering",__FILE__);
	if(created_fields == NULL || src == NULL) return bit_length;

	do {
		/*if(bits_remaining >= 32) {*/
			/* create a 32 bit scan field */
		/*	LOG_DEBUG("Creating 32 bit scan field...");
			return_fields = realloc(return_fields, sizeof(struct scan_field)*(field_count + 1));
			t = malloc(sizeof(uint32_t));
			r = malloc(sizeof(uint32_t));
			field_bits = 32;
		} else */if(bits_remaining >= 16) {
			/* create a 16 bit scan field */
			LOG_DEBUG("Creating 16 bit scan field...");
			return_fields = realloc(return_fields, sizeof(struct scan_field)*(field_count + 1));		
			t = calloc(1,sizeof(uint16_t));
			r = calloc(1,sizeof(uint16_t));
			field_bits = 16;
		} else if(bits_remaining >= 8) {
			/* create an 8 bit scan field */
			LOG_DEBUG("Creating 8 bit scan field...");
			return_fields = realloc(return_fields, sizeof(struct scan_field)*(field_count + 1));		
			t = calloc(1,sizeof(uint8_t));
			r = calloc(1,sizeof(uint8_t));
			field_bits = 8;
		} else {
			/* create a final scan field */
			LOG_DEBUG("Creating %d bit scan field...",bits_remaining);
			return_fields = realloc(return_fields, sizeof(struct scan_field)*(field_count + 1));		
			t = calloc(1,sizeof(uint8_t));
			r = calloc(1,sizeof(uint8_t));
			field_bits = bits_remaining;
		}
		if(return_fields != NULL && t != NULL && r != NULL) {
			return_fields[field_count].num_bits = field_bits;
			return_fields[field_count].out_value = t;
			return_fields[field_count].in_value = r;
			bit_copy((uint8_t *)return_fields[field_count].out_value,0,src,bit_length-bits_remaining, field_bits);
			bits_remaining -= field_bits;
			field_count++;	
		} else {
			if(t != NULL) free(t);
			if(r != NULL) free(r);
			free(return_fields);
			break;
		}
	} while(bits_remaining > 0);

	if(num_fields != NULL) *num_fields = field_count;

	LOG_DEBUG("%s Exit bits remaining = %d",__FILE__,bits_remaining);
	*created_fields = return_fields;
	return bits_remaining;
}

static void destroy_scan_fields(struct scan_field *fields, int field_cnt) {
	int i;

	LOG_DEBUG("Enter");
	if(field_cnt > 0)
		for(i = 0; i < field_cnt; i++) {
			if(fields[i].out_value != NULL) free((void *)fields[i].out_value);
			if(fields[i].in_value != NULL) free((void *)fields[i].in_value);
		}
	if(fields != NULL)
		free(fields);
	LOG_DEBUG("Exit");
}

