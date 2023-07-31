// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 * Alain Volmat <avolmat@me.com>					   *
 *									   *
 * This target for using DBU as target to write_memory has been written by *
 * performing read of the initialization code found in 4kopen.com and      *
 * especially within the targetpack package .py files used to generate     *
 * JTAG data transmitted usually using ST Microelectronics STMC2 JTAG probe*
 *									   *
 * STi ST Microelectronics SOC start by loading and executing so-called    *
 * btfmw binary and then executing it, in order to perform the very first  *
 * initialization of the SOC (including clock-tree).  At this time since   *
 * clock are running at low frequency, the idea was to tune as much as     *
 * possible the DBU JTAG commands to load the data as fast as possible,    *
 * faster that if done via the cpu0 target.				   *
 *									   *
 * Performing clock simple initialization within the tcl file prior to     *
 * loading via the DAP (cpu0) might as well be a solution, making this     *
 * stdbu target useless.						   *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <jtag/jtag.h>

#include "target.h"
#include "target_type.h"

#define DBU_IR_LEN	3

static int stdbu_init(struct command_context *cmd_ctx, struct target *target)
{
	return ERROR_OK;
}

static int stdbu_poll(struct target *target)
{
	if ((target->state == TARGET_RUNNING) || (target->state == TARGET_DEBUG_RUNNING))
		target->state = TARGET_HALTED;
	return ERROR_OK;
}

static int stdbu_halt(struct target *target)
{
	target->state = TARGET_HALTED;
	return ERROR_OK;
}

static int stdbu_reset_assert(struct target *target)
{
	target->state = TARGET_RESET;
	return ERROR_OK;
}

static int stdbu_reset_deassert(struct target *target)
{
	target->state = TARGET_RUNNING;
	return ERROR_OK;
}

#define DBU_REG_ACCESS_CHAIN_LEN	40
static int stdbu_write_reg(struct target *t, uint8_t reg, uint32_t val)
{
	/* IR SCAN */
	uint8_t inst = 0x05;
	struct scan_field field_ir = {
		.num_bits = t->tap->ir_length,
		.out_value = &inst,
		.in_value = NULL,
	};
	/* DR SCAN */
	uint8_t data[DBU_REG_ACCESS_CHAIN_LEN >> 3];
	struct scan_field field_dr = {
		.num_bits = DBU_REG_ACCESS_CHAIN_LEN,
		.out_value = data,
		.in_value = NULL,
	};

	jtag_add_ir_scan(t->tap, &field_ir, TAP_IDLE);
	buf_set_u32(data, 8, 32, val);
	data[0] = reg | 0x01;
	jtag_add_dr_scan(t->tap, 1, &field_dr, TAP_IDLE);
	if (jtag_execute_queue() != ERROR_OK) {
		LOG_ERROR("%s: setting DBU_REG 0x%x = 0x%x failed", __func__, reg, val);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int stdbu_read_reg(struct target *t, uint8_t reg, uint32_t *val)
{
	/* IR SCAN */
	uint8_t inst = 0x05;
	struct scan_field field_ir = {
		.num_bits = t->tap->ir_length,
		.out_value = &inst,
		.in_value = NULL,
	};
	/* DR SCAN */
	uint8_t data_out[DBU_REG_ACCESS_CHAIN_LEN >> 3] = { 0 };
	uint8_t data_in[DBU_REG_ACCESS_CHAIN_LEN >> 3] = { 0 };
	struct scan_field field_dr_out = {
		.num_bits = DBU_REG_ACCESS_CHAIN_LEN,
		.out_value = data_out,
		.in_value = NULL,
	};
	struct scan_field field_dr_in = {
		.num_bits = DBU_REG_ACCESS_CHAIN_LEN,
		.out_value = NULL,
		.in_value = data_in,
	};

	jtag_add_ir_scan(t->tap, &field_ir, TAP_IDLE);
	data_out[0] = reg | 0x02;
	jtag_add_dr_scan(t->tap, 1, &field_dr_out, TAP_IDLE);
	jtag_add_dr_scan(t->tap, 1, &field_dr_in, TAP_IDLE);
	if (jtag_execute_queue() != ERROR_OK) {
		LOG_ERROR("%s: reading DBU_REG 0x%x failed", __func__, reg);
		return ERROR_FAIL;
	}

	*val = buf_get_u32(data_in, 8, 32);

	return ERROR_OK;
}

/*
 * Read a value from the main bus using the DBU access.  Sequence is
 * as follow:
 * Write address to access into DBU register 0x6C
 * Kick read by setting 1 into DBU register 0x60
 * Wait for bit 1 of DBU register 0x64 to get cleared
 * Read value from DBU register 0x74
 */
/*
 * For the time being stdbu_read (aka read_memory) is not optimised
 * since for the moment the goal is to load BTFMW as fast as possible
 */
static int stdbu_read(struct target *t, target_addr_t a, uint32_t size,
			  uint32_t count, uint8_t *buf)
{
	int ret;
	uint32_t val;
	uint32_t address = (uint32_t)a;

	if (size != 4) {
		LOG_ERROR("%s: only size 4 bytes supported\n", __func__);
		return ERROR_NOT_IMPLEMENTED;
	}

	while (count--) {
		ret = stdbu_write_reg(t, 0x6C, address);
		if (ret != ERROR_OK) {
			LOG_ERROR("%s : Failed to set address, ret = %d\n", __func__, ret);
			return ret;
		}
		ret = stdbu_write_reg(t, 0x60, 1);
		if (ret != ERROR_OK) {
			LOG_ERROR("%s : failed to kick read, ret = %d\n", __func__, ret);
			return ret;
		}
		do {
			ret = stdbu_read_reg(t, 0x64, &val);
			if (ret != ERROR_OK) {
				LOG_ERROR("%s : failed to read status, ret = %d\n", __func__, ret);
				return ret;
			}
		} while (val & 0x02);
		ret = stdbu_read_reg(t, 0x74, &val);
		if (ret != ERROR_OK) {
			LOG_ERROR("%s : failed to read value register, ret = %d\n", __func__, ret);
			return ret;
		}
		h_u32_to_le(buf, val);
		buf += 4;
		address += 4;
	}

	return ERROR_OK;
}

/*
 * Write a value to the main bus using the DBU access.  Sequence is
 * as follow:
 * Write address to access into DBU register 0x6C
 * Write the value to be written into DBU register 0x70
 * Kick write by writing 0 into DBU register 0x60
 * Wait for bit 1 of DBU register 0x64 to get cleared
 */
static int stdbu_write(struct target *t, target_addr_t a, uint32_t size, uint32_t count,
		       const uint8_t *b)
{
	uint32_t val;
	uint32_t address = (uint32_t)a;
	/* IR SCAN */
	uint8_t inst = 0x05;
	struct scan_field field_ir = {
		.num_bits = t->tap->ir_length,
		.out_value = &inst,
		.in_value = NULL,
	};
	/* DR SCAN */
	uint8_t data_out[DBU_REG_ACCESS_CHAIN_LEN >> 3];
	uint8_t data_in[DBU_REG_ACCESS_CHAIN_LEN >> 3];
	struct scan_field field_dr_out = {
		.num_bits = DBU_REG_ACCESS_CHAIN_LEN,
		.out_value = data_out,
		.in_value = NULL,
	};
	struct scan_field field_dr_in = {
		.num_bits = DBU_REG_ACCESS_CHAIN_LEN,
		.out_value = data_in,
		.in_value = NULL,
	};

	if (size != 4) {
		LOG_ERROR("%s: only size 4 bytes supported\n", __func__);
		return ERROR_NOT_IMPLEMENTED;
	}

	jtag_add_ir_scan(t->tap, &field_ir, TAP_IDLE);
	while (count--) {
		val = le_to_h_u32(b);

		buf_set_u32(data_out, 8, 32, address);
		data_out[0] = 0x6C | 0x01;
		jtag_add_dr_scan(t->tap, 1, &field_dr_out, TAP_IDLE);

		buf_set_u32(data_out, 8, 32, val);
		data_out[0] = 0x70 | 0x01;
		jtag_add_dr_scan(t->tap, 1, &field_dr_out, TAP_IDLE);

		buf_set_u32(data_out, 8, 32, 0);
		data_out[0] = 0x60 | 0x01;
		jtag_add_dr_scan(t->tap, 1, &field_dr_out, TAP_IDLE);

		data_out[0] = 0x64 | 0x02;
		jtag_add_dr_scan(t->tap, 1, &field_dr_out, TAP_IDLE);
		/*
		 * Here we double the status check just to give to the DBU
		 * enough time to actually write the data.  This is quite
		 * dangerous trick which however allows to load the data
		 * way faster
		 */
		jtag_add_dr_scan(t->tap, 1, &field_dr_in, TAP_IDLE);
		jtag_add_dr_scan(t->tap, 1, &field_dr_in, TAP_IDLE);
		b += 4;
		address += 4;
	}

	if (jtag_execute_queue() != ERROR_OK) {
		LOG_ERROR("%s: reading DBU_REG failed", __func__);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

struct target_type stdbu_target = {
	.name = "stdbu",

	.init_target = &stdbu_init,
	.poll = &stdbu_poll,
	.halt = &stdbu_halt,
	.assert_reset = &stdbu_reset_assert,
	.deassert_reset = &stdbu_reset_deassert,

	.read_memory = &stdbu_read,
	.write_memory = &stdbu_write,
};
