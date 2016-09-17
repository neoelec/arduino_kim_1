/**
 * Copyright (C) 2016. Joo, Young Jin <neoelec@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/**
 * Project Name : KIM-1 on Arduino (clone of KIM Uno)
 *
 * Project Description :
 *
 * Comments : tabstop = 8, shiftwidth = 8, noexpandtab
 */

/**
 * File Name : kim_1_emu.h
 *
 * File Description :
 *
 * Author : Joo, Young Jin <neoelec@gmail.com>
 * Dept : Raccoon's Cave
 * Created Date : 25/Sep/2016
 * Version : Baby-Raccoon
 */

#ifndef __KIM_1_EMU_H__
#define __KIM_1_EMU_H__

#include <stdint.h>

#define KIM_1_EMU_UART_KEY_CODE_GO	0x07	/* Ctrl+G */
#define KIM_1_EMU_UART_KEY_CODE_ST	0x14	/* Ctrl+T */
#define KIM_1_EMU_UART_KEY_CODE_RS	0x12	/* Ctrl+R */
#define KIM_1_EMU_UART_KEY_CODE_SST	0X0E	/* Ctrl+N */
#define KIM_1_EMU_UART_KEY_CODE_AD	0x01	/* Ctrl+A */
#define KIM_1_EMU_UART_KEY_CODE_DA	0x04	/* Ctrl+D */
#define KIM_1_EMU_UART_KEY_CODE_PC	0x10	/* Ctrl+P */
#define KIM_1_EMU_UART_KEY_CODE_PLUS	'+'
#define KIM_1_EMU_UART_KEY_CODE_TERM	'\t'

#define KIM_1_EMU_KEY_CODE_GO		0x13
#define KIM_1_EMU_KEY_CODE_AD		0x10
#define KIM_1_EMU_KEY_CODE_DA		0x11
#define KIM_1_EMU_KEY_CODE_PC		0x14
#define KIM_1_EMU_KEY_CODE_PLUS		0x12

#define KIM_1_EMU_KEY_CODE_ST		0x18
#define KIM_1_EMU_KEY_CODE_RS		0x19
#define KIM_1_EMU_KEY_CODE_SST		0x1A
#define KIM_1_EMU_KEY_CODE_INVALID	0xFF

enum kim_1_emu_input_mode_t {
	E_KIM_1_EMU_INPUT_MODE_AD = 0,
	E_KIM_1_EMU_INPUT_MODE_DA,
};

class kim_1_emu {
public:
	virtual void init(void);
	virtual void loop(void);

	virtual bool is_teletype_connected(void);
	virtual bool is_key_available(void);

	virtual void display_update_fnd(uint8_t *seg);
	virtual uint8_t teletype_in(void);
	virtual void teletype_out(uint8_t out_char);
	virtual uint8_t keyboard_in(void);

protected:
	virtual void display_init(void);
	virtual void teletype_scan(void);
	virtual void teletype_init(void);
	virtual void keyboard_scan(void);
	virtual void keyboard_init(void);
	virtual void input_process_function_key(void);

	bool sst_mode;
	kim_1_emu_input_mode_t ad_da_mode;
	bool teletype_available;
	bool teletype_connected;
	uint8_t teletype_data;
	bool key_available;
	uint8_t key_data;
};

void kim_1_emu_init(void);

void kim_1_emu_loop(void);

uint8_t kim_1_emu_teletype_in(void);

void kim_1_emu_teletype_out(uint8_t out_char);

uint8_t kim_1_emu_keyboard_in(void);

void kim_1_emu_display_led(uint8_t *seg);

#endif /* __KIM_1_EMU_H__ */
