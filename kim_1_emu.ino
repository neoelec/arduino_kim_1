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
 * File Name : kim_1_emu.ino
 *
 * File Description :
 *
 * Author : Joo, Young Jin <neoelec@gmail.com>
 * Dept : Raccoon's Cave
 * Created Date : 25/Sep/2016
 * Version : Baby-Raccoon
 */

#include <avr/pgmspace.h>

#include "cpu_mos6502.h"
#include "cpu_mos6502_io_mem.h"
#include "kim_1_emu.h"

//#define _DEBUG

#if defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAMD_ZERO)
static Serial_ *kim_1_emu_serial = &SerialUSB;
#elif defined(ARDUINO_AVR_LEONARDO)
static Serial_ *kim_1_emu_serial = &Serial;
#else
static HardwareSerial *kim_1_emu_serial = &Serial;
#endif

#ifdef _DEBUG
static const uint8_t __kim_1_sample[] PROGMEM = {
	0xEA, 0xA9, 0x01, 0xA2, 0x02, 0xA0, 0x03, 0x4C, 0x00, 0x02,
};

static void __kim_1_emu_load_sample(void)
{
	uint16_t base = 0x0200;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(__kim_1_sample); i++)
		cpu_mos6502_write_io_mem(base + i,
				pgm_read_byte(&__kim_1_sample[i]));
}
#else
static void __kim_1_emu_load_sample(void) {}
#endif

void kim_1_emu::init(void)
{
	sst_mode = false;
	ad_da_mode = E_KIM_1_EMU_INPUT_MODE_AD;
	teletype_available = false;
	teletype_connected = false;
	teletype_data = 0x00;
	key_available = false;
	key_data = KIM_1_EMU_KEY_CODE_INVALID;

	display_init();
	teletype_init();
	keyboard_init();

	cpu_mos6502_write_io_mem(0x17FA, 0x00);
	cpu_mos6502_write_io_mem(0x17FB, 0x1C);
	cpu_mos6502_write_io_mem(0x17FE, 0x00);
	cpu_mos6502_write_io_mem(0x17FF, 0x1C);

	__kim_1_emu_load_sample();

	cpu_mos6502_reset();
}

void kim_1_emu::loop(void)
{
	cpu_mos6502_exec(100);

	teletype_scan();
	keyboard_scan();

	if (key_data != KIM_1_EMU_KEY_CODE_INVALID &&
	    key_data >= 0x10 /* 0x10 means function key */)
		input_process_function_key();
}

bool kim_1_emu::is_teletype_connected(void)
{
	return teletype_connected;
}

bool kim_1_emu::is_key_available(void)
{
	return key_available;
}

void kim_1_emu::teletype_scan(void)
{
	teletype_available = false;
	if (!kim_1_emu_serial->available()) {
		teletype_data = 0x00;
		return;
	}

	teletype_data = kim_1_emu_serial->read();

	/* switch statement 1: are not assumed real teletype input.
	 * teletype_available == false. this function should be returned in
	 * this switch statement if case is taken */
	switch (teletype_data) {
	case KIM_1_EMU_UART_KEY_CODE_ST:
		key_data = KIM_1_EMU_KEY_CODE_ST;
		return;
	case KIM_1_EMU_UART_KEY_CODE_RS:
		key_data = KIM_1_EMU_KEY_CODE_RS;
		return;
	case KIM_1_EMU_UART_KEY_CODE_SST:
		key_data = KIM_1_EMU_KEY_CODE_SST;
		return;
	}

	/* the others */
	teletype_available = true;
	switch (teletype_data) {
	case KIM_1_EMU_UART_KEY_CODE_TERM:
		teletype_connected = !teletype_connected;
		break;
	}

	if (!teletype_connected) {
		/* keyboard emulation from uart input */
		key_available = true;

		switch (teletype_data) {
		case '0' ... '9':
			key_data = teletype_data - '0';
			break;
		case 'A' ... 'F':
			key_data = 0x0A + teletype_data - 'A';
			break;
		case 'a' ... 'f':
			key_data = 0x0A + teletype_data - 'a';
			break;
		case KIM_1_EMU_UART_KEY_CODE_GO:
			key_data = KIM_1_EMU_KEY_CODE_GO;
			break;
		case KIM_1_EMU_UART_KEY_CODE_AD:
			key_data = KIM_1_EMU_KEY_CODE_AD;
			break;
		case KIM_1_EMU_UART_KEY_CODE_DA:
			key_data = KIM_1_EMU_KEY_CODE_DA;
			break;
		case KIM_1_EMU_UART_KEY_CODE_PC:
			key_data = KIM_1_EMU_KEY_CODE_PC;
			break;
		case KIM_1_EMU_UART_KEY_CODE_PLUS:
			key_data = KIM_1_EMU_KEY_CODE_PLUS;
			break;
		default:
			key_available = false;
		}
	}
}

uint8_t kim_1_emu::teletype_in(void)
{
	uint8_t ret_char;

	if (!teletype_available)
		return 0x00;

	ret_char = teletype_data;

	teletype_available = false;
	teletype_data = 0x00;

	return ret_char;
}

void kim_1_emu::teletype_out(uint8_t out_char)
{
	kim_1_emu_serial->write(out_char);
}

void kim_1_emu::teletype_init(void)
{
	kim_1_emu_serial->begin(9600);
	kim_1_emu_serial->println(F("MOS / KIM-1 on Arduino"));
}

uint8_t kim_1_emu::keyboard_in(void)
{
	uint8_t ret_key;

	if (!key_available)
		return KIM_1_EMU_KEY_CODE_INVALID;

	ret_key = key_data;
	key_data = KIM_1_EMU_KEY_CODE_INVALID;
	key_available = false;

	return ret_key;
}

void kim_1_emu::input_process_function_key(void)
{
	switch (key_data) {
	case KIM_1_EMU_KEY_CODE_ST:
		if (!sst_mode)
			cpu_mos6502_change_nmi_pin(E_CPU_MOS6502_IRQ_PIN_H_L_H);
		break;
	case KIM_1_EMU_KEY_CODE_RS:
		cpu_mos6502_reset();
		ad_da_mode = E_KIM_1_EMU_INPUT_MODE_AD;
		break;
	case KIM_1_EMU_KEY_CODE_SST:
		sst_mode = !sst_mode;
		cpu_mos6502_change_nmi_pin(sst_mode ? E_CPU_MOS6502_IRQ_PIN_L :
					E_CPU_MOS6502_IRQ_PIN_H);
		break;
	case KIM_1_EMU_KEY_CODE_GO:
	case KIM_1_EMU_KEY_CODE_AD:
		ad_da_mode = E_KIM_1_EMU_INPUT_MODE_AD;
		return;
	case KIM_1_EMU_KEY_CODE_DA:
		ad_da_mode = E_KIM_1_EMU_INPUT_MODE_DA;
		return;
	default:
		return;
	}

	/* invalidate key_data */
	key_data = KIM_1_EMU_KEY_CODE_INVALID;
	key_available = false;
}

void kim_1_emu::display_update_fnd(uint8_t *seg)
{
	/* FIXME: to prevent compiler warnings */
	seg = seg;
}

void kim_1_emu::display_init(void)
{
}

void kim_1_emu::keyboard_scan(void)
{
}

void kim_1_emu::keyboard_init(void)
{
}
