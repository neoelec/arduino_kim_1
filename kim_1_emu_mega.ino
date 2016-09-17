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
 * File Name : kim_1_emu_mega.ino
 *
 * File Description :
 *
 * Author : Joo, Young Jin <neoelec@gmail.com>
 * Dept : Raccoon's Cave
 * Created Date : 25/Sep/2016
 * Version : Baby-Raccoon
 */

#if defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_SAM_DUE)

#include <avr/pgmspace.h>

#include "cpu_mos6502.h"
#include "cpu_mos6502_io_mem.h"
#include "kim_1_emu_mega.h"

#define CFG_KIM_1_EMU_FND_MS		1

static const uint8_t __kim_1_emu_mega_key_row[] PROGMEM = {
	4, 3, 2
};

static const uint8_t __kim_1_emu_mega_key_col[] PROGMEM = {
	14, 16, 18, 12, 15, 17, 19, 11,
};

static const uint8_t __kim_1_emu_mega_fnd_led[] PROGMEM = {
	17, 15, 16, 14, 11, 19, 18, 12,
};

static const uint8_t __kim_1_emu_mega_fnd_common[] PROGMEM = {
	10, 9, 8, 7, 6, 5,
};

static const uint8_t __kim_1_emu_mega_fnd_code[] PROGMEM = {
	0b00111111,	/* 0 */
	0b00000110,	/* 1 */
	0b01011011,	/* 2 */
	0b01001111,	/* 3 */
	0b01100110,	/* 4 */
	0b01101101,	/* 5 */
	0b01111101,	/* 6 */
	0b00000111,	/* 7 */
	0b01111111,	/* 8 */
	0b01101111,	/* 9 */
	0b01110111,	/* A */
	0b01111100,	/* b */
	0b00111001,	/* C */
	0b01011110,	/* d */
	0b01111001,	/* E */
	0b01110001,	/* F */
};

kim_1_emu_mega::kim_1_emu_mega()
		: key_row(__kim_1_emu_mega_key_row),
		key_row_nr(ARRAY_SIZE(__kim_1_emu_mega_key_row)),
		key_col(__kim_1_emu_mega_key_col),
		key_col_nr(ARRAY_SIZE(__kim_1_emu_mega_key_col)),
		fnd_led(__kim_1_emu_mega_fnd_led),
		fnd_led_nr(ARRAY_SIZE(__kim_1_emu_mega_fnd_led)),
		fnd_common(__kim_1_emu_mega_fnd_common),
		fnd_common_nr(ARRAY_SIZE(__kim_1_emu_mega_fnd_common)),
		fnd_code(__kim_1_emu_mega_fnd_code),
		fnd_code_nr(ARRAY_SIZE(__kim_1_emu_mega_fnd_code)),
		sst_led(13)
{
}

static kim_1_emu_mega __kim_1_emu_mega;

kim_1_emu_mega *kim_1_emu_mega::singleton = &__kim_1_emu_mega;

kim_1_emu_mega *kim_1_emu_mega::get_instance(void)
{
	return singleton;
}

void kim_1_emu_mega::__display_toggle_sst_led(void)
{
	digitalWrite(sst_led, sst_mode ? HIGH : LOW);
}

void kim_1_emu_mega::__display_update_each_fnd(uint8_t pos, uint8_t num)
{
	uint8_t code;
	unsigned int i;

	if (teletype_connected)
		return;

	digitalWrite(pgm_read_byte(&fnd_common[pos]), LOW);

	for (i = 0; i < key_col_nr; i++) {
		pinMode(pgm_read_byte(&fnd_led[i]), OUTPUT);
		digitalWrite(pgm_read_byte(&fnd_led[i]), LOW);
	}

	code = pgm_read_byte(&fnd_code[num]);

	/* toggle on for each fnd mode */
	if (((pos < 4) && (E_KIM_1_EMU_INPUT_MODE_AD == ad_da_mode)) ||
	    ((pos >= 4) && (E_KIM_1_EMU_INPUT_MODE_DA == ad_da_mode)))
		code |= 0b10000000;

	for (i = 0; i < fnd_led_nr; i++) {
		if (code & (1 << i))
			digitalWrite(pgm_read_byte(&fnd_led[i]), HIGH);
		else
			digitalWrite(pgm_read_byte(&fnd_led[i]), LOW);
	}

	delay(CFG_KIM_1_EMU_FND_MS);

	/* completly off fnd */
	for (i = 0; i < fnd_led_nr; i++)
		digitalWrite(pgm_read_byte(&fnd_led[i]), LOW);

	digitalWrite(pgm_read_byte(&fnd_common[pos]), HIGH);
}

void kim_1_emu_mega::display_update_fnd(uint8_t *seg)
{
	unsigned int i;

	if (teletype_connected)
		return;

	for (i = 0; i < 3; i++) {
		__display_update_each_fnd(2 * i, seg[i] >> 4);
		__display_update_each_fnd(2 * i + 1, seg[i] & 0x0F);
	}
}

void kim_1_emu_mega::display_init(void)
{
	unsigned int i;

	for (i = 0; i < fnd_common_nr; i++) {
		pinMode(pgm_read_byte(&fnd_common[i]), OUTPUT);
		digitalWrite(pgm_read_byte(&fnd_common[i]), HIGH);
	}

	pinMode(sst_led, OUTPUT);
}

static const uint8_t __fn_key_lookup_tbl[] PROGMEM = {
	KIM_1_EMU_KEY_CODE_AD,
	KIM_1_EMU_KEY_CODE_DA,
	KIM_1_EMU_KEY_CODE_PC,
	KIM_1_EMU_KEY_CODE_PLUS,
	KIM_1_EMU_KEY_CODE_GO,
	KIM_1_EMU_KEY_CODE_ST,
	KIM_1_EMU_KEY_CODE_RS,
	KIM_1_EMU_KEY_CODE_SST,
};

void kim_1_emu_mega::keyboard_scan(void)
{
	unsigned int row, col, key = KIM_1_EMU_KEY_CODE_INVALID;
	unsigned int i;
	unsigned long timestamp = millis();
	static unsigned long timestamp_prev = 0;

	if (timestamp - timestamp_prev < 250)
		return;

	for (col = 0; col < key_col_nr; col++)
		pinMode(pgm_read_byte(&key_col[col]), INPUT_PULLUP);

	for (row = 0; row < key_row_nr; row++) {
		digitalWrite(pgm_read_byte(&key_row[row]), LOW);
		for (col = 0; col < key_col_nr; col++) {
			if (!digitalRead(pgm_read_byte(&key_col[col])))
				key = key_col_nr * row + col;
		}
		digitalWrite(pgm_read_byte(&key_row[row]), HIGH);
	}

	if (KIM_1_EMU_KEY_CODE_INVALID == key)
		return;

	key_available = true;
	if (key <= 0x10) {	/* hex numeric keys */
		key_data = key;
	} else if (key > 0x17) {
		key_data = KIM_1_EMU_KEY_CODE_INVALID;
		key_available = false;
		return;		/* return without updating timestamp */
	} else {
		key_data = pgm_read_byte(&__fn_key_lookup_tbl[key - 0x10]);
		if (KIM_1_EMU_KEY_CODE_ST == key_data ||
		    KIM_1_EMU_KEY_CODE_RS == key_data ||
		    KIM_1_EMU_KEY_CODE_SST == key_data)
			key_available = false;
	}

	timestamp_prev = timestamp;
}

void kim_1_emu_mega::keyboard_init(void)
{
	unsigned int i;

	for (i = 0; i < key_col_nr; i++) {
		pinMode(pgm_read_byte(&key_col[i]), OUTPUT);
		digitalWrite(pgm_read_byte(&key_col[i]), LOW);
	}

	for (i = 0; i < key_row_nr; i++)
		pinMode(pgm_read_byte(&key_row[i]), OUTPUT);
}

void kim_1_emu_mega::input_process_function_key(void)
{
	/* TODO: save key_data before calling a method in the parent */
	uint8_t tmp_key_data = key_data;

	kim_1_emu::input_process_function_key();

	switch (tmp_key_data) {
	case KIM_1_EMU_KEY_CODE_SST:
		__display_toggle_sst_led();
		break;
	}
}

#endif /* ARDUINO_AVR_MEGA2560 / ARDUINO_SAM_DUE */
