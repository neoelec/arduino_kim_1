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
 * File Name : kim_1_emu_uno.ino
 *
 * File Description :
 *
 * Author : Joo, Young Jin <neoelec@gmail.com>
 * Dept : Raccoon's Cave
 * Created Date : 25/Sep/2016
 * Version : Baby-Raccoon
 */

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_LEONARDO) || \
	defined(ARDUINO_SAM_ZERO) || \
	defined(ARDUINO_ESP8266_WEMOS_D1MINI)

#include <avr/pgmspace.h>

#include "cpu_mos6502.h"
#include "cpu_mos6502_io_mem.h"
#include "kim_1_emu_uno.h"

#ifdef CFG_KIM_1_UNO_USE_LCD
LiquidCrystal_I2C __kim_1_emu_uno_lcd(0x27, 16, 2);

kim_1_emu_uno::kim_1_emu_uno()
		: lcd(&__kim_1_emu_uno_lcd),
		go_mode(false)
{
}
#else
kim_1_emu_uno::kim_1_emu_uno()
		: go_mode(false)
{
}
#endif

static kim_1_emu_uno __kim_1_emu_uno;

kim_1_emu_uno *kim_1_emu_uno::singleton = &__kim_1_emu_uno;

kim_1_emu_uno *kim_1_emu_uno::get_instance(void)
{
	return singleton;
}

void kim_1_emu_uno::__display_erase_ad_da_text(void)
{
#ifdef CFG_KIM_1_UNO_USE_LCD
	lcd->setCursor(0, 1);
	lcd->print(F("    :      "));
#endif
}

void kim_1_emu_uno::__display_toggle_ad_da_mode(void)
{
#ifdef CFG_KIM_1_UNO_USE_LCD
	lcd->setCursor(9, 1);
	lcd->print(E_KIM_1_EMU_INPUT_MODE_AD == ad_da_mode ?
		F("AD") : F("DA"));
#endif
}

void kim_1_emu_uno::__display_toggle_sst_mode(void)
{
#ifdef CFG_KIM_1_UNO_USE_LCD
	lcd->setCursor(13, 1);
	lcd->print(sst_mode ? F(" ON") : F("OFF"));
#endif
}

bool kim_1_emu_uno::is_teletype_connected(void)
{
	if (teletype_connected)
		__display_erase_ad_da_text();
	else
		__display_toggle_ad_da_mode();

	return kim_1_emu::is_teletype_connected();
}

static const char __kim_1_emu_uno_fnd_fmt[] PROGMEM = "%02X%02X:%02X";

void kim_1_emu_uno::display_update_fnd(uint8_t *seg)
{
#ifdef CFG_KIM_1_UNO_USE_LCD
	char buf[16];

	if (go_mode)
		return;

	sprintf_P(buf, __kim_1_emu_uno_fnd_fmt, seg[0], seg[1], seg[2]);

	lcd->setCursor(0, 1);
	lcd->print(buf);
#else
	/* FIXME: to prevent compiler warnings */
	seg = seg;
#endif
}

void kim_1_emu_uno::display_init(void)
{
#ifdef CFG_KIM_1_UNO_USE_LCD
	lcd->begin();
	lcd->backlight();

	lcd->setCursor(0, 0);
	lcd->print(F("MOS / KIM-1  SST"));
	lcd->setCursor(0, 1);
	//lcd->print(F("0000:00  AD  OFF"));
	lcd->print(F("         AD  OFF"));
#endif
}

void kim_1_emu_uno::input_process_function_key(void)
{
	/* TODO: save key_data before calling a method in the parent */
	uint8_t tmp_key_data = key_data;

	kim_1_emu::input_process_function_key();

	switch (tmp_key_data) {
	case KIM_1_EMU_KEY_CODE_GO:
		go_mode = sst_mode ? false : true;
		__display_erase_ad_da_text();
		break;
	case KIM_1_EMU_KEY_CODE_SST:
		go_mode = false;
		__display_toggle_sst_mode();
		break;
	case KIM_1_EMU_KEY_CODE_ST:
		go_mode = false;
	case KIM_1_EMU_KEY_CODE_AD:
	case KIM_1_EMU_KEY_CODE_DA:
		__display_toggle_ad_da_mode();
		break;
	}
}

#endif /* ARDUINO_AVR_UNO / ARDUINO_AVR_LEONARDO /
	* ARDUINO_SAMD_ZERO
	*/
