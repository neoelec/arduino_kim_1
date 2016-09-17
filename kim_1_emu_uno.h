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
 * File Name : kim_1_emu_uno.h
 *
 * File Description :
 *
 * Author : Joo, Young Jin <neoelec@gmail.com>
 * Dept : Raccoon's Cave
 * Created Date : 25/Sep/2016
 * Version : Baby-Raccoon
 */

#ifndef __KIM_1_EMU_UNO_H__
#define __KIM_1_EMU_UNO_H__

//#define CFG_KIM_1_UNO_USE_LCD

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_LEONARDO) || \
	defined(ARDUINO_SAMD_ZERO) || \
	defined(ARDUINO_ARCH_ESP8266)

#include <stdint.h>
#include <stdio.h>

#ifdef CFG_KIM_1_UNO_USE_LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#endif

#include "kim_1_emu.h"

class kim_1_emu_uno : public kim_1_emu {
public:
	kim_1_emu_uno();
	static kim_1_emu_uno *get_instance(void);

	virtual bool is_teletype_connected(void);

	virtual void display_update_fnd(uint8_t *seg);

protected:
	virtual void display_init(void);
	virtual void input_process_function_key(void);

private:
	void __display_erase_ad_da_text(void);
	void __display_toggle_ad_da_mode(void);
	void __display_toggle_sst_mode(void);

	static kim_1_emu_uno *singleton;

#ifdef CFG_KIM_1_UNO_USE_LCD
	LiquidCrystal_I2C *lcd;
#endif
	bool go_mode;
};

#endif /* ARDUINO_AVR_UNO / ARDUINO_AVR_LEONARDO /
	* ARDUINO_SAMD_ZERO
	*/

#endif /* __KIM_1_EMU_UNO_H__ */
