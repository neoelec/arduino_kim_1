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
 * File Name : kim_1_emu_mega.h
 *
 * File Description :
 *
 * Author : Joo, Young Jin <neoelec@gmail.com>
 * Dept : Raccoon's Cave
 * Created Date : 25/Sep/2016
 * Version : Baby-Raccoon
 */

#ifndef __KIM_1_EMU_MEGA_H__
#define __KIM_1_EMU_MEGA_H__

#if defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_SAM_DUE)

#include <stdint.h>
#include <stdio.h>

#include "kim_1_emu.h"

class kim_1_emu_mega : public kim_1_emu {
public:
	kim_1_emu_mega();
	static kim_1_emu_mega *get_instance(void);

	virtual void display_update_fnd(uint8_t *seg);

protected:
	virtual void display_init(void);
	virtual void keyboard_scan(void);
	virtual void keyboard_init(void);
	virtual void input_process_function_key(void);

private:
	void __display_toggle_sst_led(void);
	void __display_update_each_fnd(uint8_t pos, uint8_t num);

	static kim_1_emu_mega *singleton;

	const uint8_t *key_row;
	size_t key_row_nr;
	const uint8_t *key_col;
	size_t key_col_nr;
	const uint8_t *fnd_led;
	size_t fnd_led_nr;
	const uint8_t *fnd_common;
	size_t fnd_common_nr;
	const uint8_t *fnd_code;
	size_t fnd_code_nr;
	uint8_t sst_led;
};

#endif /* ARDUINO_AVR_MEGA2560 / ARDUINO_SAM_DUE */

#endif /* __KIM_1_EMU_MEGA_H__ */
