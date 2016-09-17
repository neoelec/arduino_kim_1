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
 * File Name : arduino_kim_1.ino
 *
 * File Description :
 *
 * Author : Joo, Young Jin <neoelec@gmail.com>
 * Dept : Raccoon's Cave
 * Created Date : 18/Sep/2016
 * Version : Baby-Raccoon
 */

#include "cpu_mos6502.h"
#include "kim_1_emu.h"
#include "kim_1_emu_mega.h"
#include "kim_1_emu_uno.h"

kim_1_emu *mach_emu;

void setup(void)
{
#if defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_SAM_DUE)
	mach_emu = kim_1_emu_mega::get_instance();
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_LEONARDO) || \
	defined(ARDUINO_SAMD_ZERO)
	mach_emu = kim_1_emu_uno::get_instance();
#endif

	mach_emu->init();
}

void loop(void)
{
	mach_emu->loop();
}
