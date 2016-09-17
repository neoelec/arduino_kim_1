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
 * File Name : cpu_mos6502_io_mem.ino
 *
 * File Description :
 *
 * Author : Joo, Young Jin <neoelec@gmail.com>
 * Dept : Raccoon's Cave
 * Created Date : 18/Sep/2016
 * Version : Baby-Raccoon
 */

#include "arduino_kim_1.h"
#include "cpu_mos6502.h"
#include "cpu_mos6502_io_mem.h"
#include "kim_1_emu.h"

#define IO_MEM_RAM_BASE			0x0000
#if defined(ARDUINO_AVR_MEGA2560) || \
	defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD) || \
	defined(ARDUINO_ARCH_ESP8266)
#define IO_MEM_RAM_SIZE			0x1000
#else
#define IO_MEM_RAM_SIZE			0x0400
#endif

static uint8_t __ram[IO_MEM_RAM_SIZE];
static uint8_t *ram = __ram;

#define IO_MEM_6530_003_AIO_BASE	0x1700
#define IO_MEM_6530_002_AIO_BASE	0x1740
#define IO_MEM_6530_AIO_SIZE		0x0040

#define IO_MEM_6530_003_RAM_BASE	0x1780
#define IO_MEM_6530_002_RAM_BASE	0x17C0
#define IO_MEM_6530_RAM_SIZE		0x0040

#define IO_MEM_6530_003_ROM_BASE	0x1800
#define IO_MEM_6530_002_ROM_BASE	0x1C00
#define IO_MEM_6530_ROM_SIZE		0x0400

static uint8_t __io_6503_003_ram[IO_MEM_6530_RAM_SIZE];
static uint8_t *io_6503_003_ram = __io_6503_003_ram;

static uint8_t __io_6503_002_ram[IO_MEM_6530_RAM_SIZE];
static uint8_t *io_6503_002_ram = __io_6503_002_ram;

static const uint8_t __io_6503_003_rom[] PROGMEM = {
#include "io_mem_mos6530_003_rom.h"
};
static const uint8_t *io_6503_003_rom = __io_6503_003_rom;
#define IO_MEM_6530_003_ROM_SIZE	ARRAY_SIZE(__io_6503_003_rom)

static const uint8_t __io_6503_002_rom[] PROGMEM = {
#include "io_mem_mos6530_002_rom.h"
};
static const uint8_t *io_6503_002_rom = __io_6503_002_rom;
#define IO_MEM_6530_002_ROM_SIZE	ARRAY_SIZE(__io_6503_002_rom)

#if defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAMD_ZERO)
#define IO_MEM_RAM_2_BASE		0x2000

#ifdef ARDUINO_SAM_DUE
#define IO_MEM_RAM_2_SIZE		0x8000
#else
#define IO_MEM_RAM_2_SIZE		0x2000
#endif

static uint8_t __ram_2[IO_MEM_RAM_2_SIZE];
static uint8_t *ram_2 = __ram_2;
#endif

#define CPU_MOS6502_INSTR_NOP		0xEA

static void inline __io_mem_display_7_seg(void)
{
	uint8_t seg[3];

	seg[0] = __cpu_mos6502_io_mem_read_ram(IO_MEM_RAM_BASE, ram, 0x00FB);
	seg[1] = __cpu_mos6502_io_mem_read_ram(IO_MEM_RAM_BASE, ram, 0x00FA);
	seg[2] = __cpu_mos6502_io_mem_read_ram(IO_MEM_RAM_BASE, ram, 0x00F9);

	mach_emu->display_update_fnd(seg);
}

#define KIM_1_FW_ENTRY_DUMPT		0x1800	/* Write (Dump) to audio tape. */
#define KIM_1_FW_ENTRY_LOADT		0x1873	/* Read (Load) from audio tape. */
#define KIM_1_FW_ENTRY_ONE		0x199E	/* Send 3700 Hz tone to tape. */
#define KIM_1_FW_ENTRY_ZRO		0x19C4	/* Send 200 Hz tone to tape. */
#define KIM_1_FW_ENTRY_PLLCAL		0x1A6B	/* Send 300 Hz PLL reference tone to tape. */
#define KIM_1_FW_ENTRY_AK		0x1EFE	/* Check for key depressed. A non-zero: no key down. A equal 0, key down. */
#define KIM_1_FW_ENTRY_SCAND		0x1F19	/* Display address and contents. */
#define KIM_1_FW_ENTRY_SCANDS		0x1F1F	/* Output six hex characters on display. Stored in 0x00F9, 0x00FA, 0x00FB. */
#define KIM_1_FW_ENTRY_KEYIN		0x1F40	/* Open up keyboard channel. Call before using GETKEY (or call SCANDS). */
#define KIM_1_FW_ENTRY_INCPT		0x1F63	/* Increment display address. */
#define KIM_1_FW_ENTRY_GETKEY		0x1F6A	/* Return key from keyboard. Value 0-F, 10(AD), 11(DA), 12(+), 13(GO), 14(PC), 15 (no keypress). */
#define KIM_1_FW_ENTRY_TABLE		0x1FE7	/* Table of 7-segment patterns. */
#define KIM_1_FW_ENTRY_PRTPNT		0x1E1E	/* Prints contents of 0x00FB, 0x00FA on TTY. */
#define KIM_1_FW_ENTRY_PRTBYT		0x1E3B	/* Prints A as two hex characters on TTY. */
#define KIM_1_FW_ENTRY_GETCH		0x1E5A	/* Get one ASCII character from TTY and return in A. */
#define KIM_1_FW_ENTRY_OUTSP		0x1E9E	/* Print space on TTY. */
#define KIM_1_FW_ENTRY_OUTCH		0x1EA0	/* Print ASCII character in A on TTY. */
#define KIM_1_FW_ENTRY_GETBYT		0x1F9D	/* Get two hex characters from TTY and return them packed in A. */
#define KIM_1_FW_ENTRY_SAVE		0x1C00	/* Normal interrupt entry point. */
#define KIM_1_FW_ENTRY_RST		0x1C22	/* Reset return to monitor. */
#define KIM_1_FW_ENTRY_START		0x1C4F	/* Return to monitor entry. */

#define KIM_1_FW_ENTRY_DETCPS		0x1C2A
#define KIM_1_FW_ENTRY_GETCH_GET1	0x1E65

static bool inline __io_mem_hook_read_6530_002_rom(uint16_t addr, uint8_t &data)
{
	if (KIM_1_FW_ENTRY_OUTCH == addr) {
		mach_emu->teletype_out(reg_A);
		reg_PC = 0x1ED3;
		data = CPU_MOS6502_INSTR_NOP;
		return true;
	} else if (KIM_1_FW_ENTRY_GETCH_GET1 == addr) {
		reg_A = mach_emu->teletype_in();
		if (0x00 == reg_A) {
			reg_PC = 0x1E60;
			data = CPU_MOS6502_INSTR_NOP;
			return true;
		}
		reg_X = __cpu_mos6502_io_mem_read_ram(IO_MEM_RAM_BASE,
				ram, 0x00FD);
		reg_PC = 0x1E87;
		data = CPU_MOS6502_INSTR_NOP;
		return true;
	} else if (KIM_1_FW_ENTRY_DETCPS == addr) {
		__cpu_mos6502_io_mem_write_ram(IO_MEM_6530_002_RAM_BASE,
				io_6503_002_ram, 0x17F2, 1);
		__cpu_mos6502_io_mem_write_ram(IO_MEM_6530_002_RAM_BASE,
				io_6503_002_ram, 0x17F3, 1);
		reg_PC = 0x1C4F;
		data = CPU_MOS6502_INSTR_NOP;
		return true;
	} else if (KIM_1_FW_ENTRY_SCANDS == addr) {
		__io_mem_display_7_seg();
		reg_PC = 0x1F45;
		data = CPU_MOS6502_INSTR_NOP;
		return true;
	} else if (KIM_1_FW_ENTRY_AK == addr) {
		reg_A = mach_emu->is_key_available() ? 0x00 : 0xFF;
		reg_PC = 0x1F14;
		data = CPU_MOS6502_INSTR_NOP;
		return true;
	} else if (KIM_1_FW_ENTRY_GETKEY == addr) {
		reg_A = mach_emu->keyboard_in();
		reg_PC = 0x1F90;
		data = CPU_MOS6502_INSTR_NOP;
		return true;
	}

	return false;
}

uint8_t cpu_mos6502_read_io_mem(uint16_t addr)
{
	uint8_t tmp8;

	/* [[BEGIN>> Main Memory */
	__cpu_mos6502_io_mem_read_ram_if_in(IO_MEM_RAM_BASE, IO_MEM_RAM_SIZE,
			ram, addr);
	/* <<END]] Main Memory */

	/* [[BEGIN>> U003:6030 Application I/O / RAM / ROM */
	__cpu_mos6502_io_mem_if_in(IO_MEM_6530_003_AIO_BASE,
			IO_MEM_6530_AIO_SIZE, addr) {
		return 0x00;
	}

	__cpu_mos6502_io_mem_read_ram_if_in(IO_MEM_6530_003_RAM_BASE,
			IO_MEM_6530_RAM_SIZE, io_6503_003_ram, addr);

	__cpu_mos6502_io_mem_read_pgm_if_in(IO_MEM_6530_003_ROM_BASE,
			IO_MEM_6530_003_ROM_SIZE,
			io_6503_003_rom, addr);
	/* <<END]] U003:6030 Application I/O / RAM / ROM */

	/* [[BEGIN>> U002:6030 Application I/O / RAM / ROM */
	__cpu_mos6502_io_mem_if_in(IO_MEM_6530_002_AIO_BASE,
			IO_MEM_6530_AIO_SIZE, addr) {
		switch (addr) {
		case 0x1747:		/* CLKRDI */
			return 0xFF;	/* always completed */
		case 0x1740:
			return !mach_emu->is_teletype_connected();
		}

		return 0x00;
	}

	__cpu_mos6502_io_mem_read_ram_if_in(IO_MEM_6530_002_RAM_BASE,
			IO_MEM_6530_RAM_SIZE, io_6503_002_ram, addr);

	__cpu_mos6502_io_mem_if_in(IO_MEM_6530_002_ROM_BASE,
			IO_MEM_6530_002_ROM_SIZE, addr) {
		if (__io_mem_hook_read_6530_002_rom(addr, tmp8))
			return tmp8;
		return __cpu_mos6502_io_mem_read_pgm(IO_MEM_6530_002_ROM_BASE,
				io_6503_002_rom, addr);
	}
	/* <<END]] U003:6030 Application I/O / RAM / ROM */

	/* [[BEGIN>> NMI/IRQ Vector */
	/*  rerouting to 6530-002 rom  for vectors*/
	if (addr >= CPU_MOS6502_NMI_VECTOR_LOW)
		return(pgm_read_byte_near(io_6503_002_rom + addr - 0xFC00));
	/* <<END]] NMI/IRQ Vector */

#if defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAMD_ZERO)
	/* [[BEGIN>> Additional Memory */
	__cpu_mos6502_io_mem_read_ram_if_in(IO_MEM_RAM_2_BASE,
			IO_MEM_RAM_2_SIZE, ram_2, addr);
	/* <<END]] Additional Memory */
#endif
	return 0;
}

void cpu_mos6502_write_io_mem(uint16_t addr, uint8_t data)
{
	/* [[BEGIN>> Main Memory */
	__cpu_mos6502_io_mem_write_ram_if_in(IO_MEM_RAM_BASE, IO_MEM_RAM_SIZE,
			ram, addr, data);
	/* <<END]] Main Memory */

	/* [[BEGIN>> U003:6030 Application I/O / RAM / ROM */
	__cpu_mos6502_io_mem_if_in(IO_MEM_6530_003_AIO_BASE,
			IO_MEM_6530_AIO_SIZE, addr) {
		return;
	}

	__cpu_mos6502_io_mem_write_ram_if_in(IO_MEM_6530_003_RAM_BASE,
			IO_MEM_6530_RAM_SIZE, io_6503_003_ram, addr, data);

	__cpu_mos6502_io_mem_write_pgm_if_in(IO_MEM_6530_003_ROM_BASE,
			IO_MEM_6530_003_ROM_SIZE, io_6503_003_rom, addr, data);
	/* <<END]] U003:6030 Application I/O / RAM / ROM */

	/* [[BEGIN>> U002:6030 Application I/O / RAM / ROM */
	__cpu_mos6502_io_mem_if_in(IO_MEM_6530_002_AIO_BASE,
			IO_MEM_6530_AIO_SIZE, addr) {
		return;
	}

	__cpu_mos6502_io_mem_write_ram_if_in(IO_MEM_6530_002_RAM_BASE,
			IO_MEM_6530_RAM_SIZE, io_6503_002_ram, addr, data);

	__cpu_mos6502_io_mem_write_pgm_if_in(IO_MEM_6530_002_RAM_BASE,
			IO_MEM_6530_002_ROM_SIZE, io_6503_002_rom, addr, data);
	/* <<END]] U002:6030 Application I/O / RAM / ROM */

#if defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAMD_ZERO)
	/* [[BEGIN>> Additional Memory */
	__cpu_mos6502_io_mem_write_ram_if_in(IO_MEM_RAM_2_BASE,
			IO_MEM_RAM_2_SIZE, ram_2, addr, data);
	/* <<END]] Additional Memory */
#endif
}
