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
 * File Name : cpu_mos6502_io_mem.h
 *
 * File Description :
 *
 * Author : Joo, Young Jin <neoelec@gmail.com>
 * Dept : Raccoon's Cave
 * Created Date : 18/Sep/2016
 * Version : Baby-Raccoon
 */

#ifndef __CPU_MOS6502_IO_MEM_H__
#define __CPU_MOS6502_IO_MEM_H__

#define __cpu_mos6502_io_mem_if_in(__base, __size, __addr)		\
	if (((__addr) >= (__base)) &&					\
	    ((__addr) <= ((__base) + ((__size) - 1))))

#define __cpu_mos6502_io_mem_read_pgm(__base, __ptr, __addr)		\
	pgm_read_byte((const uint8_t *)__ptr + ((__addr) - (__base)))

#define __cpu_mos6502_io_mem_read_pgm_if_in(__base, __size, __ptr, __addr) \
do {									\
	__cpu_mos6502_io_mem_if_in((__base), (__size), (__addr))	\
		return __cpu_mos6502_io_mem_read_pgm(__base, __ptr, __addr); \
} while (0)

#define __cpu_mos6502_io_mem_write_pgm(__base, __ptr, __addr, __data)

#define __cpu_mos6502_io_mem_write_pgm_if_in(__base, __size, __ptr, __addr, \
		__data)							\
do {									\
	__cpu_mos6502_io_mem_if_in((__base), (__size), (__addr)) {	\
		__cpu_mos6502_io_mem_write_pgm(__base, __ptr, __addr, __data); \
		return;							\
	}								\
} while (0)

#define __cpu_mos6502_io_mem_read_ram(__base, __ptr, __addr)		\
	((uint8_t *)(__ptr))[((__addr) - (__base))]

#define __cpu_mos6502_io_mem_read_ram_if_in(__base, __size, __ptr, __addr)\
do {									\
	__cpu_mos6502_io_mem_if_in((__base), (__size), (__addr))	\
		return __cpu_mos6502_io_mem_read_ram(__base, __ptr, __addr); \
} while (0)

#define __cpu_mos6502_io_mem_write_ram(__base, __ptr, __addr, __data)	\
	((uint8_t *)(__ptr))[((__addr) - (__base))] = __data

#define __cpu_mos6502_io_mem_write_ram_if_in(__base, __size, __ptr, __addr, \
		__data)							\
do {									\
	__cpu_mos6502_io_mem_if_in((__base), (__size), (__addr)) {	\
		__cpu_mos6502_io_mem_write_ram(__base, __ptr, __addr, __data); \
		return;							\
	}								\
} while (0)

uint8_t cpu_mos6502_read_io_mem(uint16_t addr);

void cpu_mos6502_write_io_mem(uint16_t addr, uint8_t data);

#endif /* __CPU_MOS6502_IO_MEM_H__ */
