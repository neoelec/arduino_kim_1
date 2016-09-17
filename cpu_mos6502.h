#ifndef __CPU_MOS6502_H__
#define __CPU_MOS6502_H__

#include <stdint.h>

#define ARRAY_SIZE(a)		(sizeof((a)) / sizeof((a)[0]))

#define CPU_MOS6502_NMI_VECTOR_LOW         0xFFFA
#define CPU_MOS6502_NMI_VECTOR_HIGH        0xFFFB

#define CPU_MOS6502_RES_VECTOR_LOW         0xFFFC
#define CPU_MOS6502_RES_VECTOR_HIGH        0xFFFD

#define CPU_MOS6502_IRQ_VECTOR_LOW         0xFFFE
#define CPU_MOS6502_IRQ_VECTOR_HIGH        0xFFFF

#define CPU_MOS6502_STACK_BASE             0x0100

#define CPU_MOS6502_REG_P_CARRY		0x01
#define CPU_MOS6502_REG_P_ZERO		0x02
#define CPU_MOS6502_REG_P_INTERRUPT	0x04
#define CPU_MOS6502_REG_P_DECIMAL		0x08
#define CPU_MOS6502_REG_P_BREAK		0x10
#define CPU_MOS6502_REG_P_CONSTANT		0x20
#define CPU_MOS6502_REG_P_OVERFLOW		0x40
#define CPU_MOS6502_REG_P_SIGN		0x80

enum cpu_mos6502_reg_t {
	E_CPU_MOS6502_REG_A = 0,
	E_CPU_MOS6502_REG_Y,
	E_CPU_MOS6502_REG_X,
	E_CPU_MOS6502_REG_PC,
	E_CPU_MOS6502_REG_S,
	E_CPU_MOS6502_REG_P,
	E_CPU_MOS6502_NR_REG,
};

enum cpu_mos6502_irq_pin_t {
	E_CPU_MOS6502_IRQ_PIN_H = 0,	/* HIGH */
	E_CPU_MOS6502_IRQ_PIN_L,		/* LOW */
	E_CPU_MOS6502_IRQ_PIN_H_L_H,	/* HIGH -> LOW -> HIGH */
};

void cpu_mos6502_reset(void);

void cpu_mos6502_exec(unsigned long tickcount);

void cpu_mos6502_set_core_reg(enum cpu_mos6502_reg_t r_idx, uint16_t val);

uint16_t cpu_mos6502_get_core_reg(enum cpu_mos6502_reg_t r_idx);

void cpu_mos6502_dump_core_reg(void);

void cpu_mos6502_change_nmi_pin(enum cpu_mos6502_irq_pin_t state);

void cpu_mos6502_change_irq_pin(enum cpu_mos6502_irq_pin_t state);

#endif /* __CPU_MOS6502_H__ */
