#include <avr/pgmspace.h>
#include <stdint.h>

#include "cpu_mos6502.h"
#include "cpu_mos6502_io_mem.h"

// slower, but allows you to specify number of cycles to run for exec6502
// rather than simply a number of instructions. also uses a little more
// program memory when enabled.
//#define CFG_CPU_MOS6502_USE_TIMING

/* registers */
static uint8_t reg_A;			/* ACCUMULATOR */
static uint8_t reg_Y;			/* Y INDEX REG */
static uint8_t reg_X;			/* X INDEX REG */
static uint16_t reg_PC;			/* PROGRAM COUNTER */
static uint8_t reg_S;			/* STACK PNTR */
static union {
	uint8_t raw;
	struct {
		uint8_t C:1;		/* carry = _borrow */
		uint8_t Z:1;		/* zero result */
		uint8_t I:1;		/* IRQ disabled */
		uint8_t D:1;		/* decimal mode */
		uint8_t B:1;		/* BRK instruction */
		uint8_t U:1;		/* unused? */
		uint8_t V:1;		/* overflow */
		uint8_t N:1;		/* negative result */
	} __attribute__((packed));
} reg_P;				/* FLAGS */

/* helper variables */
static uint16_t effective_addr;
static bool fetch_from_reg_A;
static enum cpu_mos6502_irq_pin_t nmi_pin;
static enum cpu_mos6502_irq_pin_t irq_pin;

enum cpu_mos6502_irq_state_t {
	E_CPU_MOS6502_IRQ_STATE_OUT = 0,
	E_CPU_MOS6502_IRQ_STATE_IN,
	E_CPU_MOS6502_IRQ_STATE_EXIT,
};

static enum cpu_mos6502_irq_state_t irq_state;

/* timing */
#ifdef CFG_CPU_MOS6502_USE_TIMING
static int clk_per_instr, clk_goal;
static unsigned long cnt_instr;
#define __cpu_mos6502_set_clk_per_instr(n)				\
do {									\
	clk_per_instr = (n);						\
} while (0)
#else
#define __cpu_mos6502_set_clk_per_instr(n)		do { } while(0)
#endif

static void inline __cpu_mos6502_push_word(uint16_t val)
{
	cpu_mos6502_write_io_mem(CPU_MOS6502_STACK_BASE + reg_S--,
			     (uint8_t)(val >> 8));
	cpu_mos6502_write_io_mem(CPU_MOS6502_STACK_BASE + reg_S--,
			     (uint8_t)(val & 0x00FF));
}

static void inline __cpu_mos6502_push_byte(uint8_t pushval)
{
	cpu_mos6502_write_io_mem(CPU_MOS6502_STACK_BASE + reg_S--, pushval);
}

static uint16_t inline __cpu_mos6502_pull_word(void)
{
	uint16_t val;

	val = (uint16_t)cpu_mos6502_read_io_mem(
			CPU_MOS6502_STACK_BASE + ++reg_S);
	val |= (uint16_t)cpu_mos6502_read_io_mem(
			CPU_MOS6502_STACK_BASE + ++reg_S) << 8;

	return val;
}

static uint8_t inline __cpu_mos6502_pull_byte(void)
{
	return cpu_mos6502_read_io_mem(CPU_MOS6502_STACK_BASE + ++reg_S);
}

static void inline __cpu_mos6502_update_P_Z(uint8_t val)
{
	reg_P.Z = val ? 0 : 1;
}

static void inline __cpu_mos6502_update_P_N(uint8_t val)
{
	 reg_P.N = val & 0x80 ? 1 : 0;
}

static void inline __cpu_mos6502_update_P_C(uint16_t val)
{
	 reg_P.C = val & 0xFF00 ? 1 : 0;
}

static void inline __cpu_mos6502_update_P_V(uint16_t val,
		uint8_t in_0, uint8_t in_1)
{
	reg_P.V = ((val ^ (uint16_t)in_0) &
			(val ^ (uint16_t)in_1) & 0x0080) ? 1 : 0;
}

/* core addressing */
/*  : None */
static void inline __cpu_mos6502_access_none(void)
{
	__cpu_mos6502_set_clk_per_instr(1);
}

/* #n : Immediate */
static void inline __cpu_mos6502_access_immediate(void)
{
	__cpu_mos6502_set_clk_per_instr(2);

	effective_addr = reg_PC++;
}

/* n : Zero Page */
static void inline __cpu_mos6502_access_zero_page(void)
{
	__cpu_mos6502_set_clk_per_instr(2);

	effective_addr = (uint16_t)cpu_mos6502_read_io_mem(reg_PC++);
}

/* nn : Absolute */
static void inline __cpu_mos6502_access_absolute(void)
{
	__cpu_mos6502_set_clk_per_instr(3);

	effective_addr = (uint16_t)cpu_mos6502_read_io_mem(reg_PC++);
	effective_addr |= (uint16_t)cpu_mos6502_read_io_mem(reg_PC++) << 8;
}

/* (n),Y : Ind Y */
static void inline __cpu_mos6502_access_indirect_y(void)
{
	uint16_t ind_addr;
	uint16_t start_page;

	__cpu_mos6502_access_zero_page();
	// __cpu_mos6502_set_clk_per_instr(2);

	ind_addr = effective_addr;
	effective_addr = (uint16_t)cpu_mos6502_read_io_mem(ind_addr);

	ind_addr = (ind_addr & 0xFF00) | ((ind_addr + 1) & 0x00FF);
	effective_addr |= (uint16_t)cpu_mos6502_read_io_mem(ind_addr) << 8;

	start_page = effective_addr & 0xFF00;
	effective_addr += (uint16_t)reg_Y;
	__cpu_mos6502_set_clk_per_instr(clk_per_instr +
			start_page != (effective_addr & 0xFF00));
}

/* (n,X) : Ind X */
static void inline __cpu_mos6502_access_indirect_x(void)
{
	uint16_t ind_addr;

	__cpu_mos6502_access_zero_page();
	// __cpu_mos6502_set_clk_per_instr(2);

	ind_addr = effective_addr + (uint16_t)reg_X;
	ind_addr &= 0xFF;

	effective_addr = (uint16_t)cpu_mos6502_read_io_mem(ind_addr);
	ind_addr = (ind_addr + 1) & 0xFF;
	effective_addr |= (uint16_t)cpu_mos6502_read_io_mem(ind_addr) << 8;
}

/* (nn) : Indirect */
static void inline __cpu_mos6502_access_indirect(void)
{
	uint16_t ind_addr;

	__cpu_mos6502_access_absolute();
	// __cpu_mos6502_set_clk_per_instr(3);

	ind_addr = effective_addr;
	effective_addr = (uint16_t)cpu_mos6502_read_io_mem(ind_addr);

	ind_addr = (ind_addr & 0xFF00) | ((ind_addr + 1) & 0x00FF);
	effective_addr |= (uint16_t)cpu_mos6502_read_io_mem(ind_addr) << 8;
}

/* A : Accumulator */
static void inline __cpu_mos6502_access_accumulator(void)
{
	__cpu_mos6502_set_clk_per_instr(1);

	fetch_from_reg_A = true;
}

/* n,X : Zero Page X */
static void inline __cpu_mos6502_access_zero_page_x(void)
{
	__cpu_mos6502_access_zero_page();
	// __cpu_mos6502_set_clk_per_instr(2);

	effective_addr += (uint16_t)reg_X;
	effective_addr &= 0x00FF;
}

/* n,Y : Zero Page Y */
static void inline __cpu_mos6502_access_zero_page_y(void)
{
	__cpu_mos6502_access_zero_page();
	// __cpu_mos6502_set_clk_per_instr(2);

	effective_addr += (uint16_t)reg_Y;
	effective_addr &= 0x00FF;
}

/* nn,X : Absolute X */
static void inline __cpu_mos6502_access_absolute_x(void)
{
	uint16_t start_page;

	__cpu_mos6502_access_absolute();
	// __cpu_mos6502_set_clk_per_instr(3);

	start_page = effective_addr & 0xFF00;
	effective_addr += (uint16_t)reg_X;
	__cpu_mos6502_set_clk_per_instr(clk_per_instr +
			start_page != (effective_addr & 0xFF00));
}

/* nn,Y : Absolute Y */
static void inline __cpu_mos6502_access_absolute_y(void)
{
	uint16_t start_page;

	__cpu_mos6502_access_absolute();
	// __cpu_mos6502_set_clk_per_instr(3);

	start_page = effective_addr & 0xFF00;
	effective_addr += (uint16_t)reg_Y;
	__cpu_mos6502_set_clk_per_instr(clk_per_instr +
			start_page != (effective_addr & 0xFF00));
}

/* r : Relative */
static void inline __cpu_mos6502_access_relative(void)
{
	__cpu_mos6502_set_clk_per_instr(2);

	effective_addr = (uint16_t)cpu_mos6502_read_io_mem(reg_PC++);
	if (effective_addr & 0x0080)
		effective_addr |= 0xFF00;
}

static uint8_t inline __cpu_mos6502_fetch_data(void)
{
	if (fetch_from_reg_A)
		return reg_A;
	else
		return cpu_mos6502_read_io_mem(effective_addr);
}

static void inline __cpu_mos6502_write_back_data(uint8_t val)
{
	if (fetch_from_reg_A)
		reg_A = val;
	else
		cpu_mos6502_write_io_mem(effective_addr, val);
}

/* core operations */

/* ADC : Add with carry to A */
static void inline __cpu_mos6502_execute_adc(void)
{
	if (!reg_P.D) {
		uint8_t in_b = __cpu_mos6502_fetch_data();
		union {
			uint16_t w;
			uint8_t b[2];
		} out;

		out.w = (uint16_t)reg_A + (uint16_t)in_b +
				(uint16_t)(reg_P.C);

		__cpu_mos6502_update_P_N(out.b[0]);
		__cpu_mos6502_update_P_V(out.w, reg_A, in_b);
		__cpu_mos6502_update_P_Z(out.b[0]);
		__cpu_mos6502_update_P_C(out.w);

		reg_A = out.b[0];
	} else {
		union {
			uint8_t b;
			struct {
				uint8_t l_4bit:4;
				uint8_t h_4bit:4;
			};
		} in, tmp, low, high;

		in.b = __cpu_mos6502_fetch_data();
		tmp.b = reg_A;

		low.b = tmp.l_4bit + in.l_4bit + reg_P.C;
		if (low.b > 0x09)
			low.b += 0x06;

		high.b = tmp.h_4bit + in.h_4bit + low.h_4bit;
		if (high.b > 0x09)
			high.b += 0x06;

		tmp.l_4bit = low.l_4bit;
		tmp.h_4bit = high.l_4bit;

		reg_P.C = high.h_4bit;
		__cpu_mos6502_update_P_Z(tmp.b);
		reg_P.N = 0;
		reg_P.Z = 0;

		reg_A = tmp.b;

		__cpu_mos6502_set_clk_per_instr(clk_per_instr + 1);
	}
}

/* AND : AND to A */
static void inline __cpu_mos6502_execute_and(void)
{
	reg_A &= __cpu_mos6502_fetch_data();

	__cpu_mos6502_update_P_N(reg_A);
	__cpu_mos6502_update_P_Z(reg_A);
}

/* ASL : Arithmetic shift left */
static void inline __cpu_mos6502_execute_asl(void)
{
	union {
		uint16_t w;
		uint8_t b[2];
	} in;

	in.w = (uint16_t)__cpu_mos6502_fetch_data();
	in.w = in.w << 1;

	__cpu_mos6502_update_P_C(in.w);
	__cpu_mos6502_update_P_Z(in.b[0]);
	__cpu_mos6502_update_P_N(in.b[0]);

	__cpu_mos6502_write_back_data(in.b[0]);
}

/* for BCC / BCS / BEQ / BNE / BMI / BPL / BVC / BVS */
static void inline __cpu_mos6502_branch_common(void)
{
	uint8_t old_reg_PC = reg_PC;

	reg_PC += effective_addr;
	if ((old_reg_PC ^ reg_PC) & 0xFF00)
		//check if jump crossed reg_A page boundary
		__cpu_mos6502_set_clk_per_instr(clk_per_instr + 2);
	else
		__cpu_mos6502_set_clk_per_instr(clk_per_instr + 1);
}

/* BCC : Branch if Carry clear (C=0) */
static void inline __cpu_mos6502_execute_bcc(void)
{
	if (!reg_P.C)
		__cpu_mos6502_branch_common();
}

/* BCS : Branch if Carry set (C=1) */
static void inline __cpu_mos6502_execute_bcs(void)
{
	if (reg_P.C)
		__cpu_mos6502_branch_common();
}

/* BEQ : Branch if equal (Z=1) */
static void inline __cpu_mos6502_execute_beq(void)
{
	if (reg_P.Z)
		__cpu_mos6502_branch_common();
}

/* BMI : Branch if minus (N=1) */
static void inline __cpu_mos6502_execute_bmi(void)
{
	if (reg_P.N)
		__cpu_mos6502_branch_common();
}

/* BNE : Branch if not equal (Z=0) */
static void inline __cpu_mos6502_execute_bne(void)
{
	if (!reg_P.Z)
		__cpu_mos6502_branch_common();
}

/* BPL : Branch if plus (N=0) */
static void inline __cpu_mos6502_execute_bpl(void)
{
	if (!reg_P.N)
		__cpu_mos6502_branch_common();
}

/* BVC : Branch if ovfl clear (V=0) */
static void inline __cpu_mos6502_execute_bvc(void)
{
	if (!reg_P.V)
		__cpu_mos6502_branch_common();
}

/* BVS : Branch if ovf l set (V=1) */
static void inline __cpu_mos6502_execute_bvs(void)
{
	if (reg_P.V)
		__cpu_mos6502_branch_common();
}

/* BIT : AND with A (A unchanged) */
static void inline __cpu_mos6502_execute_bit(void)
{
	uint8_t in_b = __cpu_mos6502_fetch_data();

	reg_P.raw = (reg_P.raw & 0x3F) | (in_b & 0xC0);

	in_b &= reg_A;

	__cpu_mos6502_update_P_Z(in_b);
}

/* BRK : Break (force interrupt) */
static void inline __cpu_mos6502_execute_brk(void)
{
	reg_PC++;
	//push next instruction address onto stack
	__cpu_mos6502_push_word(reg_PC);
	reg_P.B = 1;
	__cpu_mos6502_push_byte(reg_P.raw);	//push CPU reg_P.raw to stack
	reg_P.B = 0;
	reg_P.I = 1;				//set interrupt flag

	reg_PC = (uint16_t)cpu_mos6502_read_io_mem(CPU_MOS6502_IRQ_VECTOR_LOW);
	reg_PC |= (uint16_t)cpu_mos6502_read_io_mem(CPU_MOS6502_IRQ_VECTOR_HIGH) << 8;

	irq_state = E_CPU_MOS6502_IRQ_STATE_IN;
}

/* CLC : Clear carry */
static void inline __cpu_mos6502_execute_clc(void)
{
	reg_P.C = 0;
}

/* CLD : Clear decimal mode */
static void inline __cpu_mos6502_execute_cld(void)
{
	reg_P.D = 0;
}

/* CLI : Clear IRQ disable */
static void inline __cpu_mos6502_execute_cli(void)
{
	reg_P.I = 0;
}

/* CLV : Clear overflow */
static void inline __cpu_mos6502_execute_clv(void)
{
	reg_P.V = 0;
}

/* for CMP / CPX / CPY */
static void inline __cpu_mos6502_compare_common(uint8_t &val)
{
	uint8_t in_b = __cpu_mos6502_fetch_data();

	reg_P.C = val >= in_b ? 1 : 0;
	reg_P.Z = val == in_b ? 1 : 0;

	in_b = val - in_b;

	__cpu_mos6502_update_P_N(in_b);
}

/* CMP : Compare with A */
static void inline __cpu_mos6502_execute_cmp(void)
{
	__cpu_mos6502_compare_common(reg_A);
}

/* CPX : Compare with X */
static void inline __cpu_mos6502_execute_cpx(void)
{
	__cpu_mos6502_compare_common(reg_X);
}

/* CPY : Compare with Y */
static void inline __cpu_mos6502_execute_cpy(void)
{
	__cpu_mos6502_compare_common(reg_Y);
}

/* DEC / DEX / DEY */
static void inline __cpu_mos6502_decrease_common(uint8_t &val)
{
	val--;

	__cpu_mos6502_update_P_Z(val);
	__cpu_mos6502_update_P_N(val);
}

/* DEC : Decrement by one */
static void inline __cpu_mos6502_execute_dec(void)
{
	uint8_t in_b = __cpu_mos6502_fetch_data();

	__cpu_mos6502_decrease_common(in_b);

	__cpu_mos6502_write_back_data(in_b);
}

/* DEX : Decrement X by one */
static void inline __cpu_mos6502_execute_dex(void)
{
	__cpu_mos6502_decrease_common(reg_X);
}

/* DEY : Decrement Y by one */
static void inline __cpu_mos6502_execute_dey(void)
{
	__cpu_mos6502_decrease_common(reg_Y);
}

/* EOR : XOR to A */
static void inline __cpu_mos6502_execute_eor(void)
{
	reg_A ^= __cpu_mos6502_fetch_data();

	__cpu_mos6502_update_P_Z(reg_A);
	__cpu_mos6502_update_P_N(reg_A);
}

/* INC / INX / INY */
static void inline __cpu_mos6502_increase_common(uint8_t &val)
{
	val++;

	__cpu_mos6502_update_P_Z(val);
	__cpu_mos6502_update_P_N(val);
}

/* INC : Increment by one */
static void inline __cpu_mos6502_execute_inc(void)
{
	uint8_t in_b = __cpu_mos6502_fetch_data();

	__cpu_mos6502_increase_common(in_b);

	__cpu_mos6502_write_back_data(in_b);
}

/* INX : Increment X by one */
static void inline __cpu_mos6502_execute_inx(void)
{
	__cpu_mos6502_increase_common(reg_X);
}

/* INY : Increment Y by one */
static void inline __cpu_mos6502_execute_iny(void)
{
	__cpu_mos6502_increase_common(reg_Y);
}

/* JMP : Jump to new location */
static void inline __cpu_mos6502_execute_jmp(void)
{
	reg_PC = effective_addr;
}

/* JSR : Jump to subroutine */
static void inline __cpu_mos6502_execute_jsr(void)
{
	__cpu_mos6502_push_word(reg_PC - 1);
	reg_PC = effective_addr;
}

/* LDA / LDX / LDY */
static void inline __cpu_mos6502_load_common(uint8_t &dest)
{
	dest = __cpu_mos6502_fetch_data();

	__cpu_mos6502_update_P_Z(dest);
	__cpu_mos6502_update_P_N(dest);
}

/* LDA : Load A */
static void inline __cpu_mos6502_execute_lda(void)
{
	__cpu_mos6502_load_common(reg_A);
}

/* LDX : Load X */
static void inline __cpu_mos6502_execute_ldx(void)
{
	__cpu_mos6502_load_common(reg_X);
}

/* LDY : Load Y */
static void inline __cpu_mos6502_execute_ldy(void)
{
	__cpu_mos6502_load_common(reg_Y);
}

/* LSR : Logical shift right */
static void inline __cpu_mos6502_execute_lsr(void)
{
	union {
		uint16_t w;
		uint8_t b[2];
	} in = { .w = 0, };

	in.b[1] = __cpu_mos6502_fetch_data();
	in.w = in.w >> 1;

	reg_P.C = !!(in.b[0]);

	__cpu_mos6502_update_P_Z(in.b[1]);
	__cpu_mos6502_update_P_N(in.b[1]);

	__cpu_mos6502_write_back_data(in.b[1]);
}

/* NOP : No operation */
static void inline __cpu_mos6502_execute_nop(void)
{
}

/* ORA : OR to A */
static void inline __cpu_mos6502_execute_ora(void)
{
	reg_A |= __cpu_mos6502_fetch_data();

	__cpu_mos6502_update_P_Z(reg_A);
	__cpu_mos6502_update_P_N(reg_A);
}

/* PHA : Push A onto stack */
static void inline __cpu_mos6502_execute_pha(void)
{
	__cpu_mos6502_push_byte(reg_A);
}

/* PHP : Push P onto stack */
static void inline __cpu_mos6502_execute_php(void)
{
	reg_P.B = 1;
	__cpu_mos6502_push_byte(reg_P.raw);
	reg_P.B = 0;
}

/* PLA : Pull (pop) A from stack */
static void inline __cpu_mos6502_execute_pla(void)
{
	reg_A = __cpu_mos6502_pull_byte();

	__cpu_mos6502_update_P_Z(reg_A);
	__cpu_mos6502_update_P_N(reg_A);
}

/* PLP : Pull (pop) P from stack */
static void inline __cpu_mos6502_execute_plp(void)
{
	reg_P.raw = __cpu_mos6502_pull_byte();
	/* FIXME: Why? */
	reg_P.U = 1;
}

/* ROL : Rotate left through carry */
static void inline __cpu_mos6502_execute_rol(void)
{
	union {
		uint16_t w;
		uint8_t b[2];
	} in;

	in.w = (uint16_t)__cpu_mos6502_fetch_data();
	in.w = in.w << 1 | reg_P.C;

	__cpu_mos6502_update_P_C(in.w);
	__cpu_mos6502_update_P_Z(in.b[0]);
	__cpu_mos6502_update_P_N(in.b[0]);

	__cpu_mos6502_write_back_data(in.b[0]);
}

/* ROR : Rotate right through carry */
static void inline __cpu_mos6502_execute_ror(void)
{
	union {
		uint16_t w;
		uint8_t b[2];
	} in = { .w = 0, };

	in.b[1] = __cpu_mos6502_fetch_data();
	in.w = in.w >> 1;
	in.b[1] |= reg_P.C << 7;

	reg_P.C = !!(in.b[0]);

	__cpu_mos6502_update_P_Z(in.b[1]);
	__cpu_mos6502_update_P_N(in.b[1]);

	__cpu_mos6502_write_back_data(in.b[1]);
}

/* RTI : Return from interrupt */
static void inline __cpu_mos6502_execute_rti(void)
{
	reg_P.raw = __cpu_mos6502_pull_byte();
	reg_PC = __cpu_mos6502_pull_word();

	irq_state = E_CPU_MOS6502_IRQ_STATE_EXIT;
}

/* RTS : Return from subroutine */
static void inline __cpu_mos6502_execute_rts(void)
{
	reg_PC = __cpu_mos6502_pull_word();
	reg_PC++;
}

/* SBC : Subtract with borrow from A */
static void inline __cpu_mos6502_execute_sbc(void)
{
	if (!reg_P.D) {
		uint8_t in_b = ~__cpu_mos6502_fetch_data();
		union {
			uint16_t w;
			uint8_t b[2];
		} out;

		out.w = (uint16_t)reg_A + (uint16_t)in_b + (uint16_t)(reg_P.C);

		__cpu_mos6502_update_P_N(out.b[0]);
		__cpu_mos6502_update_P_V(out.w, reg_A, in_b);
		__cpu_mos6502_update_P_Z(out.b[0]);
		__cpu_mos6502_update_P_C(out.w);

		reg_A = out.b[0];
	} else {
		union {
			uint8_t b;
			struct {
				uint8_t l_4bit:4;
				uint8_t h_4bit:4;
			};
		} in, tmp, low, high;

		in.b = 0x99 - __cpu_mos6502_fetch_data();
		tmp.b = reg_A;

		low.b = tmp.l_4bit + in.l_4bit + reg_P.C;
		if (low.b > 0x09)
			low.b += 0x06;
		low.l_4bit += low.h_4bit;

		high.b = tmp.h_4bit + in.h_4bit + low.h_4bit;
		if (high.b > 0x09)
			high.b += 0x06;
		high.l_4bit += high.h_4bit;

		tmp.l_4bit = low.l_4bit;
		tmp.h_4bit = high.l_4bit;

		reg_P.C = high.h_4bit;
		__cpu_mos6502_update_P_Z(tmp.b);
		reg_P.N = 0;
		reg_P.Z = 0;

		reg_A = tmp.b;

		__cpu_mos6502_set_clk_per_instr(clk_per_instr + 1);
	}
}

/* SEC : Set carry */
static void inline __cpu_mos6502_execute_sec(void)
{
	reg_P.C = 1;
}

/* SED : Set decimal mode */
static void inline __cpu_mos6502_execute_sed(void)
{
	reg_P.D = 1;
}

/* SEI : Set IRQ disable */
static void inline __cpu_mos6502_execute_sei(void)
{
	reg_P.I = 1;
}

/* STA : Store A */
static void inline __cpu_mos6502_execute_sta(void)
{
	__cpu_mos6502_write_back_data(reg_A);
}

/* STX : Store X */
static void inline __cpu_mos6502_execute_stx(void)
{
	__cpu_mos6502_write_back_data(reg_X);
}

/* STY : Store Y */
static void inline __cpu_mos6502_execute_sty(void)
{
	__cpu_mos6502_write_back_data(reg_Y);
}

/* TAX : Transfer A to X */
static void inline __cpu_mos6502_execute_tax(void)
{
	reg_X = reg_A;

	__cpu_mos6502_update_P_Z(reg_X);
	__cpu_mos6502_update_P_N(reg_X);
}

/* TAY : Transfer A to Y */
static void inline __cpu_mos6502_execute_tay(void)
{
	reg_Y = reg_A;

	__cpu_mos6502_update_P_Z(reg_Y);
	__cpu_mos6502_update_P_N(reg_Y);
}

/* TSX : Transfer S to X */
static void inline __cpu_mos6502_execute_tsx(void)
{
	reg_X = reg_S;

	__cpu_mos6502_update_P_Z(reg_X);
	__cpu_mos6502_update_P_N(reg_X);
}

/* TXA : Transfer X to A */
static void inline __cpu_mos6502_execute_txa(void)
{
	reg_A = reg_X;

	__cpu_mos6502_update_P_Z(reg_A);
	__cpu_mos6502_update_P_N(reg_A);
}

/* TXS : Transfer X to S */
static void inline __cpu_mos6502_execute_txs(void)
{
	reg_S = reg_X;
}

/* TYA : Transfer Y to A */
static void inline __cpu_mos6502_execute_tya(void)
{
	reg_A = reg_Y;

	__cpu_mos6502_update_P_Z(reg_A);
	__cpu_mos6502_update_P_N(reg_A);
}

static void inline __cpu_mos6502_nmi(void)
{
	__cpu_mos6502_push_word(reg_PC);
	__cpu_mos6502_push_byte(reg_P.raw);
	reg_P.I = 1;
	reg_PC = (uint16_t)cpu_mos6502_read_io_mem(CPU_MOS6502_NMI_VECTOR_LOW);
	reg_PC |= (uint16_t)cpu_mos6502_read_io_mem(CPU_MOS6502_NMI_VECTOR_HIGH) << 8;

	irq_state = E_CPU_MOS6502_IRQ_STATE_IN;
	if (E_CPU_MOS6502_IRQ_PIN_H_L_H == nmi_pin)
		nmi_pin = E_CPU_MOS6502_IRQ_PIN_H;
}

void cpu_mos6502_change_nmi_pin(enum cpu_mos6502_irq_pin_t state)
{
	nmi_pin = state;
}

void cpu_mos6502_reset(void)
{
	reg_A = 0;
	reg_X = 0;
	reg_Y = 0;
	reg_S = 0xFD;
	reg_P.U = 1;
	reg_PC = (uint16_t)cpu_mos6502_read_io_mem(
			CPU_MOS6502_RES_VECTOR_LOW);
	reg_PC |= (uint16_t)cpu_mos6502_read_io_mem(
			CPU_MOS6502_RES_VECTOR_HIGH) << 8;

	irq_state = E_CPU_MOS6502_IRQ_STATE_IN;
}

static void inline __cpu_mos6502_irq(void)
{
	__cpu_mos6502_push_word(reg_PC);
	__cpu_mos6502_push_byte(reg_P.raw);
	reg_P.I = 1;
	reg_PC = (uint16_t)cpu_mos6502_read_io_mem(
			CPU_MOS6502_IRQ_VECTOR_LOW);
	reg_PC |= (uint16_t)cpu_mos6502_read_io_mem(
			CPU_MOS6502_IRQ_VECTOR_HIGH) << 8;

	irq_state = E_CPU_MOS6502_IRQ_STATE_IN;
	if (E_CPU_MOS6502_IRQ_PIN_H_L_H == nmi_pin)
		nmi_pin = E_CPU_MOS6502_IRQ_PIN_H;
}

void cpu_mos6502_change_irq_pin(enum cpu_mos6502_irq_pin_t state)
{
	irq_pin = state;
}

/* main executions */
static void inline __cpu_mos6502_execute(void)
{
	uint8_t opcode = cpu_mos6502_read_io_mem(reg_PC++);

	fetch_from_reg_A = false;
	/* FIXME: why? */
	reg_P.U = 1;

	switch (opcode) {
	case 0x00:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_brk();
		__cpu_mos6502_set_clk_per_instr(7);
		break;
	case 0x01:
		__cpu_mos6502_access_indirect_x();
		__cpu_mos6502_execute_ora();
		break;
	case 0x05:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_ora();
		break;
	case 0x06:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_asl();
		__cpu_mos6502_set_clk_per_instr(5);
		break;
	case 0x08:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_php();
		__cpu_mos6502_set_clk_per_instr(3);
		break;
	case 0x09:
		__cpu_mos6502_access_immediate();
		__cpu_mos6502_execute_ora();
		break;
	case 0x0A:
		__cpu_mos6502_access_accumulator();
		__cpu_mos6502_execute_asl();
		break;
	case 0x0D:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_ora();
		break;
	case 0x0E:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_asl();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0x10:
		__cpu_mos6502_access_relative();
		__cpu_mos6502_execute_bpl();
		break;
	case 0x11:
		__cpu_mos6502_access_indirect_y();
		__cpu_mos6502_execute_ora();
		break;
	case 0x15:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_ora();
		break;
	case 0x16:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_asl();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0x18:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_clc();
		break;
	case 0x19:
		__cpu_mos6502_access_absolute_y();
		__cpu_mos6502_execute_ora();
		break;
	case 0x1D:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_ora();
		break;
	case 0x1E:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_asl();
		__cpu_mos6502_set_clk_per_instr(7);
		break;
	case 0x20:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_jsr();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0x21:
		__cpu_mos6502_access_indirect_x();
		__cpu_mos6502_execute_and();
		break;
	case 0x24:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_bit();
		break;
	case 0x25:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_and();
		break;
	case 0x26:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_rol();
		__cpu_mos6502_set_clk_per_instr(5);
		break;
	case 0x28:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_plp();
		__cpu_mos6502_set_clk_per_instr(4);
		break;
	case 0x29:
		__cpu_mos6502_access_immediate();
		__cpu_mos6502_execute_and();
		break;
	case 0x2A:
		__cpu_mos6502_access_accumulator();
		__cpu_mos6502_execute_rol();
		break;
	case 0x2C:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_bit();
		break;
	case 0x2D:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_and();
		break;
	case 0x2E:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_rol();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0x30:
		__cpu_mos6502_access_relative();
		__cpu_mos6502_execute_bmi();
		break;
	case 0x31:
		__cpu_mos6502_access_indirect_y();
		__cpu_mos6502_execute_and();
		break;
	case 0x35:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_and();
		break;
	case 0x36:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_rol();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0x38:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_sec();
		break;
	case 0x39:
		__cpu_mos6502_access_absolute_y();
		__cpu_mos6502_execute_and();
		break;
	case 0x3D:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_and();
		break;
	case 0x3E:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_rol();
		__cpu_mos6502_set_clk_per_instr(7);
		break;
	case 0x40:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_rti();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0x41:
		__cpu_mos6502_access_indirect_x();
		__cpu_mos6502_execute_eor();
		break;
	case 0x45:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_eor();
		break;
	case 0x46:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_lsr();
		__cpu_mos6502_set_clk_per_instr(5);
		break;
	case 0x48:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_pha();
		__cpu_mos6502_set_clk_per_instr(3);
		break;
	case 0x49:
		__cpu_mos6502_access_immediate();
		__cpu_mos6502_execute_eor();
		break;
	case 0x4A:
		__cpu_mos6502_access_accumulator();
		__cpu_mos6502_execute_lsr();
		break;
	case 0x4C:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_jmp();
		__cpu_mos6502_set_clk_per_instr(3);
		break;
	case 0x4D:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_eor();
		break;
	case 0x4E:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_lsr();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0x50:
		__cpu_mos6502_access_relative();
		__cpu_mos6502_execute_bvc();
		break;
	case 0x51:
		__cpu_mos6502_access_indirect_y();
		__cpu_mos6502_execute_eor();
		break;
	case 0x55:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_eor();
		break;
	case 0x56:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_lsr();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0x58:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_cli();
		break;
	case 0x59:
		__cpu_mos6502_access_absolute_y();
		__cpu_mos6502_execute_eor();
		break;
	case 0x5D:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_eor();
		break;
	case 0x5E:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_lsr();
		__cpu_mos6502_set_clk_per_instr(7);
		break;
	case 0x60:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_rts();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0x61:
		__cpu_mos6502_access_indirect_x();
		__cpu_mos6502_execute_adc();
		break;
	case 0x65:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_adc();
		break;
	case 0x66:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_ror();
		__cpu_mos6502_set_clk_per_instr(5);
		break;
	case 0x68:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_pla();
		__cpu_mos6502_set_clk_per_instr(4);
		break;
	case 0x69:
		__cpu_mos6502_access_immediate();
		__cpu_mos6502_execute_adc();
		break;
	case 0x6A:
		__cpu_mos6502_access_accumulator();
		__cpu_mos6502_execute_ror();
		break;
	case 0x6C:
		__cpu_mos6502_access_indirect();
		__cpu_mos6502_execute_jmp();
		break;
	case 0x6D:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_adc();
		break;
	case 0x6E:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_ror();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0x70:
		__cpu_mos6502_access_relative();
		__cpu_mos6502_execute_bvs();
		break;
	case 0x71:
		__cpu_mos6502_access_indirect_y();
		__cpu_mos6502_execute_adc();
		break;
	case 0x75:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_adc();
		break;
	case 0x76:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_ror();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0x78:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_sei();
		break;
	case 0x79:
		__cpu_mos6502_access_absolute_y();
		__cpu_mos6502_execute_adc();
		break;
	case 0x7D:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_adc();
		break;
	case 0x7E:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_ror();
		__cpu_mos6502_set_clk_per_instr(7);
		break;
	case 0x81:
		__cpu_mos6502_access_indirect_x();
		__cpu_mos6502_execute_sta();
		break;
	case 0x84:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_sty();
		break;
	case 0x85:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_sta();
		break;
	case 0x86:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_stx();
		break;
	case 0x88:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_dey();
		break;
	case 0x8A:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_txa();
		break;
	case 0x8C:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_sty();
		break;
	case 0x8D:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_sta();
		break;
	case 0x8E:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_stx();
		break;
	case 0x90:
		__cpu_mos6502_access_relative();
		__cpu_mos6502_execute_bcc();
		break;
	case 0x91:
		__cpu_mos6502_access_indirect_y();
		__cpu_mos6502_execute_sta();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0x94:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_sty();
		break;
	case 0x95:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_sta();
		break;
	case 0x96:
		__cpu_mos6502_access_zero_page_y();
		__cpu_mos6502_execute_stx();
		break;
	case 0x98:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_tya();
		break;
	case 0x99:
		__cpu_mos6502_access_absolute_y();
		__cpu_mos6502_execute_sta();
		__cpu_mos6502_set_clk_per_instr(5);
		break;
	case 0x9A:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_txs();
		break;
	case 0x9D:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_sta();
		__cpu_mos6502_set_clk_per_instr(5);
		break;
	case 0xA0:
		__cpu_mos6502_access_immediate();
		__cpu_mos6502_execute_ldy();
		break;
	case 0xA1:
		__cpu_mos6502_access_indirect_x();
		__cpu_mos6502_execute_lda();
		break;
	case 0xA2:
		__cpu_mos6502_access_immediate();
		__cpu_mos6502_execute_ldx();
		break;
	case 0xA4:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_ldy();
		break;
	case 0xA5:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_lda();
		break;
	case 0xA6:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_ldx();
		break;
	case 0xA8:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_tay();
		break;
	case 0xA9:
		__cpu_mos6502_access_immediate();
		__cpu_mos6502_execute_lda();
		break;
	case 0xAA:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_tax();
		break;
	case 0xAC:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_ldy();
		break;
	case 0xAD:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_lda();
		break;
	case 0xAE:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_ldx();
		break;
	case 0xB0:
		__cpu_mos6502_access_relative();
		__cpu_mos6502_execute_bcs();
		break;
	case 0xB1:
		__cpu_mos6502_access_indirect_y();
		__cpu_mos6502_execute_lda();
		break;
	case 0xB4:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_ldy();
		break;
	case 0xB5:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_lda();
		break;
	case 0xB6:
		__cpu_mos6502_access_zero_page_y();
		__cpu_mos6502_execute_ldx();
		break;
	case 0xB8:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_clv();
		break;
	case 0xB9:
		__cpu_mos6502_access_absolute_y();
		__cpu_mos6502_execute_lda();
		break;
	case 0xBA:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_tsx();
		break;
	case 0xBC:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_ldy();
		break;
	case 0xBD:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_lda();
		break;
	case 0xBE:
		__cpu_mos6502_access_absolute_y();
		__cpu_mos6502_execute_ldx();
		break;
	case 0xC0:
		__cpu_mos6502_access_immediate();
		__cpu_mos6502_execute_cpy();
		break;
	case 0xC1:
		__cpu_mos6502_access_indirect_x();
		__cpu_mos6502_execute_cmp();
		break;
	case 0xC4:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_cpy();
		break;
	case 0xC5:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_cmp();
		break;
	case 0xC6:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_dec();
		__cpu_mos6502_set_clk_per_instr(5);
		break;
	case 0xC8:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_iny();
		break;
	case 0xC9:
		__cpu_mos6502_access_immediate();
		__cpu_mos6502_execute_cmp();
		break;
	case 0xCA:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_dex();
		break;
	case 0xCC:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_cpy();
		break;
	case 0xCD:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_cmp();
		break;
	case 0xCE:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_dec();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0xD0:
		__cpu_mos6502_access_relative();
		__cpu_mos6502_execute_bne();
		break;
	case 0xD1:
		__cpu_mos6502_access_indirect_y();
		__cpu_mos6502_execute_cmp();
		break;
	case 0xD5:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_cmp();
		break;
	case 0xD6:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_dec();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0xD8:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_cld();
		break;
	case 0xD9:
		__cpu_mos6502_access_absolute_y();
		__cpu_mos6502_execute_cmp();
		break;
	case 0xDD:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_cmp();
		break;
	case 0xDE:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_dec();
		__cpu_mos6502_set_clk_per_instr(7);
		break;
	case 0xE0:
		__cpu_mos6502_access_immediate();
		__cpu_mos6502_execute_cpx();
		break;
	case 0xE1:
		__cpu_mos6502_access_indirect_x();
		__cpu_mos6502_execute_sbc();
		break;
	case 0xE4:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_cpx();
		break;
	case 0xE5:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_sbc();
		break;
	case 0xE6:
		__cpu_mos6502_access_zero_page();
		__cpu_mos6502_execute_inc();
		__cpu_mos6502_set_clk_per_instr(5);
		break;
	case 0xE8:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_inx();
		break;
	case 0xE9:
		__cpu_mos6502_access_immediate();
		__cpu_mos6502_execute_sbc();
		break;
	case 0xEA:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_nop();
		break;
	case 0xEC:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_cpx();
		break;
	case 0xED:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_sbc();
		break;
	case 0xEE:
		__cpu_mos6502_access_absolute();
		__cpu_mos6502_execute_inc();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0xF0:
		__cpu_mos6502_access_relative();
		__cpu_mos6502_execute_beq();
		break;
	case 0xF1:
		__cpu_mos6502_access_indirect_y();
		__cpu_mos6502_execute_sbc();
		break;
	case 0xF5:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_sbc();
		break;
	case 0xF6:
		__cpu_mos6502_access_zero_page_x();
		__cpu_mos6502_execute_inc();
		__cpu_mos6502_set_clk_per_instr(6);
		break;
	case 0xF8:
		__cpu_mos6502_access_none();
		__cpu_mos6502_execute_sed();
		break;
	case 0xF9:
		__cpu_mos6502_access_absolute_y();
		__cpu_mos6502_execute_sbc();
		break;
	case 0xFD:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_sbc();
		break;
	case 0xFE:
		__cpu_mos6502_access_absolute_x();
		__cpu_mos6502_execute_inc();
		__cpu_mos6502_set_clk_per_instr(7);
		break;
	}
}

void cpu_mos6502_exec(unsigned long tickcount)
{
#ifdef CFG_CPU_MOS6502_USE_TIMING
	clk_goal += tickcount;

	while (clk_goal > 0) {
#else
	while (tickcount--) {
#endif

		__cpu_mos6502_execute();

#ifdef CFG_CPU_MOS6502_USE_TIMING
		clk_goal -= clk_per_instr;
		cnt_instr++;
#endif

		if (E_CPU_MOS6502_IRQ_STATE_IN == irq_state) {
			continue;
		} else if (E_CPU_MOS6502_IRQ_STATE_EXIT == irq_state) {
			irq_state = E_CPU_MOS6502_IRQ_STATE_OUT;
			continue;
		}

		if (E_CPU_MOS6502_IRQ_PIN_H != nmi_pin)
			__cpu_mos6502_nmi();

		if ((E_CPU_MOS6502_IRQ_PIN_H != irq_pin) && !reg_P.I)
			__cpu_mos6502_irq();
	}
}

static void *__6502_regs[] = {
	[E_CPU_MOS6502_REG_A] = &reg_A,
	[E_CPU_MOS6502_REG_Y] = &reg_Y,
	[E_CPU_MOS6502_REG_X] = &reg_X,
	[E_CPU_MOS6502_REG_PC] = &reg_PC,
	[E_CPU_MOS6502_REG_S] = &reg_S,
	[E_CPU_MOS6502_REG_P] = &reg_P,
};

void cpu_mos6502_set_core_reg(enum cpu_mos6502_reg_t r_idx, uint16_t val)
{
	if (E_CPU_MOS6502_REG_PC == r_idx)
		*(uint16_t *)__6502_regs[r_idx] = val;
	else
		*(uint8_t *)__6502_regs[r_idx] = val;
}

uint16_t cpu_mos6502_get_core_reg(enum cpu_mos6502_reg_t r_idx)
{
	if (E_CPU_MOS6502_REG_PC == r_idx)
		return *(uint16_t *)__6502_regs[r_idx];
	else
		return *(uint8_t *)__6502_regs[r_idx];
}
