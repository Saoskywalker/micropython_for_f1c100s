/*
 * sys-clock.c
 *
 * Copyright(c) 2007-2018 Jianjun Jiang <8192542@qq.com>
 * Official site: http://xboot.org
 * Mobile phone: +86-18665388956
 * QQ: 8192542
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

//#include <xboot.h>
#include <stdint.h>
#include <reg-ccu.h>
#include <io.h>
#include "delay.h"

static inline void sdelay(int loops)
{
	__asm__ __volatile__ ("1:\n" "subs %0, %1, #1\n"
		"bne 1b":"=r" (loops):"0"(loops));
}

static void wait_pll_stable(uint32_t base)
{
	uint32_t rval = 0;
	uint32_t time = 0xfff;

	do {
		rval = read32(base);
		time--;
	} while(time && !(rval & (1 << 28)));
}

static void clock_set_pll_cpu(uint32_t clk)
{
	uint32_t n, k, m, p;
	uint32_t rval = 0;
	uint32_t div = 0;

	if(clk > 720000000)
		clk = 720000000;

	if((clk % 24000000) == 0)
	{
		div = clk / 24000000;
		n = div - 1;
		k = 0;
		m = 0;
		p = 0;
	}
	else if((clk % 12000000) == 0)
	{
		m = 1;
		div = clk / 12000000;
		if((div % 3) == 0)
			k = 2;
		else if((div % 4) == 0)
			k = 3;
		else
			k = 1;
		n = (div / (k + 1)) - 1;
		p = 0;
	}
	else
	{
		div = clk / 24000000;
		n = div - 1;
		k = 0;
		m = 0;
		p = 0;
	}

	rval = read32(F1C100S_CCU_BASE + CCU_PLL_CPU_CTRL);
	rval &= ~((0x3 << 16) | (0x1f << 8) | (0x3 << 4) | (0x3 << 0));
	rval |= (1U << 31) | (p << 16) | (n << 8) | (k << 4) | m;
	write32(F1C100S_CCU_BASE + CCU_PLL_CPU_CTRL, rval);
	wait_pll_stable(F1C100S_CCU_BASE + CCU_PLL_CPU_CTRL);
}

void sys_clock_init(void)
{
	uint32_t val;

	write32(F1C100S_CCU_BASE + CCU_PLL_STABLE_TIME0, 0x1ff);
	write32(F1C100S_CCU_BASE + CCU_PLL_STABLE_TIME1, 0x1ff);

	val = read32(F1C100S_CCU_BASE + CCU_CPU_CFG);
	val &= ~(0x3 << 16);
	val |= (0x1 << 16);
	write32(F1C100S_CCU_BASE + CCU_CPU_CFG, val);
	sdelay(100);

	//write32(F1C100S_CCU_BASE + CCU_PLL_VIDEO_CTRL, 0x81004107); //TIME RATE: 24000000*(0x41+1)/(0x07+1)=198Mhz
	write32(F1C100S_CCU_BASE + CCU_PLL_VIDEO_CTRL, 0x81004103); //TIME RATE: 24000000*(0x41+1)/(0x03+1)=396Mhz
	sdelay(100);
	write32(F1C100S_CCU_BASE + CCU_PLL_PERIPH_CTRL, 0x80041800);
	sdelay(100);
	write32(F1C100S_CCU_BASE + CCU_AHB_APB_CFG, 0x00003180);
	sdelay(100);

	val = read32(F1C100S_CCU_BASE + CCU_DRAM_CLK_GATE);
	val |= (0x1 << 26) | (0x1 << 24);
	write32(F1C100S_CCU_BASE + CCU_DRAM_CLK_GATE, val);
	sdelay(100);

	//clock_set_pll_cpu(408000000); //NORMALL STATE
	clock_set_pll_cpu(600000000); //overlock state, this function limit 720000000, stable run in 600000000
	val = read32(F1C100S_CCU_BASE + CCU_CPU_CFG);
	val &= ~(0x3 << 16);
	val |= (0x2 << 16);
	write32(F1C100S_CCU_BASE + CCU_CPU_CFG, val);
	sdelay(100);
}

unsigned char clk_enable(unsigned char clk_num)
{
	uint32_t val = 0;

	switch (clk_num)
	{
		case SD0_GATING:
			val = read32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE0);
			val |= 1<<8;
			write32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE0, val);
			return 1;
			break;
		
		case UART0_GATING:
			val = read32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE2);
			val |= 1<<20;
			write32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE2, val);
			return 1;
			break;

		case UART1_GATING:
			val = read32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE2);
			val |= 1<<21;
			write32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE2, val);
			return 1;
			break;

		case UART2_GATING:
			val = read32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE2);
			val |= 1<<22;
			write32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE2, val);
			return 1;
			break;

		default:
			return 0;
			break;
	}
}

unsigned char clk_disable(unsigned char clk_num)
{
	uint32_t val = 0;

	switch (clk_num)
	{
		case SD0_GATING:
			val = read32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE0);
			val &= ~(1<<8);
			write32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE0, val);
			return 1;
			break;

		case UART0_GATING:
			val = read32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE2);
			val &= ~(1<<20);
			write32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE2, val);
			return 1;
			break;

		case UART1_GATING:
			val = read32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE2);
			val &= ~(1<<21);
			write32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE2, val);
			return 1;
			break;

		case UART2_GATING:
			val = read32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE2);
			val &= ~(1<<22);
			write32(F1C100S_CCU_BASE + CCU_BUS_CLK_GATE2, val);
			return 1;
			break;
		
		default:
			return 0;
			break;
	}
}

/*打开外设时钟*/
void Open_Dev_Clock(int Dev)
{
	//使能DMA时钟
	S_BIT((F1C100S_CCU_BASE + CCU_BUS_CLK_GATE0+(((Dev>>8)&0x3)*0x4)),(Dev&0x1f));
	//使能DMA复位
	C_BIT((F1C100S_CCU_BASE + CCU_BUS_SOFT_RST0+(((Dev>>8)&0x3)*0x4)),(Dev&0x1f));
	delay_us(100);	
	S_BIT((F1C100S_CCU_BASE + CCU_BUS_SOFT_RST0+(((Dev>>8)&0x3)*0x4)),(Dev&0x1f));
	delay_us(100);
}
