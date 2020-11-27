/*
 * driver/fb-f1c100s.c
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
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <dma.h>
#include <clk-f1c100s-pll.h>
#include <reset-f1c100s.h>
#include <gpio-f1c100s.h>
#include <pwm-f1c100s.h>
#include <framebuffer.h>
#include <f1c100s-gpio.h>
#include <reg-tcon.h>
#include <reg-defe.h>
#include <reg-debe.h>
#include <reg-ccu.h>
#include <io.h>
#include "delay.h"
#include "irq_misc.h"
#include <stdio.h>

#define phys_to_virt(x) (x)

typedef struct
{
	virtual_addr_t virtdefe;
	virtual_addr_t virtdebe;
	virtual_addr_t virttcon;

	char * clkdefe;
	char * clkdebe;
	char * clktcon;
	int32_t rstdefe;
	int32_t rstdebe;
	int32_t rsttcon;
	int32_t width;
	int32_t height;
	int32_t pwidth;
	int32_t pheight;
	int32_t bits_per_pixel;
	int32_t bytes_per_pixel;
	int32_t index;
	void * vram[2];
	char swapBR; /* color BR swap */
	char layer0_to_video;
	void * cleanCacheAddr;

	struct
	{
		int32_t pixel_clock_hz;
		int32_t h_front_porch;
		int32_t h_back_porch;
		int32_t h_sync_len;
		int32_t v_front_porch;
		int32_t v_back_porch;
		int32_t v_sync_len;
		int32_t h_sync_active;
		int32_t v_sync_active;
		int32_t den_active;
		int32_t clk_active;
		int32_t DE_HV_mode;
	} timing;

	pwm_t * backlight;
	int32_t brightness;
}fb_f1c100s_pdata_t;

static pwm_t led_pwm_bl =
        {
            .virt = 0x01c21000,
            .duty = 0,
            .period = 1000000,
            .channel = 0,
            .polarity = TRUE,
            .pwm_port = &GPIO_PE,
            .pwm_pin = 12,
            .pwm_pin_cfg = 4,
        };

fb_f1c100s_pdata_t fb_f1c100s_pData;

reset_f1c100s_t reset_1 = 
{
	.virt = 0x01c202c0,
	.base = 0,
	.nreset = 32,
};
reset_f1c100s_t reset_2 = 
{
	.virt = 0x01c202c4,
	.base = 32,
	.nreset = 32,
};
reset_f1c100s_t reset_3 = 
{
	.virt = 0x01c202d0,
	.base = 64,
	.nreset = 32,
};

static clk_pll_t clk_pll_video = 
{
	.virt = F1C100S_CCU_BASE,
	.parent = NULL,
	.channel = 2,
	.prate = 24000000, /* hosc */
};

render_t *render = 0;
framebuffer_t fb_f1c100s;

static char writeBackFlag = 0;
void *writeBackRAM = 0;

void fb_f1c100s_write_back_start(void)
{
	writeBackFlag = 1;
}

void fb_f1c100s_write_back(void * data)
{
	virtual_addr_t debe = phys_to_virt(F1C100S_DEBE_BASE);
	
	write32(debe+0X8F4, (uint32_t)data<<3);
	write32(debe+0X8F8, (uint32_t)data>>29);
	write32(debe+0X8F8, fb_f1c100s.width<<5);
	write32(debe+0X8F0, 0);
	S_BIT(debe+0X8F0, 0); //start,,,auto stop (set 0) in the end
}

uint8_t fb_f1c100s_write_back_state(void)
{
	virtual_addr_t debe = phys_to_virt(F1C100S_DEBE_BASE);
	uint32_t val, time_out = 0;
	uint8_t res = 0;
	
	val = read32(debe+0X8F0);
	while ((val&(~(1<<8))))
	{
		delay_ms(1);
		printf("2\r\n");
		val = read32(debe+0X8F0);
		if((val&(~(1<<9))))
		{
			res = 1;  //write back error
			break;
		}
		if(++time_out>=100) //100ms
		{
			res = 2; //time out
			break;
		}
	}
	return res; //success = 0
}

inline static void f1c100s_debe_set_mode(fb_f1c100s_pdata_t * pdat)
{
	struct f1c100s_debe_reg_t * debe = (struct f1c100s_debe_reg_t *)(pdat->virtdebe);
	uint32_t val;

	val = read32((virtual_addr_t)&debe->mode); //enable debe
	val |= (1 << 0);
	write32((virtual_addr_t)&debe->mode, val);

	//set layer0
	write32((virtual_addr_t)&debe->disp_size, (((pdat->height) - 1) << 16) | (((pdat->width) - 1) << 0));
	write32((virtual_addr_t)&debe->layer0_size, (((pdat->height) - 1) << 16) | (((pdat->width) - 1) << 0));
	write32((virtual_addr_t)&debe->layer0_stride, ((pdat->width) << 5));
	write32((virtual_addr_t)&debe->layer0_addr_low32b, (uint32_t)(pdat->vram[(pdat->index + 1) & 0x1]) << 3);
	write32((virtual_addr_t)&debe->layer0_addr_high4b, (uint32_t)(pdat->vram[(pdat->index + 1) & 0x1]) >> 29);
	if (pdat->swapBR)
		write32((virtual_addr_t)&debe->layer0_attr1_ctrl, 0x0A << 8|1<<2); //Note: (0x0A<<8)=ARGB (1<<2 )=BR swap 
	else
		write32((virtual_addr_t)&debe->layer0_attr1_ctrl, 0x0A << 8);
	if (pdat->layer0_to_video)
		S_BIT((virtual_addr_t)&debe->layer0_attr0_ctrl, 1); //enable video
	else
		C_BIT((virtual_addr_t)&debe->layer0_attr0_ctrl, 1); //disable video

	//set layer1
	write32((virtual_addr_t)&debe->disp_size, (((pdat->height) - 1) << 16) | (((pdat->width) - 1) << 0));
	write32((virtual_addr_t)&debe->layer1_size, (((pdat->height) - 1) << 16) | (((pdat->width) - 1) << 0));
	write32((virtual_addr_t)&debe->layer1_stride, ((pdat->width) <<5));
	write32((virtual_addr_t)&debe->layer1_addr_low32b, (u32)(pdat->vram[pdat->index]) << 3);
	write32((virtual_addr_t)&debe->layer1_addr_high4b, (u32)(pdat->vram[pdat->index]) >> 29);
	if (pdat->swapBR)
		write32((virtual_addr_t)&debe->layer1_attr1_ctrl, 0x0A << 8|1<<2); //Note: (0x0A<<8)=ARGB (1<<2 )=BR swap 
	else
		write32((virtual_addr_t)&debe->layer1_attr1_ctrl, 0x0A << 8);
	S_BIT((virtual_addr_t)&debe->layer1_attr0_ctrl,10);//priority set
  	S_BIT((virtual_addr_t)&debe->layer1_attr0_ctrl,15);//1: select Pipe 1
	S_BIT((virtual_addr_t)&debe->mode,9);//layer1 enable

	//layer0 enable
	val = read32((virtual_addr_t)&debe->mode);
	val |= (1 << 8);
	write32((virtual_addr_t)&debe->mode, val);
	//load
	val = read32((virtual_addr_t)&debe->reg_ctrl);
	val |= (1 << 0);
	write32((virtual_addr_t)&debe->reg_ctrl, val);
	//debe start
	val = read32((virtual_addr_t)&debe->mode);
	val |= (1 << 1);
	write32((virtual_addr_t)&debe->mode, val);
}

inline static void f1c100s_debe_set_address(fb_f1c100s_pdata_t * pdat, void * vram)
{
	struct f1c100s_debe_reg_t * debe = (struct f1c100s_debe_reg_t *)(pdat->virtdebe);

	write32((virtual_addr_t)&debe->layer0_addr_low32b, (uint32_t)vram << 3);
	write32((virtual_addr_t)&debe->layer0_addr_high4b, (uint32_t)vram >> 29);
}

inline static void f1c100s_tcon_enable(fb_f1c100s_pdata_t * pdat)
{
	struct f1c100s_tcon_reg_t * tcon = (struct f1c100s_tcon_reg_t *)pdat->virttcon;
	uint32_t val;

	val = read32((virtual_addr_t)&tcon->ctrl);
	val |= (1 << 31);
	write32((virtual_addr_t)&tcon->ctrl, val);
}

inline static void f1c100s_tcon_disable(fb_f1c100s_pdata_t * pdat)
{
	struct f1c100s_tcon_reg_t * tcon = (struct f1c100s_tcon_reg_t *)pdat->virttcon;
	uint32_t val;

	write32((virtual_addr_t)&tcon->ctrl, 0);
	write32((virtual_addr_t)&tcon->int0, 0);

	val = read32((virtual_addr_t)&tcon->tcon0_dclk);
	val &= ~(0xf << 28);
	write32((virtual_addr_t)&tcon->tcon0_dclk, val);

	write32((virtual_addr_t)&tcon->tcon0_io_tristate, 0xffffffff);
	write32((virtual_addr_t)&tcon->tcon1_io_tristate, 0xffffffff);
}

inline static uint64_t f1c100s_tcon_clk_get_rate()
{
	return clk_f1c100s_pll_get_rate(&clk_pll_video, clk_pll_video.prate);
}

inline static void f1c100s_clk_pll_video_set_enable(bool enable)
{
	clk_f1c100s_pll_set_enable(&clk_pll_video, enable);
}

static void clk_mux_set_parent(virtual_addr_t virt, uint32_t width, uint32_t shift, uint32_t parent_val)
{
	uint32_t val;
	val = read32(virt);
	val &= ~(((1 << width) - 1) << shift);
	val |= parent_val << shift;
	write32(virt, val);
}

#define gate_clk_enable(virt, shift, invert) write32(virt, (read32(virt) & ~(0x1 << shift)) | ((invert ? 0x0 : 0x1) << shift))
#define gate_clk_disable(virt, shift, invert) write32(virt, (read32(virt) & ~(0x1 << shift)) | ((invert ? 0x1 : 0x0) << shift))

static void f1c100s_clk_tcon_init(void)
{
	clk_mux_set_parent(0x01c20118, 3, 24, 0);
}

inline static void f1c100s_clk_tcon_enable()
{
	//gate-tcon
	// virtual_addr_t virt = 0x01c20118;
	// uint32_t shift = 31;
	// bool invert = false;
	// write32(virt, (read32(virt) & ~(0x1 << shift)) | ((invert ? 0x0 : 0x1) << shift));
	gate_clk_enable(0x01c20118, 31, false);
	//gate-bus-tcon
	// virt = 0x01c20064;
	// shift = 4;
	// invert = false;
	// write32(virt, (read32(virt) & ~(0x1 << shift)) | ((invert ? 0x0 : 0x1) << shift));
	gate_clk_enable(0x01c20064, 4, false);
}

inline static void f1c100s_clk_tcon_disable()
{
	gate_clk_disable(0x01c20118, 31, false);
	gate_clk_disable(0x01c20064, 4, false);
}

static void clk_divider_set_rate(virtual_addr_t virt, uint32_t width, uint32_t shift, bool_t onebased, uint64_t prate, uint64_t rate)
{
	uint32_t mask = ((1 << (width)) - 1);
	uint32_t div;
	uint32_t val;

	if(rate == 0)
		rate = prate;

	div = prate / rate;
	if(onebased)
		div--;
	if(div > mask)
		div = mask;

	val = read32(virt);
	val &= ~(mask << shift);
	val |= div << shift;
	write32(virt, val);
}

static void f1c100s_clk_defe_init(void)
{
	clk_mux_set_parent(0x01c2010c, 3, 24, 0);
	clk_divider_set_rate(0x01c2010c, 4, 0, TRUE, f1c100s_tcon_clk_get_rate(), 198000000);
	//clk_divider_set_rate(0x01c2010c, 4, 0, TRUE, 24000000, 198000000);
}
inline static void f1c100s_clk_defe_enable()
{
	gate_clk_enable(0x01c2010c, 31, false);
	gate_clk_enable(0x01c20064, 14, false);
}

inline static void f1c100s_clk_defe_disable()
{
	gate_clk_disable(0x01c2010c, 31, false);
	gate_clk_disable(0x01c20064, 14, false);
}
	// "clk-mux@0x01c2010c": {
	// 	"parent": [
	// 		{ "name": "pll-video", "value": 0 },
	// 		{ "name": "pll-periph", "value": 2 }
	// 	],
	// 	"name": "mux-defe", "shift": 24, "width": 3,
	// 	"default": { "parent": "pll-video" }
	// },
	// "clk-divider@0x01c2010c": { "parent": "mux-defe", "name": "div-defe", "shift": 0, "width": 4, "default": { "rate": 198000000 } },
	// "clk-gate@0x01c2010c": {"parent": "div-defe", "name": "gate-defe", "shift": 31, "invert": false },
	// "clk-gate@0x01c20064": {"parent": "gate-defe", "name": "gate-bus-defe", "shift": 14, "invert": false },
	// "clk-link": { "parent": "gate-bus-defe", "name": "link-defe" },
	
static void f1c100s_clk_debe_init(void)
{
	clk_mux_set_parent(0x01c20104, 3, 24, 0);
	clk_divider_set_rate(0x01c20104, 4, 0, TRUE, f1c100s_tcon_clk_get_rate(), 198000000);
	//clk_divider_set_rate(0x01c20104, 4, 0, TRUE, 24000000, 198000000);
}

// "clk-mux@0x01c20104": {
// 	"parent": [
// 		{ "name": "pll-video", "value": 0 },
// 		{ "name": "pll-periph", "value": 2 }
// 	],
// 	"name": "mux-debe", "shift": 24, "width": 3,
// 	"default": { "parent": "pll-video" }
// },
// "clk-divider@0x01c20104": { "parent": "mux-debe", "name": "div-debe", "shift": 0, "width": 4, "divider-one-based": TRUE, "default": { "rate": 198000000 } },
// "clk-gate@0x01c20104": {"parent": "div-debe", "name": "gate-debe", "shift": 31, "invert": false },
// "clk-gate@0x01c20064": {"parent": "gate-debe", "name": "gate-bus-debe", "shift": 12, "invert": false },
inline static void f1c100s_clk_debe_enable()
{
	gate_clk_enable(0x01c20104, 31, false);
	gate_clk_enable(0x01c20064, 12, false);
}

inline static void f1c100s_clk_debe_disable()
{
	gate_clk_disable(0x01c20104, 31, false);
	gate_clk_disable(0x01c20064, 12, false);
}

inline static void f1c100s_tcon_set_mode(fb_f1c100s_pdata_t * pdat)
{
	struct f1c100s_tcon_reg_t * tcon = (struct f1c100s_tcon_reg_t *)pdat->virttcon;
	int32_t bp, total;
	uint32_t val;

	val = read32((virtual_addr_t)&tcon->ctrl);
	val &= ~(0x1 << 0);
	write32((virtual_addr_t)&tcon->ctrl, val);

	val = (pdat->timing.v_front_porch + pdat->timing.v_back_porch + pdat->timing.v_sync_len);
	write32((virtual_addr_t)&tcon->tcon0_ctrl, (1 << 31) | ((val & 0x1f) << 4));
	//val = clk_get_rate(pdat->clktcon) / pdat->timing.pixel_clock_hz;
	val = f1c100s_tcon_clk_get_rate() / pdat->timing.pixel_clock_hz;
	write32((virtual_addr_t)&tcon->tcon0_dclk, (0xf << 28) | (val << 0)); //set refresh HZ
	write32((virtual_addr_t)&tcon->tcon0_timing_active, ((pdat->width - 1) << 16) | ((pdat->height - 1) << 0));

	bp = pdat->timing.h_sync_len + pdat->timing.h_back_porch;
	total = pdat->width + pdat->timing.h_front_porch + bp;
	write32((virtual_addr_t)&tcon->tcon0_timing_h, ((total - 1) << 16) | ((bp - 1) << 0));
	bp = pdat->timing.v_sync_len + pdat->timing.v_back_porch;
	total = pdat->height + pdat->timing.v_front_porch + bp;
	write32((virtual_addr_t)&tcon->tcon0_timing_v, ((total * 2) << 16) | ((bp - 1) << 0));
	write32((virtual_addr_t)&tcon->tcon0_timing_sync, ((pdat->timing.h_sync_len - 1) << 16) | ((pdat->timing.v_sync_len - 1) << 0));

	write32((virtual_addr_t)&tcon->tcon0_hv_intf, 0);
	write32((virtual_addr_t)&tcon->tcon0_cpu_intf, 0);

	if(pdat->bits_per_pixel == 18 || pdat->bits_per_pixel == 16)
	{
		write32((virtual_addr_t)&tcon->tcon0_frm_seed[0], 0x11111111);
		write32((virtual_addr_t)&tcon->tcon0_frm_seed[1], 0x11111111);
		write32((virtual_addr_t)&tcon->tcon0_frm_seed[2], 0x11111111);
		write32((virtual_addr_t)&tcon->tcon0_frm_seed[3], 0x11111111);
		write32((virtual_addr_t)&tcon->tcon0_frm_seed[4], 0x11111111);
		write32((virtual_addr_t)&tcon->tcon0_frm_seed[5], 0x11111111);
		write32((virtual_addr_t)&tcon->tcon0_frm_table[0], 0x01010000);
		write32((virtual_addr_t)&tcon->tcon0_frm_table[1], 0x15151111);
		write32((virtual_addr_t)&tcon->tcon0_frm_table[2], 0x57575555);
		write32((virtual_addr_t)&tcon->tcon0_frm_table[3], 0x7f7f7777);
		write32((virtual_addr_t)&tcon->tcon0_frm_ctrl, (pdat->bits_per_pixel == 18) ? ((1 << 31) | (0 << 4)) : ((1 << 31) | (5 << 4)));
	}
	//set lcd polarity
	val = (1 << 28);
	if(!pdat->timing.h_sync_active)
		val |= (1 << 25);
	if(!pdat->timing.v_sync_active)
		val |= (1 << 24);
	if(!pdat->timing.den_active)
		val |= (1 << 27);
	if(!pdat->timing.clk_active)
		val |= (1 << 26);
	write32((virtual_addr_t)&tcon->tcon0_io_polarity, val);
	write32((virtual_addr_t)&tcon->tcon0_io_tristate, 0);
}

inline static void fb_f1c100s_cfg_gpios(fb_f1c100s_pdata_t * pdat, const gpio_port_t* port, int32_t pin_start, 
										int32_t pin_end, int32_t cfg, gpio_pull_t pull, gpio_drv_t drv)
{
	int32_t pin;
	for(pin = pin_start; pin < pin_end; pin++)
	{
		gpio_f1c100s_set_cfg(port, pin, cfg);
		gpio_f1c100s_set_pull(port, pin, pull);
		gpio_f1c100s_set_drv(port, pin, drv);
	}
	if (pdat->timing.DE_HV_mode == 2) //only HV mode
	{
		gpio_f1c100s_set_cfg(port, 19, 1); //close de
		gpio_f1c100s_set_value(port, 19, 1);
	}
	else if (pdat->timing.DE_HV_mode == 1) //only DE mode
	{
		gpio_f1c100s_set_cfg(port, 20, 1); //close hv
		gpio_f1c100s_set_value(port, 20, 0);
		gpio_f1c100s_set_cfg(port, 21, 1);
		gpio_f1c100s_set_value(port, 21, 0);
	}
	else //DE and HV mode
	{
	}
}

/*cache操作 flush*/
#define CONFIG_SYS_CACHELINE_SIZE 32 /*Cache line组织宽度为32byte*/
static inline void dcache_flush_line(unsigned int addr)
{
	/* 清空并使无效数据.d cache.. */
	// __asm__ __volatile__(
	// "mcr p15,0,%0,c7,c14,1\n"
	// "mcr p15,0,%0,c7,c5,1\n"
	// :
	// : "r" (addr)
	// : "memory");
	__asm__ __volatile__(
		"mcr p15,0,%0,c7,c14,1\n"
		:
		: "r"(addr)
		: "memory");
	/*F1C100S没有写缓冲*/
}
static inline void flush_dcache_range(unsigned long start, unsigned long end)
{
	unsigned int addr;

	start = start & (~(CONFIG_SYS_CACHELINE_SIZE - 1));
	end = end & (~(CONFIG_SYS_CACHELINE_SIZE - 1));
	for (addr = start; addr <= end; addr += CONFIG_SYS_CACHELINE_SIZE)
		dcache_flush_line(addr);
}
static inline void flush_cache(unsigned long start, unsigned long size)
{
	//	sysFlushCache(I_D_CACHE);
	flush_dcache_range(start, start + size);
}

void framebuffer_flush(void)
{
	flush_cache((unsigned long)fb_f1c100s_pData.vram[fb_f1c100s_pData.index],
				(unsigned long)(fb_f1c100s_pData.width * fb_f1c100s_pData.height * fb_f1c100s_pData.bytes_per_pixel));
}

void TCON_ISR(void *arg)
{
	fb_f1c100s_pdata_t * pdat = (fb_f1c100s_pdata_t *)arg;

	write32(pdat->virttcon + 0x4, read32(pdat->virttcon + 0x4) & (~((1) << 15))); //清中断
	if(writeBackFlag)
	{
		writeBackFlag = 0;
		fb_f1c100s_write_back(writeBackRAM);
	}
	flush_cache((unsigned long)pdat->cleanCacheAddr,
				(unsigned long)(fb_f1c100s.user_width * fb_f1c100s.user_height * pdat->bytes_per_pixel));
}

void fb_f1c100s_init_hw(fb_f1c100s_pdata_t * pdat)
{
	fb_f1c100s_cfg_gpios(pdat, &GPIO_PD, 0, 22, 0x2, GPIO_PULL_NONE, GPIO_DRV_STRONG);
	f1c100s_tcon_disable(pdat);
	f1c100s_debe_set_mode(pdat);
	f1c100s_tcon_set_mode(pdat);
	request_irq(IRQ_LCD, TCON_ISR, pdat); //register irq
	write32(pdat->virttcon + 0x4, (u32_t)(1) << 31); //enable lcd frame irq
	f1c100s_tcon_enable(pdat);

	pwm_f1c100s_init(pdat->backlight);
}

void fb_f1c100s_change_hw(char BR, char Video)
{
	fb_f1c100s_pData.swapBR = BR;
	fb_f1c100s_pData.layer0_to_video = Video;

	f1c100s_tcon_disable(&fb_f1c100s_pData);
	f1c100s_clk_defe_disable();
	f1c100s_clk_debe_disable();
	f1c100s_debe_set_mode(&fb_f1c100s_pData);
	f1c100s_tcon_set_mode(&fb_f1c100s_pData);
	write32(F1C100S_TCON_BASE + 0x4, read32(F1C100S_TCON_BASE + 0x4) & (~((1) << 15))); //清中断
	write32(F1C100S_TCON_BASE + 0x4, (u32_t)(1) << 31); //enable lcd frame irq
	f1c100s_clk_defe_enable();
	f1c100s_clk_debe_enable();
	f1c100s_tcon_enable(&fb_f1c100s_pData);
}

void fb_f1c100s_flush_cache_set(void *addr)
{
	fb_f1c100s_pData.cleanCacheAddr = addr;
}

void fb_f1c100s_setbl(framebuffer_t * fb, int32_t brightness)
{
	fb_f1c100s_pdata_t * pdat = (fb_f1c100s_pdata_t *)fb->priv;
	//led_set_brightness(pdat->backlight, brightness);
	pwm_f1c100s_set_duty(pdat->backlight, brightness);
}

int32_t fb_f1c100s_getbl(framebuffer_t * fb)
{
	fb_f1c100s_pdata_t * pdat = (fb_f1c100s_pdata_t *)fb->priv;
	// return led_get_brightness(pdat->backlight);
	return pwm_f1c100s_get_duty(pdat->backlight);
}

uint32_t PixelsTotal = 0;
render_t * fb_f1c100s_create(framebuffer_t * fb)
{
	fb_f1c100s_pdata_t * pdat = (fb_f1c100s_pdata_t *)fb->priv;
	render_t * render;
	void * pixels;
	size_t pixlen;

	pixlen = pdat->width * pdat->height * pdat->bytes_per_pixel;
	// pixels = memalign(4, pixlen); //framebuffer way refresh
	// if(!pixels)
	// 	return NULL;
	pixels = pdat->vram[pdat->index]; //vram way refresh

	render = malloc(sizeof(render_t));
	if(!render)
	{
		free(pixels);
		return NULL;
	}

	render->width = pdat->width;
	render->height = pdat->height;
	render->pitch = (pdat->width * pdat->bytes_per_pixel + 0x3) & ~0x3;
	render->format = PIXEL_FORMAT_ARGB32;
	render->pixels = pixels;
	render->pixels2 = pdat->vram[(pdat->index + 1) & 0x1];
	render->pixlen = pixlen;
	render->priv = NULL;

	PixelsTotal = render->pixlen / PIXEL_LEN;

	return render;
}

void fb_f1c100s_destroy(framebuffer_t * fb, render_t * render)
{
	if(render)
	{
		dma_free_coherent(render->pixels);
		free(render);
	}
}
    
void fb_f1c100s_present(framebuffer_t * fb, render_t * render)
{
	fb_f1c100s_pdata_t * pdat = (fb_f1c100s_pdata_t *)fb->priv;

	if(render && render->pixels)
	{
		pdat->index = (pdat->index + 1) & 0x1;
		memcpy(pdat->vram[pdat->index], render->pixels, render->pixlen);
		dma_cache_sync(pdat->vram[pdat->index], render->pixlen, DMA_TO_DEVICE);
		f1c100s_debe_set_address(pdat, pdat->vram[pdat->index]);
	}
}

	// "fb-f1c100s@0": {
	// 	"clock-name-defe": "link-defe",
	// 	"clock-name-debe": "link-debe",
	// 	"clock-name-tcon": "link-tcon",
	// 	"reset-defe": 46,
	// 	"reset-debe": 44,
	// 	"reset-tcon": 36,
	// 	"width": 800,
	// 	"height": 480,
	// 	"physical-width": 216,
	// 	"physical-height": 135,
	// 	"bits-per-pixel": 18,
	// 	"bytes-per-pixel": 4,
	// 	"clock-frequency": 33000000,
	// 	"hfront-porch": 40,
	// 	"hback-porch": 87,
	// 	"hsync-len": 1,
	// 	"vfront-porch": 13,
	// 	"vback-porch": 31,
	// 	"vsync-len": 1,
	// 	"hsync-active": false,
	// 	"vsync-active": false,
	// 	"den-active": false,
	// 	"clk-active": false,
	// 	"backlight": "led-pwm-bl.0"
	// },
void fb_f1c100s_init(framebuffer_t * fb)
{
	int32_t i;
	fb_f1c100s_pdata_t * pdat = &fb_f1c100s_pData;

	pdat->virtdefe = phys_to_virt(F1C100S_DEFE_BASE);
	pdat->virtdebe = phys_to_virt(F1C100S_DEBE_BASE);
	pdat->virttcon = phys_to_virt(F1C100S_TCON_BASE);
	pdat->clkdefe = "clkdefe";
	pdat->clkdebe = "clkdebe";
	pdat->clktcon = "clktcon";
	pdat->rstdefe = 46;
	pdat->rstdebe = 44;
	pdat->rsttcon = 36;
	pdat->width = 800; //800 , set display resolution ratio
	pdat->height = 480;	//480
	pdat->pwidth = 216;
	pdat->pheight = 135;
	pdat->bits_per_pixel = 18;
	pdat->bytes_per_pixel = 4;
	pdat->index = 0;
	pdat->vram[0] = dma_alloc_coherent(pdat->width * pdat->height * pdat->bytes_per_pixel);
	pdat->vram[1] = dma_alloc_coherent(pdat->width * pdat->height * pdat->bytes_per_pixel);
	pdat->cleanCacheAddr = pdat->vram[0];
	writeBackRAM = pdat->vram[0];
	// writeBackRAM = dma_alloc_coherent(pdat->width * pdat->height * pdat->bytes_per_pixel);
	// if(writeBackRAM==0)
	// 	printf("*******dma_alloc full********\r\n");

	/*************************************************
	DOTCLK = fframe × (X + HBP + HFP+HSPW) × (Y + VBP + VFP+VSPW)(单位：MHz)
	pixclock = 1012/ DOTCLK=1012/ （fframe × (X + HBP + HFP+HSPW) × (Y + VBP + VFP+VSPW)） (单位：皮秒)
	fframe表帧率，一般取60
	************************************************/
/****************************************
	__u32 pixclock; //像素时钟(皮秒) 
　　__u32 left_margin; //行切换，从同步到绘图之间的延迟 
　　__u32 right_margin; //行切换，从绘图到同步之间的延迟 
　　__u32 upper_margin; //帧切换，从同步到绘图之间的延迟 
　　__u32 lower_margin; //帧切换，从绘图到同步之间的延迟 
　　__u32 hsync_len; //水平同步的长度 
　　__u32 vsync_len; //垂直同步的长度

VBP(vertical back porch)：表示在一帧图像开始时，垂直同步信号以后的无效的行数，对应驱动中的upper_margin；
VFB(vertical front porch)：表示在一帧图像结束后，垂直同步信号以前的无效的行数，对应驱动中的lower_margin；
VSPW(vertical sync pulse width)：表示垂直同步脉冲的宽度，用行数计算，对应驱动中的vsync_len；
HBP(horizontal back porch)：表示从水平同步信号开始到一行的有效数据开始之间的VCLK的个数，对应驱动中的left_margin；
HFP(horizontal front porth)：表示一行的有效数据结束到下一个水平同步信号开始之间的VCLK的个数，对应驱动中的right_margin；
HSPW(horizontal sync pulse width)：表示水平同步信号的宽度，用VCLK计算，对应驱动中的hsync_len；
**************************************/
	//note: pdat->timing.pixel_clock_hz高于33000000, F1C100S_CCU_BASE + CCU_PLL_VIDEO_CTRL需改
	//HJ080IA-01E 1024*768
	// pdat->timing.pixel_clock_hz = 65000000; 
	// pdat->timing.h_front_porch = 160;
	// pdat->timing.h_back_porch = 160;
	// pdat->timing.h_sync_len = 1;
	// pdat->timing.v_front_porch = 15;
	// pdat->timing.v_back_porch = 23;
	// pdat->timing.v_sync_len = 1;
	//7 IPS 1024*600
	// pdat->timing.pixel_clock_hz = 51200000; 
	// pdat->timing.h_front_porch = 160;
	// pdat->timing.h_back_porch = 140;
	// pdat->timing.h_sync_len = 20;
	// pdat->timing.v_front_porch = 12;
	// pdat->timing.v_back_porch = 20;
	// pdat->timing.v_sync_len = 3;
	//TM080SDH01 800*600
	// pdat->timing.pixel_clock_hz = 40000000; 
	// pdat->timing.h_front_porch = 112;
	// pdat->timing.h_back_porch = 88;
	// pdat->timing.h_sync_len = 1;
	// pdat->timing.v_front_porch = 21;
	// pdat->timing.v_back_porch = 39;
	// pdat->timing.v_sync_len = 3;
	//LSA40AT9001 800*600
	// pdat->timing.pixel_clock_hz = 40000000; 
	// pdat->timing.h_front_porch = 210;
	// pdat->timing.h_back_porch = 46;
	// pdat->timing.h_sync_len = 1;
	// pdat->timing.v_front_porch = 12;
	// pdat->timing.v_back_porch = 23;
	// pdat->timing.v_sync_len = 1;
	//AT070TN94 800*480
	pdat->timing.pixel_clock_hz = 33300000;
	pdat->timing.h_front_porch = 210;
	pdat->timing.h_back_porch = 46;
	pdat->timing.h_sync_len = 1;
	pdat->timing.v_front_porch = 22;
	pdat->timing.v_back_porch = 23;
	pdat->timing.v_sync_len = 1;
	//4.3 inch 800*480
	// pdat->timing.pixel_clock_hz = 33000000;
	// pdat->timing.h_front_porch = 40;
	// pdat->timing.h_back_porch = 87;
	// pdat->timing.h_sync_len = 1;
	// pdat->timing.v_front_porch = 13;
	// pdat->timing.v_back_porch = 31;
	// pdat->timing.v_sync_len = 1;
	//AT043TN24 480*272
	// pdat->timing.pixel_clock_hz = 9000000;
	// pdat->timing.h_front_porch = 40;
	// pdat->timing.h_back_porch = 87;
	// pdat->timing.h_sync_len = 1;
	// pdat->timing.v_front_porch = 13;
	// pdat->timing.v_back_porch = 31;
	// pdat->timing.v_sync_len = 1;

	pdat->timing.DE_HV_mode = 1; //new add 0: DE_HV 1:DE 2:HV
	pdat->timing.h_sync_active = FALSE; //false: polarity invert
	pdat->timing.v_sync_active = FALSE;
	pdat->timing.den_active = TRUE;
	pdat->timing.clk_active = FALSE;
	pdat->backlight = &led_pwm_bl; //back light function
	pdat->swapBR = TRUE; //FALSE: close BR swap, TRUE: enable BR swap
	pdat->layer0_to_video = FALSE;

	fb->name = "fb-f1c100s";
	fb->width = pdat->width;
	fb->height = pdat->height;
	fb->user_width = pdat->width;
	fb->user_height = pdat->height;
	fb->pwidth = pdat->pwidth;
	fb->pheight = pdat->pheight;
	fb->bpp = pdat->bytes_per_pixel * 8;
	fb->priv = pdat;

	f1c100s_clk_pll_video_set_enable(TRUE);
	f1c100s_clk_debe_init();
	f1c100s_clk_defe_init();
	f1c100s_clk_tcon_init();
	// clk_enable(pdat->clkdefe);
	// clk_enable(pdat->clkdebe);
	// clk_enable(pdat->clktcon);
	f1c100s_clk_defe_enable();
	f1c100s_clk_debe_enable();
	f1c100s_clk_tcon_enable();
	if(pdat->rstdefe >= 0)
		reset_f1c100s_deassert(&reset_2, pdat->rstdefe - reset_2.base);
	if(pdat->rstdebe >= 0)
		reset_f1c100s_deassert(&reset_2, pdat->rstdebe - reset_2.base);
	if(pdat->rsttcon >= 0)
		reset_f1c100s_deassert(&reset_2, pdat->rsttcon - reset_2.base);
	for(i = 0x0800; i < 0x1000; i += 4)
		write32(pdat->virtdebe + i, 0);
	fb_f1c100s_init_hw(pdat);

}

void fb_f1c100s_remove(framebuffer_t * fb)
{
	fb_f1c100s_pdata_t * pdat = (fb_f1c100s_pdata_t *)fb->priv;

	if(fb)
	{
		// clk_disable(pdat->clkdefe);
		// clk_disable(pdat->clkdebe);
		// clk_disable(pdat->clktcon);
		f1c100s_clk_defe_disable();
		f1c100s_clk_debe_disable();
		f1c100s_clk_tcon_disable();
		// free(pdat->clkdefe);
		// free(pdat->clkdebe);
		// free(pdat->clktcon);
		dma_free_noncoherent(pdat->vram[0]);
		dma_free_noncoherent(pdat->vram[1]);

		// free_device_name(fb->name);
		// free(fb->priv);
		// free(fb);
	}
}

void fb_f1c100s_suspend(framebuffer_t * fb)
{
	fb_f1c100s_pdata_t * pdat = (fb_f1c100s_pdata_t *)fb->priv;

	//pdat->brightness = led_get_brightness(pdat->backlight);
	pdat->brightness = pwm_f1c100s_get_duty(pdat->backlight);
	//led_set_brightness(pdat->backlight, 0);
	pwm_f1c100s_set_duty(pdat->backlight, 0);
}

void fb_f1c100s_resume(framebuffer_t * fb)
{
	fb_f1c100s_pdata_t * pdat = (fb_f1c100s_pdata_t *)fb->priv;

	//led_set_brightness(pdat->backlight, pdat->brightness);
	pwm_f1c100s_set_duty(pdat->backlight, pdat->brightness);
}
