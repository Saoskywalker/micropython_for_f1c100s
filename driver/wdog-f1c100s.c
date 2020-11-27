/*
 * driver/wdog-f1c100s.c
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

#include "wdog-f1c100s.h"

#define WDOG_IRQ_EN_REG (0x01C20C00 + 0xA0)
#define WDOG_IRQ_STA_REG (0x01C20C00 + 0xA4)
#define WDOG_CTRL_REG (0x01C20C00 + 0xB0)
#define WDOG_CFG_REG (0x01C20C00 + 0xB4)
#define WDOG_MODE_REG (0x01C20C00 + 0xB8)

void wdog_f1c100s_set(u32_t timeout)
{
	write32(WDOG_CFG_REG, 0x1); //To whole system
    write32(WDOG_MODE_REG, timeout); //Interval Value set 1s
    write32(WDOG_MODE_REG, read32(WDOG_MODE_REG)|(1<<0)); //Enable Watchdog
    //delay_ms(500);
    //Writel 0xA57 at Key Field and Restart Watchdog
    write32(WDOG_CTRL_REG, read32(WDOG_MODE_REG)|(0xA57<<1)|(1<<0)); 
}

void wdog_f1c100s_feed(u32_t timeout)
{
	write32(WDOG_CFG_REG, 0x1); //To whole system
    write32(WDOG_MODE_REG, timeout); //Interval Value set 1s
    write32(WDOG_MODE_REG, read32(WDOG_MODE_REG)|(1<<0)); //Enable Watchdog
    //Writel 0xA57 at Key Field and Restart Watchdog
    write32(WDOG_CTRL_REG, read32(WDOG_MODE_REG)|(0xA57<<1)|(1<<0)); 
}
