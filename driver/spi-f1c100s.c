/*
 * sys-spi-flash.c
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

#include <stdint.h>
#include <string.h>
#include <io.h>
#include <spi-f1c100s.h>
#include <gpio-f1c100s.h>
#include <types_plus.h>

#define SPI0_BASE_ADDR 0x01c05000
#define SPI1_BASE_ADDR 0x01c06000

enum {
	SPI_GCR	= 0x04,
	SPI_TCR	= 0x08,
	SPI_IER	= 0x10,
	SPI_ISR	= 0x14,
	SPI_FCR	= 0x18,
	SPI_FSR	= 0x1c,
	SPI_WCR	= 0x20,
	SPI_CCR	= 0x24,
	SPI_MBC	= 0x30,
	SPI_MTC	= 0x34,
	SPI_BCC	= 0x38,
	SPI_TXD	= 0x200,
	SPI_RXD	= 0x300,
};

void spi_flash_init(uint8_t port)
{
	virtual_addr_t addr;
	uint32_t val;

	if (port == SPI1)
	{
		/* Config GPIOE7, GPIOE8, GPIOE9 and GPIOE10 */
		gpio_f1c100s_set_cfg(&GPIO_PE, 7, 4);  //CS
		gpio_f1c100s_set_cfg(&GPIO_PE, 8, 4);  //MOSI
		gpio_f1c100s_set_cfg(&GPIO_PE, 9, 4);  //CLK
		gpio_f1c100s_set_cfg(&GPIO_PE, 10, 4); //MISO

		// gpio_f1c100s_set_pull(&GPIO_PE, 7, GPIO_PULL_UP);
		// gpio_f1c100s_set_pull(&GPIO_PE, 8, GPIO_PULL_UP);
		// gpio_f1c100s_set_pull(&GPIO_PE, 9, GPIO_PULL_UP);
		// gpio_f1c100s_set_pull(&GPIO_PE, 10, GPIO_PULL_UP);

		/* Deassert spi1 reset */
		addr = 0x01c202c0;
		val = read32(addr);
		val |= (1 << 21);
		write32(addr, val);

		/* Open the spi1 bus gate */
		addr = 0x01c20000 + 0x60;
		val = read32(addr);
		val |= (1 << 21);
		write32(addr, val);

		addr = SPI1_BASE_ADDR;
	}
	else
	{
		/* Config GPIOC0, GPIOC1, GPIOC2 and GPIOC3 */
		addr = 0x01c20848 + 0x00;
		val = read32(addr);
		val &= ~(0xf << ((0 & 0x7) << 2));
		val |= ((0x2 & 0x7) << ((0 & 0x7) << 2));
		write32(addr, val);

		val = read32(addr);
		val &= ~(0xf << ((1 & 0x7) << 2));
		val |= ((0x2 & 0x7) << ((1 & 0x7) << 2));
		write32(addr, val);

		val = read32(addr);
		val &= ~(0xf << ((2 & 0x7) << 2));
		val |= ((0x2 & 0x7) << ((2 & 0x7) << 2));
		write32(addr, val);

		val = read32(addr);
		val &= ~(0xf << ((3 & 0x7) << 2));
		val |= ((0x2 & 0x7) << ((3 & 0x7) << 2));
		write32(addr, val);

		// gpio_f1c100s_set_pull(&GPIO_PC, 0, GPIO_PULL_UP);
		// gpio_f1c100s_set_pull(&GPIO_PC, 1, GPIO_PULL_UP);
		// gpio_f1c100s_set_pull(&GPIO_PC, 2, GPIO_PULL_UP);
		// gpio_f1c100s_set_pull(&GPIO_PC, 3, GPIO_PULL_UP);

		/* Deassert spi0 reset */
		addr = 0x01c202c0;
		val = read32(addr);
		val |= (1 << 20);
		write32(addr, val);

		/* Open the spi0 bus gate */
		addr = 0x01c20000 + 0x60;
		val = read32(addr);
		val |= (1 << 20);
		write32(addr, val);

		addr = SPI0_BASE_ADDR;
	}

	/* Set spi clock rate control register, divided by 4 */
	/*this use CDR2, SPI_CLK = AHB_CLK / (2*(n + 1))*/
	//NOTE: I change it to rate up(divided 2)
	write32(addr + SPI_CCR, 0x00001000);
	//write32(addr + SPI_CCR, 0x00001001);	//src divided 4

	/* Enable spi and do a soft reset */
	val = read32(addr + SPI_GCR);
	val |= (1 << 31) | (1 << 7) | (1 << 1) | (1 << 0);
	write32(addr + SPI_GCR, val);
	while (read32(addr + SPI_GCR) & (1 << 31));

	val = read32(addr + SPI_TCR);
	val &= ~(0x3 << 0);
	val |= (1 << 6) | (1 << 2);
	write32(addr + SPI_TCR, val);

	val = read32(addr + SPI_FCR);
	val |= (1 << 31) | (1 << 15);
	write32(addr + SPI_FCR, val);
}

void spi_flash_exit(uint8_t port)
{
	virtual_addr_t addr = SPI0_BASE_ADDR;
	uint32_t val;

	if (port==SPI1)
	{
		addr = SPI1_BASE_ADDR;
	}
	
	/* Disable the spi controller */
	val = read32(addr + SPI_GCR);
	val &= ~((1 << 1) | (1 << 0));
	write32(addr + SPI_GCR, val);
}

void spi_select(uint8_t port)
{
	virtual_addr_t addr = SPI0_BASE_ADDR;
	uint32_t val;

	if (port==SPI1)
	{
		addr = SPI1_BASE_ADDR;
	}

	val = read32(addr + SPI_TCR);
	val &= ~((0x3 << 4) | (0x1 << 7));
	val |= ((0 & 0x3) << 4) | (0x0 << 7);
	write32(addr + SPI_TCR, val);
}

void spi_deselect(uint8_t port)
{
	virtual_addr_t addr = SPI0_BASE_ADDR;
	uint32_t val;

	if (port==SPI1)
	{
		addr = SPI1_BASE_ADDR;
	}
	
	val = read32(addr + SPI_TCR);
	val &= ~((0x3 << 4) | (0x1 << 7));
	val |= ((0 & 0x3) << 4) | (0x1 << 7);
	write32(addr + SPI_TCR, val);
}

static void spi_write_txbuf(uint8_t port, uint8_t * buf, int len)
{
	virtual_addr_t addr = SPI0_BASE_ADDR;
	int i;

	if (port==SPI1)
	{
		addr = SPI1_BASE_ADDR;
	}

	if(!buf)
		len = 0;

	write32(addr + SPI_MTC, len & 0xffffff);
	write32(addr + SPI_BCC, len & 0xffffff);
	for(i = 0; i < len; ++i)
		write8(addr + SPI_TXD, *buf++);
}

int spi_transfer(uint8_t port, void * txbuf, void * rxbuf, int len)
{
	virtual_addr_t addr = SPI0_BASE_ADDR;
	int count = len;
	uint8_t * tx = txbuf;
	uint8_t * rx = rxbuf;
	uint8_t val;
	int n, i;

	if (port==SPI1)
	{
		addr = SPI1_BASE_ADDR;
	}

	while(count > 0)
	{
		n = (count <= 64) ? count : 64;
		write32(addr + SPI_MBC, n);
		spi_write_txbuf(port, tx, n);
		write32(addr + SPI_TCR, read32(addr + SPI_TCR) | (1 << 31));

		while((read32(addr + SPI_FSR) & 0xff) < n);
		for(i = 0; i < n; i++)
		{
			val = read8(addr + SPI_RXD);
			if(rx)
				*rx++ = val;
		}

		if(tx)
			tx += n;
		count -= n;
	}
	return len;
}

int spi_write_then_read(uint8_t port, void * txbuf, int txlen, void * rxbuf, int rxlen)
{
	if(spi_transfer(port, txbuf, NULL, txlen) != txlen)
		return -1;
	if(spi_transfer(port, NULL, rxbuf, rxlen) != rxlen)
		return -1;
	return 0;
}

void spi_flash_read(uint8_t port, int addr, void * buf, int count)
{
	uint8_t tx[4];

	tx[0] = 0x03;
	tx[1] = (uint8_t)(addr >> 16);
	tx[2] = (uint8_t)(addr >> 8);
	tx[3] = (uint8_t)(addr >> 0);
	spi_select(port);
	spi_write_then_read(port, tx, 4, buf, count);
	spi_deselect(port);
}

void spi_set_rate(uint8_t port, uint32_t rate)
{
	virtual_addr_t addr = SPI0_BASE_ADDR;

	if (port==SPI1)
	{
		addr = SPI1_BASE_ADDR;
	}
 	write32(addr + SPI_CCR, rate);
}
