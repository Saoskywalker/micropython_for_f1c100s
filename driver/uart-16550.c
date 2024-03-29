/*
 * driver/uart-16550.c
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

#include <uart-16550.h>
#include <reg-ccu.h>
#include <reset-f1c100s.h>
#include <gpio-f1c100s.h>
#include <io.h>
#include <malloc.h>
#include <stdio.h>

/*
 * Uart 16550 - Universal Asynchronous Receiver Transmitter
 *
 * Required properties:
 * - clock-name: uart parant clock name
 *
 * Optional properties:
 * - txd-gpio: uart txd gpio
 * - txd-gpio-config: uart txd gpio config
 * - rxd-gpio: uart rxd gpio
 * - rxd-gpio-config: uart rxd gpio config
 * - baud-rates: uart baud rates, default is 115200
 * - data-bits: uart data bits, default is 8
 * - parity-bits: uart parity bits, default is 0
 * - stop-bits: uart stop bits, default is 1
 *
 * Example:
 *   "uart-16550@0x01c28000": {
 *       "clock-name": "link-uart0",
 *       "txd-gpio": 40,
 *       "txd-gpio-config": 3,
 *       "rxd-gpio": 41,
 *       "rxd-gpio-config": 3,
 *       "baud-rates": 115200,
 *       "data-bits": 8,
 *       "parity-bits": 0,
 *       "stop-bits": 1
 *   }
 */

#define UART_RBR	(0x00)
#define UART_THR	(0x00)
#define UART_DLL	(0x00)
#define UART_DLH	(0x04)
#define UART_IER	(0x04)
#define UART_IIR	(0x08)
#define UART_FCR	(0x08)
#define UART_LCR	(0x0c)
#define UART_MCR	(0x10)
#define UART_LSR	(0x14)
#define UART_MSR	(0x18)
#define UART_SCH	(0x1c)
#define UART_USR	(0x7c)
#define UART_TFL	(0x80)
#define UART_RFL	(0x84)
#define UART_HALT	(0xa4)

static int uart_16550_set(struct uart_t * uart, int baud, int data, int parity, int stop)
{
	struct uart_16550_pdata_t * pdat = (struct uart_16550_pdata_t *)uart->priv;
	u8_t dreg, preg, sreg;
	u32_t val; 
	u32_t udiv;

	if(baud < 0)
		return 0;
	if((data < 5) || (data > 8))
		return 0;
	if((parity < 0) || (parity > 2))
		return 0;
	if((stop < 0) || (stop > 2))
		return 0;

	switch(data)
	{
	case 5:	/* Data bits = 5 */
		dreg = 0x0;
		break;
	case 6:	/* Data bits = 6 */
		dreg = 0x1;
		break;
	case 7:	/* Data bits = 7 */
		dreg = 0x2;
		break;
	case 8:	/* Data bits = 8 */
		dreg = 0x3;
		break;
	default:
		return 0;
	}

	switch(parity)
	{
	case 0:	/* Parity none */
		preg = 0x0;
		break;
	case 1:	/* Parity odd */
		preg = 0x1;
		break;
	case 2:	/* Parity even */
		preg = 0x3;
		break;
	default:
		return 0;
	}

	switch(stop)
	{
	case 1:	/* Stop bits = 1 */
		sreg = 0;
		break;
	case 2:	/* Stop bits = 2 */
		sreg = 1;
		break;
	case 0:	/* Stop bits = 1.5 */
	default:
		return 0;
	}

	pdat->baud = baud;
	pdat->data = data;
	pdat->parity = parity;
	pdat->stop = stop;

	val = read32(pdat->virt + UART_LCR);
	val |= (1 << 7);
	write32(pdat->virt + UART_LCR, val);

	udiv = 102000000 / baud / 16; //APBclk/Baudrate/16; APBclk = 102M
	write32(pdat->virt + UART_DLL, udiv & 0xff); //can adjust baud
	write32(pdat->virt + UART_DLH, (udiv >> 8) & 0xff);
	// udiv = 0x36; //115200
	// write32(pdat->virt + UART_DLL, udiv & 0xff); //note: baud cannot adjust 
	// write32(pdat->virt + UART_DLH, (udiv >> 8) & 0xff);

	val = read32(pdat->virt + UART_LCR);
	val &= ~(1 << 7);
	write32(pdat->virt + UART_LCR, val);

	val = read32(pdat->virt + UART_LCR);
	val &= ~0x1f;
	val |= (dreg << 0) | (sreg << 2) | (preg << 3);
	write32(pdat->virt + UART_LCR, val);

	return 1;
}

static int uart_16550_get(struct uart_t * uart, int * baud, int * data, int * parity, int * stop)
{
	struct uart_16550_pdata_t * pdat = (struct uart_16550_pdata_t *)uart->priv;

	if(baud)
		*baud = pdat->baud;
	if(data)
		*data = pdat->data;
	if(parity)
		*parity = pdat->parity;
	if(stop)
		*stop = pdat->stop;
	return 1;
}

static ssize_t uart_16550_read(struct uart_t * uart, u8_t * buf, size_t count)
{
	struct uart_16550_pdata_t * pdat = (struct uart_16550_pdata_t *)uart->priv;
	ssize_t i;

	for(i = 0; i < count; i++)
	{
		if((read32(pdat->virt + UART_LSR) & (0x1 << 0)) != 0)
			buf[i] = (u8_t)read32(pdat->virt + UART_RBR);
		else
			break;
	}
	return i;
}

static ssize_t uart_16550_write(struct uart_t * uart, const u8_t * buf, size_t count)
{
	struct uart_16550_pdata_t * pdat = (struct uart_16550_pdata_t *)uart->priv;
	ssize_t i;

	for(i = 0; i < count; i++)
	{
		while((read32(pdat->virt + UART_LSR) & (1 << 6)) == 0);
		write32(pdat->virt + UART_THR, buf[i]);
	}
	return i;
}

unsigned char uart_16550_rec_empty(struct uart_t * uart)
{
	struct uart_16550_pdata_t * pdat = (struct uart_16550_pdata_t *)uart->priv;
	return read32(pdat->virt + UART_LSR) & (1 << 0); //0 empty
}

unsigned char uart_16550_send_empty(struct uart_t * uart)
{
	struct uart_16550_pdata_t * pdat = (struct uart_16550_pdata_t *)uart->priv;
	return read32(pdat->virt + UART_LSR) & (1 << 6); //1 empty
}

void uart_16550_putc(struct uart_t * uart, u8_t c)
{
	struct uart_16550_pdata_t * pdat = (struct uart_16550_pdata_t *)uart->priv;

	while((read32(pdat->virt + UART_USR) & (0x1 << 1)) == 0);
	write32(pdat->virt + UART_THR, c);
}

u8_t uart_16550_rec(struct uart_t * uart, u8_t *rec_char)
{
	struct uart_16550_pdata_t * pdat = (struct uart_16550_pdata_t *)uart->priv;

	volatile unsigned int *rec_buf;
    volatile unsigned int *sta;
    rec_buf = (unsigned int *)(pdat->virt + UART_RBR);
    sta = (unsigned int *)(pdat->virt + UART_USR);

	/* Receive Data Available */
	if (*sta & 0x08)
	{
		*rec_char = *rec_buf & 0xff;
		// uart_16550_putc(uart, '6');
		return 0;    
	}
	else
	{
		return 1;
	}
}  

struct uart_t * uart_16550_init(u8_t port, int baud, int data, int parity, int stop)
{
	struct uart_16550_pdata_t * pdat;
	struct uart_t * uart;
	reset_f1c100s_t re_v;

	pdat = malloc(sizeof(struct uart_16550_pdata_t));
	if(!pdat)
		return 0;

	uart = malloc(sizeof(struct uart_t));
	if(!uart)
	{
		free(pdat);
		return 0;
	}

	if(port==UART0)
	{
		pdat->virt = UART0_BASE_ADDR;
		pdat->clk = UART0_GATING;
		pdat->reset = 20;
		pdat->txd = 1;
		pdat->txdcfg = 5;
		pdat->rxd = 0;
		pdat->rxdcfg = 5;
		pdat->baud = baud;
		pdat->data = data;
		pdat->parity = parity;
		pdat->stop = stop;
	}
	else if (port == UART1)
	{
		pdat->virt = UART1_BASE_ADDR;
		pdat->clk = UART1_GATING;
		pdat->reset = 21;
		pdat->txd = 3;
		pdat->txdcfg = 5;
		pdat->rxd = 2;
		pdat->rxdcfg = 5;
		pdat->baud = baud;
		pdat->data = data;
		pdat->parity = parity;
		pdat->stop = stop;
	}
	else if (port == UART2)
	{
		pdat->virt = UART2_BASE_ADDR;
		pdat->clk = UART2_GATING;
		pdat->reset = 22;
		pdat->txd = 7;
		pdat->txdcfg = 3;
		pdat->rxd = 8;
		pdat->rxdcfg = 3;
		pdat->baud = baud;
		pdat->data = 8;
		pdat->parity = parity;
		pdat->stop = stop;
	}

	uart->name = "UART";
	uart->set = uart_16550_set;
	uart->get = uart_16550_get;
	uart->read = uart_16550_read;
	uart->write = uart_16550_write;
	uart->priv = pdat;

	clk_enable(pdat->clk);
	re_v.virt = 0x01c202d0;
	reset_f1c100s_deassert(&re_v, pdat->reset);

	if (port == UART0)
	{
		gpio_f1c100s_set_cfg(&GPIO_PE, pdat->rxd, pdat->rxdcfg); //RX
		gpio_f1c100s_set_pull(&GPIO_PE, pdat->rxd, GPIO_PULL_UP);
		gpio_f1c100s_set_cfg(&GPIO_PE, pdat->txd, pdat->txdcfg); //TX
		gpio_f1c100s_set_pull(&GPIO_PE, pdat->txd, GPIO_PULL_UP);
	}
	else if (port == UART1)
	{
		gpio_f1c100s_set_cfg(&GPIO_PA, pdat->rxd, pdat->rxdcfg); //RX
		gpio_f1c100s_set_pull(&GPIO_PA, pdat->rxd, GPIO_PULL_UP);
		gpio_f1c100s_set_cfg(&GPIO_PA, pdat->txd, pdat->txdcfg); //TX
		gpio_f1c100s_set_pull(&GPIO_PA, pdat->txd, GPIO_PULL_UP);
	}
	else if (port == UART2)
	{
		gpio_f1c100s_set_cfg(&GPIO_PE, pdat->rxd, pdat->rxdcfg); //RX
		gpio_f1c100s_set_pull(&GPIO_PE, pdat->rxd, GPIO_PULL_UP);
		gpio_f1c100s_set_cfg(&GPIO_PE, pdat->txd, pdat->txdcfg); //TX
		gpio_f1c100s_set_pull(&GPIO_PE, pdat->txd, GPIO_PULL_UP);
	}

	write32(pdat->virt + UART_IER, 0x0);
	write32(pdat->virt + UART_FCR, 0xf7);
	write32(pdat->virt + UART_MCR, 0x0);
	uart_16550_set(uart, pdat->baud, pdat->data, pdat->parity, pdat->stop);

	return uart;
}

void uart_16550_close(struct uart_t **uart)
{
	struct uart_16550_pdata_t * pdat = (*uart)->priv;
	reset_f1c100s_t re_v;
	
	re_v.virt = 0x01c202d0;
	reset_f1c100s_deassert(&re_v, pdat->reset);
	clk_disable(pdat->clk);
	free(pdat);
	free(*uart);
	*uart = 0;
}
