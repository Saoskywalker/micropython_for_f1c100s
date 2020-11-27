#ifndef __UART_16550_H__
#define __UART_16550_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <types.h>

#define UART0 0
#define UART1 1
#define UART2 2

#define UART0_BASE_ADDR 0x01c25000
#define UART1_BASE_ADDR 0x01c25400
#define UART2_BASE_ADDR 0x01c25800

struct uart_16550_pdata_t 
{
	virtual_addr_t virt;
	u8_t clk;
	int reset;
	int txd;
	int txdcfg;
	int rxd;
	int rxdcfg;
	int baud;
	int data;
	int parity;
	int stop;
};

struct uart_t
{
	/* The uart name */
	char * name;

	/* Set uart param */
	int (*set)(struct uart_t * uart, int baud, int data, int parity, int stop);

	/* Get uart param */
	int (*get)(struct uart_t * uart, int * baud, int * data, int * parity, int * stop);

	/* Read uart */
	ssize_t (*read)(struct uart_t * uart, u8_t * buf, size_t count);

	/* Write uart */
	ssize_t (*write)(struct uart_t * uart, const u8_t * buf, size_t count);

	/* Private data */
	void * priv;
};

struct uart_t * uart_16550_init(u8_t port, int baud, int data, int parity, int stop);
void uart_16550_putc(struct uart_t * uart, u8_t c);
u8_t uart_16550_rec(struct uart_t * uart, u8_t *rec_char);
void uart_16550_close(struct uart_t **uart);
unsigned char uart_16550_rec_empty(struct uart_t * uart);
unsigned char uart_16550_send_empty(struct uart_t * uart);

#ifdef __cplusplus
}
#endif

#endif
