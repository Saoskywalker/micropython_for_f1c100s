#ifndef _SYS_SPI_FLASH_H
#define _SYS_SPI_FLASH_H

#include <types.h>

void sys_spi_flash_init(void);
void sys_spi_flash_exit(void);
void sys_spi_select(void);
void sys_spi_deselect(void);
int sys_spi_transfer(void * txbuf, void * rxbuf, int len);
int sys_spi_write_then_read(void * txbuf, int txlen, void * rxbuf, int rxlen);
void sys_spi_flash_read(int addr, void * buf, int count);

#endif
