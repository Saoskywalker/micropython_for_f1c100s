#ifndef _SPI_F1C_FLASH_H
#define _SPI_F1C_FLASH_H

#include <types.h>
#include <stdint.h>

#define SPI0 0
#define SPI1 1

void spi_flash_init(uint8_t port);
void spi_flash_exit(uint8_t port);
void spi_select(uint8_t port);
void spi_deselect(uint8_t port);
int spi_transfer(uint8_t port, void * txbuf, void * rxbuf, int len);
int spi_write_then_read(uint8_t port, void * txbuf, int txlen, void * rxbuf, int rxlen);
void spi_flash_read(uint8_t port, int addr, void * buf, int count);
void spi_set_rate(uint8_t port, uint32_t rate);

#endif
