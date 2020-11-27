#ifndef _WDOG_F1C_H
#define _WDOG_F1C_H

#include <io.h>
#include <types.h>

enum{
    WDOG_05S = 0x00,
    WDOG_1S = 0x10, 
    WDOG_2S = 0x20, 
    WDOG_3S = 0x30, 
    WDOG_4S = 0x40, 
    WDOG_5S = 0x50, 
    WDOG_6S = 0x60, 
	WDOG_8S = 0x70, 
    WDOG_10S = 0x80, 
    WDOG_12S = 0x90, 
    WDOG_14S = 0xA0, 
    WDOG_16S = 0xB0 //NOTE: this value has a bug, after 16s, will reboot
};

void wdog_f1c100s_set(u32_t timeout); //START WDOG
void wdog_f1c100s_feed(u32_t timeout); //FEED WDOG

#endif
