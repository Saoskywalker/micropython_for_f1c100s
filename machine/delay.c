#include "delay.h"

#define CPU_FREQUENCY 720 //CPU: 720M

#if CPU_FREQUENCY == 720
#define _MS_CNT 90000
#define _US_CNT 87
#else
#define _MS_CNT 51000
#define _US_CNT 36
#endif

void delay(volatile uint32_t ms)
{
	while(ms--)
	{
		volatile uint32_t n = _MS_CNT;
		while(n--);
	}
}

void delay_us(volatile uint32_t us)
{
	while(us--)
	{
		volatile uint32_t n = _US_CNT;
		while(n--);
	}
}

void delay_ms(volatile uint32_t ms)
{
	while(ms--)
	{
		volatile uint32_t n = _MS_CNT;
		while(n--);
	}
}

