#ifndef _TIMER_F1C100S_H
#define _TIMER_F1C100S_H

#include "types_plus.h"

enum
{
    TIMR_CLK_PRES_1 = 0,
    TIMR_CLK_PRES_2 = 1,
    TIMR_CLK_PRES_4 = 2,
    TIMR_CLK_PRES_8 = 3,
    TIMR_CLK_PRES_16 = 4,
    TIMR_CLK_PRES_32 = 5,
    TIMR_CLK_PRES_64 = 6,
    TIMR_CLK_PRES_128 = 7,
};

void TIMER0_init(u32 value, u32 pres);

/***********************************************/
#define TIMER0 0
#define TIMER1 1
#define TIMER2 2

#define F1C100S_TIMER_BASE	(0x01C20C00)
#define TIMER_IRQ_EN				F1C100S_TIMER_BASE+(0x00)
#define TIMER_IRQ_STA				F1C100S_TIMER_BASE+(0x04)
#define TIMER_CTRL(x)				F1C100S_TIMER_BASE+((x + 1) * 0x10 + 0x00)
#define TIMER_INTV(x)				F1C100S_TIMER_BASE+((x + 1) * 0x10 + 0x04)
#define TIMER_CUR(x)				F1C100S_TIMER_BASE+((x + 1) * 0x10 + 0x08)

/*音视频同步时间初始化-100us*/
#define AVS_TIME_0 		0
#define AVS_TIME_1 		1

void AVS_Time_Init(void);
/*开始并计数器清0*/
void AVS_Time_Start(int AVS_TIME_Inx);
void AVS_Time_Stop(int AVS_TIME_Inx);
/*读计数值*/
unsigned int AVS_Time_Read(int AVS_TIME_Inx);
/*计数暂停*/
void AVS_Time_Pause(int AVS_TIME_Inx);
/*计数继续*/
void AVS_Time_Recover(int AVS_TIME_Inx);

void Timer_Demo(void);
void Tdelay_ms(int ms);
void Timer_enable(u8 timer);
void Timer_disable(u8 timer);
void Timer_Init(u8 timer,u32 time_us,u8 IRQ_EN);
unsigned int Read_time_ms(void);
void _timer_delay_us(u32 time_us);

#endif
