#include <io.h>
#include "timer_f1c100s.h"
#include "irq_misc.h"
#include <gpio-f1c100s.h>
#include <stdio.h>
#include "wdog-f1c100s.h"
#include "touch.h"
#include <pwm-f1c100s.h>
#include "ComPort.h"
#include "UI_engine.h"

#define TMR_IRQ_EN_REG (0x01C20C00 + 0x00)
#define TMR_IRQ_STA_REG (0x01C20C00 + 0x04)
#define TMR0_CTRL_REG (0x01C20C00 + 0x10)
#define TMR0_INTV_VALUE_REG (0x01C20C00 + 0x14)
#define TMR0_CUR_VALUE_REG (0x01C20C00 + 0x18)

extern pwm_t BeepControl;
extern int beep_enable, back_light_control_enable;
extern int user_touch_switch, user_touch_mode, user_touch_down_keep_up;
extern u8 reset_flag;

/*Bee function*/
static const u16 BeeModPeriod[] = {99, 410, 1000, 1010, 3010};
static const u16 BeeModCompare[] = {84, 400, 500, 1000, 3000};
u8 BeeMod = 0, BeeTime = 0, BeeVol = 70; //禁止VOL:100, 必须方波
void BeeFunction(void)
{
	static u16 BeeTimeCnt = 0;

	if (BeeTime > 0)
	{
		if (++BeeTimeCnt >= BeeModPeriod[BeeMod])
		{
			BeeTimeCnt = 0;
			BeeTime--;
		}
		else
		{
			if (BeeTimeCnt >= BeeModCompare[BeeMod])
				pwm_f1c100s_set_duty(&BeepControl, 100); //不响
			else
				pwm_f1c100s_set_duty(&BeepControl, BeeVol); //响
		}
	}
	else
	{
		BeeTimeCnt = 0;
		pwm_f1c100s_set_duty(&BeepControl, 100); //不响
	}
}

//timer0 中断
u8 diskCheckBusy(void);
void _WAV_Play2(void);
u8 timer_3ms_flag = 0, timer_9ms_flag = 0, timer_6s_flag = 0, timer_450ms_flag = 0;
void timer0_irq(void *arg)
{
	static int ioio = 0;
	volatile unsigned int *temp_addr = (unsigned int *)(0x01C20C00 + 0x04);
	static u8 touch_down = 0;
	static u8 timer9msCnt = 0;
	static u32 timer6sCnt = 0, timer450msCnt = 0;
	static u8 i = 0;

	timer_3ms_flag = 1;
	if(diskCheckBusy()==0) //确认总线是否空闲
		_WAV_Play2();
	if (i)
		i = 0;
	else
		i = 1;
	// gpio_f1c100s_set_value(&GPIO_PE, 7, i);
	if (++timer9msCnt >= 3)
	{
		timer9msCnt = 0;
		timer_9ms_flag = 1;
	}
	if(++timer450msCnt>=150)
	{
		timer450msCnt = 0;
		timer_450ms_flag = 1;
	}
	if (++timer6sCnt >= 2000)
	{
		timer6sCnt = 0;
		timer_6s_flag = 1;
	}

	if (TPAjustFlag)
	{
		tp_dev.scan(1); //扫描物理坐标
	}
	else
	{
		tp_dev.scan(0);				   //扫描屏幕坐标
		if (tp_dev.sta & TP_PRES_DOWN) //触摸屏被按下
			UI_touchAdapt(tp_dev.x[0], tp_dev.y[0], 1);
		else
			UI_touchAdapt(tp_dev.x[0], tp_dev.y[0], 0);
	}
	if (tp_dev.sta & TP_PRES_DOWN) //触摸屏被按下
	{
		// printf("touch: %d, %d\r\n", UI_pressTouchX, UI_pressTouchY);
		touch_down = 1;
	}
	else
	{
		touch_down = 0;
	}

	if(reset_flag==0)
		wdog_f1c100s_feed(WDOG_14S); //feed wdog
	
	BeeFunction();
	ComModel.recProcess();

	if (++ioio >= 200)
	{
		ioio = 0;
		//printf("timer0 irq\r\n");
	}

	// write32(TMR_IRQ_STA_REG, (1)); //清除TIMER0中断标识
	*temp_addr |= 0x01; /* clear timer */
}

void TIMER0_init(u32 value, u32 pres)
{
	// gpio_f1c100s_set_dir(&GPIO_PE, 7, GPIO_DIRECTION_OUTPUT);

	write32(TMR_IRQ_EN_REG, 1);
	write32(IRQ_ADDR_BASE + IRQ_ENABLE0, (read32(IRQ_ADDR_BASE + IRQ_ENABLE0)) | 1 << 13);
	write32(TMR_IRQ_EN_REG, (read32(TMR_IRQ_EN_REG)) | 1 << 13);
	write32(TMR0_INTV_VALUE_REG, value * 24); //Set interval value  set 20:1ms 2:150us
	write32(TMR0_CTRL_REG, 0);
	write32(TMR0_CTRL_REG, read32(TMR0_CTRL_REG) | (pres << 4));
	write32(TMR0_CTRL_REG, read32(TMR0_CTRL_REG) | (1 << 2)); //there24M losc:about 36k
	write32(TMR0_CTRL_REG, read32(TMR0_CTRL_REG) | (1 << 1)); //Set Reload bit
	while ((read32(TMR0_CTRL_REG) >> 1) & 1);		 //Waiting Reload bit turns to 0
	write32(TMR0_CTRL_REG, read32(TMR0_CTRL_REG) | (1 << 0)); //Enable Timer0

	request_irq(IRQ_TIMER0, timer0_irq, 0); //register int
}

/*******************************************************/
/*
定时器初始化
timer=TIMER0,TIMER1,TIMER2【f1c100s有三个32位定时器】
IRQ_EN=中断使能
time_us=中断时间 us 最大178956970
*/
void Timer_Init(u8 timer, u32 time_us, u8 IRQ_EN)
{
	u32 val = 0;
	write32(TIMER_CTRL(timer), 0);
	//Timer 模式 0连续 1单
	val = 0;
	write32(TIMER_CTRL(timer), read32(TIMER_CTRL(timer)) | ((val) << 7));
	//Timer 分频
	val = 0;
	write32(TIMER_CTRL(timer), read32(TIMER_CTRL(timer)) | ((val) << 4));
	//Timer 时钟源
	val = 1; //there is 24Mhz , losc about 36k
	write32(TIMER_CTRL(timer), read32(TIMER_CTRL(timer)) | ((val) << 2));
	//Timer 重装计数值
	val = 1; //=1重装计数值
	write32(TIMER_CTRL(timer), read32(TIMER_CTRL(timer)) | ((val) << 1));

	//Timer 计数值
	val = 24 * time_us;
	write32(TIMER_INTV(timer), val);

	//使能中断
	if (IRQ_EN == 1)
	{
		write32(TIMER_IRQ_EN, read32(TIMER_IRQ_EN) | (1 << timer));
	}
}
/*
定时器开
*/
void Timer_enable(u8 timer)
{
	write32(TIMER_CTRL(timer), read32(TIMER_CTRL(timer)) | ((1) << 0));
}
/*
定时器关
*/
void Timer_disable(u8 timer)
{
	write32(TIMER_CTRL(timer), read32(TIMER_CTRL(timer)) & (~((1) << 0)));
}
/*
定时器中断
*/
unsigned int sys_count_timer0 = 0;
void TIMER0_ISR(void *arg)
{
	//清中断
	write32(TIMER_IRQ_STA, read32(TIMER_IRQ_STA) | (1 << TIMER0));
	//
	sys_count_timer0++;
	if (sys_count_timer0 >= 0xffffffff)
		sys_count_timer0 = 0;
}
void TIMER1_ISR(void *arg)
{
	static u8 i = 0;
	//清中断
	write32(TIMER_IRQ_STA, read32(TIMER_IRQ_STA) | (1 << TIMER1));
	if (i)
		i = 0;
	else
		i = 1;
	//printf("timer1 irq\r\n");
	// gpio_f1c100s_set_value(&GPIO_PE, 7, i);
}
void TIMER2_ISR(void *arg)
{
	//清中断
	write32(TIMER_IRQ_STA, read32(TIMER_IRQ_STA) | (1 << TIMER2));
	//
}
/*
读定时器计数
*/
unsigned int Read_time_ms(void)
{
	return sys_count_timer0;
}
/*
延时函数(定时器)
*/
void Tdelay_ms(int ms)
{
	unsigned int time = 0;
	time = sys_count_timer0;
	while ((sys_count_timer0 - time) < ms);
}
void _timer_delay_us(u32 time_us)
{
	u8 timer = TIMER2;
	u32 val = 0;
	write32(TIMER_CTRL(timer), 0);
	// val = 0; //Timer 模式 0连续 1单
	// write32(TIMER_CTRL(timer), read32(TIMER_CTRL(timer)) | ((val) << 7));
	// val = 0; //Timer 分频
	// write32(TIMER_CTRL(timer), read32(TIMER_CTRL(timer)) | ((val) << 4));
	val = 1; //Timer 时钟源 24Mhz
	write32(TIMER_CTRL(timer), read32(TIMER_CTRL(timer)) | ((val) << 2));
	// val = 1; //=1重装计数值
	// write32(TIMER_CTRL(timer), read32(TIMER_CTRL(timer)) | ((val) << 1));
	val = 24 * time_us; //Timer 计数值
	write32(TIMER_INTV(timer), val);
	// write32(TIMER_CUR(timer), val);
	//write32(TIMER_IRQ_EN,read32(TIMER_IRQ_EN)	&	(~(1<<timer))); //关中断
	write32(TIMER_CTRL(timer), read32(TIMER_CTRL(timer)) | ((1) << 0)); //enable
	while ((read32(TIMER_IRQ_STA) & (1 << timer)) == 0); //检查标志
	write32(TIMER_IRQ_STA, read32(TIMER_IRQ_STA) | (1 << timer));		 //清标志
	write32(TIMER_CTRL(timer), read32(TIMER_CTRL(timer)) & (~(1) << 0)); //disable
}
/*
定时器测试
*/
void Timer_Demo(void)
{
	// gpio_f1c100s_set_dir(&GPIO_PE, 7, GPIO_DIRECTION_OUTPUT);
	Timer_Init(TIMER0, 3000, 1);
	request_irq(IRQ_TIMER0, timer0_irq, 0); //register int
	Timer_enable(TIMER0);
	Timer_Init(TIMER1, 200000, 1);
	request_irq(IRQ_TIMER1, TIMER1_ISR, 0); //register int
	Timer_enable(TIMER1);
	//	while(1)
	//	{
	//		printf("TIMER %d \r\n",sys_count_timer0);
	//		Tdelay_ms(1000);
	//	}
}

/*音视频同步时间初始化-100us*/
void AVS_Time_Init(void)
{
	/*24M时钟通过*/
	S_BIT(0x01C20000 + 0x0144, 31);
	/*初始为100us/次*/
	write32(F1C100S_TIMER_BASE + 0x8c, (2400 - 1) << 0 | (2400 - 1) << 16);
}
/*停止计数器*/
void AVS_Time_Stop(int AVS_TIME_Inx)
{
	if (AVS_TIME_Inx == AVS_TIME_0)
	{
		write32(F1C100S_TIMER_BASE + 0x84, 0);
		write32(F1C100S_TIMER_BASE + 0x80, read32(F1C100S_TIMER_BASE + 0x80) & (~((1) << 0)));
	}
	if (AVS_TIME_Inx == AVS_TIME_1)
	{
		write32(F1C100S_TIMER_BASE + 0x88, 0);
		write32(F1C100S_TIMER_BASE + 0x80, read32(F1C100S_TIMER_BASE + 0x80) & (~((1) << 1)));
	}
}
/*开始并计数器清0*/
void AVS_Time_Start(int AVS_TIME_Inx)
{
	if (AVS_TIME_Inx == AVS_TIME_0)
	{
		write32(F1C100S_TIMER_BASE + 0x84, 0);
		write32(F1C100S_TIMER_BASE + 0x80, read32(F1C100S_TIMER_BASE + 0x80) | (1) << 0);
	}
	if (AVS_TIME_Inx == AVS_TIME_1)
	{
		write32(F1C100S_TIMER_BASE + 0x88, 0);
		write32(F1C100S_TIMER_BASE + 0x80, read32(F1C100S_TIMER_BASE + 0x80) | (1) << 1);
	}
}
/*读计数值 33位计数器读出的为高32位*/
unsigned int AVS_Time_Read(int AVS_TIME_Inx)
{
	if (AVS_TIME_Inx == AVS_TIME_0)
		return read32(F1C100S_TIMER_BASE + 0x84) * 2;
	if (AVS_TIME_Inx == AVS_TIME_1)
		return read32(F1C100S_TIMER_BASE + 0x88) * 2;
	return 0;
}
/*计数暂停*/
void AVS_Time_Pause(int AVS_TIME_Inx)
{
	if (AVS_TIME_Inx == AVS_TIME_0)
		write32(F1C100S_TIMER_BASE + 0x80, read32(F1C100S_TIMER_BASE + 0x80) | (1) << 8);
	if (AVS_TIME_Inx == AVS_TIME_1)
		write32(F1C100S_TIMER_BASE + 0x80, read32(F1C100S_TIMER_BASE + 0x80) | (1) << 9);
}
/*计数继续*/
void AVS_Time_Recover(int AVS_TIME_Inx)
{
	if (AVS_TIME_Inx == AVS_TIME_0)
		write32(F1C100S_TIMER_BASE + 0x80, read32(F1C100S_TIMER_BASE + 0x80) & (~((1) << 8)));
	if (AVS_TIME_Inx == AVS_TIME_1)
		write32(F1C100S_TIMER_BASE + 0x80, read32(F1C100S_TIMER_BASE + 0x80) & (~((1) << 9)));
}
