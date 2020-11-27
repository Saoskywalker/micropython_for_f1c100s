#include "sys_TPAdc.h"
#include <stdio.h>
#include "delay.h"
#include <gpio-f1c100s.h>
#include <io.h>
#include "touch.h"

/*寄存器地址*/
#define F1C100S_TPADC_BASE (0x01C24800)

#define TP_CTRL_REG0 F1C100S_TPADC_BASE + (0x00)		 //TP Control Register 0
#define TP_CTRL_REG1 F1C100S_TPADC_BASE + (0x04)		 //TP Control Register 1
#define TP_CTRL_REG2 F1C100S_TPADC_BASE + (0x08)		 //TP Control Register 2
#define TP_CTRL_REG3 F1C100S_TPADC_BASE + (0x0C)		 //TP Control Register 3
#define TP_INT_FIFO_CTRL_REG F1C100S_TPADC_BASE + (0x10) //TP Interrupt FIFO Control Register
#define TP_INT_FIFO_STAT_REG F1C100S_TPADC_BASE + (0x14) //TP Interrupt FIFO Status Register
#define TP_COM_DATA_REG F1C100S_TPADC_BASE + (0x1C)		 //TP Common Data Register
#define TP_DATA_REG F1C100S_TPADC_BASE + (0x24)			 //TP Data Register

#define TP_MODE 0
#define ADC_MODE 1

/*
TP ADC 配置IO
*/
static void Init_TPADC_IO(void)
{
	gpio_f1c100s_set_cfg(&GPIO_PA, 0, 2);
	gpio_f1c100s_set_cfg(&GPIO_PA, 1, 2);
	gpio_f1c100s_set_cfg(&GPIO_PA, 2, 2);
	gpio_f1c100s_set_cfg(&GPIO_PA, 3, 2);
}

/*TP ADC 初始化
mode =0 TP模式
mode =1 ADC模式
*/
void Init_TP_ADC(int mode)
{
	/*配置IO*/
	Init_TPADC_IO();

	/*24M / 6 = 4Mhz(CLK_IN)/ 2^13(8192) = 488.28125 hz*/
	/*Conversion Time = 1 / (4MHz/13Cycles) = 3.25us*/
	/*触摸ADC获取时间T_ACQ  = CLK_IN /(16*(1+63)) = 3906.25hz 左右*/
	write32(TP_CTRL_REG0, (0x2 << 20) | (0x1f << 23) | (0x7 << 16) | 63);

	/*模式设置*/
	write32(TP_CTRL_REG1, (5 << 12) | (1 << 9));
	if (mode == ADC_MODE)
		S_BIT(TP_CTRL_REG1, 4); //ADC
	else
		C_BIT(TP_CTRL_REG1, 4); //TP

	write32(TP_CTRL_REG2, (0x08 << 28) | (0x0 << 26) | (0 << 24) | 0xFFF);

	/*滤波设置*/
	// write32(TP_CTRL_REG3, read32(TP_CTRL_REG3) | (1) << 2 | (11) << 0); //原版本
	write32(TP_CTRL_REG3, read32(TP_CTRL_REG3) | (1) << 2 | (0) << 0); //note:已改, 滤波级数减少

	/*使能ADC*/
	S_BIT(TP_CTRL_REG1, 5);
}

/*设置ADC通道*/
void Set_ADC_Channel(int Channel)
{
	C_Vlue(TP_CTRL_REG1, 0, 0xf);
	S_Vlue(TP_CTRL_REG1, 0, Channel);
}

/*读出AD值*/
int TP_ADC_Read_Value(void)
{
	return (read32(TP_DATA_REG) & 0xfff);
}

/***************************滤波函数****************************/
//读取一个坐标值(x或者y)
//连续读取READ_TIMES次数据,对这些数据升序排列,
//然后去掉最低和最高LOST_VAL个数,取平均值
//xy:指令（CMD_RDX/CMD_RDY）
//返回值:读到的数据
/* #define READ_TIMES 5 //读取次数
#define LOST_VAL 1   //丢弃值
static u16 TP_Read_XOY(unsigned char xy)
{
    u16 i, j;
    u16 buf[READ_TIMES];
    u16 sum = 0;
    u16 temp;
    for (i = 0; i < READ_TIMES; i++)
        buf[i] = TP_Read_AD(xy);
    for (i = 0; i < READ_TIMES - 1; i++) //排序
    {
        for (j = i + 1; j < READ_TIMES; j++)
        {
            if (buf[i] > buf[j]) //升序排列
            {
                temp = buf[i];
                buf[i] = buf[j];
                buf[j] = temp;
            }
        }
    }
    sum = 0;
    for (i = LOST_VAL; i < READ_TIMES - LOST_VAL; i++)
        sum += buf[i];
    temp = sum / (READ_TIMES - 2 * LOST_VAL);
    return temp;
} */

//读取x,y坐标
//最小值不能少于100.
//x,y:读取到的坐标值
//返回值:0,失败;1,成功。
static unsigned char TP_Read_XY(int *x, int *y)
{
    int xtemp, ytemp;

    if (TPxy)
    { 
        ytemp = TP_ADC_Read_Value();
        xtemp = TP_ADC_Read_Value();
    }
    else
    {
		xtemp = TP_ADC_Read_Value();
        ytemp = TP_ADC_Read_Value();
    }
    //if(xtemp<100||ytemp<100)return 0;//读数失败
    *x = xtemp;
    *y = ytemp;
    return 1; //读数成功
}

//连续2次读取触摸屏IC,且这两次的偏差不能超过
//ERR_RANGE,满足条件,则认为读数正确,否则读数错误.
//该函数能大大提高准确度
//x,y:读取到的坐标值
//返回值:0,失败;1,成功。
#define ERR_RANGE 50 //误差范围
static unsigned char TP_Read_XY2(int *x, int *y)
{
	static unsigned char i = 0;
    static int x1, y1;
    static int x2, y2;

	if((x==0)&&(y==0)) //无按下时清理数据
	{
		x1 = 0;
		y1 = 0;
		x2 = 0;
		y2 = 0;
		i = 0;
		return 0;
	}
	if(i)
	{
		i = 0;
		TP_Read_XY(&x1, &y1);
	}
	else
	{
		i = 1;
		TP_Read_XY(&x2, &y2);
        return 0;
	}

    // printf("point: %d, %d, %d, %d\r\n", x1, y1, x2, y2);
    if (((x2 <= x1 && x1 < x2 + ERR_RANGE) || (x1 <= x2 && x2 < x1 + ERR_RANGE)) //前后两次采样在+-误差内
        && ((y2 <= y1 && y1 < y2 + ERR_RANGE) || (y1 <= y2 && y2 < y1 + ERR_RANGE)))
    {
        *x = (x1 + x2) / 2;
        *y = (y1 + y2) / 2;
        return 1;
    }
    else
        return 0;
}

///////////////////////////////////////////////////////////////////
//与touch库对接函数

//触摸屏扫描(带滤波)
//tp:0,屏幕坐标;1,物理坐标(校准等特殊场合用)
//返回值:当前触屏状态.
//0,触屏无触摸;1,触屏有触摸
/* unsigned char F1C_TP_Scan(unsigned char tp)
{
	unsigned int as = 0;
	unsigned int n;
	static int status = 0; //1=按下 2抬起
	static int x = 0, y = 0;
	static unsigned char i = 0;
	int temp = 0;

	as = read32(TP_INT_FIFO_STAT_REG); //读取笔状态
	if (as & 0x2) //抬起
	{
		//printf("TP_CTRL_REG0=0x%08x\r\n",as);
		// printf("TP UP...\r\n");
		S_BIT(TP_INT_FIFO_STAT_REG, 1); //清抬起标志位
		TP_Read_XY2(0, 0); //清数据
		status = 2;
		i = 0;
	}
	if (as & 0x1)					   //按下
	{
		// printf("TP_CTRL_REG0=0x%08x\r\n",as);
		// printf("TP DOWN...\r\n");
		S_BIT(TP_INT_FIFO_STAT_REG, 0); //清按下标志位
		i = 1;
	}
	if (i)
	{
		n = (as >> 8) & 0x1f; //数据计数
		if (n >= 2)
		{
			if (TP_Read_XY2(&x, &y)) //读数据
			{
				status = 1;
			}
		}
	}

	if (status == 1) //按下后读AD值
	{
		if (tp)
		{ //读取物理坐标
			tp_dev.x[0] = x;
			tp_dev.y[0] = y;
			//printf("dev: x=%d y=%d \r\n", tp_dev.x[0], tp_dev.y[0]);
		}
		else //读取屏幕坐标
		{
			tp_dev.x[0] = x;
			tp_dev.y[0] = y;
			tp_dev.x[0] = tp_dev.xfac * tp_dev.x[0] + tp_dev.xoff; //将结果转换为屏幕坐标
			tp_dev.y[0] = tp_dev.yfac * tp_dev.y[0] + tp_dev.yoff;

			//方向转换
			if(((tp_dev.touchtype&0x06)>>1)==0) //90度
			{
				temp = tp_dev.y[0];
				tp_dev.y[0] = tp_dev.x[0];
				tp_dev.x[0] = tp_dev.width-1-temp;			
			}
			else if(((tp_dev.touchtype&0x06)>>1)==2) //270度
			{
				temp = tp_dev.x[0];
				tp_dev.x[0] = tp_dev.y[0];
				tp_dev.y[0] = tp_dev.height-1-temp;		
			}
			else if(((tp_dev.touchtype&0x06)>>1)==3) //180度
			{
				tp_dev.x[0] = tp_dev.width-1-tp_dev.x[0];
				tp_dev.y[0] = tp_dev.height-1-tp_dev.y[0];
			}
			else //0度
			{

			}
			// printf("dis: %d, %d\r\n", tp_dev.x[0],tp_dev.y[0]);
		}
		if ((tp_dev.sta & TP_PRES_DOWN) == 0) //之前没有被按下
		{
			tp_dev.sta = TP_PRES_DOWN | TP_CATH_PRES; //按键按下
			tp_dev.x[4] = tp_dev.x[0];				  //记录第一次按下时的坐标
			tp_dev.y[4] = tp_dev.y[0];
		}
	}
	else
	{
		if (tp_dev.sta & TP_PRES_DOWN) //之前是被按下的
		{
			tp_dev.sta &= ~(1 << 7); //标记按键松开
		}
		else //之前就没有被按下
		{
			tp_dev.x[4] = 0;
			tp_dev.y[4] = 0;
			tp_dev.x[0] = 0xffff;
			tp_dev.y[0] = 0xffff;
		}
	}

	return tp_dev.sta & TP_PRES_DOWN; //返回当前的触屏状态
} */

//触摸屏扫描(不带滤波)
unsigned char F1C_TP_Scan(unsigned char tp)
{
	unsigned int as = 0;
	unsigned int n;
	static int status = 0; //1=按下 2抬起
	int x = 0;
	int y = 0;
	int temp = 0;
	static unsigned char FifoErrorCnt = 0;

	as = read32(TP_INT_FIFO_STAT_REG);
	if (as & 0x1) //按下
	{
		// printf("TP_CTRL_REG0=0x%08x\r\n",as);
		// printf("TP DOWN...\r\n");
		S_BIT(TP_INT_FIFO_STAT_REG, 0);
		status = 1;
	}
	if (as & 0x2) //抬起
	{
		// printf("TP_CTRL_REG0=0x%08x\r\n",as);
		// printf("TP UP...\r\n");
		S_BIT(TP_INT_FIFO_STAT_REG, 1);
		status = 2;
	}
	if (status == 1) //按下后读AD值
	{
		// printf("TP_CTRL_REG0=0x%08x\r\n",as);
		n = (as >> 8) & 0x1f; //数据计数
		if (n == 2)
		{
			FifoErrorCnt = 0;
			TP_Read_XY(&x, &y);
			// printf("TP:n=%d x=%d y=%d \r\n", n, x, y);

			if (tp)
			{ //读取物理坐标
				tp_dev.x[0] = x;
				tp_dev.y[0] = y;
				// printf("dev: x=%d y=%d \r\n", tp_dev.x[0], tp_dev.y[0]);
			}
			else //读取屏幕坐标
			{
				tp_dev.x[0] = x;
				tp_dev.y[0] = y;
				tp_dev.x[0] = tp_dev.xfac * tp_dev.x[0] + tp_dev.xoff; //将结果转换为屏幕坐标
				tp_dev.y[0] = tp_dev.yfac * tp_dev.y[0] + tp_dev.yoff;

				//方向转换
				if (((tp_dev.touchtype & 0x06) >> 1) == 0) //90度
				{
					temp = tp_dev.y[0];
					tp_dev.y[0] = tp_dev.x[0];
					tp_dev.x[0] = tp_dev.width - 1 - temp;
				}
				else if (((tp_dev.touchtype & 0x06) >> 1) == 2) //270度
				{
					temp = tp_dev.x[0];
					tp_dev.x[0] = tp_dev.y[0];
					tp_dev.y[0] = tp_dev.height - 1 - temp;
				}
				else if (((tp_dev.touchtype & 0x06) >> 1) == 3) //180度
				{
					tp_dev.x[0] = tp_dev.width - 1 - tp_dev.x[0];
					tp_dev.y[0] = tp_dev.height - 1 - tp_dev.y[0];
				}
				else //0度
				{
				}
				// printf("dis: %d, %d\r\n", tp_dev.x[0],tp_dev.y[0]);
			}
			if ((tp_dev.sta & TP_PRES_DOWN) == 0) //之前没有被按下
			{	//TP_PRES_DOWN按键按下; TP_CATH_PRES用于松开后标记曾经有按下过(需手动清除)
				tp_dev.sta = TP_PRES_DOWN | TP_CATH_PRES; 
				tp_dev.x[4] = tp_dev.x[0];				  //记录第一次按下时的坐标
				tp_dev.y[4] = tp_dev.y[0];
			}
		}
		else
		{
			if(++FifoErrorCnt>=20) //防止硬件错误,,,此时始终N=1, 需清除
			{
				// printf("tp error\r\n");
				FifoErrorCnt = 0;
				//clear fifo flush
				write32(TP_INT_FIFO_CTRL_REG, read32(TP_INT_FIFO_CTRL_REG)|0X00000010);
			}
		}
	}
	else
	{
		FifoErrorCnt = 0;
		if (tp_dev.sta & TP_PRES_DOWN) //之前是被按下的
		{
			tp_dev.sta &= ~(1 << 7); //标记按键松开
		}
		else //之前就没有被按下
		{
			tp_dev.x[4] = 0;
			tp_dev.y[4] = 0;
			// tp_dev.x[0] = 0xffff;
			// tp_dev.y[0] = 0xffff;
		}
	}

	return tp_dev.sta & TP_PRES_DOWN; //返回当前的触屏状态
}

//触摸屏初始化
unsigned char F1C_TP_Init(void)
{
	Init_TP_ADC(TP_MODE);
	Set_ADC_Channel(ADC_all);
	return 0;
}

//——————————————————————————————————————————————————————————————————
//Demo代码
//冒泡排序
static void maopao(unsigned int a[], unsigned char n)
{
	unsigned char i, j;
	for (i = 0; i < n - 1; i++)
	{
		for (j = 0; j < n - i - 1; j++)
		{
			if (a[j] > a[j + 1])
			{
				int t = a[j];
				a[j] = a[j + 1];
				a[j + 1] = t;
			}
		}
	}
}

/*TP模式 测试*/
void TP_MODE_Demo(void)
{
	unsigned int as = 0;
	unsigned int n;
	int status = 0; //1=按下 2抬起
	int x = 0;
	int y = 0;

	printf("TP Test...\r\n");
	Init_TP_ADC(TP_MODE);
	Set_ADC_Channel(ADC_all);
	while (1)
	{
		as = read32(TP_INT_FIFO_STAT_REG);
		if (as & 0x1) //按下
		{
			// printf("TP_CTRL_REG0=0x%08x\r\n",as);
			// printf("TP DOWN...\r\n");
			S_BIT(TP_INT_FIFO_STAT_REG, 0);
			status = 1;
		}
		if (as & 0x2) //抬起
		{
			// printf("TP_CTRL_REG0=0x%08x\r\n",as);
			// printf("TP UP...\r\n");
			S_BIT(TP_INT_FIFO_STAT_REG, 1);
			status = 2;
		}
		if (status == 1) //按下后读AD值
		{
			n = (as >> 8) & 0x1f; //数据计数
			if (n >= 2)
			{
				x = TP_ADC_Read_Value();
				y = TP_ADC_Read_Value();
				printf("TP:n=%d x=%d y=%d \r\n", n, x, y);
			}
		}
	}
}

/*ADC模式 测试*/
void ADC_MODE_Demo(void)
{
	int key_vl[9] = {24, 360, 700, 1000, 1380, 1710, 2040, 2400, 2740};
	unsigned int i = 0, _x1, _x2, _y1, _y2, x1[8], x2[8], y1[8], y2[8], x1_key_f = 0, x2_key_f = 0;

	Init_TP_ADC(ADC_MODE);
	Set_ADC_Channel(ADC_all);
	printf("TP_CTRL_REG0=0x%08x\r\n", read32(TP_CTRL_REG0));
	printf("TP_CTRL_REG1=0x%08x\r\n", read32(TP_CTRL_REG1));
	printf("TP_CTRL_REG2=0x%08x\r\n", read32(TP_CTRL_REG2));
	printf("TP_CTRL_REG3=0x%08x\r\n", read32(TP_CTRL_REG3));
	while (1)
	{
		//等待FIFO满
		// i = Read_time_ms(); note
		while (!(read32(TP_INT_FIFO_STAT_REG) & (1 << 17)))
			;
		S_BIT(TP_INT_FIFO_STAT_REG, 17);
		//		printf("Time=%d us\r\n",Read_time_ms()-i);
		//读出
		for (i = 0; i < 8; i++)
		{
			x1[i] = TP_ADC_Read_Value();
			x2[i] = TP_ADC_Read_Value();
			y1[i] = TP_ADC_Read_Value();
			y2[i] = TP_ADC_Read_Value();
		}

		//【排序-中值-平均】
		maopao(x1, 8);
		maopao(x2, 8);
		maopao(y1, 8);
		maopao(y2, 8);
		_x1 = 0;
		_x2 = 0, _y1 = 0, _y2 = 0;
		for (i = 0; i < 4; i++)
		{
			_x1 += x1[2 + i];
			_x2 += x2[2 + i];
			_y1 += y1[2 + i];
			_y2 += y2[2 + i];
		}
		_x1 /= 4;
		_x2 /= 4;
		_y1 /= 4;
		_y2 /= 4;
		//
		//		printf("x1=%d\r\n",_x1);
		//		printf("x2=%d\r\n",_x2);
		//		printf("y1=%d\r\n",_y1);
		//		printf("y2=%d\r\n",_y2);
		/*x1-按键处理*/
		if ((_x1 < 3000) && (x1_key_f == 0))
		{
			x1_key_f = 1;
		}
		else if ((_x1 < 3000) && (x1_key_f == 1))
		{
			x1_key_f = 2;

			for (i = 0; i < 9; i++)
			{
				if ((_x1 > (key_vl[i] - 50)) && (_x1 < (key_vl[i] + 50)))
				{
					printf("X1-KEY DOWN-%d\r\n", i);
					break;
				}
			}
		}
		else if ((_x1 > 3000) && (x1_key_f > 1))
		{
			printf("X1-KEY UP\r\n\r\n");
			x1_key_f = 0;
		}
		/*x2-按键处理*/
		if ((_x2 < 3000) && (x2_key_f == 0))
		{
			x2_key_f = 1;
		}
		else if ((_x2 < 3000) && (x2_key_f == 1))
		{
			x2_key_f = 2;

			for (i = 0; i < 9; i++)
			{
				if ((_x2 > (key_vl[i] - 50)) && (_x2 < (key_vl[i] + 50)))
				{
					printf("X2-KEY DOWN-%d\r\n", i);
					break;
				}
			}
		}
		else if ((_x2 > 3000) && (x2_key_f > 1))
		{
			printf("X2-KEY UP\r\n\r\n");
			x2_key_f = 0;
		}
	}
}
