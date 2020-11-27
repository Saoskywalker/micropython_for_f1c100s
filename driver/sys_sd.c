#include "sys_sd.h"
#include <io.h>
#include "stdio.h"
#include "delay.h"
// #include "sys_interrupt.h"
// #include "sys_cache.h"
#include "string.h"
#include <gpio-f1c100s.h>
#include "types_plus.h"

#define SD_DBG_PRINTF 0

#define CCU_Base_Address (u32_t)0x01C20000
#define CCU_BUS_SOFT_RST_REG0 (u32_t) CCU_Base_Address + 0x02C0
#define CCU_BUS_CLK_GATING_REG0 (u32_t) CCU_Base_Address + 0x0060
#define CCU_SDMMC0_CLK_REG (u32_t) CCU_Base_Address + 0x0088
#define CCU_SDMMC1_CLK_REG (u32_t) CCU_Base_Address + 0x008c
#define SD0_BASE_ADDR (0x01C0F000)
#define SD1_BASE_ADDR (0x01C10000)

//MMC复位
#define SD_GCTRL_SOFT_RESET (0x1 << 0)
#define SD_GCTRL_FIFO_RESET (0x1 << 1)
#define SD_GCTRL_DMA_RESET (0x1 << 2)
#define SD_GCTRL_RESET (SD_GCTRL_SOFT_RESET | SD_GCTRL_FIFO_RESET | SD_GCTRL_DMA_RESET)

//卡总线宽度
#define Card_width_1bit 0
#define Card_width_4bit 1
#define Card_width_8bit 2
//卡块大小
#define Card_Block_size 512
#define SD_BLOCK_SIZE Card_Block_size

//
#define SUNXI_MMC_CMD_RESP_EXPIRE (0x1 << 6)
#define SUNXI_MMC_CMD_LONG_RESPONSE (0x1 << 7)
#define SUNXI_MMC_CMD_CHK_RESPONSE_CRC (0x1 << 8)
#define SUNXI_MMC_CMD_DATA_EXPIRE (0x1 << 9)
#define SUNXI_MMC_CMD_WRITE (0x1 << 10)
#define SUNXI_MMC_CMD_AUTO_STOP (0x1 << 12)
#define SUNXI_MMC_CMD_WAIT_PRE_OVER (0x1 << 13)
#define SUNXI_MMC_CMD_SEND_INIT_SEQ (0x1 << 15)
#define SUNXI_MMC_CMD_UPCLK_ONLY (0x1 << 21)
#define SUNXI_MMC_CMD_START ((u64_t)0x1 << 31)

#define SUNXI_MMC_CLK_ENABLE 16

//超时
#define ETIMEDOUT 0x98 //超时标志
#define SUNXI_MMC_RINT_RESP_ERROR (0x1 << 1)
#define SUNXI_MMC_RINT_COMMAND_DONE (0x1 << 2)
#define SUNXI_MMC_RINT_DATA_OVER (0x1 << 3)
#define SUNXI_MMC_RINT_TX_DATA_REQUEST (0x1 << 4)
#define SUNXI_MMC_RINT_RX_DATA_REQUEST (0x1 << 5)
#define SUNXI_MMC_RINT_RESP_CRC_ERROR (0x1 << 6)
#define SUNXI_MMC_RINT_DATA_CRC_ERROR (0x1 << 7)
#define SUNXI_MMC_RINT_RESP_TIMEOUT (0x1 << 8)
#define SUNXI_MMC_RINT_DATA_TIMEOUT (0x1 << 9)
#define SUNXI_MMC_RINT_VOLTAGE_CHANGE_DONE (0x1 << 10)
#define SUNXI_MMC_RINT_FIFO_RUN_ERROR (0x1 << 11)
#define SUNXI_MMC_RINT_HARD_WARE_LOCKED (0x1 << 12)
#define SUNXI_MMC_RINT_START_BIT_ERROR (0x1 << 13)
#define SUNXI_MMC_RINT_AUTO_COMMAND_DONE (0x1 << 14)
#define SUNXI_MMC_RINT_END_BIT_ERROR (0x1 << 15)
#define SUNXI_MMC_RINT_SDIO_INTERRUPT (0x1 << 16)
#define SUNXI_MMC_RINT_CARD_INSERT (0x1 << 30)
#define SUNXI_MMC_RINT_CARD_REMOVE (0x1 << 31)
#define SUNXI_MMC_RINT_INTERRUPT_ERROR_BIT \
	(SUNXI_MMC_RINT_RESP_ERROR |           \
	 SUNXI_MMC_RINT_RESP_CRC_ERROR |       \
	 SUNXI_MMC_RINT_DATA_CRC_ERROR |       \
	 SUNXI_MMC_RINT_RESP_TIMEOUT |         \
	 SUNXI_MMC_RINT_DATA_TIMEOUT |         \
	 SUNXI_MMC_RINT_VOLTAGE_CHANGE_DONE |  \
	 SUNXI_MMC_RINT_FIFO_RUN_ERROR |       \
	 SUNXI_MMC_RINT_HARD_WARE_LOCKED |     \
	 SUNXI_MMC_RINT_START_BIT_ERROR |      \
	 SUNXI_MMC_RINT_END_BIT_ERROR) /* 0xbfc2 */

#define SUNXI_MMC_GCTRL_ACCESS_BY_AHB 31

#define SUNXI_MMC_STATUS_FIFO_EMPTY (0x1 << 2)
#define SUNXI_MMC_STATUS_FIFO_FULL (0x1 << 3)

#define SUNXI_MMC_STATUS_CARD_DATA_BUSY (0x1 << 9)
#define SUNXI_MMC_CMD_RESP_EXPIRE (0x1 << 6)
#define SUNXI_MMC_CMD_LONG_RESPONSE (0x1 << 7)
#define SUNXI_MMC_CMD_CHK_RESPONSE_CRC (0x1 << 8)

#define MMC_RSP_PRESENT (1 << 0)
#define MMC_RSP_136 (1 << 1)	/* 136 bit response */
#define MMC_RSP_CRC (1 << 2)	/* expect valid crc */
#define MMC_RSP_BUSY (1 << 3)   /* card may send busy */
#define MMC_RSP_OPCODE (1 << 4) /* response contains opcode */

#define MMC_DATA_READ 1
#define MMC_DATA_WRITE 2

#define SUNXI_MMC_CLK_DIVIDER_MASK (0xff)

//CCU SD时钟设置相关
#define CCM_MMC_CTRL_ENABLE ((u64_t)0x1 << 31)
#define CCM_MMC_CTRL_M(x) ((x)-1)
#define CCM_MMC_CTRL_OCLK_DLY(x) ((x) << 8)
#define CCM_MMC_CTRL_N(x) ((x) << 16)
#define CCM_MMC_CTRL_SCLK_DLY(x) ((x) << 20)
//SD时钟选择位
#define CCM_MMC_CTRL_OSCM24 (0x0 << 24)
#define CCM_MMC_CTRL_PLL6 (0x1 << 24)
//返回时钟
#define CCM_PLL6_CTRL_N_SHIFT 8
#define CCM_PLL6_CTRL_N_MASK (0x1f << CCM_PLL6_CTRL_N_SHIFT)
#define CCM_PLL6_CTRL_K_SHIFT 4
#define CCM_PLL6_CTRL_K_MASK (0x3 << CCM_PLL6_CTRL_K_SHIFT)
//卡类型
#define CARD_TYPE_UNKNOWN 0
#define CARD_TYPE_SD_HIGH 1
#define CARD_TYPE_SD_LOW 2
#define CARD_TYPE_MMC 3
//DMA发送接收状态
#define DMA_TX_Finished 1
#define DMA_RX_Finished 2
//MMC SD 描述符结构体
struct mmc_des_v4p1
{
	u32 : 1,			  //null
		dic : 1,		  /* disable interrupt on completion */
		last_des : 1,	 /* 1-this data buffer is the last buffer */
		first_des : 1,	/* 1-data buffer is the first buffer,0-data buffer contained in the next descriptor is 1st buffer */
		des_chain : 1,	/* 1-the 2nd address in the descriptor is the next descriptor address */
		end_of_ring : 1,  /* 1-last descriptor flag when using dual data buffer in descriptor */
		: 24,			  //null
		card_err_sum : 1, /* transfer error flag */
		own : 1;		  /* des owner:1-idma owns it, 0-host owns it */

#define SDXC_DES_NUM_SHIFT 12 /* smhc2!! */
#define SDXC_DES_BUFFER_MAX_LEN (1 << SDXC_DES_NUM_SHIFT)

	u32 data_buf1_sz : 16,
		data_buf2_sz : 16;

	u32 buf_addr_ptr1;
	u32 buf_addr_ptr2;
};
/*描述符表*/
#define DES_NUM 256
struct mmc_des_v4p1 _pdes[DES_NUM];

struct mmc_cmd
{
	unsigned short cmdidx;
	unsigned int resp_type;
	unsigned int cmdarg;
	unsigned int response[4];
};

struct mmc_data
{
	union {
		unsigned char *dest;
		unsigned char *src; /* src buffers don't get written to */
	} b;
	unsigned int flags;
	unsigned int blocks;
	unsigned int blocksize;
};

DISK_DATA_T SD_INFO;

unsigned int DMA_TR_SD; //DMA中断收发完成标志
unsigned int CARD_TYPE = 0;
unsigned int CARD_uRCA = 0;
unsigned int CIDBuffer[4];

unsigned char __attribute__((aligned(4096))) _card_ucSDHCBuffer[64]; //卡设置缓存数据

void flush_cache(unsigned long start, unsigned long size);
static void _OS_Time(void);
void SD0_ISR(void); /*SD中断0*/
void SD1_ISR(void); /*SD中断1*/

/*SD IO 初始化*/
void SD_IO_Init(int SDinx)
{
	int i = 0;
	if (SDinx == SD0)
	{
		for (i = 0; i < 6; i++)
			gpio_f1c100s_set_cfg(&GPIO_PF, 0 + i, 2); //PF0-PF5      6线
		gpio_f1c100s_set_pull(&GPIO_PF, 2, GPIO_PULL_UP);
	}
	else if (SDinx == SD1)
		for (i = 0; i < 3; i++)
			gpio_f1c100s_set_cfg(&GPIO_PC, 0 + i, 3); //PC0-PC2 3线
}

/*返回基地址*/
unsigned int Get_SDaddr(int SDinx)
{
	if (SDinx == SD0)
		return SD0_BASE_ADDR;
	else if (SDinx == SD1)
		return SD1_BASE_ADDR;
	else
		return 0;
}

/*校验参数*/
//--- 2014/3/27, check the sector number is valid or not for current SD card.
unsigned int g_max_valid_sector; // The max valid sector number for current SD card.
int fmiSDCheckSector(u32 uSector, u32 uBufcnt)
{
	if ((uSector + uBufcnt - 1) > g_max_valid_sector)
	{
		printf("ERROR: Fail to access invalid sector number %d from SD card !!\r\n", uSector + uBufcnt - 1);
		printf("       The max valid sector number for current SD card is %d.\r\n", g_max_valid_sector);
		return -0x3565d; // invalid sector
	}
	return 0; // valid sector
}
/*等待
timeout_msecs 超时
done_bit 等待位
*/
int mmc_rint_wait(int SDinx, unsigned int timeout_msecs, unsigned int done_bit)
{
	unsigned int status;
	unsigned int SD_ADDR = 0;
	SD_ADDR = Get_SDaddr(SDinx); //返回地址
	timeout_msecs *= 1000; 
	do
	{
		status = read32(SD_ADDR + 0x038);
		if (!timeout_msecs-- || (status & SUNXI_MMC_RINT_INTERRUPT_ERROR_BIT))
		{
			printf("ERR: mmc%d tmier=[%d] timeout %x\r\n", SDinx, timeout_msecs, status & SUNXI_MMC_RINT_INTERRUPT_ERROR_BIT);
			return -ETIMEDOUT;
		}
		delay_us(1);
	} while (!(status & done_bit));

	return 0;
}

/*更新时钟*/
int mmc_update_clk(int SDinx)
{
	unsigned int cmd;
	unsigned int timeout_msecs = 2000;
	unsigned int SD_ADDR = 0;
#if SD_DBG_PRINTF
	printf("mmc_update_clk...\r\n");
#endif
	SD_ADDR = Get_SDaddr(SDinx); //返回地址

	cmd = SUNXI_MMC_CMD_START | SUNXI_MMC_CMD_UPCLK_ONLY | SUNXI_MMC_CMD_WAIT_PRE_OVER;
	write32(SD_ADDR + 0x018, cmd);
	//等待完成
	while (read32(SD_ADDR + 0x018) & SUNXI_MMC_CMD_START)
	{
		if (!timeout_msecs--)
		{
			printf("timeout...\r\n");
			return -1;
		}
		delay_us(1);
	}
	//清状态位

	return 0;
}

/*返回时钟*/
unsigned int clock_get_pll6(void)
{
	uint32_t rval = read32(CCU_Base_Address + 0x28);
	//int n = ((rval & CCM_PLL6_CTRL_N_MASK) >> CCM_PLL6_CTRL_N_SHIFT);
	//int k = ((rval & CCM_PLL6_CTRL_K_MASK) >> CCM_PLL6_CTRL_K_SHIFT) + 1;
	// return 24000000 * n * k / 2; //原, 有错误
	int n = ((rval & CCM_PLL6_CTRL_N_MASK) >> CCM_PLL6_CTRL_N_SHIFT) + 1;
	int k = ((rval & CCM_PLL6_CTRL_K_MASK) >> CCM_PLL6_CTRL_K_SHIFT) + 1;
	int p = ((rval >> 16) & 0x3) + 1;
	return 24000000 * n * k / p;
}

/*配置时钟*/
int mmc_set_mod_clk(int SDinx, unsigned int hz)
{
	unsigned int pll, pll_hz, div, n, oclk_dly, sclk_dly;

	if (hz <= 24000000)
	{
		pll = CCM_MMC_CTRL_OSCM24;
		pll_hz = 24000000;
	}
	else
	{
		pll = CCM_MMC_CTRL_PLL6;
		pll_hz = clock_get_pll6();
	}

	div = pll_hz / hz;
	if (pll_hz % hz)
		div++;

	n = 0;
	while (div > 16)
	{
		n++;
		div = (div + 1) / 2;
	}

	if (n > 3)
	{
		printf("mmc %u error cannot set clock to %u\r\n", SDinx, hz);
		return -1;
	}

	/* determine delays */
	if (hz <= 400000)
	{
		oclk_dly = 0;
		sclk_dly = 0;
	}
	else if (hz <= 25000000)
	{
		oclk_dly = 0;
		sclk_dly = 5;
	}
	else if (hz <= 50000000)
	{
		oclk_dly = 3;
		sclk_dly = 4;
	}
	else
	{
		/* hz > 50000000 */
		oclk_dly = 1;
		sclk_dly = 4;
	}

	write32(CCU_SDMMC0_CLK_REG + SDinx * 0x4, CCM_MMC_CTRL_ENABLE | pll |
												  CCM_MMC_CTRL_SCLK_DLY(sclk_dly) |
												  CCM_MMC_CTRL_N(n) |
												  CCM_MMC_CTRL_OCLK_DLY(oclk_dly) |
												  CCM_MMC_CTRL_M(div));

#if SD_DBG_PRINTF
	printf("mmc %u set [mod-clk req %u]-[parent %u]-[n %u]-[m %u]-[rate %u]\r\n",
		   SDinx, hz, pll_hz, 1u << n, div,
		   pll_hz / (1u << n) / div);
#endif
	return 0;
}
/*设置卡时钟空闲时无*/
int Card_IDLE_Clock_OFF(int SDinx)
{
	unsigned int SD_ADDR = 0;
	SD_ADDR = Get_SDaddr(SDinx); //返回地址
	S_BIT(SD_ADDR + 0x004, 17);
	return 0;
}
/*时钟开*/
int mmc_clock_enable(int SDinx)
{
	unsigned int SD_ADDR = 0;
	SD_ADDR = Get_SDaddr(SDinx); //返回地址
	S_BIT(SD_ADDR + 0x004, SUNXI_MMC_CLK_ENABLE);
	if (mmc_update_clk(SDinx) != 0)
		return -1;
	return 0;
}
/*时钟关*/
int mmc_clock_disable(int SDinx)
{
	unsigned int SD_ADDR = 0;
	SD_ADDR = Get_SDaddr(SDinx); //返回地址
	C_BIT(SD_ADDR + 0x004, SUNXI_MMC_CLK_ENABLE);
	if (mmc_update_clk(SDinx) != 0)
		return -1;
	return 0;
}
/*关data3 卡检测脚*/
int mmc_close_Card_Detectint(int SDinx)
{
	unsigned int SD_ADDR = 0;
	SD_ADDR = Get_SDaddr(SDinx); //返回地址
	C_BIT(SD_ADDR + 0x000, 8);
	return 0;
}

/*配置更新时钟*/
int mmc_config_clock(int SDinx, unsigned int hz)
{
	unsigned int SD_ADDR = 0;
	unsigned int rval = 0;

	SD_ADDR = Get_SDaddr(SDinx); //返回地址

	/* Disable Clock */
	if (mmc_clock_disable(SDinx) != 0)
	{
		printf("err: mmc_clock_disable\r\n");
		return -1;
	}
	//

	/* 设置新时钟 */
	if (mmc_set_mod_clk(SDinx, hz) != 0)
	{
		printf("err: mmc_set_mod_clk\r\n");
		return -1;
	}
	/* 清内部分频器为0 */
	rval = read32(SD_ADDR + 0x004);
	rval &= ~SUNXI_MMC_CLK_DIVIDER_MASK;
	write32(SD_ADDR + 0x004, rval);

	/* Re-enable Clock */
	if (mmc_clock_enable(SDinx) != 0)
	{
		printf("err: mmc_clock_enable\r\n");
		return -1;
	}
	printf("mmc_config_clock: %d ok...\r\n", hz);
	return 0;
}

//控制器复位
int sunxi_mmc_core_init(int SDinx)
{
	unsigned int SD_ADDR = 0;

	SD_ADDR = Get_SDaddr(SDinx); //返回地址
	write32(SD_ADDR + 0x000, read32(SD_ADDR + 0x000) | SD_GCTRL_RESET);
	delay_us(1000);

	return 0;
}

/*传输数据通过CPU
data 数据指针 
*/
int mmc_trans_data_by_cpu(int SDinx, struct mmc_data *data)
{
	unsigned int reading = !!(data->flags & MMC_DATA_READ);
	unsigned int status_bit = reading ? SUNXI_MMC_STATUS_FIFO_EMPTY : SUNXI_MMC_STATUS_FIFO_FULL; //读空等待	//写满等待
	unsigned int *buff = (unsigned int *)(reading ? data->b.dest : data->b.src);
	unsigned int byte_cnt = data->blocksize * data->blocks;
	unsigned int timeout_usecs = (byte_cnt >> 8) * 1000;

	unsigned int i;
	unsigned int SD_ADDR = 0;
#if SD_DBG_PRINTF
	printf("Trans data by cpu...\r\n");
	if (reading == 1)
		printf("Data Read...\r\n");
	else
		printf("Data Write...\r\n");
#endif

	SD_ADDR = Get_SDaddr(SDinx); //返回地址

	if (timeout_usecs < 2000000)
		timeout_usecs = 2000000;

	/* CPU方式读写数据*/
	S_BIT(SD_ADDR + 0x000, SUNXI_MMC_GCTRL_ACCESS_BY_AHB);

	for (i = 0; i < (byte_cnt >> 2); i++)
	{
		while (read32(SD_ADDR + 0x03c) & status_bit)
		{
			if (!timeout_usecs--)
			{
				printf("Data[%d] timeout BIT=0x%08x...\r\n", i, read32(SD_ADDR + 0x03c));
				return -1;
			}

			delay_us(1);
		}
		//读写
		if (reading) //读
			buff[i] = read32(SD_ADDR + 0x200);
		else
			write32(SD_ADDR + 0x200, buff[i]);
	}

	return 0;
}
/*传输数据通过DMA
data 数据指针 
pdes 表指针*/
int mmc_trans_data_by_dma(int SDinx, struct mmc_data *data, struct mmc_des_v4p1 *pdes)
{
	unsigned byte_cnt = data->blocksize * data->blocks;
	unsigned char *buff;
	unsigned des_idx = 0;
	unsigned buff_frag_num = 0;
	unsigned remain;
	unsigned i, rval;
	unsigned int SD_ADDR = 0;
#if SD_DBG_PRINTF
	printf("Trans data by dma...\r\n");
	if ((data->flags & MMC_DATA_READ) == 1)
		printf("Data Read...\r\n");
	else
		printf("Data Write...\r\n");
#endif
	SD_ADDR = Get_SDaddr(SDinx);

	buff = data->flags & MMC_DATA_READ ? (unsigned char *)data->b.dest : (unsigned char *)data->b.src;

	buff_frag_num = byte_cnt >> SDXC_DES_NUM_SHIFT;	//除4096取整，描述表个数
	remain = byte_cnt & (SDXC_DES_BUFFER_MAX_LEN - 1); //取余数 ，有余数则描述表+1个

	if (remain) //有余数
		buff_frag_num++;
	else //是整数
		remain = SDXC_DES_BUFFER_MAX_LEN;

	if (buff_frag_num > DES_NUM)
		return -1; /*描述符超出定义数*/

	//开启cache后刷新缓存
	flush_cache((unsigned long)buff, (unsigned long)byte_cnt);

	//写入描述表
	for (i = 0; i < buff_frag_num; i++, des_idx++)
	{
		//清0
		memset((void *)&pdes[des_idx], 0, sizeof(struct mmc_des_v4p1));
		//
		pdes[des_idx].des_chain = 1; //置1表示第二个地址为下一个表地址
		pdes[des_idx].own = 1;		 //传输结束位清除
		pdes[des_idx].dic = 1;		 //使能中断
		//写入缓存大小
		if (buff_frag_num > 1 && i != buff_frag_num - 1) //为整
			pdes[des_idx].data_buf1_sz = SDXC_DES_BUFFER_MAX_LEN;
		else //为余
			pdes[des_idx].data_buf1_sz = remain;
		//写入BUFF地址
		pdes[des_idx].buf_addr_ptr1 = (unsigned long)buff + i * SDXC_DES_BUFFER_MAX_LEN;
		//写入第一个表标志符
		if (i == 0)
			pdes[des_idx].first_des = 1;
		//写入下一个表地址，否则写入结束标志
		if (i == buff_frag_num - 1) //表结束
		{
			pdes[des_idx].dic = 0;			 //中断关
			pdes[des_idx].last_des = 1;		 //最后一个BUFF
			pdes[des_idx].end_of_ring = 1;   //最后一个描述表
			pdes[des_idx].buf_addr_ptr2 = 0; //最后一个地址空
		}
		else //下一表地址
		{
			pdes[des_idx].buf_addr_ptr2 = (unsigned long)&pdes[des_idx + 1];
		}
		//打印表
#if SD_DBG_PRINTF
		printf("frag %d, remain %d, des[%d](%08x): "
			   "[0] = %08x, [1] = %08x, [2] = %08x, [3] = %08x\r\n",
			   i, remain, des_idx, (u32)&pdes[des_idx],
			   (u32)((u32 *)&pdes[des_idx])[0], (u32)((u32 *)&pdes[des_idx])[1],
			   (u32)((u32 *)&pdes[des_idx])[2], (u32)((u32 *)&pdes[des_idx])[3]);
#endif
	}

	//开启cache后刷新缓存
	flush_cache((unsigned long)pdes, sizeof(struct mmc_des_v4p1) * (des_idx + 1));

	/*****************************************************************************
下面进入寄存器设置相关*/
	//	__asm("DSB");   chang by A-GAN
	//	__asm("ISB");

	/*
	 * GCTRLREG
	 * GCTRL[2]	: DMA reset
	 * GCTRL[5]	: DMA enable
	 *
	 * IDMACREG
	 * IDMAC[0]	: IDMA soft reset
	 * IDMAC[1]	: IDMA fix burst flag
	 * IDMAC[7]	: IDMA on
	 *
	 * IDIECREG
	 * IDIE[0]	: IDMA transmit interrupt flag
	 * IDIE[1]	: IDMA receive interrupt flag
	 */

	/* DMA方式读写数据*/
	C_BIT(SD_ADDR + 0x000, SUNXI_MMC_GCTRL_ACCESS_BY_AHB);
	//
	rval = read32(SD_ADDR + 0x00);
	write32(SD_ADDR + 0x00, rval | (1 << 5) | (1 << 2)); /* dma enable */
	write32(SD_ADDR + 0x80, (1 << 0));					 /* idma reset */
	while (read32(SD_ADDR + 0x80) & 0x1)
	{
	}; /* wait idma reset done */

	write32(SD_ADDR + 0x80, (1 << 1) | (1 << 7)); /* idma on */

	/*读写中断标志*/
	rval = read32(SD_ADDR + 0x8c) & (~3);
	if (data->flags & MMC_DATA_WRITE)
		rval |= (1 << 0); //发送中断
	else
		rval |= (1 << 1); //接收中断
	write32(SD_ADDR + 0x8c, rval);

	/*写入表地址*/
	write32(SD_ADDR + 0x84, (unsigned long)pdes);
	/*写入-trigger level*/
	write32(SD_ADDR + 0x40, (2U << 28) | (7U << 16) | 8);
	//
	return 0;
}
/* 设置总线宽度 
#define Card_width_1bit		  0
#define Card_width_4bit			1
#define Card_width_8bit			2
*/
int Change_bus_width(int SDinx, unsigned int width)
{
	unsigned int SD_ADDR = 0;
	SD_ADDR = Get_SDaddr(SDinx); //返回地址
	write32(SD_ADDR + 0x00c, width);
	return 0;
}
/*返回DMA发送接收状态*/
unsigned int Get_Dma_TR_S(int SDinx)
{
	int DMA_S = 0, Seg;
	unsigned int SD_ADDR = 0;
	SD_ADDR = Get_SDaddr(SDinx); //返回地址
	Seg = read32(SD_ADDR + 0x88);
	if (Seg & DMA_TX_Finished)
		DMA_S |= DMA_TX_Finished;
	if (Seg & DMA_RX_Finished)
		DMA_S |= DMA_RX_Finished;

	if (DMA_S > 0)
		write32(SD_ADDR + 0x88, read32(SD_ADDR + 0x88) | DMA_S);

	return DMA_S;
}
/*
cmd  命令指针
data 数据指针 
*/
int sunxi_mmc_send_cmd(int SDinx, struct mmc_cmd *cmd, struct mmc_data *data)
{
	unsigned char *buff_;
	unsigned byte_cnt_ = 0;
	unsigned int cmdval = SUNXI_MMC_CMD_START;
	unsigned int timeout_msecs;
	unsigned int status = 0;
	unsigned int bytecnt = 0;
	unsigned int SD_ADDR = 0;
	int usedma = 0; //DMA使用标志
	int error = 0;

	SD_ADDR = Get_SDaddr(SDinx); //返回地址

	if (cmd->cmdidx == 12)
		return 0;

	if (!cmd->cmdidx)
		cmdval |= SUNXI_MMC_CMD_SEND_INIT_SEQ; //发送初始化
	if (cmd->resp_type & MMC_RSP_PRESENT)
		cmdval |= SUNXI_MMC_CMD_RESP_EXPIRE; //命令响应
	if (cmd->resp_type & MMC_RSP_136)
		cmdval |= SUNXI_MMC_CMD_LONG_RESPONSE; //长响应
	if (cmd->resp_type & MMC_RSP_CRC)
		cmdval |= SUNXI_MMC_CMD_CHK_RESPONSE_CRC; //校验响应CRC

	//设置数据相关标志
	if (data)
	{
		if ((u32)(long)data->b.dest & 0x3)
		{
			error = -1;
			printf("Addr Align Err ...\r\n");
			goto out;
		}

		//设置标志位
		cmdval |= SUNXI_MMC_CMD_DATA_EXPIRE | SUNXI_MMC_CMD_WAIT_PRE_OVER;

		if (data->flags & MMC_DATA_WRITE)
			cmdval |= SUNXI_MMC_CMD_WRITE; //数据写
		if (data->blocks > 1)
			cmdval |= SUNXI_MMC_CMD_AUTO_STOP; //自动停止

		//设置块大小，计数
		write32(SD_ADDR + 0x010, data->blocksize); //块大小
		write32(SD_ADDR + 0x014, data->blocks * data->blocksize);
	}
#if SD_DBG_PRINTF
	printf("SEG 0x18=%08x\r\n", cmdval | cmd->cmdidx);
#endif
	//设置命令内容寄存器
	write32(SD_ADDR + 0x01c, cmd->cmdarg);

	//命令发送
	if (!data)
		write32(SD_ADDR + 0x018, cmdval | cmd->cmdidx);

	//数据发送
	/*
	 * transfer data and check status
	 * STATREG[2] : FIFO empty
	 * STATREG[3] : FIFO full
	 */
	if (data)
	{
		int ret = 0;

		bytecnt = data->blocksize * data->blocks;
#if SD_DBG_PRINTF
		printf("trans data %d bytes\r\n", bytecnt);
#endif
		if (bytecnt > 64)
		{
			usedma = 1; //DMA使用标志
			mmc_trans_data_by_dma(SDinx, data, _pdes);
			write32(SD_ADDR + 0x018, cmdval | cmd->cmdidx); /*写入命令寄存器*/
		}
		else
		{
			usedma = 0;
			write32(SD_ADDR + 0x018, cmdval | cmd->cmdidx); /*写入命令寄存器*/
			ret = mmc_trans_data_by_cpu(SDinx, data);		/*传送数据*/
		}
		if (ret) //错误返回
		{
			error = read32(SD_ADDR + 0x038) & SUNXI_MMC_RINT_INTERRUPT_ERROR_BIT;
			printf("error %08x \r\n", error);
			error = -ETIMEDOUT;
			goto out;
		}
	}
	//等待命令完成，超时返回
	error = mmc_rint_wait(SDinx, 1000, SUNXI_MMC_RINT_COMMAND_DONE);
	if (error)
		goto out;
	//
	//等待dma完成
	if (usedma == 1)
	{
#if SD_DBG_PRINTF
		printf("Cal OS...\r\n");
#endif

		//		while(DMA_TR_SD==0) _OS_Time();//如果有系统这里调度
		//    DMA_TR_SD=0;

		while (Get_Dma_TR_S(SDinx) == 0)
			_OS_Time(); //如果有系统这里调度
	}
	//等待数据完成或自动命令完成
	if (data)
	{
		timeout_msecs = 120;
#if SD_DBG_PRINTF
		printf("Wait CONNADN_DONE or DATA_OVER\r\n");
#endif
		//
		error = mmc_rint_wait(SDinx, timeout_msecs, data->blocks > 1 ? SUNXI_MMC_RINT_AUTO_COMMAND_DONE : SUNXI_MMC_RINT_DATA_OVER);
		if (error)
			goto out; //超时返回
	}
	//总线忙判断
	if (cmd->resp_type & MMC_RSP_BUSY)
	{
		timeout_msecs = 2000*1000;
		do
		{
			status = read32(SD_ADDR + 0x03c);
			if (!timeout_msecs--)
			{
				printf("busy timeout\r\n");
				error = -ETIMEDOUT;
				goto out;
			}
			delay_us(1);
		} while (status & SUNXI_MMC_STATUS_CARD_DATA_BUSY);
	}
	//读响应
	if (cmd->resp_type & MMC_RSP_136) //长响应
	{
		cmd->response[0] = read32(SD_ADDR + 0x020);
		cmd->response[1] = read32(SD_ADDR + 0x024);
		cmd->response[2] = read32(SD_ADDR + 0x028);
		cmd->response[3] = read32(SD_ADDR + 0x02c);
#if SD_DBG_PRINTF
		printf("mmc resp[0]=0x%08x\r\n", cmd->response[0]);
		printf("mmc resp[1]=0x%08x\r\n", cmd->response[1]);
		printf("mmc resp[2]=0x%08x\r\n", cmd->response[2]);
		printf("mmc resp[3]=0x%08x\r\n", cmd->response[3]);
#endif
	}
	else //短
	{
		cmd->response[0] = read32(SD_ADDR + 0x020);
		cmd->response[1] = read32(SD_ADDR + 0x024);
#if SD_DBG_PRINTF
		printf("mmc resp[0]=0x%08x\r\n", cmd->response[0]);
#endif
	}

out:
	if (error < 0) //错误复位控制器
	{
		printf("err_reset...\r\n");
		write32(SD_ADDR + 0x000, SD_GCTRL_RESET);
		mmc_update_clk(SDinx);
	}

	if (data && usedma)
	{
		write32(SD_ADDR + 0x088, read32(SD_ADDR + 0x088));
		write32(SD_ADDR + 0x08C, 0);
		write32(SD_ADDR + 0x080, 0);
	}

	write32(SD_ADDR + 0x038, read32(SD_ADDR + 0x038));
	write32(SD_ADDR + 0x000, read32(SD_ADDR + 0x000) | SD_GCTRL_FIFO_RESET | SD_GCTRL_DMA_RESET);
	//数据清到内存上
	if (data && (data->flags & MMC_DATA_READ))
	{
		buff_ = (unsigned char *)data->b.dest;
		byte_cnt_ = data->blocksize * data->blocks;
		flush_cache((unsigned long)buff_, (unsigned long)byte_cnt_);
#if SD_DBG_PRINTF
		printf("invald cache after read complete\r\n");
#endif
	}

	return error;
}
/*SD硬件复位*/
void SD_Hardware_Reset(int SDinx)
{
	unsigned int SD_ADDR = 0;
	SD_ADDR = Get_SDaddr(SDinx);
	S_BIT(SD_ADDR + 0x078, 1);
	C_BIT(SD_ADDR + 0x078, 1);
	delay_us(1000);
	S_BIT(SD_ADDR + 0x078, 1);
	delay_us(1000);
}

/*发送命令*/
int SD_CMD_SEND(int SDinx, struct mmc_cmd *cmd, struct mmc_data *data, unsigned short Cmdidx, unsigned int Cmdarg, unsigned int Resp_type)
{
	cmd->cmdidx = Cmdidx;
	cmd->cmdarg = Cmdarg;
	cmd->resp_type = Resp_type;
	return sunxi_mmc_send_cmd(SDinx, cmd, data);
}

/*返SD信息*/
void Get_SD_info(int SDinx, struct mmc_cmd *cmd, DISK_DATA_T *_info)
{
	unsigned int i;
	unsigned int R_LEN, C_Size, MULT, size;
	unsigned int Buffer[4];
	unsigned char *ptr;
	/*读CSD*/
	SD_CMD_SEND(SDinx, cmd, NULL, 9, CARD_uRCA, MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_136);
	Buffer[0] = cmd->response[3];
	Buffer[1] = cmd->response[2];
	Buffer[2] = cmd->response[1];
	Buffer[3] = cmd->response[0];
	//
	if ((Buffer[0] & 0xc0000000) && (CARD_TYPE != CARD_TYPE_MMC))
	{
		C_Size = ((Buffer[1] & 0x0000003f) << 16) | ((Buffer[2] & 0xffff0000) >> 16);
		size = (C_Size + 1) * 512; // Kbytes

		_info->diskSize = size;
		_info->totalSectorN = size << 1;
	}
	else
	{
		R_LEN = (Buffer[1] & 0x000f0000) >> 16;
		C_Size = ((Buffer[1] & 0x000003ff) << 2) | ((Buffer[2] & 0xc0000000) >> 30);
		MULT = (Buffer[2] & 0x00038000) >> 15;
		size = (C_Size + 1) * (1 << (MULT + 2)) * (1 << R_LEN);

		_info->diskSize = size / 1024;
		_info->totalSectorN = size / 512;
	}
	g_max_valid_sector = _info->totalSectorN;
	_info->sectorSize = 512;

	/*读CID*/
	SD_CMD_SEND(SDinx, cmd, NULL, 10, CARD_uRCA, MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_136);
	Buffer[0] = cmd->response[3];
	Buffer[1] = cmd->response[2];
	Buffer[2] = cmd->response[1];
	Buffer[3] = cmd->response[0];
	//
	_info->vendor[0] = (Buffer[0] & 0xff000000) >> 24;
	ptr = (unsigned char *)Buffer;
	ptr = ptr + 4;
	for (i = 0; i < 5; i++)
		_info->product[i] = *ptr++;
	ptr = ptr + 10;
	for (i = 0; i < 4; i++)
		_info->serial[i] = *ptr++;
	//

	printf("SD_info print...\r\n");
	printf("SD_info->diskSize=[%d]\r\n", _info->diskSize);
	printf("SD_info->totalSectorN=[%d]\r\n", _info->totalSectorN);
	printf("SD_info->sectorSize=[%d]\r\n", _info->sectorSize);
}
/*设置卡到高速模式*/
int CardSwitchToHighSpeed(int SDinx, struct mmc_cmd *cmd)
{
	struct mmc_data data;
	int volatile status = -1;
	unsigned short current_comsumption,
		//			fun1_info, switch_status,
		busy_status0, busy_status1;

	//CMD6
	data.blocks = 1;
	data.blocksize = 64; //8*64=512位
	data.flags = MMC_DATA_READ;
	data.b.dest = _card_ucSDHCBuffer;
	if ((status = SD_CMD_SEND(SDinx, cmd, &data, 6, 0x00ffff01, MMC_RSP_PRESENT | MMC_RSP_CRC)) != Successful)
		return status;

	//
	current_comsumption = _card_ucSDHCBuffer[0] << 8 | _card_ucSDHCBuffer[1];
	if (!current_comsumption)
		return Fail;

	//    fun1_info =  _card_ucSDHCBuffer[12]<<8 | _card_ucSDHCBuffer[13];
	//    switch_status = _card_ucSDHCBuffer[16] & 0xf;//等于1为支持高速模式
	busy_status0 = _card_ucSDHCBuffer[28] << 8 | _card_ucSDHCBuffer[29];

	if (!busy_status0) // function ready
	{
		//CMD6
		data.blocks = 1;
		data.blocksize = 64; //8*64=512位
		data.flags = MMC_DATA_READ;
		data.b.dest = _card_ucSDHCBuffer;
		if ((status = SD_CMD_SEND(SDinx, cmd, &data, 6, 0x80ffff01, MMC_RSP_PRESENT | MMC_RSP_CRC)) != Successful)
			return status;
		//
		current_comsumption = _card_ucSDHCBuffer[0] << 8 | _card_ucSDHCBuffer[1];
		if (!current_comsumption)
			return Fail;

		busy_status1 = _card_ucSDHCBuffer[28] << 8 | _card_ucSDHCBuffer[29];

		if (!busy_status1)
			printf("switch into high speed mode !!!\r\n");

		return Successful;
	}
	else
		return Fail;
}

/*选择卡，设置各项参数 CARD_WIDTH=卡宽度*/
int SelectCard(int SDinx, struct mmc_cmd *cmd, char CARD_WIDTH)
{
	struct mmc_data data;
	int volatile status = -1;
	printf("SelectCard...\r\n");
	//CMD7，使SD卡进Transfer状态
	if ((status = SD_CMD_SEND(SDinx, cmd, NULL, 7, CARD_uRCA, MMC_RSP_PRESENT | MMC_RSP_CRC)) != Successful)
		return status;

	// 设置位宽
	if (CARD_TYPE == CARD_TYPE_SD_HIGH)
	{
		printf("CARD_TYPE_SD_HIGH \r\n");
		//读SCR寄存器
		//SD数据线默认1位宽度传输
		if ((status = SD_CMD_SEND(SDinx, cmd, NULL, 55, CARD_uRCA, MMC_RSP_PRESENT | MMC_RSP_CRC)) != Successful)
			return status;
		data.blocks = 1;
		data.blocksize = 8; //8*8=64位
		data.flags = MMC_DATA_READ;
		data.b.dest = _card_ucSDHCBuffer;
		if ((status = SD_CMD_SEND(SDinx, cmd, &data, 51, 0, MMC_RSP_PRESENT | MMC_RSP_CRC)) != Successful)
			return status;
#if SD_DBG_PRINTF
		printf("buf=0x");
		for (int i = 0; i < 8; i++)
			printf("%02x", _card_ucSDHCBuffer[i]);
		printf(" \r\n");
#endif
		//
		if ((_card_ucSDHCBuffer[0] & 0xf) == 0x2) //判断SD版本56-59位
		{
			//设置卡到高速模式 1位总线最高到20MHZ,如果设置大于12M只能设置到12M，因为分频只能是2
			status = CardSwitchToHighSpeed(SDinx, cmd);
			//设置控制器到高速
			if (status == Successful)
			{
				mmc_config_clock(SDinx, 144000000);
			}
		}
		//设置SD卡到4位
		if (CARD_WIDTH == Card_width_4bit)
		{
			printf("Card_width_4bit \r\n");
			//设置CARD位宽  参数=0x2为4位，00为1位
			if ((status = SD_CMD_SEND(SDinx, cmd, NULL, 55, CARD_uRCA, MMC_RSP_PRESENT | MMC_RSP_CRC)) != Successful)
				return status;
			if ((status = SD_CMD_SEND(SDinx, cmd, NULL, 6, 0x02, MMC_RSP_PRESENT | MMC_RSP_CRC)) != Successful)
				return status;
			//设置控制器到4位
			Change_bus_width(SDinx, Card_width_4bit); /* 设置总线宽度*/
		}
	}
	else if (CARD_TYPE == CARD_TYPE_SD_LOW)
	{
		printf("CARD_TYPE_SD_LOW \r\n");
		//设置控制器到高速
		mmc_config_clock(SDinx, 12000000);
		//设置SD卡到4位
		if (CARD_WIDTH == Card_width_4bit)
		{
			printf("Card_width_4bit \r\n");
			//设置位宽  参数=0x2为4位，00为1位
			if ((status = SD_CMD_SEND(SDinx, cmd, NULL, 55, CARD_uRCA, MMC_RSP_PRESENT | MMC_RSP_CRC)) != Successful)
				return status;
			if ((status = SD_CMD_SEND(SDinx, cmd, NULL, 6, 0x02, MMC_RSP_PRESENT | MMC_RSP_CRC)) != Successful)
				return status;
			//设置控制器到4位
			Change_bus_width(SDinx, Card_width_4bit); /* 设置总线宽度*/
		}
	}
	else if (CARD_TYPE == CARD_TYPE_MMC)
	{
		//MMC为1位
	}
	printf("SD_BLOCK_SIZE 512 \r\n");
	//设置块长度
	if ((status = SD_CMD_SEND(SDinx, cmd, NULL, 16, SD_BLOCK_SIZE, MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_BUSY)) != Successful)
		;									//return status;
	SD_CMD_SEND(SDinx, cmd, NULL, 7, 0, 0); //取消所有选中
											//
	return Successful;
}

/*-----------------------------------------------------------------------------
读扇区
sector  扇区地址
count   块数量
buff    读出指针
 *---------------------------------------------------------------------------*/
int SD_Read_in(int SDinx, unsigned int sector, unsigned int count, unsigned char *buff)
{
	struct mmc_cmd cmd;
	struct mmc_data data;
	unsigned int _sector;
	unsigned int cmd_inx;
	int status;

	//--- 校验参数
	status = fmiSDCheckSector(sector, count);
	if (status < 0)
		return status; // invalid sector
	//
	if (buff == 0)
	{
		printf("ERROR: fmiSD_Read_in(): uBufcnt cannot be 0!!\r\n");
		return -745435;
	}
	if ((u32)(long)buff & 0x3)
	{
		printf("SD_Read Addr Align Err[%08x] ...\r\n", buff);
		return -474385;
	} //地址对齐检测吗？
	//高容量卡与低容量卡 小于等于2G为低速 高容量卡为写入扇区地址，低容量卡为写入直接地址
	if (CARD_TYPE == CARD_TYPE_SD_HIGH)
		_sector = sector;
	else
		_sector = sector * SD_BLOCK_SIZE;
	//设置命令
	if (count == 1)
		cmd_inx = 17;
	else
		cmd_inx = 18;
	/*发送CMD7-选中*/
	if ((status = SD_CMD_SEND(SDinx, &cmd, NULL, 7, CARD_uRCA, MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_BUSY)) != Successful)
		return status;
	/*读取数据*/
	data.blocks = count;
	data.blocksize = SD_BLOCK_SIZE;
	data.flags = MMC_DATA_READ;
	data.b.dest = buff;
	if ((status = SD_CMD_SEND(SDinx, &cmd, &data, cmd_inx, _sector, MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_BUSY)) != Successful)
		return status;
	/*发送CMD7-取消选中*/
	SD_CMD_SEND(SDinx, &cmd, NULL, 7, 0, 0); //取消所有选中
											 //
	return Successful;
}
/*-----------------------------------------------------------------------------
写扇区
sector  扇区地址
count   块数量
buff    读出指针
 *---------------------------------------------------------------------------*/
int SD_Write_out(int SDinx, unsigned int sector, unsigned int count, unsigned char *buff)
{
	struct mmc_cmd cmd;
	struct mmc_data data;
	unsigned int _sector;
	unsigned int cmd_inx;
	int status;

	//--- 校验参数
	status = fmiSDCheckSector(sector, count);
	if (status < 0)
		return status; // invalid sector
	//
	if (buff == 0)
	{
		printf("ERROR: fmiSD_Read_in(): uBufcnt cannot be 0!!\r\n");
		return -745435;
	}
	if ((u32)(long)buff & 0x3)
	{
		printf("Addr Align Err ...\r\n");
		return -474385;
	} //地址对齐检测吗？
	//高容量卡与低容量卡 小于等于2G为低速 高容量卡为写入扇区地址，低容量卡为写入直接地址
	if (CARD_TYPE == CARD_TYPE_SD_HIGH)
		_sector = sector;
	else
		_sector = sector * SD_BLOCK_SIZE;
	//设置命令
	if (count == 1)
		cmd_inx = 24;
	else
		cmd_inx = 25;
	/*发送CMD7-选中*/
	if ((status = SD_CMD_SEND(SDinx, &cmd, NULL, 7, CARD_uRCA, MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_BUSY)) != Successful)
		return status;
	/*读取数据*/
	data.blocks = count;
	data.blocksize = SD_BLOCK_SIZE;
	data.flags = MMC_DATA_WRITE;
	data.b.dest = buff;
	if ((status = SD_CMD_SEND(SDinx, &cmd, &data, cmd_inx, _sector, MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_BUSY)) != Successful)
		return status;
	/*发送CMD7-取消选中*/
	SD_CMD_SEND(SDinx, &cmd, NULL, 7, 0, 0); //取消所有选中
											 //
	return Successful;
}

/*cache操作 flush*/
#define CONFIG_SYS_CACHELINE_SIZE 32 /*Cache line组织宽度为32byte*/
void dcache_flush_line(unsigned int addr)
{
	/* 清空并使无效数据... */
	/* invalidate I cache */
	__asm__ __volatile__(
	"mcr p15,0,%0,c7,c14,1\n"
	"mcr p15,0,%0,c7,c5,1\n"
	: 
	: "r" (addr)
	: "memory");
	/*F1C100S没有写缓冲*/
}
void flush_dcache_range(unsigned long start, unsigned long end)
{
	unsigned int addr;

	start = start & (~(CONFIG_SYS_CACHELINE_SIZE - 1));
	end = end & (~(CONFIG_SYS_CACHELINE_SIZE - 1));
	for (addr = start; addr <= end; addr += CONFIG_SYS_CACHELINE_SIZE)
		dcache_flush_line(addr);
}
void flush_cache(unsigned long start, unsigned long size)
{
	//	sysFlushCache(I_D_CACHE);
	flush_dcache_range(start, start + size);
}

/*SD初始化*/
int SD_Init(int SDinx)
{
	int returni;
	struct mmc_cmd cmd;
	unsigned int SD_ADDR = 0;
	SD_ADDR = Get_SDaddr(SDinx);
	//GPIO_操作
	SD_IO_Init(SDinx);
	//使能时钟操作
	write32(CCU_BUS_CLK_GATING_REG0, read32(CCU_BUS_CLK_GATING_REG0) | (1) << (8 + SDinx));
	//使能时钟复位
	write32(CCU_BUS_SOFT_RST_REG0, read32(CCU_BUS_SOFT_RST_REG0) | (1) << (8 + SDinx));
	delay_ms(1);
	mmc_set_mod_clk(SDinx, 400000); /*设置时钟 初始化为400KB*/
	//
	SD_Hardware_Reset(SDinx);   /*SD硬件复位*/
	sunxi_mmc_core_init(SDinx); /*SD控制器复位*/
	//
	mmc_close_Card_Detectint(SDinx);		  /*关data3 卡检测脚*/
	Card_IDLE_Clock_OFF(SDinx);				  /*空闲时时钟关闭*/
	Change_bus_width(SDinx, Card_width_1bit); /* 设置总线宽度*/
	write32(SD_ADDR + 0x008, 0xffffffff);	 //超时
	/*设置中断FOR DMA*/
	//	if(SDinx==SD0)
	//		IRQ_Init(IRQ_LEVEL_1,IRQ_SDC0,SD0_ISR,3);
	//	else if(SDinx==SD1)
	//		IRQ_Init(IRQ_LEVEL_1,IRQ_SDC1,SD1_ISR,3);
	//	S_BIT(SD_ADDR+0x000,4);//开SD全局中断
	//  sysSetLocalInterrupt(ENABLE_IRQ);//开IRQ中断
	//
	mmc_clock_enable(SDinx); /*使能时钟*/
	//------------------------------------------------------------------------------------------------------------------------------
	//命令开始
	delay_ms(50);
	SD_CMD_SEND(SDinx, &cmd, NULL, 0, 0, 0); //发送复位命令 CMD0
	delay_ms(500);
	returni = SD_CMD_SEND(SDinx, &cmd, NULL, 8, 0x155, MMC_RSP_PRESENT | MMC_RSP_CRC); //发送命令 CMD8
	if (returni == 0)																   //SD2.0
	{
		printf("SD2.0...\r\n");
		SD_CMD_SEND(SDinx, &cmd, NULL, 55, 0, MMC_RSP_PRESENT | MMC_RSP_CRC);						 /* CMD55,特殊指令前命令,在发送ACMD类指令前,需要发送此命令 */
		SD_CMD_SEND(SDinx, &cmd, NULL, 41, 0x40ff8000, MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_136); /* CMD41,获取SD电压值 */
		while (!(cmd.response[3] & 0x80000000))														 // 电压值没写成功-重写电压值
		{
			SD_CMD_SEND(SDinx, &cmd, NULL, 55, 0, MMC_RSP_PRESENT | MMC_RSP_CRC);						 /* CMD55,特殊指令前命令,在发送ACMD类指令前,需要发送此命令 */
			SD_CMD_SEND(SDinx, &cmd, NULL, 41, 0x40ff8000, MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_136); /* CMD41,获取SD电压值 */
		}
		if (cmd.response[3] & 0x40000000)
		{
			CARD_TYPE = CARD_TYPE_SD_HIGH;
			printf("TYPE_SD_HIGH...\r\n");
		}
		else
		{
			CARD_TYPE = CARD_TYPE_SD_LOW;
			printf("TYPE_SD_LOW...\r\n");
		}
	}
	else //SD1.0
	{
		printf("SD1.1...\r\n");
		SD_CMD_SEND(SDinx, &cmd, NULL, 0, 0, 0); //发送复位命令 CMD0
		delay_ms(500);

		returni = SD_CMD_SEND(SDinx, &cmd, NULL, 55, 0, MMC_RSP_PRESENT | MMC_RSP_CRC);
		if (returni == ETIMEDOUT) // MMC memory
		{
			SD_CMD_SEND(SDinx, &cmd, NULL, 0, 0, 0); //发送复位命令 CMD0
			delay_ms(1);
			if (SD_CMD_SEND(SDinx, &cmd, NULL, 1, 0x80ff8000, MMC_RSP_PRESENT | MMC_RSP_CRC) != ETIMEDOUT) // MMC memory
			{
				while (!(cmd.response[3] & 0x80000000)) // check if card is ready
				{
					SD_CMD_SEND(SDinx, &cmd, NULL, 1, 0x80ff8000, MMC_RSP_PRESENT | MMC_RSP_CRC); // high voltage
				}
				CARD_TYPE = CARD_TYPE_MMC;
				printf("TYPE_MMC...\r\n");
			}
			else
			{
				CARD_TYPE = CARD_TYPE_UNKNOWN;
				printf("ERR_DEVICE...\r\n");
				return -1;
			}
		}
		else if (returni == 0) // SD Memory
		{
			SD_CMD_SEND(SDinx, &cmd, NULL, 41, 0x00ff8000, MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_136);
			while (!(cmd.response[3] & 0x80000000)) // check if card is ready
			{
				SD_CMD_SEND(SDinx, &cmd, NULL, 55, 0x00, MMC_RSP_PRESENT | MMC_RSP_CRC);
				SD_CMD_SEND(SDinx, &cmd, NULL, 41, 0x00ff8000, MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_136);
			}
			CARD_TYPE = CARD_TYPE_SD_LOW;
			printf("TYPE_SD_LOW...\r\n");
		}
		else
		{
			CARD_TYPE = CARD_TYPE_UNKNOWN;
			printf("Init Error !!\n");
			return -1;
		}
	}
	// CMD2, CMD3
	// Get 16 bytes CID or CSD
	if (CARD_TYPE != CARD_TYPE_UNKNOWN)
	{
		SD_CMD_SEND(SDinx, &cmd, NULL, 2, 0x0, MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_136);
		CIDBuffer[0] = cmd.response[3];
		CIDBuffer[1] = cmd.response[2];
		CIDBuffer[2] = cmd.response[1];
		CIDBuffer[3] = cmd.response[0];
		if (CARD_TYPE == CARD_TYPE_MMC)
		{
			returni = SD_CMD_SEND(SDinx, &cmd, NULL, 3, 0x10000, MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_136);
			if (returni != 0)
				return -1; // set RCA
			CARD_uRCA = 0x10000;
		}
		else
		{
			returni = SD_CMD_SEND(SDinx, &cmd, NULL, 3, 0x00, MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_136);
			if (returni != 0)
				return -1; // get RCA
			CARD_uRCA = cmd.response[3];
		}
	}
	//返回文件信息
	Get_SD_info(SDinx, &cmd, &SD_INFO);
	/*选择卡，设置各项参数*/
	SelectCard(SDinx, &cmd, Card_width_4bit);

	return 0;
}

/*SD测试*/
unsigned char Test_Buff[512 * 128];
void SD_Demo(void)
{
	int i;
	int SDx = SD0;
	printf("SD Demo...\r\n");
	if (SD_Init(SDx) == 0)
	{
		//设置buff
		for (i = 0; i < 512; i++)
			Test_Buff[i] = 0x66;
		//写测试
		printf("sd write\r\n");
		SD_Write_out(SDx, g_max_valid_sector - 1100, 1, Test_Buff);
		//设置buff
		for (i = 0; i < 512; i++)
			Test_Buff[i] = 0x25;
		//写测试
		SD_Write_out(SDx, g_max_valid_sector - 1100 + 1, 1, Test_Buff);

		//清buff
		for (i = 0; i < 1024; i++)
			Test_Buff[i] = 0;
		//读测试
		printf("sd read\r\n");
		SD_Read_in(SDx, g_max_valid_sector - 1100, 1, Test_Buff);
		//打印
		for (i = 0; i < 512; i += 2)
			printf("Test_Buff[%d]=0x%02x\r\n", i, Test_Buff[i]);

		//清buff
		for (i = 0; i < 1024; i++)
			Test_Buff[i] = 0;
		//读测试
		SD_Read_in(SDx, g_max_valid_sector - 1100 + 1, 1, Test_Buff);
		//打印
		for (i = 0; i < 512; i += 2)
			printf("Test_Buff[%d]=0x%02x\r\n", i, Test_Buff[i]);
	}
	printf("SDIO OK\r\n");
	while(1);
}

/*系统调度,有优化时不能空函数*/
static void _OS_Time(void)
{
		// delay_us(1);
	//	printf("----------------------\r\n");
	//
}
/*SD0中断*/
void SD0_ISR(void)
{
	unsigned int Seg = 0;
	DMA_TR_SD = 0;
	Seg = read32(SD0_BASE_ADDR + 0x88);
	if (Seg & DMA_TX_Finished)
		DMA_TR_SD |= DMA_TX_Finished;
	if (Seg & DMA_RX_Finished)
		DMA_TR_SD |= DMA_RX_Finished;
	if (DMA_TR_SD > 0)
		write32(SD0_BASE_ADDR + 0x88, Seg | DMA_TR_SD);
}
/*SD1中断*/
void SD1_ISR(void)
{
	unsigned int Seg = 0;
	DMA_TR_SD = 0;
	Seg = read32(SD1_BASE_ADDR + 0x88);
	if (Seg & DMA_TX_Finished)
		DMA_TR_SD |= DMA_TX_Finished;
	if (Seg & DMA_RX_Finished)
		DMA_TR_SD |= DMA_RX_Finished;
	if (DMA_TR_SD > 0)
		write32(SD1_BASE_ADDR + 0x88, Seg | DMA_TR_SD);
}
