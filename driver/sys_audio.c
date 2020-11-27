#include "sys_audio.h"
#include "types.h"
#include "types_plus.h"
#include "stdio.h"
#include "delay.h"
#include <gpio-f1c100s.h>
#include "io.h"
#include "ff.h"
#include "timer_f1c100s.h"
#include <string.h>
//mp3
// #include "mp3dec.h" //汇编文件没改为GCC版本,编不了
// #include "mp3common.h"

/*DMA-寄存器地址*/
#define DMA_Base_Address (0x01C02000)
/*寄存器地址*/
#define F1C100S_AUDIO_BASE  (0x01C23C00)
#define AUDIO_BASE    F1C100S_AUDIO_BASE
#define codec         F1C100S_AUDIO_BASE 
//
#define CCU_Base_Address     	  		(u32_t)0x01C20000
#define CCU_BUS_CLK_GATING_REG0 		(u32_t)CCU_Base_Address+0x0060
#define CCU_BUS_CLK_GATING_REG1 		(u32_t)CCU_Base_Address+0x0064
#define CCU_BUS_CLK_GATING_REG2 		(u32_t)CCU_Base_Address+0x0068
#define CCU_BUS_SOFT_RST_REG0 			(u32_t)CCU_Base_Address+0x02C0
#define CCU_BUS_SOFT_RST_REG1 			(u32_t)CCU_Base_Address+0x02C4
#define CCU_BUS_SOFT_RST_REG2 			(u32_t)CCU_Base_Address+0x02D0

//AUDIO CLK
#define PLL_AUDIO_CTRL_REG 						(u32_t)CCU_Base_Address+0x0008
#define AUDIO_CODEC_CLK_REG						(u32_t)CCU_Base_Address+0x0140
#define PLL_AUDIO_BIAS_REG						(u32_t)CCU_Base_Address+0x0224
#define PLL_AUDIO_PAT_CTRL_REG  	    (u32_t)CCU_Base_Address+0x0284


#define AC_DAC_DPC		    0x00
#define AC_DAC_FIFOC	    0x04
#define AC_DAC_FIFOS	    0x08
#define AC_ADC_TXDATA	    0x0c
#define AC_ADC_FIFOC	    0x10
#define AC_ADC_FIFOS	    0x14
#define AC_ADC_RXDATA	    0x18
#define DAC_MIXER_CTRL	    0x20
#define ADC_MIXER_CTRL	    0x24
#define ADDA_TUNE    	      0x28
#define BIAS_DA16_CAL_CTRL0	0x2C
#define BIAS_DA16_CAL_CTRL1	0x34

void MP3WAVplay(char *path);
// int _MP3_Play(char *path);
int _WAV_Play(char *path);	
int play_exit=0;//播放退出标志
static u8 audioVolValue = 8; //音量大小
unsigned char playMusicState = 0;
unsigned char wav_buff[2][70000] __attribute__ ((aligned (4)));;
struct wav_
{
  u16 audio_format;//格式
  u16	num_channels;//通道数
  u32	format_blok_lenght;//格式块长度
	u32 sample_rate;//采集频率
	u32 byte_rate;//数据传输率
	u16 bits_per_sample;//采样位宽度
	u32 data_size;//数据区大小
	u32 play_time_ms;//播放时长
};
struct wav_ wav;

#define SampleRate48KHZ 	0//000: 48KHz
#define SampleRate24KHZ 	2//010: 24KHz
#define SampleRate12KHZ		4//100: 12KHz
#define SampleRate192KHZ	6//110: 192KHz
#define SampleRate32KHZ		1//001: 32KHz
#define SampleRate16KHZ		3//011: 16KHz
#define SampleRate8KHZ		5//101: 8KHz
#define SampleRate96KHZ		7//111: 96KHz
//
#define SampleRate44_1KHZ			0//000: 48KHz
#define SampleRate22_05KHZ		2//010: 24KHz
#define SampleRate11_025KHZ		4//100: 12KHz

#define snd_soc_update_bits(Base_Add,Offset,bit,Clear,Value)	write32(Base_Add+Offset,read32(Base_Add+Offset) & (~((u64_t)(Clear)<<bit)) | ((u64_t)(Value)<<bit)) 
//#define snd_soc_update_bits(Base_Add,Offset,Clear,Value)	write32(Base_Add+Offset,read32(Base_Add+Offset) & (~((u64_t)(Clear))) | ((u64_t)(Value))    ) 

/////////////////////////////////////////////
/*DMA初始化*/
void DMA_Init(void)
{
static int Inif_f=0;
	if(Inif_f==0)//初始化一次
	{
		//使能DMA时钟
		write32(CCU_BUS_CLK_GATING_REG0,read32(CCU_BUS_CLK_GATING_REG0)|(1)<<6);
		//使能DMA复位
		write32(CCU_BUS_SOFT_RST_REG0,read32(CCU_BUS_SOFT_RST_REG0)|((1)<<6));
		delay_us(100);
		Inif_f=1;
	}
}

/*音频DMA参数初始化
dma_ch ：dma通道0-3
Source ：目标指针
bcnt	：计数 byte
*/
void SET_AUDIO_DMA_DATA(int dma_ch,unsigned int *Source,unsigned int bcnt)
{
int n=0;
//	printf("DMA Update...\r\n");
  n=dma_ch;
	
  //自动模式下【Auto Clock】一定要设置为0，不然两个时钟会传一个数据过去
  write32(DMA_Base_Address+0x8,read32(DMA_Base_Address+0x8) &(~ ((0x1)<<16)) );	
	write32(DMA_Base_Address+0x8,0);	
	
	//设置源地址
	write32(DMA_Base_Address+(0x100+n*0x20+0x4),(u32_t)Source);
	//设置目标地址
	write32(DMA_Base_Address+(0x100+n*0x20+0x8),(u32_t)(AUDIO_BASE+0x0C));
	//设置BCNT
	write32(DMA_Base_Address+(0x100+n*0x20+0xC),bcnt);

	//设置DMA连续使能
//	write32(DMA_Base_Address+(0x100+n*0x20+0x0),read32(DMA_Base_Address+(0x100+n*0x20+0x0))|((0x1)<<29));	

	//设置源类型 SDRAM memory
	write32(DMA_Base_Address+(0x100+n*0x20+0x0),read32(DMA_Base_Address+(0x100+n*0x20+0x0))|((0x11)<<0));
	//设置源宽度为16位  0-8 1-16 2-32
	write32(DMA_Base_Address+(0x100+n*0x20+0x0),read32(DMA_Base_Address+(0x100+n*0x20+0x0))|((0x1)<<8));
	//设置源地址模式 IO  0-线 1-IO
//	write32(DMA_Base_Address+(0x100+n*0x20+0x0),read32(DMA_Base_Address+(0x100+n*0x20+0x0))|((0x1)<<5));	
	//源 Burst Length 0-1 1-4
//	write32(DMA_Base_Address+(0x100+n*0x20+0x0),read32(DMA_Base_Address+(0x100+n*0x20+0x0))|((0x1)<<7));	
	
	//设置目标类型 AUDIO
	write32(DMA_Base_Address+(0x100+n*0x20+0x0),read32(DMA_Base_Address+(0x100+n*0x20+0x0))|((0xc)<<16));	
	//设置目标宽度为16位  0-8 1-16 2-32
	write32(DMA_Base_Address+(0x100+n*0x20+0x0),read32(DMA_Base_Address+(0x100+n*0x20+0x0))|((0x1)<<24));		
	//设置目标地址模式 IO 0-线 1-IO
	write32(DMA_Base_Address+(0x100+n*0x20+0x0),read32(DMA_Base_Address+(0x100+n*0x20+0x0))|((0x1)<<21));		
	//目标 Burst Length0-1 1-4
// write32(DMA_Base_Address+(0x100+n*0x20+0x0),read32(DMA_Base_Address+(0x100+n*0x20+0x0))|((0x1)<<23));	

	write32(DMA_Base_Address+(0x100+n*0x20+0x0),read32(DMA_Base_Address+(0x100+n*0x20+0x0))|((0x1)<<15));
			
	//设置DMA加载使能
	S_BIT(DMA_Base_Address+(0x100+n*0x20+0x0),31);
}

/*dma disable*/
void dmd_disable(int dma_ch)
{
	C_BIT(DMA_Base_Address+(0x100+dma_ch*0x20+0x0),31);
}

/*音频初始化*/
void AUDIO_Init(void)
{
static int af=0;
	if(af==0)
	{
	af=1;
		printf("AUDIO Init...\r\n");
		//使能总线AUDIO时钟
		write32(CCU_BUS_CLK_GATING_REG2,read32(CCU_BUS_CLK_GATING_REG2)|(1)<<0);
		//使能AUDIO复位
		write32(CCU_BUS_SOFT_RST_REG2,read32(CCU_BUS_SOFT_RST_REG2)|((1)<<0));
		delay_ms(1);

		
		//PLL使能
		S_BIT(PLL_AUDIO_CTRL_REG,31);
		//等PLL有效
		while((read32(PLL_AUDIO_CTRL_REG)&(1<<28))==0);
		
		#define DAC_DRQ_CLR_CNT			21
		#define FIFO_FLUSH					0
		#define FIR_VER							28
		#define ADC_FIFO_FLUSH			0
		#define PH_COM_FC        		22
		#define PH_COM_PROTEC      	21
		#define HP_VOL              0
		#define EN_DAC							31
		#define HP_POWER_EN					15	
		#define R_MIXER_MUTE_R_DAC	17
		#define R_MIXER_MUTE_L_DAC	16
		#define L_MIXER_MUTE_R_DAC	9
		#define L_MIXER_MUTE_L_DAC	8
		#define PH_R_MUTE						27  
		#define PH_L_MUTE						26  
		#define DAC_AG_R_EN					31  
		#define DAC_AG_L_EN					30  
		
		snd_soc_update_bits(codec, AC_DAC_FIFOC,DAC_DRQ_CLR_CNT,0x3,0x3);
		snd_soc_update_bits(codec, AC_DAC_FIFOC,FIFO_FLUSH,0x1,0x1);
		/*
		*	0:64-Tap FIR
		*	1:32-Tap FIR
		*/	
		snd_soc_update_bits(codec, AC_DAC_FIFOC,FIR_VER,0x1,0x0);	
		snd_soc_update_bits(codec, AC_ADC_FIFOC,ADC_FIFO_FLUSH,0x1,0x1);
		
		snd_soc_update_bits(codec, DAC_MIXER_CTRL,PH_COM_PROTEC,0x1,0x1);
		snd_soc_update_bits(codec, DAC_MIXER_CTRL,PH_COM_FC,0x3,0x3);	
		if(audioVolValue>63) //over
			audioVolValue = 8;
		snd_soc_update_bits(codec, DAC_MIXER_CTRL,HP_VOL,0x3f,audioVolValue);/* set hp volume */
		snd_soc_update_bits(codec, AC_DAC_DPC,EN_DAC,0x1,0x1);
		
		snd_soc_update_bits(codec, DAC_MIXER_CTRL,HP_POWER_EN,0x1,0x1);	/* power hp */
		
		snd_soc_update_bits(codec, DAC_MIXER_CTRL,L_MIXER_MUTE_R_DAC,0x1,0x1);	
		snd_soc_update_bits(codec, DAC_MIXER_CTRL,L_MIXER_MUTE_L_DAC,0x1,0x1);
		
		snd_soc_update_bits(codec, DAC_MIXER_CTRL,R_MIXER_MUTE_R_DAC,0x1,0x1);	
		snd_soc_update_bits(codec, DAC_MIXER_CTRL,R_MIXER_MUTE_L_DAC,0x1,0x1);	
		
		snd_soc_update_bits(codec, DAC_MIXER_CTRL,PH_L_MUTE,0x1,0x1);	
		snd_soc_update_bits(codec, DAC_MIXER_CTRL,PH_R_MUTE,0x1,0x1);	

		snd_soc_update_bits(codec, DAC_MIXER_CTRL,DAC_AG_L_EN,0x1,0x1);	
		snd_soc_update_bits(codec, DAC_MIXER_CTRL,DAC_AG_R_EN,0x1,0x1);	
		
		/*音频放大开关控制,需要先初始化，再开外部放大器，不然初始化时有杂音*/
		delay_ms(500);	
		// gpio_f1c100s_set_dir(&GPIO_PE, 9, GPIO_DIRECTION_OUTPUT);
		// gpio_f1c100s_set_value(&GPIO_PE, 9, 0);
		delay_ms(500);

		//DMA使用时要开空中断
		S_BIT(AUDIO_BASE+4,4);
		//发送最后采样
		S_BIT(AUDIO_BASE+4,26);
	}
}


/*播放初始化*/
int AUDIO_PLAY_Init(struct wav_ * wav_f)	
{
int s=0,n=0,m=0;	

	switch(wav_f->sample_rate)
	{
		case 48000:
			s=SampleRate48KHZ;n=0x55;m=0x14;
			break;
		case 24000:
			s=SampleRate24KHZ;n=0x55;m=0x14;
			break;	
		case 12000:
			s=SampleRate12KHZ;n=0x55;m=0x14;
			break;
		case 192000:
			s=SampleRate192KHZ;n=0x55;m=0x14;
			break;	
		case 32000:
			s=SampleRate32KHZ;n=0x55;m=0x14;
			break;
		case 16000:
			s=SampleRate16KHZ;n=0x55;m=0x14;
			break;			
		case 8000:
			s=SampleRate8KHZ;n=0x55;m=0x14;
			break;
		case 96000:
			s=SampleRate96KHZ;n=0x55;m=0x14;
			break;	
		case 44100:
			s=SampleRate44_1KHZ;n=0x4E;m=0x14;
			break;			
		case 22050:
			s=SampleRate22_05KHZ;n=0x4E;m=0x14;
			break;
		case 11025:
			s=SampleRate11_025KHZ;n=0x4E;m=0x14;
			break;	
		default:
			return -1;
	}
	//PLL关
	C_BIT(PLL_AUDIO_CTRL_REG,31);
	
	//-设置PLL-N
	C_Vlue(PLL_AUDIO_CTRL_REG,8,0x7f);
	S_Vlue(PLL_AUDIO_CTRL_REG,8,n);
	//-设置PLL-M
	C_Vlue(PLL_AUDIO_CTRL_REG,0,0x1f);
	S_Vlue(PLL_AUDIO_CTRL_REG,0,m);

	//PLL使能
	S_BIT(PLL_AUDIO_CTRL_REG,31);
	//等PLL有效
	while((read32(PLL_AUDIO_CTRL_REG)&(1<<28))==0);
			
		
	/*设置采样周期*/
	C_Vlue(AUDIO_BASE+4,29,0xf);
	S_Vlue(AUDIO_BASE+4,29,s);
	
	
	/*设置采样位宽*/
	if(wav_f->bits_per_sample==16)
		C_BIT(AUDIO_BASE+4,5); //16位
	else if(wav_f->bits_per_sample==24)
		S_BIT(AUDIO_BASE+4,5); //24位
	else return -1;
	
	/*设置采样*/
	if(wav_f->bits_per_sample==16)
	{
		/*设置采样低16位*/
		C_Vlue(AUDIO_BASE+4,24,0x3);
		S_Vlue(AUDIO_BASE+4,24,0x3);	
	}
	else if(wav_f->bits_per_sample==24)
	{
		/*设置采样高24位*/
		C_Vlue(AUDIO_BASE+4,24,0x3);
	}
	else return -1;
	
	/*设置为单声道*/
	if(wav_f->num_channels==1)
		S_BIT(AUDIO_BASE+4,6);//单声道
	else if(wav_f->num_channels==2)
		C_BIT(AUDIO_BASE+4,6);//双声道
	else return -1;
	
	//清采样计数
	write32(AUDIO_BASE+0x40,0);
	
return 0;
}

void AudioVol(unsigned char i)
{
	if(i<=63)
	{
		audioVolValue = i;
		snd_soc_update_bits(codec, DAC_MIXER_CTRL,HP_VOL,0x3f,audioVolValue);
	}
}

/*音频测试*/
void AUDIO_Demo(void)
{
char *path[13]=
{	
	"0:/a1.wav",		
	"0:/a2.mp3",		
	"0:/m1.wav",
  "0:/m2.mp3",
	"0:/m3.mp3",
  "0:/m4.mp3",	
	0
};	

	int s=0;
	while(1)
	{
		/*播放*/
    MP3WAVplay(path[s]);
		/*下一曲*/
		s++;
		if(s>=6)s=0;
	}
}

/*播放WAV MP3文件*/
void MP3WAVplay(char *path)
{
	printf("path %s \r\n",path);
	char *Fn=path+(strlen(path)-4);
	if(strcmp(Fn,".mp3")==0)
	{
		// _MP3_Play(path);
	}
	else if(strcmp(Fn,".wav")==0)
	{
		_WAV_Play(path);
	}else
	{
		printf("it isn't mp3/wav\r\n");
	}	
}
/*退出播放*/
void MP3WAVplay_exit(void)
{
	play_exit=1;
}

//输入路径
int _WAV_Play(char *path)
{
#define ReadSize (512*40)/*一次读取数据大小*/
int i,tj=1;
int a,b,u,Inx,r = 0;
char buff[8192];
int res=0;
FIL fp1;	
int wav_play_time=0;//播放时长
	
		printf("WAV play\r\n");
	
		if(path==NULL)
		{
			printf("no path input..\r\n");	
			return -4;
		}
		res=f_open(&fp1,path,FA_OPEN_EXISTING|FA_READ);
		if(res==5)
		{
			printf("no path..\r\n");	
			return -3;
		}
		if(res==4)
		{
			printf("WAV file not exist..\r\n");
			return -2;
		}
		if(res==0)
		{
			printf("WAV file open successful..\r\n");

			/*解WAV文件*/
			f_read(&fp1,buff,58,NULL);
			//for(i=0;i<44;i++)printf("%02x ",buff[i]);
			wav.audio_format=buff[21]<<8|buff[20];
			wav.num_channels=buff[23]<<8|buff[22];		
			wav.bits_per_sample=buff[35]<<8|buff[34];
			wav.byte_rate=buff[31]<<24|buff[30]<<16|buff[29]<<8|buff[28];
			wav.sample_rate=buff[27]<<24|buff[26]<<16|buff[25]<<8|buff[24];	
			wav.format_blok_lenght=buff[19]<<24|buff[18]<<16|buff[17]<<8|buff[16];	
			i=16+wav.format_blok_lenght+4+4;
			wav.data_size=buff[i+3]<<24|buff[i+2]<<16|buff[i+1]<<8|buff[i+0];
			wav.play_time_ms=(int)((float)wav.data_size*(float)1000/(float)wav.byte_rate);
								
			/*输出*/
     	printf("sample format:%d \r\n",wav.audio_format);
      printf("sample channel:%d \r\n",wav.num_channels);
     	printf("sample bit:%d bit\r\n",wav.bits_per_sample);
      printf("sample cycle:%d Hz\r\n",wav.sample_rate);
      printf("sample rate:%d Byte/S\r\n",wav.byte_rate);
			printf("format block lenght:%d Byte\r\n",wav.format_blok_lenght);
			printf("data size:%d Byte\r\n",wav.data_size);
			wav_play_time=wav.play_time_ms/1000;
			printf("play time:%02d:%02d:%02d \r\n",
					wav_play_time/3600,wav_play_time%3600/60,wav_play_time%3600%60);
						
			/*无压缩格式*/
			if(wav.audio_format==1)
			{	
				/*音频初始化*/
				AUDIO_Init();
				DMA_Init();
				AVS_Time_Init();	
				
				a=wav.data_size/ReadSize;
				b=wav.data_size%ReadSize;
				/*读出数据区*/
				if(a==1)
				{
					f_lseek(&fp1,i+4);	
					f_read(&fp1,wav_buff[0],ReadSize,NULL);
					if(b>0)
					{
						f_lseek(&fp1,i+4+ReadSize);	
						f_read(&fp1,wav_buff[1],b,NULL);
					}
					r=ReadSize;
				}
				else if(a>=2)
				{
					f_lseek(&fp1,i+4);	
					f_read(&fp1,wav_buff[0],ReadSize,NULL);
					f_lseek(&fp1,i+4+ReadSize);	
					f_read(&fp1,wav_buff[1],ReadSize,NULL);	
					r=ReadSize;
				}
				else if((a==0)&&(b>0))
				{
					f_lseek(&fp1,i+4);	
					f_read(&fp1,wav_buff[0],b,NULL);
					r=b;
					b=0;
				}
				if(AUDIO_PLAY_Init(&wav)==0)
				{
					//dma初始化
					SET_AUDIO_DMA_DATA(0,(unsigned int *)wav_buff[0],r);
					printf("play...\r\n");		
					//时钟选通
					S_BIT(AUDIO_CODEC_CLK_REG,31);
					//开始计时器
					AVS_Time_Start(AVS_TIME_0);
					//检测发送全中断
					Inx=0;u=0;
					while(1)
					{
						if(read32(DMA_Base_Address+0x4)&0x2)//全传输
						{
							  S_BIT(DMA_Base_Address+0x4,1);
							  a--;
						    u++;
							  Inx++;
								if(a>=1)
								{							
									dmd_disable(0);
								  SET_AUDIO_DMA_DATA(0,(unsigned int *)wav_buff[Inx%2],ReadSize);
									f_lseek(&fp1,i+4+((1+u)*ReadSize));
									if(a==1)//读剩余数据
									{			
										if((Inx%2)==0)f_read(&fp1,wav_buff[1],b,NULL);
										if((Inx%2)==1)f_read(&fp1,wav_buff[0],b,NULL);
									}else
									{
										if((Inx%2)==0)f_read(&fp1,wav_buff[1],ReadSize,NULL);
										if((Inx%2)==1)f_read(&fp1,wav_buff[0],ReadSize,NULL);				
									}
//									printf("用时= %d ms\r\n",Read_time_ms()-t);
								}else
								{							
									dmd_disable(0);
									if(b>0)
									{
										SET_AUDIO_DMA_DATA(0,(unsigned int *)wav_buff[Inx%2],b);
										b=0;
									}else
									{
										printf("play end\r\n");
										//停止计时器
										AVS_Time_Stop(AVS_TIME_0);
										C_BIT(AUDIO_CODEC_CLK_REG,31);
										break;
									}
								}	
								
								int t=AVS_Time_Read(AVS_TIME_0)/10/1000;
								if(t!=tj)
								{
									tj=t;
									printf("%02d:%02d:%02d----%02d:%02d:%02d \r\n",
									wav_play_time/3600,wav_play_time%3600/60,wav_play_time%3600%60,t/3600,t%3600/60,t%3600%60);	
								}									
						}
						/*播放退出*/
						if(play_exit==1)
						{
							play_exit=0;
							dmd_disable(0);
							printf("paly exit\r\n");
							AVS_Time_Stop(AVS_TIME_0);
							C_BIT(AUDIO_CODEC_CLK_REG,31);
						}
					}
	
				}else printf("parameter error\r\n");
			}else printf("format error\r\n");
			
			printf("file close\r\n");
			f_close(&fp1);
		}
		return 0;
}

static u32 read_size = 512 * 40; /*一次读取数据大小*/
static int iblocklen;
static int _integer, _decimal, _unit, WavBuffInx;
static FIL fpWav;
static int wavTotalTime = 0; //文件播放时长
static int _timeTemp = 1;
static int audio_keep_time = 0;

int wav_init(char *path, int play_time)
{
    int res = 0;
    int r = 0;
    char buff[8192];

	if(playMusicState) //强制停止
	{
		if (read32(DMA_Base_Address + 0x4) & 0x2) //全传输
        {
            S_BIT(DMA_Base_Address + 0x4, 1);
		}
		dmd_disable(0);
		printf("paly exit\r\n");
		AVS_Time_Stop(AVS_TIME_0);
		C_BIT(AUDIO_CODEC_CLK_REG, 31);
		_timeTemp = 1;
		play_exit = 1;
		playMusicState = 0;
		printf("file close\r\n");
		f_close(&fpWav);
	}

    printf("WAV play\r\n");

    if (path == NULL)
    {
        printf("no path input..\r\n");
        return -4;
    }
    res = f_open(&fpWav, path, FA_OPEN_EXISTING | FA_READ);
    if (res == 5)
    {
        printf("no path..\r\n");
        return -3;
    }
    if (res == 4)
    {
        printf("WAV file not exist..\r\n");
        return -2;
    }
    if (res == 0)
    {
        printf("WAV file open successful..\r\n");

        /*解WAV文件*/
        f_read(&fpWav, buff, 58, NULL);
        //for(iblocklen=0;iblocklen<44;iblocklen++)printf("%02x ",buff[iblocklen]);
        wav.audio_format = buff[21] << 8 | buff[20];
        wav.num_channels = buff[23] << 8 | buff[22];
        wav.bits_per_sample = buff[35] << 8 | buff[34];
        wav.byte_rate = buff[31] << 24 | buff[30] << 16 | buff[29] << 8 | buff[28];
        wav.sample_rate = buff[27] << 24 | buff[26] << 16 | buff[25] << 8 | buff[24];
        wav.format_blok_lenght = buff[19] << 24 | buff[18] << 16 | buff[17] << 8 | buff[16];
        iblocklen = 16 + wav.format_blok_lenght + 4 + 4;
        wav.data_size = buff[iblocklen + 3] << 24 | buff[iblocklen + 2] << 16 | buff[iblocklen + 1] << 8 | buff[iblocklen + 0];
        wav.play_time_ms = (int)((float)wav.data_size * (float)1000 / (float)wav.byte_rate);

        /*输出*/
        printf("sample format:%d \r\n", wav.audio_format);
        printf("sample channel:%d \r\n", wav.num_channels);
        printf("sample bit:%d bit\r\n", wav.bits_per_sample);
        printf("sample cycle:%d Hz\r\n", wav.sample_rate);
        printf("sample rate:%d Byte/S\r\n", wav.byte_rate);
        printf("format block lenght:%d Byte\r\n", wav.format_blok_lenght);
        printf("data size:%d Byte\r\n", wav.data_size);
        wavTotalTime = wav.play_time_ms / 1000;
        printf("play time:%02d:%02d:%02d \r\n",
               wavTotalTime / 3600, wavTotalTime % 3600 / 60, wavTotalTime % 3600 % 60);

        /*无压缩格式*/
        if (wav.audio_format == 1)
        {
            /*音频初始化*/
            AUDIO_Init();
            DMA_Init();
            AVS_Time_Init();

            _integer = wav.data_size / read_size;
            _decimal = wav.data_size % read_size;
            /*读出数据区*/
            if (_integer == 1)
            {
                f_lseek(&fpWav, iblocklen + 4);
                f_read(&fpWav, wav_buff[0], read_size, NULL);
                if (_decimal > 0)
                {
                    f_lseek(&fpWav, iblocklen + 4 + read_size);
                    f_read(&fpWav, wav_buff[1], _decimal, NULL);
                }
                r = read_size;
            }
            else if (_integer >= 2)
            {
                f_lseek(&fpWav, iblocklen + 4);
                f_read(&fpWav, wav_buff[0], read_size, NULL);
                f_lseek(&fpWav, iblocklen + 4 + read_size);
                f_read(&fpWav, wav_buff[1], read_size, NULL);
                r = read_size;
            }
            else if ((_integer == 0) && (_decimal > 0))
            {
                f_lseek(&fpWav, iblocklen + 4);
                f_read(&fpWav, wav_buff[0], _decimal, NULL);
                r = _decimal;
                _decimal = 0;
            }
            if (AUDIO_PLAY_Init(&wav) == 0)
            {
                //dma初始化
                SET_AUDIO_DMA_DATA(0, (unsigned int *)wav_buff[0], r);
                printf("play...\r\n");
                //时钟选通
                S_BIT(AUDIO_CODEC_CLK_REG, 31);
                //开始计时器
                AVS_Time_Start(AVS_TIME_0);
                //检测发送全中断
                WavBuffInx = 0;
                _unit = 0;
                play_exit = 0; //开始播放
				audio_keep_time = play_time;
            }
            else
                printf("parameter error\r\n");
        }
        else
            printf("format error\r\n");
    }
    return 0;
}

void _WAV_Play2(void)
{	
	if(play_exit&&playMusicState) //强制停止
	{
		if (read32(DMA_Base_Address + 0x4) & 0x2) //全传输
        {
            S_BIT(DMA_Base_Address + 0x4, 1);
		}
		dmd_disable(0);
		printf("paly exit\r\n");
		AVS_Time_Stop(AVS_TIME_0);
		C_BIT(AUDIO_CODEC_CLK_REG, 31);
		_timeTemp = 1;
		play_exit = 1;
		playMusicState = 0;
		printf("file close\r\n");
		f_close(&fpWav);
	}

    if (play_exit == 0)
    {
        playMusicState = 1;                            //播放处理中
        if (read32(DMA_Base_Address + 0x4) & 0x2) //全传输
        {
            S_BIT(DMA_Base_Address + 0x4, 1);
            _integer--;
            _unit++;
            WavBuffInx++;
            if (_integer >= 1)
            {
                dmd_disable(0);
                SET_AUDIO_DMA_DATA(0, (unsigned int *)wav_buff[WavBuffInx % 2], read_size);
                f_lseek(&fpWav, iblocklen + 4 + ((1 + _unit) * read_size));
                if (_integer == 1) //读剩余数据
                {
                    if ((WavBuffInx % 2) == 0)
                        f_read(&fpWav, wav_buff[1], _decimal, NULL);
                    if ((WavBuffInx % 2) == 1)
                        f_read(&fpWav, wav_buff[0], _decimal, NULL);
                }
                else
                {
                    if ((WavBuffInx % 2) == 0)
                        f_read(&fpWav, wav_buff[1], read_size, NULL);
                    if ((WavBuffInx % 2) == 1)
                        f_read(&fpWav, wav_buff[0], read_size, NULL);
                }
                //printf("用时= %d ms\r\n",Read_time_ms()-t);
            }
            else
            {
                dmd_disable(0);
                if (_decimal > 0)
                {
                    SET_AUDIO_DMA_DATA(0, (unsigned int *)wav_buff[WavBuffInx % 2], _decimal);
                    _decimal = 0;
                }
                else
                {
                    printf("play end\r\n");
                    //停止计时器
                    AVS_Time_Stop(AVS_TIME_0);
                    C_BIT(AUDIO_CODEC_CLK_REG, 31);
                    _timeTemp = 1;
                    play_exit = 1; //停止播放
                    playMusicState = 0; //已停止播放处理
                    printf("file close\r\n");
                    f_close(&fpWav);
                    return;
                }
            }
            int t = AVS_Time_Read(AVS_TIME_0) / 10 / 1000;
            if (t != _timeTemp) //播放持续时间, 单位1s
            {
                _timeTemp = t;
                printf("%02d:%02d:%02d----%02d:%02d:%02d \r\n",
                       wavTotalTime / 3600, wavTotalTime % 3600 / 60,
                       wavTotalTime % 3600 % 60, t / 3600, t % 3600 / 60, t % 3600 % 60);
                if(audio_keep_time!=0XFF) //0XFF为播放整个文件
				{
					if (t >= audio_keep_time) //停止播放
					{
						dmd_disable(0);
						printf("paly exit\r\n");
						AVS_Time_Stop(AVS_TIME_0);
						C_BIT(AUDIO_CODEC_CLK_REG, 31);
						_timeTemp = 1;
						play_exit = 1;
						playMusicState = 0;
						printf("file close\r\n");
						f_close(&fpWav);
					}
				}
            }
        }
    }
}

// /*mp3解码测试*/
// int Get_ID3V2(FIL f,char *TIT2,char *TPE1,char *TALB);
// int Get_ID3V1(FIL f,char *TIT2,char *TPE1,char *TALB);
// int Get_mp3_play_time_ms(int ID3size,int ID3V1_size,FIL f,MP3FrameInfo _mp3FrameInfo);
// int MP3_to_PCM_DATA(HMP3Decoder Mp3Decoder,MP3FrameInfo *mp3FrameInfo,FIL f,unsigned char *outbuff,int data_init,int read_addr);
// //网页资料链接
// //https://blog.csdn.net/wlsfling/article/details/5875959
// //https://blog.csdn.net/zftzftzft/article/details/79550396
// //https://blog.csdn.net/zftzftzft/article/details/79550396
// //输入路径
// int _MP3_Play(char *path)
// {
// 	int tj=1;
// 	int Frame_size;
// 	int mp3_play_time=0;//播放时长
// 	int offset=0;
// 	unsigned char *readPtr=0;//输入数据缓冲
// 	int Padding=0;//Padding位
// 	int bytesLeft=0;
// 	MP3FrameInfo mp3FrameInfo;
// 	HMP3Decoder Mp3Decoder;
// 		//TIT2：歌曲标题名字
// 		//TPE1：作者名字
// 		//TALB：作品专辑	
// 	char TIT2[100];
// 	char TPE1[100];
// 	char TALB[100];
// 	int ID3v1=0,ID3v2=0;//ID3大小
// 	//
// 	int res;
// 	FIL f;	

// 	printf("MP3播放\r\n");
	
// 	if(path==NULL)
// 	{
// 		printf("没有路径输入..\r\n");	
// 		return -4;
// 	}
// 	res=f_open(&f,path,FA_OPEN_EXISTING|FA_READ);
// 	if(res==5)
// 	{
// 		printf("文件夹路径不存在..\r\n");	
// 		return -3;
// 	}		
// 	if(res==4)
// 	{
// 		printf("MP3文件不在..\r\n");	
// 		return -3;
// 	}
// 	if(res==0)printf("MP3文件打开成功..\r\n");	
// 	//
// 	readPtr=wav_buff[0];
// 	/*读入一段mp3文件*/
// 	f_read(&f,readPtr,10000,NULL);
// 	bytesLeft=10000;
// 	//读ID3
// 	ID3v1=Get_ID3V1(f,TIT2,TPE1,TALB);
// 	ID3v2=Get_ID3V2(f,TIT2,TPE1,TALB);	
// 	//超出，从新读入数据
//   if(ID3v2>10000)
// 	{
// 		printf("ID3v2超长\r\n");
// 		readPtr=wav_buff[0];
// 		f_lseek(&f,ID3v2);
// 		f_read(&f,readPtr,10000,NULL);
// 		bytesLeft=10000;
// 	}else
// 	{
// 		if(ID3v2>0)offset+=ID3v2;//如果有ID3V2偏移到下一个帧
// 		readPtr+=offset;
// 		bytesLeft-=offset;
// 	}
// 	/*【参数1：输入数据的缓存指针】【 参数2: 输入数据帧的剩余大小】*/
// 	offset= MP3FindSyncWord(readPtr,bytesLeft);//找帧头
// 	readPtr+=offset;
// 	bytesLeft-=offset;
	
// 	if(offset<0)
// 	{
// 		printf("没有帧头\r\n");
// 		return -1;
// 	}
// 	else
// 	{
// 		Mp3Decoder=MP3InitDecoder(); 	
// 		if(Mp3Decoder==0)
// 		{
// 			printf("Init Mp3Decoder failed!\r\n");
// 			return -1;
// 		}		
// 		/*返回帧头信息*/
//     if(ERR_MP3_NONE==MP3GetNextFrameInfo(Mp3Decoder,&mp3FrameInfo,readPtr))	
// 		{			
// 			/*打印帧头信息*/
// 			printf("FrameInfo bitrate=%d \r\n",mp3FrameInfo.bitrate);
// 			printf("FrameInfo nChans=%d \r\n",mp3FrameInfo.nChans);
// 			printf("FrameInfo samprate=%d \r\n",mp3FrameInfo.samprate);
// 			printf("FrameInfo bitsPerSample=%d \r\n",mp3FrameInfo.bitsPerSample);
// 			printf("FrameInfo outputSamps=%d \r\n",mp3FrameInfo.outputSamps);	
// 			printf("FrameInfo layer=%d \r\n",mp3FrameInfo.layer);
// 			printf("FrameInfo version=%d \r\n",mp3FrameInfo.version);
// 			/*计算数据帧大小 帧大小 = ( 每帧采样次数 × 比特率(bit/s) ÷ 8 ÷采样率) + Padding*/	
// 			Frame_size=(int)samplesPerFrameTab[mp3FrameInfo.version][mp3FrameInfo.layer - 1] * mp3FrameInfo.bitrate / 8 / mp3FrameInfo.samprate + Padding;
// 			printf("FrameInfo Frame_size=%d \r\n",Frame_size);
// 			//返回播放时长信息
// 			int t=Get_mp3_play_time_ms(ID3v2,ID3v1,f,mp3FrameInfo);
// 			mp3_play_time=t/1000;
// 			printf("播放时长=%02d:%02d:%02d \r\n",mp3_play_time/3600,mp3_play_time%3600/60,mp3_play_time%3600%60);		
// 		}			
// 	}
// 	/*只能播放MP3文件*/
// 	if((mp3FrameInfo.layer<3) || (mp3FrameInfo.bitrate<=0) || (mp3FrameInfo.samprate<=0))
// 	{
// 		printf("格式不是MP3... \r\n");	
// 		MP3FreeDecoder(Mp3Decoder);
// 		f_close(&f);
// 		return -1;
// 	}
	

// 	/*音频初始化*/
// 	AUDIO_Init();
// 	DMA_Init();
// 	AVS_Time_Init();	
// 	/*初始化参数*/
// 	wav.audio_format=1;
// 	wav.num_channels=mp3FrameInfo.nChans;	
//   wav.bits_per_sample=mp3FrameInfo.bitsPerSample;
// 	wav.sample_rate=mp3FrameInfo.samprate;	
//   /*写入参数*/
// 	if(wav.audio_format==1)//PCM格式
// 	{
// 		if(AUDIO_PLAY_Init(&wav)==0)
// 		{
// 			//dma初始化-需要先解码两个缓存-播放一个，一个备用
// 			int pcm_data_size;
// 			pcm_data_size=MP3_to_PCM_DATA(Mp3Decoder,&mp3FrameInfo,f,wav_buff[0],1,ID3v2);	
// 			if(pcm_data_size>0)SET_AUDIO_DMA_DATA(0,(unsigned int *)wav_buff[0],pcm_data_size);
// 			pcm_data_size=MP3_to_PCM_DATA(Mp3Decoder,&mp3FrameInfo,f,wav_buff[1],0,0);
// 			if(pcm_data_size<0)
// 			{
// 				printf("解码错误D1...\r\n");	
// 				MP3FreeDecoder(Mp3Decoder);
// 				f_close(&f);					
// 				return -2;
// 			}		
// 			printf("播放...\r\n");		
// 			//时钟选通
// 			S_BIT(AUDIO_CODEC_CLK_REG,31);
// 			//开始计时器
// 			AVS_Time_Start(AVS_TIME_0);
// 			//检测发送完成中断-中断DMA指向下一缓存，读SD后解码一个缓存
// 			int Inx=0;
// 			while(1)
// 			{
// 				if(read32(DMA_Base_Address+0x4)&0x2)//全传输完标志
// 				{
// 					/*清标志*/
// 					S_BIT(DMA_Base_Address+0x4,1);
// 					/*设置DMA一下缓存*/
// 					Inx++;
// 					dmd_disable(0);
// 					if(pcm_data_size>0)
// 					{
// 						int x=Inx%2;
// 						SET_AUDIO_DMA_DATA(0,(unsigned int *)wav_buff[x],pcm_data_size);
// 						/*解一下缓存*/
// 						//int times=Read_time_ms();
// 						if(x==0)pcm_data_size=MP3_to_PCM_DATA(Mp3Decoder,&mp3FrameInfo,f,wav_buff[1],0,0);	
// 						if(x==1)pcm_data_size=MP3_to_PCM_DATA(Mp3Decoder,&mp3FrameInfo,f,wav_buff[0],0,0);
// 						//printf("解码用时= %d ms\r\n",Read_time_ms()-times);
// 						int t=AVS_Time_Read(AVS_TIME_0)/10/1000;
// 						if(t!=tj)
// 						{
// 							tj=t;
// 							printf("%02d:%02d:%02d----%02d:%02d:%02d \r\n",mp3_play_time/3600,mp3_play_time%3600/60,mp3_play_time%3600%60,t/3600,t%3600/60,t%3600%60);	
// 						}	
// 					}else
// 					{		
// 						printf("播放完成\r\n");	
// 						AVS_Time_Stop(AVS_TIME_0);
// 						C_BIT(AUDIO_CODEC_CLK_REG,31);						
// 						break;//播放放成退出
// 					}
// 				}
// 			}
// 			/*播放退出*/
// 			if(play_exit==1)
// 			{
// 				play_exit=0;
// 				dmd_disable(0);
// 				printf("播放退出\r\n");
// 				AVS_Time_Stop(AVS_TIME_0);
// 				C_BIT(AUDIO_CODEC_CLK_REG,31);
// 			}
// 		}else printf("参数错误\r\n");
// 	}else printf("格式错误\r\n");
// 	MP3FreeDecoder(Mp3Decoder);
//   f_close(&f);	
// 	return 0;
// }

// /*
// 解码1帧
// 返回数据大小
// Mp3Decoder 初始化指针
// mp3FrameInfo 解码信息指针
// f FATFS文件指针
// outbuff 输出数据指针
// data_init 初始化参数
// read_addr 第一帧地址 配合data_init用
// 返回-3 找帧头错误
// 返回-2 解码错误过多
// 返回-1 播放完成
// */
// #define DATSIZE (512*30)
// #define FM 5 	//连续解码FM帧
// unsigned char rbuff[DATSIZE] __attribute__ ((aligned (4)));	
// unsigned char rbuff2[DATSIZE] __attribute__ ((aligned (4)));	
// int MP3_to_PCM_DATA(HMP3Decoder Mp3Decoder,MP3FrameInfo *mp3FrameInfo,FIL f,unsigned char *outbuff,int data_init,int read_addr)
// {
// int offset;
// static int res=0;
// static int 	ssddtt=0;
// static int _bytesLeft=0;//有效长度
// static unsigned char *readPtr;//读出指针
// unsigned int br;
// int outsize;
// 	/*初始化参数*/
// 	if(data_init==1)
// 	{
// 		res=0;
// 		_bytesLeft=0;
// 		ssddtt=read_addr;
// 		readPtr=NULL;
// 	}
// 	/*连续解码FM帧*/
// 	for(int s=0;s<FM;s++)	
// 	{	
// 		/*判断是否需要读入数据*/
// 		if(_bytesLeft<(MAINBUF_SIZE*2))
// 		{
// 			memcpy(&rbuff[0],&rbuff[DATSIZE-_bytesLeft],_bytesLeft); 
// 			/*读入数据*/
// 			f_lseek(&f,ssddtt);
// 			f_read(&f,rbuff+_bytesLeft,DATSIZE-_bytesLeft,&br);
// 			/*数据读完-播放完成*/
// 			if(br==0)
// 			{	
// 				if(s>0)//有解码数据-返回大小
// 				{
// 					outsize=(mp3FrameInfo->outputSamps*2*s);
// 					return outsize;
// 				}				
// 				return -1;//没有解码返回
// 			}
// 			ssddtt+=(DATSIZE-_bytesLeft);//偏移到下一下地址
// 			_bytesLeft=DATSIZE;//有效字节数
// 			readPtr=rbuff;//数据指针偏移到0	
// 		}
// 		/*找帧头*/
// 		offset= MP3FindSyncWord(readPtr,_bytesLeft);	
// 		if(offset>=0)
// 		{
// 			readPtr+=offset;
// 			_bytesLeft-=offset;
// 	  	/*解码帧-【数据指针+帧长度】与【有效长度-帧长度】*/
// 			res=MP3Decode(Mp3Decoder,&readPtr,&_bytesLeft,(short *)outbuff, 0);
// 			outbuff+=(mp3FrameInfo->outputSamps*2);
// 			if(res<0)return -2;
// 		}
// 		else return -3;//没有帧头
// 	}
// 	/*返回解码大小*/
// 	outsize=(mp3FrameInfo->outputSamps*2*FM);		
// 	return outsize;
// }

// /*识别ID3V2+
// f 文件句柄
// //TIT2：歌曲标题名字
// //TPE1：作者名字
// //TALB：作品专辑
// //编码UNICODE
// 返回数据大小
// //https://blog.csdn.net/qq_18661257/article/details/54908775
// */
// int Get_ID3V2(FIL f,char *TIT2,char *TPE1,char *TALB)
// {
// int s=0,i,l;
// int ID3_VER;	
// int ID3_SIZE;	
// char readbuff[5000];	
// int TIT2_size=0;
// int TPE1_size=0;
// int TALB_size=0;	
// int TIT2_F=0;
// int TPE1_F=0;
// int TALB_F=0;
// 	/*读入一段mp3文件*/
// 	f_lseek(&f,0);
// 	f_read(&f,readbuff,5000,NULL);		
// 		/*ID3_V2识别*/
// 	for(i=0;i<50;i++)
// 	{
// 		if((readbuff[i+0]=='I')&&(readbuff[i+1]=='D')&&(readbuff[i+2]=='3'))
// 		{ 
// 			ID3_VER=readbuff[i+3]*10+readbuff[i+4];
// 			ID3_SIZE = ((readbuff[i+9] & 0x7f) | ((readbuff[i+8] & 0x7f) << 7) | ((readbuff[i+7] & 0x7f) << 14) | ((readbuff[i+6] & 0x7f) << 21)) + 10;
// 		  printf("ID3V%d.%d \r\n",ID3_VER/10,ID3_VER%10);
// 			printf("标签大小=%d\r\n",ID3_SIZE);	
// 			s=ID3_SIZE;
// 			if(ID3_SIZE>5000)ID3_SIZE=5000;//超长ID3 可能有图片类
// 			for(;i<ID3_SIZE;i++)
// 			{
// 				if(TIT2_F==0)
// 				if((readbuff[i+0]=='T')&&(readbuff[i+1]=='I')&&(readbuff[i+2]=='T')&&(readbuff[i+3]=='2'))//TIT2
// 				{
// 					printf("TIT2=%d \r\n",i);
// 					TIT2_size=readbuff[i+4+0]<<24 | readbuff[i+4+1]<<16 | readbuff[i+4+2]<<8 | readbuff[i+4+3]<<0;
// //					printf("TIT2_size=%d \r\n",TIT2_size);
// 					for(l=0;l<(TIT2_size-3);l++)TIT2[l]=readbuff[i+10+3+l];
// //					for(l=0;l<(TIT2_size-3);l++)printf("0x%02x,",TIT2[l]);
// //					printf("\r\n");
// 					TIT2_F=1;
// 				}
// 				if(TPE1_F==0)
// 				if((readbuff[i+0]=='T')&&(readbuff[i+1]=='P')&&(readbuff[i+2]=='E')&&(readbuff[i+3]=='1'))//TPE1
// 				{
// 					printf("TPE1=%d \r\n",i);
// 					TPE1_size=readbuff[i+4+0]<<24 | readbuff[i+4+1]<<16 | readbuff[i+4+2]<<8 | readbuff[i+4+3]<<0;
// //					printf("TPE1_size=%d \r\n",TPE1_size);
// 					for(l=0;l<(TPE1_size-3);l++)TPE1[l]=readbuff[i+10+3+l];
// //					for(l=0;l<(TPE1_size-3);l++)printf("0x%02x,",TPE1[l]);
// //					printf("\r\n");
// 					TPE1_F=1;
// 				}
// 				if(TALB_F==0)
// 				if((readbuff[i+0]=='T')&&(readbuff[i+1]=='A')&&(readbuff[i+2]=='L')&&(readbuff[i+3]=='B'))//TALB
// 				{
// 					printf("TALB=%d \r\n",i);
// 					TALB_size=readbuff[i+4+0]<<24 | readbuff[i+4+1]<<16 | readbuff[i+4+2]<<8 | readbuff[i+4+3]<<0;
// //					printf("TALB_size=%d \r\n",TALB_size);
// 					for(l=0;l<(TALB_size-3);l++)TALB[l]=readbuff[i+10+3+l];
// //					for(l=0;l<(TALB_size-3);l++)printf("0x%02x,",TALB[l]);
// //					printf("\r\n");
// 					TALB_F=1;
// 				}
// 			}		
// 		}	
// 	}
// 	return s;
// }

// /*识别ID3V1
// f 文件句柄
// 返回数据大小
// //编码GBK2312
// */
// int Get_ID3V1(FIL f,char *TIT2,char *TPE1,char *TALB)
// {
// 	int l;
// 	/*读最后128字节*/
// 	char buffl[128];
// 	int Fsize=f_size(&f);
// 	f_lseek(&f,Fsize-128);
// 	f_read(&f,buffl,128,NULL);		
// 	if((buffl[0]=='T')&&(buffl[1]=='A')&&(buffl[2]=='G'))
// 	{
// 		printf("ID3V1\r\n");
// 		for(l=0;l<30;l++)TIT2[l]=buffl[3+l];
// 		for(l=0;l<30;l++)TPE1[l]=buffl[33+l];
// 		for(l=0;l<30;l++)TALB[l]=buffl[63+l];
// 		//编码GBK2312
// //		for(l=0;l<10;l++)printf("0x%02x,",TIT2[l]);
// //		printf("\r\n");	
// 		return 128;
// 	}
// 	return 0;
// }


// /*返回播放时长-ms
// ID3size ID3大小
// ID3V1_size ID3V1 大小
// f 文件句柄
// _mp3FrameInfo mp3信息指针
// */
// int Get_mp3_play_time_ms(int ID3size,int ID3V1_size,FIL f,MP3FrameInfo _mp3FrameInfo)
// {
// int time,i;
// int VBR_Frame_num;//总帧数
// int Fsize;//文件大小	
// char readbuff[5000];	
	
// 	/*读入一段mp3文件*/
// 	f_lseek(&f,ID3size);
// 	f_read(&f,readbuff,5000,NULL);
// 	//VBR 可变码率
// 	for(i=0;i<60;i++)
// 	{
// 		if(((readbuff[i+0]=='X')&&(readbuff[i+1]=='i')&&(readbuff[i+2]=='n')&&(readbuff[i+3]=='g'))||
// 			((readbuff[i+0]=='I')&&(readbuff[i+1]=='n')&&(readbuff[i+2]=='f')&&(readbuff[i+3]=='o')))
// 		{
// 			printf("Xing=%d \r\n",i);
// 			/*读总帧数*/
// 			VBR_Frame_num=readbuff[i+8+0]<<24 | readbuff[i+8+1]<<16 | readbuff[i+8+2]<<8 | readbuff[i+8+3]<<0;
// 			/*计算播放时长*/
// 			time=(float)VBR_Frame_num*((float)1000000/(float)_mp3FrameInfo.samprate*(float)samplesPerFrameTab[_mp3FrameInfo.version][_mp3FrameInfo.layer - 1])/(float)1000;
// 			return time;
// 		}
// 		if((readbuff[i+0]=='V')&&(readbuff[i+1]=='B')&&(readbuff[i+2]=='R')&&(readbuff[i+3]=='I'))
// 		{
// 			printf("VBRI=%d \r\n",i);
// 			/*读总帧数*/
// 			VBR_Frame_num=readbuff[i+14+0]<<24 | readbuff[i+14+1]<<16 | readbuff[i+14+2]<<8 | readbuff[i+14+3]<<0;
// 			/*计算播放时长*/
// 			time=(float)VBR_Frame_num*((float)1000000/(float)_mp3FrameInfo.samprate*(float)samplesPerFrameTab[_mp3FrameInfo.version][_mp3FrameInfo.layer - 1])/(float)1000;
// 			return time;
// 		}
// 	}
// 	//CBR 固定码率
// 	Fsize=f_size(&f); 
// 	//播放时长 = ( 文件大小 – ID3大小 ) × 8 ÷ 比特率(kbit/s)
// 	time=( Fsize - ID3size -ID3V1_size) * 8 / (_mp3FrameInfo.bitrate/1000);
// 	return time;
// }

					
//	i=0;
//  t=Read_time_ms();		
//S_BIT(AUDIO_CODEC_CLK_REG,31);	
//AVS_Time_Start(AVS_TIME_0);					
//	while(1)
//	{
//		if(((read32(AUDIO_BASE+8)>>8)&0xff)>0)
//		{
//			write32(AUDIO_BASE+0x0C,(wav_buff[0][i+1]<<8)+wav_buff[0][i]);
//			i+=2;
//			//printf("i=%d\r\n",i);
//		}	
//		if(i>=wav.data_size)
//		{
//			i=0;
//			printf("用时= %d ms\r\n",Read_time_ms()-t);
//			printf("Time: %d ms \r\n",AVS_Time_Read(AVS_TIME_0)/10);
//			t=Read_time_ms();
//			AVS_Time_Start(AVS_TIME_0);
//		}
//	}					

