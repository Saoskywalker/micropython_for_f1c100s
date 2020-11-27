#ifndef _TP_F1C_H
#define _TP_F1C_H

//ADC模式变换顺序为 [X1->X2->Y1->Y2] 32位FIFO连续存储
#define ADC_X1 1
#define ADC_X2 2
#define ADC_Y1 4
#define ADC_Y2 8
#define ADC_all 0xf

void Init_TP_ADC(int mode);
void Set_ADC_Channel(int Channel);
int TP_ADC_Read_Value(void);
unsigned char F1C_TP_Init(void);
unsigned char F1C_TP_Scan(unsigned char tp);

void TP_MODE_Demo(void);	
void ADC_MODE_Demo(void);

#endif
