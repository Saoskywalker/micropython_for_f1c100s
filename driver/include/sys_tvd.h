/*
*********************************************************************************************************

* File    : tvd.h
* Version : v1.0
* Update  : date                auther         ver     notes
*           2016-01-14    charles cheng   3.0     upgrade the driver
*********************************************************************************************************
*/

#ifndef _TVD_H_
#define _TVD_H_
#define __s32 int
#define __u32 unsigned int 
#define EPDK_OK 1


typedef enum  e_TVD_SEL_SOURCE
{
    TVD_SOURCE_CVBS=1,
    TVD_SOURCE_YPbPr,

    TVD_SEL_SOURCE_MAX

}__drv_TVD_sel_source;

typedef enum  e_TVD_PAL_NTSC
{
	TVD_SOURCE_NTSC=1,
	TVD_SOURCE_PAL,
	TVD_SOURCE_PAL_M,
	TVD_SOURCE_PAL_N,
	TVD_SOURCE_SECAM,
	TVD_SYSTEM_FORMAT_MAX

}__drv_TVD_system;

typedef enum
{
    TVD_PL_YUV420                       =   0,  // ????????a??
    TVD_MB_YUV420                       =   1,  // ?????????
    TVD_PL_YUV422                       =   2,  // ????????a??
}TVD_FMT_T;


typedef enum
{
    TVD_NTSC=1,
    TVD_PAL,
}tvd_mode_t;

typedef enum
{
    TVD_FRAME_DONE=1,
    TVD_LOCK,
    TVD_UNLOCK,
}tvd_irq_t;

char TVD_state(void);
void TVD_Remove(void);
void f1c100s_tvd_AutoMode(void);
void f1c100s_tvd_AutoMode2(void);
void f1c100s_tvd_Init(void);
void	BSP_TVD_init(void);

__s32 com_video_set_contrast(__s32 val);
__s32 com_video_set_bright(__s32 val);
__s32 com_video_set_saturation(__s32 val);
__s32 com_video_set_hue(__s32 val);

void BSP_TVD_irq_enable(__u32 id,tvd_irq_t irq);
void BSP_TVD_irq_disable(__u32 id,tvd_irq_t irq);
__u32 BSP_TVD_irq_status_get(__u32 id,tvd_irq_t irq);
void BSP_TVD_irq_status_clear(__u32 id,tvd_irq_t irq);

void BSP_TVD_capture_on(__u32 id);
void BSP_TVD_capture_off(__u32 id);

void BSP_TVD_set_addr_y(__u32 id,__u32 addr);
void BSP_TVD_set_addr_c(__u32 id,__u32 addr);

#ifdef CONFIG_SOC_AW1707
void BSP_TVD_set_addr(__u32 id,__u32 addr_y, __u32 addr_c);
#endif

void BSP_TVD_set_width(__u32 id,__u32 w);
void BSP_TVD_set_width_jump(__u32 id,__u32 j);
void BSP_TVD_set_height(__u32 id,__u32 h);
void BSP_TVD_set_hor_start(__u32 id,__u32 h_start);
void BSP_TVD_set_ver_start(__u32 id,__u32 v_start);
__u32 BSP_TVD_get_hor_start(__u32 id);
__u32 BSP_TVD_get_ver_start(__u32 id);

#ifdef CONFIG_SOC_AW1707
void BSP_TVD_set_fmt(__u32 id,TVD_FMT_T fmt);
void BSP_TVD_config(__u32 interface, TVD_SYSTEM *system, __u32 format, __u32 input);

void BSP_TVD_3d_comb_fliter(void* _3d_addr);
void BSP_TVD_3d_comb_fliter_disable(void *_3d_addr);
void BSP_TVD_dig_filter(void);
void BSP_TVD_input_select(__u32 input);
__u32 BSP_TVD_get_system_status(__u32 id);
__s32 BSP_TVD_get_signal_dect(__u32 id);

__s32 BSP_TVD_set_saturation(__u32 id, __u32 saturation);
__s32 BSP_TVD_set_luma(__u32 id, __u32 luma);
__s32 BSP_TVD_set_contrast(__u32 id, __u32 contrast);
__u32 BSP_TVD_get_saturation(__u32 id);
__u32 BSP_TVD_get_luma(__u32 id);
__u32 BSP_TVD_get_contrast(__u32 id);
__s32 BSP_TVD_SET_BLUE(void);
void BSP_TVD_clk_select(__u32 input);
#endif 

#ifdef CONFIG_SOC_AW1663

//void	BSP_TVD_set_fmt(__u32 id,tvd_fmt_t fmt);
void	BSP_TVD_set_fmt(__u32 id,TVD_FMT_T fmt);
void	BSP_TVD_config(__u32 interface, __u32 system,  __u32 format);
__u32	BSP_TVD_get_status(__u32 id);
void 	BSP_TVD_3D_COMB_Filter(__u32 enable,__u32 addr);
void 	BSP_TVD_input_select(__u32 input);

#endif

#endif  /* _TVD_H_ */

