#ifndef __FRAMEBUFFER_H__
#define __FRAMEBUFFER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <types.h>

#define PIXEL_LEN 4
extern uint32_t PixelsTotal;

typedef enum 
{
	PIXEL_FORMAT_ARGB32		= 0,
	PIXEL_FORMAT_RGB24		= 1,
	PIXEL_FORMAT_A8			= 2,
	PIXEL_FORMAT_A1			= 3,
	PIXEL_FORMAT_RGB16_565	= 4,
	PIXEL_FORMAT_RGB30		= 5,
}pixel_format_t;

typedef struct render_st{
	/* The width of render */
	uint32_t width;

	/* The height of render */
	uint32_t height;

	/* The pitch of one scan line */
	uint32_t pitch;

	/* Pixel format */
	pixel_format_t format;

	/* Pixel data */
	void * pixels;   

	/* Pixel data 2 */
	void * pixels2;   

	/* FULL DISPLAY data length */
	size_t pixlen;

	/* Private data */
	void * priv;
}render_t;

typedef struct framebuffer_st
{
	/* Framebuffer name */
	char * name;

	/* The width and height in pixel */
	int32_t width, height;

	/* display ram width and height, mainly use for scale display */
	int user_width, user_height;

	/* The physical size in millimeter */
	int32_t pwidth, pheight;

	/* The bit per pixel */
	int32_t bpp;

	/* Alone render - create by register */
	render_t * alone;

	/* Private data */
	void * priv;
}framebuffer_t;

extern render_t* render;
extern framebuffer_t fb_f1c100s;
extern void *writeBackRAM;

void fb_f1c100s_write_back_start(void);
void fb_f1c100s_write_back(void * data);
uint8_t fb_f1c100s_write_back_state(void);
void framebuffer_flush(void);
void fb_f1c100s_setbl(framebuffer_t * fb, int32_t brightness);
int32_t fb_f1c100s_getbl(framebuffer_t * fb);
void fb_f1c100s_change_hw(char BR, char Video);
void fb_f1c100s_flush_cache_set(void *addr);


render_t * fb_f1c100s_create(framebuffer_t * fb);

void fb_f1c100s_destroy(framebuffer_t * fb, render_t * render);

void fb_f1c100s_present(framebuffer_t * fb, render_t * render);

void fb_f1c100s_init(framebuffer_t * fb);

void fb_f1c100s_remove(framebuffer_t * fb);

void fb_f1c100s_suspend(framebuffer_t * fb);

void fb_f1c100s_resume(framebuffer_t * fb);


#ifdef __cplusplus
}
#endif

#endif /* __FRAMEBUFFER_H__ */
