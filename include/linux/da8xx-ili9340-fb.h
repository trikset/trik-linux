#ifndef __DA8XX_ILI9340_PLATFORM_DATA
#define __DA8XX_ILI9340_PLATFORM_DATA __FILE__

struct da8xx_ili9340_pdata {
	int	xres;
	int	yres;
	int	yres_screens;
	int	bits_per_pixel;
	int	screen_height;
	int	screen_width;
	int	fps;

	int	lidd_cs;
};


#endif /* __DA8XX_ILI9340_PLATFORM_DATA */
