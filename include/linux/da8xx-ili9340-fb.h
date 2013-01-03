#ifndef __DA8XX_ILI9340_PLATFORM_DATA
#define __DA8XX_ILI9340_PLATFORM_DATA __FILE__


enum da8xx_ili9340_pdata_lcdc_lidd_bus_mode {
	DA8XX_LCDC_LIDD_MODE_INVALID = 0,
	DA8XX_LCDC_LIDD_MODE_6800SYNC,
	DA8XX_LCDC_LIDD_MODE_6800ASYNC,
	DA8XX_LCDC_LIDD_MODE_8080SYNC,
	DA8XX_LCDC_LIDD_MODE_8080ASYNC,
};

enum da8xx_ili9340_pdata_lcdc_lidd_bus_cs {
	DA8XX_LCDC_LIDD_CS0 = 0,
	DA8XX_LCDC_LIDD_CS1,
};

enum da8xx_ili9340_pdata_lcdc_lidd_bus_polarity {
	DA8XX_LCDC_LIDD_POL_DEFAULT = 0,
	DA8XX_LCDC_LIDD_POL_ACTIVE_HIGH,
	DA8XX_LCDC_LIDD_POL_ACTIVE_LOW,
};


struct da8xx_ili9340_pdata {
	int	xres;
	int	yres;
	int	yres_screens;
	int	bits_per_pixel;
	int	screen_height;
	int	screen_width;
	int	fps;

	enum da8xx_ili9340_pdata_lcdc_lidd_bus_mode	lcdc_lidd_mode;
	enum da8xx_ili9340_pdata_lcdc_lidd_bus_cs	lcdc_lidd_cs;		//async mode only
	enum da8xx_ili9340_pdata_lcdc_lidd_bus_polarity	lcdc_lidd_ale_pol;
	enum da8xx_ili9340_pdata_lcdc_lidd_bus_polarity	lcdc_lidd_rs_en_pol;
	enum da8xx_ili9340_pdata_lcdc_lidd_bus_polarity	lcdc_lidd_ws_dir_pol;
	enum da8xx_ili9340_pdata_lcdc_lidd_bus_polarity	lcdc_lidd_cs_pol;
	enum da8xx_ili9340_pdata_lcdc_lidd_bus_polarity	lcdc_lidd_reset_pol;
	unsigned long					lcdc_lidd_edma_burst;

	unsigned long					lcdc_lidd_mclk_ns;
	unsigned long 					lcdc_t_ta_ns;
	unsigned long					lcdc_t_rhold_ns;
	unsigned long					lcdc_t_rstrobe_ns;
	unsigned long					lcdc_t_rsu_ns;
	unsigned long					lcdc_t_whold_ns;
	unsigned long					lcdc_t_wstrobe_ns;
	unsigned long					lcdc_t_wsu_ns;
	unsigned long					lcdc_t_reset_ns;
	unsigned long					lcdc_t_reset_wait_ns;

};


#endif /* __DA8XX_ILI9340_PLATFORM_DATA */
