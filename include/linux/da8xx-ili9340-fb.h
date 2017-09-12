#ifndef __DA8XX_ILI9340_PLATFORM_DATA
#define __DA8XX_ILI9340_PLATFORM_DATA __FILE__


enum da8xx_ili9340_pdata_lcdc_visual_mode {
	DA8XX_LCDC_VISUAL_INVALID = 0,
	DA8XX_LCDC_VISUAL_565,	// 16bps
	DA8XX_LCDC_VISUAL_8880,	// 32bps, 6bits per color, byte-aligned and 1 empty byte
};

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

enum da8xx_ili9340_pdata_display_gamma {
	DA8XX_LCDC_DISPLAY_GAMMA_DEFAULT = 0,
	DA8XX_LCDC_DISPLAY_GAMMA_1_0,
	DA8XX_LCDC_DISPLAY_GAMMA_1_8,
	DA8XX_LCDC_DISPLAY_GAMMA_2_2,
	DA8XX_LCDC_DISPLAY_GAMMA_2_5,
};

struct da8xx_ili9340_pdata {
	int	xres;
	int	yres;
	bool	xflip;
	bool	yflip;
	bool	xyswap;
	enum da8xx_ili9340_pdata_lcdc_visual_mode	visual_mode;

	int	screen_height;
	int	screen_width;
	int	fps;

	enum da8xx_ili9340_pdata_lcdc_lidd_bus_mode	lcdc_lidd_mode;
	enum da8xx_ili9340_pdata_lcdc_lidd_bus_cs	lcdc_lidd_cs;		//async mode only
	enum da8xx_ili9340_pdata_lcdc_lidd_bus_polarity	lcdc_lidd_ale_pol;
	enum da8xx_ili9340_pdata_lcdc_lidd_bus_polarity	lcdc_lidd_rs_en_pol;
	enum da8xx_ili9340_pdata_lcdc_lidd_bus_polarity	lcdc_lidd_ws_dir_pol;
	enum da8xx_ili9340_pdata_lcdc_lidd_bus_polarity	lcdc_lidd_cs_pol;
	unsigned long					lcdc_lidd_edma_burst;

	unsigned long					lcdc_lidd_mclk_ns;
	unsigned long 					lcdc_t_ta_ns;
	unsigned long					lcdc_t_rhold_ns;
	unsigned long					lcdc_t_rstrobe_ns;
	unsigned long					lcdc_t_rsu_ns;
	unsigned long					lcdc_t_whold_ns;
	unsigned long					lcdc_t_wstrobe_ns;
	unsigned long					lcdc_t_wsu_ns;

	unsigned long					display_t_reset_to_ready_ms;
	unsigned long					display_t_sleep_in_out_ms;

	bool						display_idle;
	bool						display_backlight;
	unsigned					display_brightness;
	bool						display_inversion;
	enum da8xx_ili9340_pdata_display_gamma		display_gamma;

	void	(*cb_power_ctrl)(bool _power_up);
	void	(*cb_backlight_ctrl)(bool _backlight);
};


#endif /* __DA8XX_ILI9340_PLATFORM_DATA */
