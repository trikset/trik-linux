#define DEBUG


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/da8xx-ili9340-fb.h>

#define DRIVER_NAME "da8xx_lcdc_ili9340"

#define __devinitexit




#define REGDEFSPLIT_MASK(reg_ofs, reg_sz)			(((1ull<<(reg_sz))-1ull) << (reg_ofs))
#define REGDEFSPLIT_SET_VALUE(reg_ofs, reg_sz, val)		(((val) << (reg_ofs)) & REGDEFSPLIT_MASK(reg_ofs, reg_sz))
#define REGDEFSPLIT_GET_VALUE(reg_ofs, reg_sz, reg_val)		(((reg_val) & REGDEFSPLIT_MASK(reg_ofs, reg_sz)) >> (reg_ofs))

#define REGDEF_MASK(reg_def)			REGDEFSPLIT_MASK(reg_def)
#define REGDEF_SET_VALUE(reg_def, val)		REGDEFSPLIT_SET_VALUE(reg_def, val)
#define REGDEF_GET_VALUE(reg_def, reg_val)	REGDEFSPLIT_GET_VALUE(reg_def, reg_val)




#define DA8XX_LCDCREG_REVID			0x00
#define DA8XX_LCDCREG_LCD_CTRL			0x04
#define DA8XX_LCDCREG_LCD_STAT			0x08
#define DA8XX_LCDCREG_LIDD_CTRL			0x0c
#define DA8XX_LCDCREG_LIDD_CS0_CONF		0x10
#define DA8XX_LCDCREG_LIDD_CS0_ADDR		0x14
#define DA8XX_LCDCREG_LIDD_CS0_DATA		0x18
#define DA8XX_LCDCREG_LIDD_CS1_CONF		0x1c
#define DA8XX_LCDCREG_LIDD_CS1_ADDR		0x20
#define DA8XX_LCDCREG_LIDD_CS1_DATA		0x24

#define DA8XX_LCDCREG_DMA_CTRL			0x40
#define DA8XX_LCDCREG_DMA_FB0_BASE		0x44
#define DA8XX_LCDCREG_DMA_FB0_CEILING		0x48
#define DA8XX_LCDCREG_DMA_FB1_BASE		0x4c
#define DA8XX_LCDCREG_DMA_FB1_CEILING		0x50


#define DA8XX_LCDCREG_REVID__REV			4,  (28)

#define DA8XX_LCDCREG_LCD_CTRL__MODESEL			0,  (1)
#define DA8XX_LCDCREG_LCD_CTRL__CLKDIV			8,  (8)

#define DA8XX_LCDCREG_LCD_STAT__DONE			0,  (1)
#define DA8XX_LCDCREG_LCD_STAT__EOF0			8,  (1)
#define DA8XX_LCDCREG_LCD_STAT__EOF1			9,  (1)

#define DA8XX_LCDCREG_LIDD_CTRL__MODE_SEL		0,  (3)
#define DA8XX_LCDCREG_LIDD_CTRL__ALEPOL			3,  (1)
#define DA8XX_LCDCREG_LIDD_CTRL__RS_EN_POL		4,  (1)
#define DA8XX_LCDCREG_LIDD_CTRL__WS_DIR_POL		5,  (1)
#define DA8XX_LCDCREG_LIDD_CTRL__CS0_E0_POL		6,  (1)
#define DA8XX_LCDCREG_LIDD_CTRL__CS1_E1_POL		7,  (1)
#define DA8XX_LCDCREG_LIDD_CTRL__LIDD_DMA_EN		8,  (1)
#define DA8XX_LCDCREG_LIDD_CTRL__DMA_CS0_CS1		9,  (1)
#define DA8XX_LCDCREG_LIDD_CTRL__DONE_INT_EN		10, (1)

#define DA8XX_LCDCREG_LIDD_CSn_CONF__TA			0,  (2)
#define DA8XX_LCDCREG_LIDD_CSn_CONF__R_HOLD		2,  (4)
#define DA8XX_LCDCREG_LIDD_CSn_CONF__R_STROBE		6,  (6)
#define DA8XX_LCDCREG_LIDD_CSn_CONF__R_SU		12, (5)
#define DA8XX_LCDCREG_LIDD_CSn_CONF__W_HOLD		17, (4)
#define DA8XX_LCDCREG_LIDD_CSn_CONF__W_STROBE		21, (6)
#define DA8XX_LCDCREG_LIDD_CSn_CONF__W_SU		27, (5)
#define DA8XX_LCDCREG_LIDD_CSn_ADDR__ADR_INDX		0,  (16)
#define DA8XX_LCDCREG_LIDD_CSn_DATA__DATA		0,  (16)

#define DA8XX_LCDCREG_DMA_CTRL__FRAME_MODE		0,  (1)
#define DA8XX_LCDCREG_DMA_CTRL__BIGENDIAN		1,  (1)
#define DA8XX_LCDCREG_DMA_CTRL__EOF_INT_EN		2,  (1)
#define DA8XX_LCDCREG_DMA_CTRL__BURST_SIZE		4,  (3)

#define DA8XX_LCDCREG_DMA_FBn_BASE__FBn_BASE		0,  (32)
#define DA8XX_LCDCREG_DMA_FBn_CEILING__FBn_CEIL		0,  (32)


#define DA8XX_LCDCREG_REVID__REV__id				0x4c10010 //Expected LCD controller ID: 4c10010x

#define DA8XX_LCDCREG_LCD_CTRL__MODESEL__lidd			0x0 //LCD controller in LIDD mode
#define DA8XX_LCDCREG_LCD_CTRL__CLKDIV__default			0x1 //MCLK to LCD_CLK by default

#define DA8XX_LCDCREG_LIDD_CTRL__MODE_SEL__6800sync		0x0 //MPU6800 sync
#define DA8XX_LCDCREG_LIDD_CTRL__MODE_SEL__6800async		0x1 //MPU6800 async
#define DA8XX_LCDCREG_LIDD_CTRL__MODE_SEL__8080sync		0x2 //8080 sync
#define DA8XX_LCDCREG_LIDD_CTRL__MODE_SEL__8080async		0x3 //8080 async
#define DA8XX_LCDCREG_LIDD_CTRL__xxxPOL__norm			0x0 //do not invert ALE / RS/EN / WS/DIR /CSn/En
#define DA8XX_LCDCREG_LIDD_CTRL__xxxPOL__inv			0x1 //invert ALE / RS/EN / WS/DIR /CSn/En
#define DA8XX_LCDCREG_LIDD_CTRL__LIDD_DMA_EN__deactivate	0x0 //deactivate DMA
#define DA8XX_LCDCREG_LIDD_CTRL__LIDD_DMA_EN__activate		0x1 //activate DMA
#define DA8XX_LCDCREG_LIDD_CTRL__DMA_CS0_CS1__cs0		0x0 //DMA writes to LIDD CS0
#define DA8XX_LCDCREG_LIDD_CTRL__DMA_CS0_CS1__cs1		0x1 //DMA writes to LIDD CS1
#define DA8XX_LCDCREG_LIDD_CTRL__DONE_INT_EN__enable		0x1 //Enable LIDD frame done interrupt
#define DA8XX_LCDCREG_LIDD_CTRL__DONE_INT_EN__disable		0x0 //Disable LIDD frame done interrupt

#define DA8XX_LCDCREG_DMA_CTRL__FRAME_MODE__single		0x0 //Single frame buffer (fb0)
#define DA8XX_LCDCREG_DMA_CTRL__BIGENDIAN__disable		0x0 //Big endian data reordering disabled
#define DA8XX_LCDCREG_DMA_CTRL__EOF_INT_EN__enable		0x1 //End of frame (CS0/CS1) interrupt enabled
#define DA8XX_LCDCREG_DMA_CTRL__EOF_INT_EN__disable		0x0 //End of frame (CS0/CS1) interrupt disabled
#define DA8XX_LCDCREG_DMA_CTRL__BURST_SIZE__1			0x0 //Burst size of 1
#define DA8XX_LCDCREG_DMA_CTRL__BURST_SIZE__2			0x1 //Burst size of 2
#define DA8XX_LCDCREG_DMA_CTRL__BURST_SIZE__4			0x2 //Burst size of 4
#define DA8XX_LCDCREG_DMA_CTRL__BURST_SIZE__8			0x3 //Burst size of 8
#define DA8XX_LCDCREG_DMA_CTRL__BURST_SIZE__16			0x4 //Burst size of 16

#define DA8XX_LCDCREG_DMA_FBn_BASE__ALIGNMENT			0x4 //DMA address alignment


#warning NOTE ON TIMINGS:
/* Page 216:
   Hardware reset -> Level2 command: 120ms
   Sleep out -> Display on sequence: 60ms

   Page 211:
   Sleep in -> power off: 120ms

   Page 90, 100, 101:
   Software reset -> any command: 5ms
   Software reset -> sleep out: 120ms
   Sleep in/out -> sleep out/in: 120ms
   Sleep in/out -> any command: 5ms
   Sleep out -> self diagnostics: 5ms
   Sleep out -> supplier's value loaded to registers: 120ms
*/
#warning TODO there is 120ms delay in power_ctrl(1) now, but maybe it will be better to use drivers timers; two timers should be added - any cmd and sleep cmd only
#warning TODO remove or configure msleeps


#define ILI9340_CMD_NOP				0x00
#define ILI9340_CMD_READID			0x04
#define ILI9340_CMD_READ_SELFDIAG		0x0f
#define ILI9340_CMD_SLEEP_IN			0x10
#define ILI9340_CMD_SLEEP_OUT			0x11
#define ILI9340_CMD_DISPLAY_OFF			0x28
#define ILI9340_CMD_DISPLAY_ON			0x29
#define ILI9340_CMD_MEMORY_WRITE		0x2c


#define ILI9340_CMD_READ_SELFDIAG__RLD		7, (1)
#define ILI9340_CMD_READ_SELFDIAG__FD		6, (1)



static void	lcdc_schedule_redraw_work(struct fb_info* _info);
static void	lcdc_wait_redraw_work_completion(struct fb_info* _info);
static void	lcdc_redraw_work(struct work_struct* _work);
static void	lcdc_redraw_work_done(struct fb_info* _info);
static void 		lcdc_edma_start(struct device* _dev);
static irqreturn_t	lcdc_edma_done(int _irq, void* _dev);


static ssize_t	fbops_write(struct fb_info* _info, const char __user* _buf, size_t _count, loff_t* _ppos);
static int	fbops_pan_display(struct fb_var_screeninfo* _var, struct fb_info* _info);
static int	fbops_blank(int _blank, struct fb_info* _info);
static int	fbops_sync(struct fb_info* _info);

static void	defio_redraw(struct fb_info* _info, struct list_head* _pagelist);




struct da8xx_ili9340_par {
	__u32		pseudo_palette[16];

	struct fb_info*			fb_info;
	enum da8xx_ili9340_pdata_lcdc_visual_mode	fb_visual_mode;
	struct mutex			lcdc_lock;

	dma_addr_t			fb_dma_phaddr;
	size_t				fb_dma_phsize;

	resource_size_t			lcdc_reg_start;
	resource_size_t			lcdc_reg_size;
	void __iomem*			lcdc_reg_base;
	int				lcdc_irq;
	struct clk*			lcdc_clk;

	struct work_struct		lcdc_redraw_work;

	loff_t		lidd_reg_cs_conf;
	loff_t		lidd_reg_cs_addr;
	loff_t		lidd_reg_cs_data;

	void (*cb_backlight_ctrl)(bool _backlight);
	void (*cb_power_ctrl)(bool _power_up);
};

static const struct fb_fix_screeninfo da8xx_ili9340_fix_init __devinitconst = {
	.id		= "DA8xx ILI9340",
	//.smem_start, .smem_len
	.type		= FB_TYPE_PACKED_PIXELS,
	.type_aux	= 0,
	.visual		= FB_VISUAL_TRUECOLOR,
	.xpanstep	= 0,
	.ypanstep	= 1, // allow vertical panning
	.ywrapstep	= 0,
	//.line_length
	.mmio_start	= 0,
	.mmio_len	= 0,
	.accel		= FB_ACCEL_NONE,
	.capabilities	= 0,
};

static const struct fb_var_screeninfo da8xx_ili9340_var_init __devinitconst = {
	//.xres, .yres, .xres_virtual, .yres_virtual
	.xoffset	= 0,
	.yoffset	= 0,
	//.bits_per_pixel
	.grayscale	= 0,
	.red		= { .msb_right = 0 },
	.green		= { .msb_right = 0 },
	.blue		= { .msb_right = 0 },
	.transp		= { .msb_right = 0, .offset = 0, .length = 0 },
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	//.height, .width
	.accel_flags	= 0,
	//.pixclock, .left_margin, .right_margin, .upper_margin, .lower_margin, .hsync_len, .vsync_len
	.sync		= FB_SYNC_VERT_HIGH_ACT,
	.vmode		= FB_VMODE_NONINTERLACED,
	.rotate		= FB_ROTATE_UR,
	.colorspace	= 0,
};

static struct fb_deferred_io da8xx_ili9340_defio = {
	//.delay
	.deferred_io		= defio_redraw,
};

static struct fb_ops da8xx_ili9340_fbops = {
	.owner		= THIS_MODULE,
	.fb_read	= &fb_sys_read,
	.fb_write	= &fbops_write,
	.fb_fillrect	= &sys_fillrect,
	.fb_copyarea	= &sys_copyarea,
	.fb_imageblit	= &sys_imageblit,
	.fb_pan_display	= &fbops_pan_display,
	.fb_blank	= &fbops_blank,
	.fb_sync	= &fbops_sync,
};





static inline void lcdc_lock(struct da8xx_ili9340_par* _par)
{
	mutex_lock(&_par->lcdc_lock);
}

static inline void lcdc_unlock(struct da8xx_ili9340_par* _par)
{
	mutex_unlock(&_par->lcdc_lock);
}

static inline __u32 lcdc_reg_read(struct device* _dev, struct da8xx_ili9340_par* _par, loff_t _reg)
{
	void __iomem*	reg_ptr		= _par->lcdc_reg_base + _reg;

	BUG_ON(_par->lcdc_reg_base == NULL);
	BUG_ON(!mutex_is_locked(&_par->lcdc_lock));

	return __raw_readl(reg_ptr);
}

static inline void lcdc_reg_write(struct device* _dev, struct da8xx_ili9340_par* _par, loff_t _reg, __u32 _value)
{
	void __iomem*	reg_ptr		= _par->lcdc_reg_base + _reg;

	BUG_ON(_par->lcdc_reg_base == NULL);
	BUG_ON(!mutex_is_locked(&_par->lcdc_lock));

	__raw_writel(_value, reg_ptr);
}

static inline void lcdc_reg_change(struct device* _dev, struct da8xx_ili9340_par* _par, loff_t _reg, __u32 _mask, __u32 _value)
{
	void __iomem*	reg_ptr		= _par->lcdc_reg_base + _reg;

	BUG_ON(_par->lcdc_reg_base == NULL);
	BUG_ON(!mutex_is_locked(&_par->lcdc_lock));

	__raw_writel((__raw_readl(reg_ptr) & ~_mask) | _value, reg_ptr);
}

static inline __u32 display_read_data(struct device* _dev, struct da8xx_ili9340_par* _par)
{
	return lcdc_reg_read(_dev, _par, _par->lidd_reg_cs_data);
}

static inline void display_write_cmd(struct device* _dev, struct da8xx_ili9340_par* _par, __u32 _cmd)
{
	lcdc_reg_write(_dev, _par, _par->lidd_reg_cs_addr, _cmd);
}

static inline void display_write_data(struct device* _dev, struct da8xx_ili9340_par* _par, __u32 _data)
{
	lcdc_reg_write(_dev, _par, _par->lidd_reg_cs_data, _data);
}




static ssize_t	fbops_write(struct fb_info* _info, const char __user* _buf, size_t _count, loff_t* _ppos)
{
	ssize_t ret;
	struct device* dev		= _info->device;

	dev_dbg(dev, "%s: called\n", __func__);
	ret = fb_sys_write(_info, _buf, _count, _ppos);
	lcdc_schedule_redraw_work(_info);
	dev_dbg(dev, "%s: done\n", __func__);

	return ret;
}

static int fbops_pan_display(struct fb_var_screeninfo* _var, struct fb_info* _info)
{
	struct device* dev		= _info->device;

	dev_dbg(dev, "%s: called\n", __func__);
#warning TODO pan ?
	lcdc_schedule_redraw_work(_info);
	dev_dbg(dev, "%s: done\n", __func__);

	return 0;
}

static int fbops_blank(int _blank, struct fb_info* _info)
{
	struct device* dev		= _info->device;
	dev_dbg(dev, "%s: called\n", __func__);
#warning TODO backlight control?
	dev_dbg(dev, "%s: done\n", __func__);
	return 0;
}

static int fbops_sync(struct fb_info* _info)
{
	struct device* dev		= _info->device;

	dev_dbg(dev, "%s: called\n", __func__);
	lcdc_wait_redraw_work_completion(_info);
	dev_dbg(dev, "%s: done\n", __func__);
	return 0;
}




static void defio_redraw(struct fb_info* _info, struct list_head* _pagelist)
{
	lcdc_schedule_redraw_work(_info);
}

static void lcdc_edma_start(struct device* _dev)
{
	struct fb_info* info		= dev_get_drvdata(_dev);
	struct da8xx_ili9340_par* par	= info->par;
	dma_addr_t phaddr_base;
	dma_addr_t phaddr_ceil;

	lcdc_lock(par);

	if (info->var.xoffset != 0)
		dev_warn(_dev, "%s: xoffset != 0\n", __func__);
	phaddr_base = par->fb_dma_phaddr + info->var.yoffset*info->fix.line_length;
	phaddr_ceil = phaddr_base + info->var.yres*info->fix.line_length - 1;

	display_write_cmd(_dev, par, ILI9340_CMD_NOP);
	display_write_cmd(_dev, par, ILI9340_CMD_MEMORY_WRITE);

	// Set EDMA address
	lcdc_reg_write(_dev, par,
			DA8XX_LCDCREG_DMA_FB0_BASE,
			REGDEF_SET_VALUE(DA8XX_LCDCREG_DMA_FBn_BASE__FBn_BASE, phaddr_base));
	lcdc_reg_write(_dev, par,
			DA8XX_LCDCREG_DMA_FB0_CEILING,
			REGDEF_SET_VALUE(DA8XX_LCDCREG_DMA_FBn_CEILING__FBn_CEIL, phaddr_ceil));

	// Kick EDMA on
	lcdc_reg_change(_dev, par,
			DA8XX_LCDCREG_LIDD_CTRL,
			REGDEF_MASK(DA8XX_LCDCREG_LIDD_CTRL__LIDD_DMA_EN),
			REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CTRL__LIDD_DMA_EN, DA8XX_LCDCREG_LIDD_CTRL__LIDD_DMA_EN__activate));
	lcdc_reg_change(_dev, par,
			DA8XX_LCDCREG_LIDD_CTRL,
			REGDEF_MASK(DA8XX_LCDCREG_LIDD_CTRL__LIDD_DMA_EN),
			REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CTRL__LIDD_DMA_EN, DA8XX_LCDCREG_LIDD_CTRL__LIDD_DMA_EN__deactivate));
}

static irqreturn_t lcdc_edma_done(int _irq, void* _dev)
{
	struct device* dev		= _dev;
	struct fb_info* info		= dev_get_drvdata(dev);
	struct da8xx_ili9340_par* par	= info->par;

	lcdc_reg_change(dev, par, DA8XX_LCDCREG_LCD_STAT, 0x0, 0x0); //Write STAT register back
	display_write_cmd(dev, par, ILI9340_CMD_NOP);

	lcdc_unlock(par);
	lcdc_redraw_work_done(info);

	return IRQ_HANDLED;
}

static void lcdc_schedule_redraw_work(struct fb_info* _info)
{
	struct device* dev			= _info->device;
	struct da8xx_ili9340_par* par		= _info->par;

	dev_dbg(dev, "%s: called\n", __func__);
	schedule_work(&par->lcdc_redraw_work);
	dev_dbg(dev, "%s: done\n", __func__);
}

static void lcdc_wait_redraw_work_completion(struct fb_info* _info)
{
	struct device* dev			= _info->device;
	struct da8xx_ili9340_par* par		= _info->par;

	dev_dbg(dev, "%s: called\n", __func__);

#warning TODO wait for redraw work done

	dev_dbg(dev, "%s: done\n", __func__);
}




static void lcdc_redraw_work(struct work_struct* _work)
{
	struct da8xx_ili9340_par* par	= container_of(_work, struct da8xx_ili9340_par, lcdc_redraw_work);
	struct fb_info* info		= par->fb_info;
	struct device* dev		= info->device;

	dev_dbg(dev, "%s: starting redraw work\n", __func__);
	lcdc_edma_start(dev);
}

static void lcdc_redraw_work_done(struct fb_info* _info)
{
	struct device* dev			= _info->device;
	struct da8xx_ili9340_par* par		= _info->par;

	dev_dbg(dev, "%s: called\n", __func__);

#warning TODO notify work done

	dev_dbg(dev, "%s: done\n", __func__);
}




static int __devinit da8xx_ili9340_fb_init(struct platform_device* _pdevice, struct da8xx_ili9340_pdata* _pdata)
{
	int ret		= -EINVAL;
	struct device* dev			= &_pdevice->dev;
	struct fb_info* info			= platform_get_drvdata(_pdevice);
	struct da8xx_ili9340_par* par		= info->par;

	dev_dbg(dev, "%s: called\n", __func__);

	info->flags		= FBINFO_DEFAULT | FBINFO_VIRTFB | FBINFO_HWACCEL_NONE;
	info->fix		= da8xx_ili9340_fix_init;
	info->var		= da8xx_ili9340_var_init;
	info->fbops		= &da8xx_ili9340_fbops;
	info->pseudo_palette	= par->pseudo_palette;

	info->var.xres		= _pdata->xres;
	info->var.xres_virtual	= info->var.xres;
	info->var.yres		= _pdata->yres;
	info->var.yres_virtual	= info->var.yres * (info->fix.ypanstep?2:1);
	info->var.height	= _pdata->screen_height;
	info->var.width		= _pdata->screen_width;

	switch (_pdata->visual_mode) {
		case DA8XX_LCDC_VISUAL_565: // RGB565, 16bits
			info->var.blue.offset		= 0;
			info->var.blue.length		= 5;
			info->var.green.offset		= 5;
			info->var.green.length		= 6;
			info->var.red.offset		= 11;
			info->var.red.length		= 5;
			info->var.bits_per_pixel	= 16;
			break;
		case DA8XX_LCDC_VISUAL_666: // RGB666, 24bits
			info->var.blue.offset		= 0;
			info->var.blue.length		= 6;
			info->var.green.offset		= 6;
			info->var.green.length		= 6;
			info->var.red.offset		= 12;
			info->var.red.length		= 6;
			info->var.bits_per_pixel	= 24;
			break;
		case DA8XX_LCDC_VISUAL_888: // RGB888, 24bits
			info->var.blue.offset		= 0;
			info->var.blue.length		= 8;
			info->var.green.offset		= 8;
			info->var.green.length		= 8;
			info->var.red.offset		= 16;
			info->var.red.length		= 8;
			info->var.bits_per_pixel	= 24;
			break;
		case DA8XX_LCDC_VISUAL_8880: // RGB888, 32bits
			info->var.blue.offset		= 0;
			info->var.blue.length		= 8;
			info->var.green.offset		= 8;
			info->var.green.length		= 8;
			info->var.red.offset		= 16;
			info->var.red.length		= 8;
			info->var.bits_per_pixel	= 32;
			break;
		default:
			dev_err(dev, "%s: unsupported visual mode %u\n", __func__, (unsigned)_pdata->visual_mode);
			ret = -EINVAL;
			goto exit;
	}
	par->fb_visual_mode = _pdata->visual_mode;

	info->fix.line_length		= DIV_ROUND_UP(info->var.bits_per_pixel, BITS_PER_BYTE) * info->var.xres_virtual;
	if (ALIGN(info->fix.line_length, DA8XX_LCDCREG_DMA_FBn_BASE__ALIGNMENT) != info->fix.line_length) {
		dev_warn(dev, "%s: framebuffer line is not alignment on LCD controller DMA boundary, y-axis padding disabled\n", __func__);
		info->fix.ypanstep = 0; // disable panning
	}
	info->screen_size		= info->fix.line_length * info->var.yres_virtual;
	par->fb_dma_phsize		= ALIGN(info->screen_size, DA8XX_LCDCREG_DMA_FBn_BASE__ALIGNMENT);
	if (par->fb_dma_phsize != info->screen_size)
		dev_warn(dev, "%s: whole framebuffer is not alignment on LCD controller DMA boundary\n", __func__);

	info->screen_base	= dma_alloc_coherent(dev, par->fb_dma_phsize, &par->fb_dma_phaddr, GFP_KERNEL | GFP_DMA);
	if (info->screen_base == NULL) {
		dev_err(dev, "%s: cannot allocate EDMA screen buffer\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	memset(info->screen_base, 0, info->screen_size);
	info->fix.smem_start	= virt_to_phys(info->screen_base);
	info->fix.smem_len	= info->screen_size;

	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret) {
		dev_err(dev, "%s: cannot allocated colormap: %d\n", __func__, ret);
		goto exit_free_dma;
	}

	info->fbdefio		= &da8xx_ili9340_defio;
	info->fbdefio->delay	= (HZ / _pdata->fps);

	fb_deferred_io_init(info);

	dev_dbg(dev, "%s: done\n", __func__);

	return 0;


 //exit_defio_cleanup:
	fb_deferred_io_cleanup(info);
 //exit_dealloc_cmap:
	fb_dealloc_cmap(&info->cmap);
 exit_free_dma:
	dma_free_coherent(dev, par->fb_dma_phsize, info->screen_base, par->fb_dma_phaddr);
 exit:
	return ret;
}




static void __devinitexit da8xx_ili9340_fb_shutdown(struct platform_device* _pdevice)
{
	struct device* dev			= &_pdevice->dev;
	struct fb_info* info			= platform_get_drvdata(_pdevice);
	struct da8xx_ili9340_par* par		= info->par;

	dev_dbg(dev, "%s: called\n", __func__);

	fb_deferred_io_cleanup(info);
	fb_dealloc_cmap(&info->cmap);
	dma_free_coherent(dev, par->fb_dma_phsize, info->screen_base, par->fb_dma_phaddr);

	dev_dbg(dev, "%s: done\n", __func__);
}




static int __devinit da8xx_ili9340_lidd_regs_init(struct platform_device* _pdevice, struct da8xx_ili9340_pdata* _pdata)
{
	int ret					= -EINVAL;
	struct device* dev			= &_pdevice->dev;
	struct fb_info* info			= platform_get_drvdata(_pdevice);
	struct da8xx_ili9340_par* par		= info->par;

	__u32 regval;
	__u32 revid;

	unsigned long lcdc_clk_khz;
	unsigned long lcdc_mclk_ns;
	__u32 lidd_mclk_div;

	bool lidd_is_async;
	__u32 lidd_ctrl_mode;
	__u32 lidd_dma_cs;
	__u32 lidd_ctrl_ale_pol;
	__u32 lidd_ctrl_rs_en_pol;
	__u32 lidd_ctrl_ws_dir_pol;
	__u32 lidd_ctrl_cs0_e0_pol;
	__u32 lidd_ctrl_cs1_e1_pol;
	__u32 lidd_dma_burst_size;

	dev_dbg(dev, "%s: called\n", __func__);

	lcdc_lock(par);
	regval = lcdc_reg_read(dev, par, DA8XX_LCDCREG_REVID);
	lcdc_unlock(par);
	revid = REGDEF_GET_VALUE(DA8XX_LCDCREG_REVID__REV, regval);
	if (revid != DA8XX_LCDCREG_REVID__REV__id) {
		dev_err(dev, "%s: detected wrong LCD controller REVID %x, expected %x\n", __func__, (unsigned)revid, (unsigned)DA8XX_LCDCREG_REVID__REV__id);
		ret = -EINVAL;
		goto exit;
	}

	lcdc_clk_khz	= clk_get_rate(par->lcdc_clk);
	lidd_mclk_div	= DIV_ROUND_UP(lcdc_clk_khz * _pdata->lcdc_lidd_mclk_ns, USEC_PER_SEC);
	if (lidd_mclk_div == 0)
		lidd_mclk_div = 1;
	lcdc_mclk_ns = USEC_PER_SEC / (lcdc_clk_khz/lidd_mclk_div);
        dev_dbg(dev, "%s: configuring LCD controller MCLK div %lu, MCLK will be %uns with %uns configured\n",
			__func__, (unsigned long)lidd_mclk_div, (unsigned)_pdata->lcdc_lidd_mclk_ns, (unsigned)lcdc_mclk_ns);

	switch (_pdata->lcdc_lidd_mode) {
		case DA8XX_LCDC_LIDD_MODE_6800SYNC:
			lidd_ctrl_mode = DA8XX_LCDCREG_LIDD_CTRL__MODE_SEL__6800sync;
			lidd_is_async  = false;
			break;
		case DA8XX_LCDC_LIDD_MODE_6800ASYNC:
			lidd_ctrl_mode = DA8XX_LCDCREG_LIDD_CTRL__MODE_SEL__6800async;
			lidd_is_async  = true;
			break;
		case DA8XX_LCDC_LIDD_MODE_8080SYNC:
			lidd_ctrl_mode = DA8XX_LCDCREG_LIDD_CTRL__MODE_SEL__8080sync;
			lidd_is_async  = false;
			break;
		case DA8XX_LCDC_LIDD_MODE_8080ASYNC:
			lidd_ctrl_mode = DA8XX_LCDCREG_LIDD_CTRL__MODE_SEL__8080async;
			lidd_is_async  = true;
			break;
		default:
			dev_err(dev, "%s: unsupported LCD controller LIDD mode %d\n", __func__, (int)_pdata->lcdc_lidd_mode);
			ret = -EINVAL;
			goto exit;
	}

	if (lidd_is_async) {
		switch (_pdata->lcdc_lidd_cs) {
			case DA8XX_LCDC_LIDD_CS0:
				par->lidd_reg_cs_conf	= DA8XX_LCDCREG_LIDD_CS0_CONF;
				par->lidd_reg_cs_addr	= DA8XX_LCDCREG_LIDD_CS0_ADDR;
				par->lidd_reg_cs_data	= DA8XX_LCDCREG_LIDD_CS0_DATA;
				lidd_dma_cs		= DA8XX_LCDCREG_LIDD_CTRL__DMA_CS0_CS1__cs0;
				dev_dbg(dev, "%s: using CS0\n", __func__);
				break;
			case DA8XX_LCDC_LIDD_CS1:
				par->lidd_reg_cs_conf	= DA8XX_LCDCREG_LIDD_CS1_CONF;
				par->lidd_reg_cs_addr	= DA8XX_LCDCREG_LIDD_CS1_ADDR;
				par->lidd_reg_cs_data	= DA8XX_LCDCREG_LIDD_CS1_DATA;
				lidd_dma_cs		= DA8XX_LCDCREG_LIDD_CTRL__DMA_CS0_CS1__cs1;
				dev_dbg(dev, "%s: using CS1\n", __func__);
				break;
			default:
				dev_err(dev, "%s: unsupported LCD controller chip-select %d\n", __func__, (int)_pdata->lcdc_lidd_cs);
				ret = -EINVAL;
				goto exit;
		}
	} else {
		if (_pdata->lcdc_lidd_cs != DA8XX_LCDC_LIDD_CS0)
			dev_warn(dev, "%s: ignoring CS %d in sync mode, enforcing CS0\n", __func__, (int)_pdata->lcdc_lidd_cs);
		par->lidd_reg_cs_conf	= DA8XX_LCDCREG_LIDD_CS0_CONF;
		par->lidd_reg_cs_addr	= DA8XX_LCDCREG_LIDD_CS0_ADDR;
		par->lidd_reg_cs_data	= DA8XX_LCDCREG_LIDD_CS0_DATA;
		lidd_dma_cs		= DA8XX_LCDCREG_LIDD_CTRL__DMA_CS0_CS1__cs0;
		dev_dbg(dev, "%s: enforcing CS0\n", __func__);
	}

	switch (_pdata->lcdc_lidd_ale_pol) {
		case DA8XX_LCDC_LIDD_POL_DEFAULT:
		case DA8XX_LCDC_LIDD_POL_ACTIVE_LOW:	lidd_ctrl_ale_pol = DA8XX_LCDCREG_LIDD_CTRL__xxxPOL__norm; break;
		case DA8XX_LCDC_LIDD_POL_ACTIVE_HIGH:	lidd_ctrl_ale_pol = DA8XX_LCDCREG_LIDD_CTRL__xxxPOL__inv; break;
		default:
			dev_err(dev, "%s: unsupported LCD controller ALE polarity %d\n", __func__, (int)_pdata->lcdc_lidd_ale_pol);
			ret = -EINVAL;
			goto exit;
	}

	switch (_pdata->lcdc_lidd_rs_en_pol) {
		case DA8XX_LCDC_LIDD_POL_DEFAULT:
		case DA8XX_LCDC_LIDD_POL_ACTIVE_LOW:	lidd_ctrl_rs_en_pol = DA8XX_LCDCREG_LIDD_CTRL__xxxPOL__norm; break;
		case DA8XX_LCDC_LIDD_POL_ACTIVE_HIGH:	lidd_ctrl_rs_en_pol = DA8XX_LCDCREG_LIDD_CTRL__xxxPOL__inv; break;
		default:
			dev_err(dev, "%s: unsupported LCD controller RS/EN polarity %d\n", __func__, (int)_pdata->lcdc_lidd_rs_en_pol);
			ret = -EINVAL;
			goto exit;
	}

	switch (_pdata->lcdc_lidd_ws_dir_pol) {
		case DA8XX_LCDC_LIDD_POL_DEFAULT:
		case DA8XX_LCDC_LIDD_POL_ACTIVE_LOW:	lidd_ctrl_ws_dir_pol = DA8XX_LCDCREG_LIDD_CTRL__xxxPOL__norm; break;
		case DA8XX_LCDC_LIDD_POL_ACTIVE_HIGH:	lidd_ctrl_ws_dir_pol = DA8XX_LCDCREG_LIDD_CTRL__xxxPOL__inv; break;
		default:
			dev_err(dev, "%s: unsupported LCD controller WS/DIR polarity %d\n", __func__, (int)_pdata->lcdc_lidd_ws_dir_pol);
			ret = -EINVAL;
			goto exit;
	}

	switch (_pdata->lcdc_lidd_cs_pol) {
		case DA8XX_LCDC_LIDD_POL_DEFAULT:	lidd_ctrl_cs0_e0_pol = DA8XX_LCDCREG_LIDD_CTRL__xxxPOL__norm;
							lidd_ctrl_cs1_e1_pol = DA8XX_LCDCREG_LIDD_CTRL__xxxPOL__norm;	break;
		case DA8XX_LCDC_LIDD_POL_ACTIVE_LOW:	lidd_ctrl_cs0_e0_pol = DA8XX_LCDCREG_LIDD_CTRL__xxxPOL__norm;
							lidd_ctrl_cs1_e1_pol = DA8XX_LCDCREG_LIDD_CTRL__xxxPOL__inv;	break;
		case DA8XX_LCDC_LIDD_POL_ACTIVE_HIGH:	lidd_ctrl_cs0_e0_pol = DA8XX_LCDCREG_LIDD_CTRL__xxxPOL__inv;
							lidd_ctrl_cs1_e1_pol = DA8XX_LCDCREG_LIDD_CTRL__xxxPOL__norm;	break;
		default:
			dev_err(dev, "%s: unsupported LCD controller CS polarity %d\n", __func__, (int)_pdata->lcdc_lidd_cs_pol);
			ret = -EINVAL;
			goto exit;
	}

	if (_pdata->lcdc_lidd_edma_burst < 2)
		lidd_dma_burst_size	= DA8XX_LCDCREG_DMA_CTRL__BURST_SIZE__1;
	else if (_pdata->lcdc_lidd_edma_burst < 4)
		lidd_dma_burst_size	= DA8XX_LCDCREG_DMA_CTRL__BURST_SIZE__2;
	else if (_pdata->lcdc_lidd_edma_burst < 8)
		lidd_dma_burst_size	= DA8XX_LCDCREG_DMA_CTRL__BURST_SIZE__4;
	else if (_pdata->lcdc_lidd_edma_burst < 16)
		lidd_dma_burst_size	= DA8XX_LCDCREG_DMA_CTRL__BURST_SIZE__8;
	else
		lidd_dma_burst_size	= DA8XX_LCDCREG_DMA_CTRL__BURST_SIZE__16;

	dev_dbg(dev, "%s: applying registers\n", __func__);
	lcdc_lock(par);

	lcdc_reg_write(dev, par, DA8XX_LCDCREG_LCD_CTRL,
			0
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LCD_CTRL__MODESEL,	DA8XX_LCDCREG_LCD_CTRL__MODESEL__lidd)
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LCD_CTRL__CLKDIV,	lidd_mclk_div));

	lcdc_reg_write(dev, par, DA8XX_LCDCREG_LIDD_CTRL,
			0
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CTRL__MODE_SEL,		lidd_ctrl_mode)
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CTRL__ALEPOL,		lidd_ctrl_ale_pol)
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CTRL__RS_EN_POL,		lidd_ctrl_rs_en_pol)
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CTRL__WS_DIR_POL,		lidd_ctrl_ws_dir_pol)
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CTRL__CS0_E0_POL,		lidd_ctrl_cs0_e0_pol)
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CTRL__CS1_E1_POL,		lidd_ctrl_cs1_e1_pol)
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CTRL__LIDD_DMA_EN,	DA8XX_LCDCREG_LIDD_CTRL__LIDD_DMA_EN__deactivate)
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CTRL__DMA_CS0_CS1,	lidd_dma_cs)
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CTRL__DONE_INT_EN,	DA8XX_LCDCREG_LIDD_CTRL__DONE_INT_EN__enable));

	lcdc_reg_write(dev, par, par->lidd_reg_cs_conf,
			0
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CSn_CONF__TA,		DIV_ROUND_UP(_pdata->lcdc_t_ta_ns,	lcdc_mclk_ns))
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CSn_CONF__R_HOLD,		DIV_ROUND_UP(_pdata->lcdc_t_rhold_ns,	lcdc_mclk_ns))
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CSn_CONF__R_STROBE,	DIV_ROUND_UP(_pdata->lcdc_t_rstrobe_ns,	lcdc_mclk_ns))
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CSn_CONF__R_SU,		DIV_ROUND_UP(_pdata->lcdc_t_rsu_ns,	lcdc_mclk_ns))
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CSn_CONF__W_HOLD,		DIV_ROUND_UP(_pdata->lcdc_t_whold_ns,	lcdc_mclk_ns))
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CSn_CONF__W_STROBE,	DIV_ROUND_UP(_pdata->lcdc_t_wstrobe_ns,	lcdc_mclk_ns))
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CSn_CONF__W_SU,		DIV_ROUND_UP(_pdata->lcdc_t_wsu_ns,	lcdc_mclk_ns)));

	lcdc_reg_write(dev, par, DA8XX_LCDCREG_DMA_CTRL,
			0
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_DMA_CTRL__FRAME_MODE,	DA8XX_LCDCREG_DMA_CTRL__FRAME_MODE__single)
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_DMA_CTRL__BIGENDIAN,	DA8XX_LCDCREG_DMA_CTRL__BIGENDIAN__disable)
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_DMA_CTRL__EOF_INT_EN,	DA8XX_LCDCREG_DMA_CTRL__EOF_INT_EN__disable)
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_DMA_CTRL__BURST_SIZE,	lidd_dma_burst_size));

	regval = lcdc_reg_read(dev, par, DA8XX_LCDCREG_LCD_STAT);
	if (regval) {
		dev_warn(dev, "%s: non-zero LCD_STAT value %x at initialization\n", __func__, (unsigned)regval);
		lcdc_reg_write(dev, par, DA8XX_LCDCREG_LCD_STAT, regval);
	}

	lcdc_unlock(par);

	dev_dbg(dev, "%s: done\n", __func__);

	return 0;


 //exit_unlock:
	lcdc_unlock(par);
 exit:
	return ret;
}

static void __devinitexit da8xx_ili9340_lidd_regs_shutdown(struct platform_device* _pdevice)
{
	struct device* dev			= &_pdevice->dev;
	struct fb_info* info			= platform_get_drvdata(_pdevice);
	struct da8xx_ili9340_par* par		= info->par;

	dev_dbg(dev, "%s: called\n", __func__);

	lcdc_lock(par);

	lcdc_reg_change(dev, par, DA8XX_LCDCREG_LIDD_CTRL,
			0
			| REGDEF_MASK(DA8XX_LCDCREG_LIDD_CTRL__LIDD_DMA_EN)
			| REGDEF_MASK(DA8XX_LCDCREG_LIDD_CTRL__DONE_INT_EN),
			0
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CTRL__LIDD_DMA_EN,	DA8XX_LCDCREG_LIDD_CTRL__LIDD_DMA_EN__deactivate)
			| REGDEF_SET_VALUE(DA8XX_LCDCREG_LIDD_CTRL__DONE_INT_EN,	DA8XX_LCDCREG_LIDD_CTRL__DONE_INT_EN__disable));

	lcdc_unlock(par);

	dev_dbg(dev, "%s: done\n", __func__);
}




static int __devinit da8xx_ili9340_lcdc_init(struct platform_device* _pdevice, struct da8xx_ili9340_pdata* _pdata)
{
	int ret					= -EINVAL;
	struct device* dev			= &_pdevice->dev;
	struct fb_info* info			= platform_get_drvdata(_pdevice);
	struct da8xx_ili9340_par* par		= info->par;
	struct resource* res_ptr;

	dev_dbg(dev, "%s: called\n", __func__);

	res_ptr = platform_get_resource(_pdevice, IORESOURCE_MEM, 0);
	if (res_ptr == NULL) {
		dev_err(dev, "%s: no memory resource specified\n", __func__);
		ret = -ENXIO;
		goto exit;
	}

	par->lcdc_reg_size	= resource_size(res_ptr);
	par->lcdc_reg_start	= res_ptr->start;
	res_ptr = request_mem_region(par->lcdc_reg_start, par->lcdc_reg_size, _pdevice->name);
	if (res_ptr == NULL) {
		dev_err(dev, "%s: cannot reserve LCD controller registers memory\n", __func__);
		ret = -ENXIO;
		goto exit;
	}

	par->lcdc_reg_base	= ioremap(par->lcdc_reg_start, par->lcdc_reg_size);
	if (par->lcdc_reg_base == NULL) {
		dev_err(dev, "%s: cannot ioremap LCD controller registers memory region\n", __func__);
		ret = -ENXIO;
		goto exit_release_lcdc_reg;
	}

	res_ptr = platform_get_resource(_pdevice, IORESOURCE_IRQ, 0);
	if (res_ptr == NULL) {
		dev_err(dev, "%s: no IRQ resource specified\n", __func__);
		ret = -ENXIO;
		goto exit_iounmap_lcdc_reg;
	}

	par->lcdc_irq = res_ptr->start;
	ret = request_irq(par->lcdc_irq, &lcdc_edma_done, 0, DRIVER_NAME, dev);
	if (ret) {
		dev_err(dev, "%s: cannot request LCD controller EDMA completion IRQ: %d\n", __func__, ret);
		goto exit_iounmap_lcdc_reg;
	}

	ret = irq_set_irq_type(par->lcdc_irq, IRQ_TYPE_EDGE_RISING);
	if (ret) {
		dev_err(dev, "%s: cannot set LCD controller EDMA completion IRQ type: %d\n", __func__, ret);
		goto exit_free_lcdc_irq;
	}

	par->lcdc_clk = clk_get(dev, "da8xx_lcdc");
	if (IS_ERR(par->lcdc_clk)) {
		dev_err(dev, "%s: cannot get LCD controller clock\n", __func__);
		ret = -ENODEV;
		goto exit_free_lcdc_irq;
	}

	ret = clk_enable(par->lcdc_clk);
	if (ret) {
		dev_err(dev, "%s: cannot enable LCD controller clock\n", __func__);
		goto exit_put_lcdc_clk;
	}

	mutex_init(&par->lcdc_lock);
	INIT_WORK(&par->lcdc_redraw_work, &lcdc_redraw_work);

	ret = da8xx_ili9340_lidd_regs_init(_pdevice, _pdata);
	if (ret) {
		dev_err(dev, "%s: cannot setup LCD controller LIDD registers: %d\n", __func__, ret);
		goto exit_destroy_lcdc_lock;
	}

	dev_dbg(dev, "%s: done\n", __func__);

	return 0;


 //exit_shutdown_lidd_regs:
	da8xx_ili9340_lidd_regs_shutdown(_pdevice);
 exit_destroy_lcdc_lock:
	mutex_destroy(&par->lcdc_lock);
 //exit_disable_lcdc_clk:
	clk_disable(par->lcdc_clk);
 exit_put_lcdc_clk:
	clk_put(par->lcdc_clk);
 exit_free_lcdc_irq:
	free_irq(par->lcdc_irq, dev);
 exit_iounmap_lcdc_reg:
	iounmap(par->lcdc_reg_base);
 exit_release_lcdc_reg:
	release_mem_region(par->lcdc_reg_start, par->lcdc_reg_size);
 exit:
	return ret;
}

static void __devinitexit da8xx_ili9340_lcdc_shutdown(struct platform_device* _pdevice)
{
	struct device* dev			= &_pdevice->dev;
	struct fb_info* info			= platform_get_drvdata(_pdevice);
	struct da8xx_ili9340_par* par		= info->par;

	dev_dbg(dev, "%s: called\n", __func__);

#warning Think here about locking?..
	da8xx_ili9340_lidd_regs_shutdown(_pdevice);
	mutex_destroy(&par->lcdc_lock);
	clk_disable(par->lcdc_clk);
	clk_put(par->lcdc_clk);
	free_irq(par->lcdc_irq, dev);
	iounmap(par->lcdc_reg_base);
	release_mem_region(par->lcdc_reg_start, par->lcdc_reg_size);

	dev_dbg(dev, "%s: done\n", __func__);
}




static int __devinit da8xx_ili9340_display_init(struct platform_device* _pdevice, struct da8xx_ili9340_pdata* _pdata)
{
	int ret					= -EINVAL;
	struct device* dev			= &_pdevice->dev;
	struct fb_info* info			= platform_get_drvdata(_pdevice);
	struct da8xx_ili9340_par* par		= info->par;
	__u32 disp_mfc, disp_ver, disp_id;
	__u32 disp_self_diag1, disp_self_diag2;

	dev_dbg(dev, "%s: called\n", __func__);

	par->cb_backlight_ctrl	= _pdata->cb_backlight_ctrl;
	par->cb_power_ctrl	= _pdata->cb_power_ctrl;

	// Startup routine should be asyncronous for faster bootup
#warning TODO make code below asynchronous
	par->cb_power_ctrl(1);

	// Checking display ID
	display_write_cmd(dev, par, ILI9340_CMD_READID);
	/*ignore*/	display_read_data(dev, par);
	disp_mfc	= display_read_data(dev, par);
	disp_ver	= display_read_data(dev, par);
	disp_id		= display_read_data(dev, par);
	pr_info(DRIVER_NAME ": detected display: manufacturer 0x%x, version 0x%x, id 0x%x\n",
			(unsigned)disp_mfc, (unsigned)disp_ver, (unsigned)disp_id);

	// Sleep-out
	display_write_cmd(dev, par, ILI9340_CMD_READ_SELFDIAG);
	/*ignore*/	display_read_data(dev, par);
	disp_self_diag1	= display_read_data(dev, par);

	display_write_cmd(dev, par, ILI9340_CMD_SLEEP_OUT);
	msleep(120);	// Sleep-in to Sleep-out is 120ms, ch11.2, 210
#warning Maybe it might work to wait for 5ms and then loop for up to 120ms until reg value changes

	display_write_cmd(dev, par, ILI9340_CMD_READ_SELFDIAG);
	/*ignore*/	display_read_data(dev, par);
	disp_self_diag2	= display_read_data(dev, par);

	if (REGDEF_GET_VALUE(ILI9340_CMD_READ_SELFDIAG__RLD, disp_self_diag1) ==
		REGDEF_GET_VALUE(ILI9340_CMD_READ_SELFDIAG__RLD, disp_self_diag2)) {
		dev_err(dev, "%s: ILI9340 reports register loading failure\n", __func__);
		goto exit_sleep_in;
	}
	if (REGDEF_GET_VALUE(ILI9340_CMD_READ_SELFDIAG__FD, disp_self_diag1) ==
		REGDEF_GET_VALUE(ILI9340_CMD_READ_SELFDIAG__FD, disp_self_diag2)) {
		dev_err(dev, "%s: ILI9340 reports functionality failure\n", __func__);
		goto exit_sleep_in;
	}


#warning TODO setup display


	lcdc_schedule_redraw_work(info);
	lcdc_wait_redraw_work_completion(info);

	display_write_cmd(dev, par, ILI9340_CMD_DISPLAY_ON);

#warning Temporary enabling backlight this way
	par->cb_backlight_ctrl(1);

	dev_dbg(dev, "%s: done\n", __func__);

	return 0;


 //exit_display_off:
	display_write_cmd(dev, par, ILI9340_CMD_DISPLAY_OFF);
 exit_sleep_in:
	display_write_cmd(dev, par, ILI9340_CMD_SLEEP_IN);
	msleep(120); // Sleep-in to power off delay, ch12, pg211
 //exit_power_off:
	par->cb_power_ctrl(0);
 //exit:
	return ret;
}

static void __devinitexit da8xx_ili9340_display_shutdown(struct platform_device* _pdevice)
{
	struct device* dev			= &_pdevice->dev;
	struct fb_info* info			= platform_get_drvdata(_pdevice);
	struct da8xx_ili9340_par* par		= info->par;

	dev_dbg(dev, "%s: called\n", __func__);

	// Shutdown routine may be synchronous since it must wait for real shutdown completion
	lcdc_wait_redraw_work_completion(info);

#warning Temporary enabling backlight this way
	par->cb_backlight_ctrl(0);

	display_write_cmd(dev, par, ILI9340_CMD_DISPLAY_OFF);
	display_write_cmd(dev, par, ILI9340_CMD_SLEEP_IN);
	msleep(120); // Sleep-in to power off delay, ch12, pg211

	par->cb_power_ctrl(0);

	dev_dbg(dev, "%s: done\n", __func__);
}






























































static int __devinit da8xx_ili9340_probe(struct platform_device* _pdevice)
{
	int ret				= -EINVAL;
	struct device* dev		= &_pdevice->dev;
	struct da8xx_ili9340_pdata* pdata = _pdevice->dev.platform_data;
	struct da8xx_ili9340_par* par	= NULL;
	struct fb_info* info		= NULL;

	dev_dbg(dev, "%s: called, pdevice %s.%d (%p), dev %s (%p)\n",
		__func__,
		_pdevice->name, _pdevice->id, _pdevice,
		dev_name(dev), dev);

	info = framebuffer_alloc(sizeof(struct da8xx_ili9340_par), dev);
	if (info == NULL) {
		dev_err(dev, "%s: cannot allocate framebuffer\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	platform_set_drvdata(_pdevice, info);
	dev_set_drvdata(dev, info);
	par = info->par;
	par->fb_info = info;

	ret = da8xx_ili9340_fb_init(_pdevice, pdata);
	if (ret) {
		dev_err(dev, "%s: cannot initialize framebuffer: %d\n", __func__, ret);
		goto exit_release_framebuffer;
	}

	ret = da8xx_ili9340_lcdc_init(_pdevice, pdata);
	if (ret) {
		dev_err(dev, "%s: cannot initialize da8xx LCD controller: %d\n", __func__, ret);
		goto exit_fb_shutdown;
	}

	ret = da8xx_ili9340_display_init(_pdevice, pdata);
	if (ret) {
		dev_err(dev, "%s: cannot initialize da8xx display data: %d\n", __func__, ret);
		goto exit_lcdc_shutdown;
	}


#warning TODO allow user access


	ret = register_framebuffer(info);
	if (ret) {
		dev_err(dev, "%s: cannot register framebuffer: %d\n", __func__, ret);
		goto exit_display_shutdown;
	}

	dev_dbg(dev, "%s: done\n", __func__);

	return 0;


 //exit_unregister_framebuffer:
	unregister_framebuffer(info);
 exit_display_shutdown:
	da8xx_ili9340_display_shutdown(_pdevice);
 exit_lcdc_shutdown:
	da8xx_ili9340_lcdc_shutdown(_pdevice);
 exit_fb_shutdown:
	da8xx_ili9340_fb_shutdown(_pdevice);
 exit_release_framebuffer:
	framebuffer_release(info);
 exit:
	return ret;
}

static int __devexit da8xx_ili9340_remove(struct platform_device* _pdevice)
{
	struct device* dev	= &_pdevice->dev;
	struct fb_info* info	= platform_get_drvdata(_pdevice);

	dev_dbg(dev, "%s: called\n", __func__);

	unregister_framebuffer(info);


#warning TODO block user access
#warning TODO ensure there is no pending user actions


	da8xx_ili9340_display_shutdown(_pdevice);
	da8xx_ili9340_lcdc_shutdown(_pdevice);
	da8xx_ili9340_fb_shutdown(_pdevice);

	framebuffer_release(info);

	dev_dbg(dev, "%s: done\n", __func__);

	return 0;
}




static struct platform_driver da8xx_ili9340_driver = {
	.probe		= da8xx_ili9340_probe,
	.remove		= __devexit_p(da8xx_ili9340_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init da8xx_ili9340_init_module(void)
{
	int ret = -EINVAL;

	ret = platform_driver_register(&da8xx_ili9340_driver);

	if (ret) {
		pr_err("%s: cannot register platform driver %s: %d\n", __func__, da8xx_ili9340_driver.driver.name, ret);
		goto exit;
	}

	pr_debug("%s: registered platform driver %s\n", __func__, da8xx_ili9340_driver.driver.name);

	return 0;


 //exit_driver_unregister:
	platform_driver_unregister(&da8xx_ili9340_driver);
 exit:
	return ret;
}

module_init(da8xx_ili9340_init_module);

#ifdef MODULE
static void __exit da8xx_ili9340_exit_module(void)
{
	platform_driver_unregister(&da8xx_ili9340_driver);
}

module_exit(da8xx_ili9340_exit_module);
#endif /* MODULE */

MODULE_LICENSE("GPL");

