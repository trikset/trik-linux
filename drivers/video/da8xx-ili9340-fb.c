#define DEBUG


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/da8xx-ili9340-fb.h>

#define DRIVER_NAME "da8xx_lcdc_ili9340"

// Utility helpers
//#define DEV_BY_FBINFO(info)	(info->device)
//#define DEV_BY_PDEVICE(pdev)	(&pdev->dev)
//#define FBINFO_BY_PDEVICE(pdev)	(platform_get_drvdata(pdev))
//#define FBINFO_BY_DEV(dev)	(dev_get_drvdata(dev))
//#define FBINFO_BY_PAR(par)	(par->fb_info)
//#define PAR_BY_FBINFO(info)	(info->par)
//#define PDATA_BY_DEV(dev)	(dev->platform_data)

#ifdef MODULE
#define __devinitexit
#else
#define __devinitexit __devinit
#endif

#define DA8XX_LIDD_EDMA_ALIGN 4

static void da8xx_ili9340_defio_redraw(struct fb_info* _info, struct list_head* _pagelist);
static int da8xx_ili9340_fbops_check_var(struct fb_var_screeninfo* _var, struct fb_info* _info);
static int da8xx_ili9340_fbops_set_par(struct fb_info* _info);




struct da8xx_ili9340_par {
	unsigned	yres_screens;

	u32		pseudo_palette[16];

	resource_size_t	lidd_reg_start;
	resource_size_t	lidd_reg_size;
	void __iomem*	lidd_reg_base;
	int		lidd_irq;
	struct clk*	lidd_clk;

	dma_addr_t	lidd_dma_phaddr;
	size_t		lidd_dma_phsize;
};

static const struct fb_fix_screeninfo da8xx_ili9340_fix_init __devinitconst = {
	.id		= "DA8xx ILI9340",
	//.smem_start
	//.smem_len
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
	.deferred_io		= da8xx_ili9340_defio_redraw,
};

static struct fb_ops da8xx_ili9340_fbops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= da8xx_ili9340_fbops_check_var,
	.fb_set_par	= da8xx_ili9340_fbops_set_par,
	.fb_read	= fb_sys_read,
	//.fb_write	= da8xx_ili9340_fbops_write,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
	//.fb_pan_display	= da8xx_ili9340_fbops_pan_display,
	//.fb_blank	= da8xx_ili9340_fbops_blank,
	//.fb_sync	= da8xx_ili9340_fbops_sync,
	//.fb_ioctl	= da8xx_ili9340_fbops_ioctl,
};





static size_t da8xx_ili9340_line_length(const struct fb_var_screeninfo* _var)
{
	return DIV_ROUND_UP(_var->bits_per_pixel, BITS_PER_BYTE) * _var->xres_virtual;
}

static int da8xx_ili9340_fbops_check_var(struct fb_var_screeninfo* _var, struct fb_info* _info)
{
	int ret				= -EINVAL;
	struct device* dev		= _info->device;
	struct da8xx_ili9340_par* par	= _info->par;
	unsigned bytes_per_pixel;

	dev_dbg(dev, "%s: called\n", __func__);

	if (_var->bits_per_pixel <= 16) { // RGB565 mode
		_var->blue.offset	= 0;
		_var->blue.length	= 5;
		_var->green.offset	= 5;
		_var->green.length	= 6;
		_var->red.offset	= 11;
		_var->red.length	= 5;
		_var->bits_per_pixel	= 16;
	} else if (_var->bits_per_pixel <= 24) { // RGB666 mode, 3-byte padding
		_var->blue.offset	= 0;
		_var->blue.length	= 6;
		_var->green.offset	= 6;
		_var->green.length	= 6;
		_var->red.offset	= 12;
		_var->red.length	= 6;
		_var->bits_per_pixel	= 24;
	} else { // RGB666 mode, 4-byte padding, full byte per color
		_var->blue.offset	= 0;
		_var->blue.length	= 6;
		_var->green.offset	= 8;
		_var->green.length	= 6;
		_var->red.offset	= 16;
		_var->red.length	= 6;
		_var->bits_per_pixel	= 32;
	}

	_var->xres		= _info->var.xres;
	_var->xres_virtual	= _info->var.xres_virtual;
	_var->yres		= _info->var.yres;
	_var->yres_virtual	= _info->var.yres * (likely((da8xx_ili9340_line_length(&_info->var) % DA8XX_LIDD_EDMA_ALIGN) == 0) ?
						     par->yres_screens :
						     1);
	_var->grayscale		= _info->var.grayscale;
	_var->nonstd		= _info->var.nonstd;

	dev_dbg(dev, "%s: done\n", __func__);

	return 0;

 exit:
	return ret;
}

static int da8xx_ili9340_fbops_set_par(struct fb_info* _info)
{
	int ret				= -EINVAL;
	struct device* dev		= _info->device;
	struct da8xx_ili9340_par* par	= _info->par;

	u32			line_length;
	unsigned long		screen_size;
	char __iomem*		screen_base;
	size_t			dma_phsize;
	dma_addr_t		dma_phaddr;

	dev_dbg(dev, "%s: called\n", __func__);

#warning LOCKING REQUIRED!


	line_length	= da8xx_ili9340_line_length(&_info->var);
	screen_size	= line_length * _info->var.yres_virtual;

#warning TODO only realloc when required?

	dma_phsize	= ALIGN(screen_size, DA8XX_LIDD_EDMA_ALIGN);
	screen_base	= dma_alloc_coherent(dev, dma_phsize, &dma_phaddr, GFP_KERNEL | GFP_DMA);
	if (screen_base == NULL) {
		dev_err(dev, "%s: cannot allocate EDMA screen buffer\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	dma_free_coherent(dev, par->lidd_dma_phsize, _info->screen_base, par->lidd_dma_phaddr);

	_info->screen_base	= screen_base;
	_info->screen_size	= screen_size;
	_info->fix.line_length	= line_length;
	_info->fix.smem_start	= virt_to_phys(screen_base);
	_info->fix.smem_len	= screen_size;
	par->lidd_dma_phaddr	= dma_phaddr;
	par->lidd_dma_phsize	= dma_phsize;

	memset(_info->screen_base, 0, _info->screen_size);

#warning TODO set LIDD registers, LCD registers, and start screen flush, maybe reset

#warning TODO unlock
	dev_dbg(dev, "%s: done\n", __func__);

	return 0;

 exit:
#warning TODO unlock
	return ret;
}

static void da8xx_ili9340_defio_redraw(struct fb_info* _info, struct list_head* _pagelist)
{
#warning TODO
}

static irqreturn_t da8xx_ili9340_lidd_edma_done(int _irq, void* _dev)
{
#warning TODO
	return IRQ_HANDLED;
}





static int __devinit da8xx_ili9340_fb_init(struct platform_device* _pdevice)
{
	int ret		= -EINVAL;
	struct device* dev			= &_pdevice->dev;
	struct fb_info* info			= platform_get_drvdata(_pdevice);
	struct da8xx_ili9340_pdata* pdata	= &dev->platform_data;
	struct da8xx_ili9340_par* par		= info->par;

	dev_dbg(dev, "%s: called\n", __func__);

	info->flags		= FBINFO_DEFAULT | FBINFO_VIRTFB | FBINFO_HWACCEL_NONE;
	info->fix		= da8xx_ili9340_fix_init;
	info->var		= da8xx_ili9340_var_init;
	info->fbops		= &da8xx_ili9340_fbops;
	info->pseudo_palette	= par->pseudo_palette;

	par->yres_screens	= (pdata->yres_screens>1?pdata->yres_screens:1);
	info->var.xres		= pdata->xres;
	info->var.xres_virtual	= info->var.xres;
	info->var.yres		= pdata->yres;
	info->var.yres_virtual	= info->var.yres * par->yres_screens;
	info->var.bits_per_pixel= pdata->bits_per_pixel;
	info->var.height	= pdata->screen_height;
	info->var.width		= pdata->screen_width;

	info->screen_base	= 0;
	info->screen_size	= 0;
	info->fix.smem_len	= 0;
	info->fix.smem_start	= 0;

#warning TODO allocate fake colormap
	ret = fb_alloc_cmap(&info->cmap, 0, 0);
	if (ret) {
		dev_err(dev, "%s: cannot allocated colormap: %d\n", __func__, ret);
		goto exit;
	}

	info->fbdefio		= &da8xx_ili9340_defio;
	info->fbdefio->delay	= (HZ / pdata->fps);

	fb_deferred_io_init(info);

	dev_dbg(dev, "%s: done\n", __func__);

	return 0;


 exit_defio_cleanup:
	fb_deferred_io_cleanup(info);
 exit_dealloc_cmap:
	fb_dealloc_cmap(&info->cmap);
 exit:
	return ret;
}

static void __devinitexit da8xx_ili9340_fb_shutdown(struct platform_device* _pdevice)
{
	struct device* dev			= &_pdevice->dev;
	struct fb_info* info			= platform_get_drvdata(_pdevice);
	struct da8xx_ili9340_par* par		= info->par;

	dev_dbg(dev, "%s: called\n", __func__);

	dma_free_coherent(dev, par->lidd_dma_phsize, info->screen_base, par->lidd_dma_phaddr);
	fb_deferred_io_cleanup(info);
	fb_dealloc_cmap(&info->cmap);

	dev_dbg(dev, "%s: done\n", __func__);
}




static int __devinit da8xx_ili9340_lidd_init(struct platform_device* _pdevice)
{
	int ret					= -EINVAL;
	struct device* dev			= &_pdevice->dev;
	struct fb_info* info			= platform_get_drvdata(_pdevice);
	struct da8xx_ili9340_pdata* pdata	= &dev->platform_data;
	struct da8xx_ili9340_par* par		= info->par;
	struct resource* res_ptr;

	dev_dbg(dev, "%s: called\n", __func__);

	res_ptr = platform_get_resource(_pdevice, IORESOURCE_MEM, 0);
	if (res_ptr == NULL) {
		dev_err(dev, "%s: no memory resource specified\n", __func__);
		ret = -ENXIO;
		goto exit;
	}

	par->lidd_reg_size	= resource_size(res_ptr);
	par->lidd_reg_start	= res_ptr->start;
	res_ptr = request_mem_region(par->lidd_reg_start, par->lidd_reg_size, _pdevice->name);
	if (res_ptr == NULL) {
		dev_err(dev, "%s: cannot reserve LIDD registers memory\n", __func__);
		ret = -ENXIO;
		goto exit;
	}

	par->lidd_reg_base	= ioremap(par->lidd_reg_start, par->lidd_reg_size);
	if (par->lidd_reg_base == NULL) {
		dev_err(dev, "%s: cannot ioremap LIDD registers memory region\n", __func__);
		ret = -ENXIO;
		goto exit_release_lidd_reg;
	}

	res_ptr = platform_get_resource(_pdevice, IORESOURCE_IRQ, 0);
	if (res_ptr == NULL) {
		dev_err(dev, "%s: no IRQ resource specified\n", __func__);
		ret = -ENXIO;
		goto exit_iounmap_lidd_reg;
	}

	par->lidd_irq	= res_ptr->start;
	ret = request_irq(par->lidd_irq, da8xx_ili9340_lidd_edma_done, 0, DRIVER_NAME, dev);
	if (ret) {
		dev_err(dev, "%s: cannot request LIDD EDMA completion IRQ: %d\n", __func__, ret);
		goto exit_iounmap_lidd_reg;
	}

	ret = irq_set_irq_type(par->lidd_irq, IRQ_TYPE_EDGE_RISING);
	if (ret) {
		dev_err(dev, "%s: cannot set LIDD EDMA completion IRQ type: %d\n", __func__, ret);
		goto exit_free_lidd_irq;
	}

	par->lidd_clk		= clk_get(dev, NULL);
	if (IS_ERR(par->lidd_clk)) {
		dev_err(dev, "%s: cannot get LIDD clock\n", __func__);
		ret = -ENODEV;
		goto exit_free_lidd_irq;
	}

	ret = clk_enable(par->lidd_clk);
	if (ret) {
		dev_err(dev, "%s: cannot enable LIDD clock\n", __func__);
		goto exit_put_lidd_clk;
	}




#warning TODO create LIDD lock and workqueue




#warning TODO set LIDD registers




	dev_dbg(dev, "%s: done\n", __func__);

	return 0;


 exit_disable_lidd_clk:
	clk_disable(par->lidd_clk);
 exit_put_lidd_clk:
	clk_put(par->lidd_clk);
 exit_free_lidd_irq:
	free_irq(par->lidd_irq, dev);
 exit_iounmap_lidd_reg:
	iounmap(par->lidd_reg_base);
 exit_release_lidd_reg:
	release_mem_region(par->lidd_reg_start, par->lidd_reg_size);
 exit:
	return ret;
}

static void __devinitexit da8xx_ili9340_lidd_shutdown(struct platform_device* _pdevice)
{
	struct device* dev			= &_pdevice->dev;
	struct fb_info* info			= platform_get_drvdata(_pdevice);
	struct da8xx_ili9340_par* par		= info->par;

	dev_dbg(dev, "%s: called\n", __func__);

	clk_disable(par->lidd_clk);
	clk_put(par->lidd_clk);
	free_irq(par->lidd_irq, dev);
	iounmap(par->lidd_reg_base);
	release_mem_region(par->lidd_reg_start, par->lidd_reg_size);

	dev_dbg(dev, "%s: done\n", __func__);
}




static int __devinit da8xx_ili9340_lcd_init(struct platform_device* _pdevice)
{
	int ret		= -EINVAL;
	struct device* dev			= &_pdevice->dev;
	struct fb_info* info			= platform_get_drvdata(_pdevice);
	struct da8xx_ili9340_pdata* pdata	= &dev->platform_data;
	struct da8xx_ili9340_par* par		= info->par;

	dev_dbg(dev, "%s: called\n", __func__);

#warning TODO

	dev_dbg(dev, "%s: done\n", __func__);

	return 0;


 exit:
	return ret;
}

static void __devinitexit da8xx_ili9340_lcd_shutdown(struct platform_device* _pdevice)
{
	struct device* dev			= &_pdevice->dev;
	struct fb_info* info			= platform_get_drvdata(_pdevice);

	dev_dbg(dev, "%s: called\n", __func__);

#warning TODO

	dev_dbg(dev, "%s: done\n", __func__);
}


























































static int __devinit da8xx_ili9340_probe(struct platform_device* _pdevice)
{
	int ret			= -EINVAL;
	struct device* dev	= &_pdevice->dev;
	struct fb_info* info	= NULL;

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


	ret = da8xx_ili9340_fb_init(_pdevice);
	if (ret) {
		dev_err(dev, "%s: cannot initialize framebuffer: %d\n", __func__, ret);
		goto exit_release_framebuffer;
	}

	ret = da8xx_ili9340_lidd_init(_pdevice);
	if (ret) {
		dev_err(dev, "%s: cannot initialize da8xx lcd controller: %d\n", __func__, ret);
		goto exit_fb_shutdown;
	}

	ret = da8xx_ili9340_lcd_init(_pdevice);
	if (ret) {
		dev_err(dev, "%s: cannot initialize ili9340 lcd controller: %d\n", __func__, ret);
		goto exit_lidd_shutdown;
	}


	ret = register_framebuffer(info);
	if (ret) {
		dev_err(dev, "%s: cannot register framebuffer: %d\n", __func__, ret);
		goto exit_lcd_shutdown;
	}




#warning TODO kick start?
#warning TODO temporary kick start
#if 0
	ret = da8xx_ili9340_fbops_check_var(&info->var, info);
	if (ret) {
		dev_err(dev, "%s: default var check failed: %d\n", __func__, ret);
		goto exit_unregister_framebuffer;
	}

	ret = da8xx_ili9340_fbops_set_par(info);
	if (ret) {
		dev_err(dev, "%s: default par set failed: %d\n", __func__, ret);
		goto exit_unregister_framebuffer;
	}
#endif




	dev_dbg(dev, "%s: done\n", __func__);

	return 0;


 exit_unregister_framebuffer:
	unregister_framebuffer(info);
 exit_lcd_shutdown:
	da8xx_ili9340_lcd_shutdown(_pdevice);
 exit_lidd_shutdown:
	da8xx_ili9340_lidd_shutdown(_pdevice);
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

#warning TODO make sure pending operations completed!

	unregister_framebuffer(info);

	da8xx_ili9340_lcd_shutdown(_pdevice);
	da8xx_ili9340_lidd_shutdown(_pdevice);
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


 exit_driver_unregister:
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

