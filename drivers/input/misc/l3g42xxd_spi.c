#include <linux/input.h>	/* BUS_SPI */
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/pm.h>
#include <linux/types.h>
#include <linux/miscdevice.h>

#include "l3g42xxd.h"

#define L3G42XXD_CMD_MULTB	(1 << 6)
#define L3G42XXD_CMD_READ	(1 << 7)

#define L3G42XXD_WRITECMD(reg)	(reg & 0x3F)
#define L3G42XXD_READCMD(reg)	(L3G42XXD_CMD_READ | (reg & 0x3F))

#define L3G42XXD_READMB_CMD(reg) (L3G42XXD_CMD_READ | L3G42XXD_CMD_MULTB \
					| (reg & 0x3F))

static int l3g42xxd_spi_read(struct device *dev, unsigned char reg)
{
	struct spi_device *spi = to_spi_device(dev);
	unsigned char cmd;
	cmd = L3G42XXD_READCMD(reg);
	return spi_w8r8(spi, cmd);
}
static int l3g42xxd_spi_write(struct device *dev, unsigned char reg, unsigned char val)
{
	struct spi_device *spi = to_spi_device(dev);
	unsigned char buf[2];
	buf[0] = L3G42XXD_WRITECMD(reg);
	buf[1] = val;

	return spi_write(spi, buf, sizeof(buf));
}
static  int l3g42xxd_spi_read_block (struct device * dev,unsigned char reg,   unsigned char  count, void *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	ssize_t status;

	reg = L3G42XXD_READMB_CMD(reg);
	status = spi_write_then_read(spi, &reg, 1, buf, count);

	return (status < 0) ? status : 0;
}
static int __devinit l3g42xxd_spi_probe(struct spi_device *spi)
{
	struct l3g42xxd_chip* chip ;
	int ret;

	if (spi->max_speed_hz > L3G42XXD_MAX_SPI_FREQ_HZ) {
		pr_err("%s : SPI CLK %d Hz too fast\n",__func__, spi->max_speed_hz);
		return -EINVAL;
	}
	ret = l3g42xxd_probe(&spi->dev,BUS_SPI,spi->irq,l3g42xxd_spi_read,l3g42xxd_spi_write,l3g42xxd_spi_read_block,&chip);
	if (ret){
		pr_err("%s: l3g42xxd_probe failed\n",__func__);
		return ret;
	}
	spi_set_drvdata(spi, chip);
	return 0;
}
static int __devexit l3g42xxd_spi_remove(struct spi_device *spi)
{
	struct l3g42xxd_chip *chip = spi_get_drvdata(spi);
	l3g42xxd_remove(chip);
	spi_set_drvdata(spi, NULL);

	return 0;
}

static int l3g42xxd_spi_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
        struct l3g42xxd_chip *chip = dev_get_drvdata(&spi->dev);
	l3g42xxd_suspend(chip);
	return 0;
}
static int l3g42xxd_spi_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
        struct l3g42xxd_chip *chip = dev_get_drvdata(&spi->dev);
        l3g42xxd_resume(chip);
	return 0;
}

static SIMPLE_DEV_PM_OPS(l3g42xxd_spi_pm, l3g42xxd_spi_suspend,
			 l3g42xxd_spi_resume);

static struct spi_driver l3g42xxd_spi_driver = {
	.driver = {
		.name 	= "l3g42xxd",
		.owner 	= THIS_MODULE,
		.pm 	= &l3g42xxd_spi_pm, 
	},
	.probe = l3g42xxd_spi_probe,
	.remove = __devexit_p(l3g42xxd_spi_remove),

};

module_spi_driver(l3g42xxd_spi_driver);

MODULE_AUTHOR("Roman Meshkevich <romik.momik@trikset.com>");
MODULE_DESCRIPTION("L3gd20 Three-Axis Digital Accelerometer SPI Bus Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:l3g42xxd");
