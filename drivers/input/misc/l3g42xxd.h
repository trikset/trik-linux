#ifndef _L3G42XXD__INT_H_
#define _L3G42XXD__INT_H_

#include <linux/types.h>


#define L3G42XXD_MAX_SPI_FREQ_HZ	10000000

struct device;
struct l3g42xxd_platform_data;
struct l3g42xxd_data;
struct l3g42xxd_chip;
struct work_struct irq_work;

typedef int (*l3g42xxd_read_t)(struct device * dev,  unsigned char reg);
typedef int (*l3g42xxd_write_t)(struct device * dev , unsigned char reg, unsigned char val);
typedef int (*l3g42xxd_read_block_t)(struct device * dev,unsigned char reg,   unsigned char  count, void *buf);

struct l3g42xxd_chip {
 	u16 bus_type;
	struct device* dev;
	
	l3g42xxd_read_t read;
	l3g42xxd_write_t write;
	l3g42xxd_read_block_t read_block;	

	struct l3g42xxd_data *data;
	short device_address;
	struct work_struct irq_work;
	struct mutex mutex;
	int irq;
};
void l3g42xxd_suspend(struct l3g42xxd_chip *chip);
void l3g42xxd_resume(struct l3g42xxd_chip *chip);

int l3g42xxd_probe(struct device *dev, u16 bus_type,int irq,l3g42xxd_read_t read,l3g42xxd_write_t write,l3g42xxd_read_block_t read_block,struct l3g42xxd_chip **chip_data);
void l3g42xxd_remove(struct l3g42xxd_chip *chip);


#endif  /*_L3G42XXD__INT_H_ */
