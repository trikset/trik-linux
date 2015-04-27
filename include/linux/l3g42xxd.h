#ifndef __L3G42XXD_H__
#define __L3G42XXD_H__

#include <linux/ioctl.h>  /* For IOCTL macros */


#define L3G42XXD_ID 					0xD3 
#define L3GD20_ID 						0xD4 

struct l3g42xxd_platform_data {
        int gpio_drdy;
};

#endif  /* __L3G42XXD_H__ */
