#ifndef __L3G42XXD_H__
#define __L3G42XXD_H__

#include <linux/ioctl.h>  /* For IOCTL macros */


#define L3G42XXD_ID 					0xD3 
#define L3GD20_ID 						0xD4 
#define L3GD20H_ID 						0xD7 


#define L3G42XXD_IOCTL_BASE 'l'
/** The following define the IOCTL command values via the ioctl macros */
#define L3G42XXD_IOCTL_SET_FS_MODE		_IOW(L3G42XXD_IOCTL_BASE, 0, int)
#define L3G42XXD_IOCTL_GET_FS_MODE		_IOR(L3G42XXD_IOCTL_BASE, 1, int)
#define L3G42XXD_IOCTL_SET_STATE		_IOW(L3G42XXD_IOCTL_BASE, 2, int)
#define L3G42XXD_IOCTL_GET_STATE		_IOR(L3G42XXD_IOCTL_BASE, 3, int)
#define L3G42XXD_IOCTL_SET_ODR			_IOW(L3G42XXD_IOCTL_BASE, 4, int)
#define L3G42XXD_IOCTL_GET_ODR			_IOR(L3G42XXD_IOCTL_BASE, 5, int)



struct l3g42xxd_platform_data {
        int gpio_drdy;
};

#endif  /* __L3G42XXD_H__ */
