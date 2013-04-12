#ifndef __L3G4200D_H__
#define __L3G4200D_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

struct l3g42xxd_platform_data {
        int gpio_drdy;
};

#endif  /* __L3G4200D_H__ */
