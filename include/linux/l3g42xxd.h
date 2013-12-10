#ifndef __L3G4200D_H__
#define __L3G4200D_H__

#include <linux/ioctl.h>  /* For IOCTL macros */


#define L3G4200D_WHO_AM_I               0x0f
#define L3G4200D_CTRL_REG1              0x20
#define L3G4200D_CTRL_REG2              0x21
#define L3G4200D_CTRL_REG3              0x22
#define L3G4200D_CTRL_REG4              0x23
#define L3G4200D_CTRL_REG5              0x24

#define L3G4200D_REF_DATA_CAP           0x25
#define L3G4200D_OUT_TEMP               0x26
#define L3G4200D_STATUS_REG             0x27

#define L3G4200D_STATUS_REG_EXPECTED	0x0f


#define L3G4200D_OUT_X_L                0x28
#define L3G4200D_OUT_X_H                0x29
#define L3G4200D_OUT_Y_L                0x2a
#define L3G4200D_OUT_Y_H                0x2b
#define L3G4200D_OUT_Z_L                0x2c
#define L3G4200D_OUT_Z_H                0x2d

#define L3G4200D_FIFO_CTRL              0x2e
#define L3G4200D_FIFO_SRC               0x2f

#define L3G4200D_INTERRUPT_CFG          0x30
#define L3G4200D_INTERRUPT_SRC          0x31
#define L3G4200D_INTERRUPT_THRESH_X_H   0x32
#define L3G4200D_INTERRUPT_THRESH_X_L   0x33
#define L3G4200D_INTERRUPT_THRESH_Y_H   0x34
#define L3G4200D_INTERRUPT_THRESH_Y_L   0x35
#define L3G4200D_INTERRUPT_THRESH_Z_H   0x36
#define L3G4200D_INTERRUPT_THRESH_Z_L   0x37
#define L3G4200D_INTERRUPT_DURATION     0x38

#define PM_MASK                         0x08

#define I2C_RETRY_DELAY                 5
#define I2C_RETRIES                     5
#define AUTO_INCREMENT                  0x80

#define L3G4200D_ID 					0xD3 
#define L3GD20_ID 						0xD4 

struct l3g42xxd_platform_data {
        int gpio_drdy;
};

#endif  /* __L3G4200D_H__ */
