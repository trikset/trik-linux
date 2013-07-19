#ifndef __MMA8451Q_H__
#define __MMA8451Q_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define F_STATUS			0x00		/* Read Only*/

#define OUT_X_MSB			0x01		/* Read Only*/
#define OUT_X_LSB			0x02		/* Read Only*/
#define OUT_Y_MSB			0x03		/* Read Only*/
#define OUT_Y_LSB			0x04		/* Read Only*/
#define OUT_Z_MSB			0x05		/* Read Only*/
#define OUT_Z_LSB			0x06		/* Read Only*/

#define F_SETUP				0x09		/* R/W 		*/
#define TRIG_CFG			0x0A		/* R/W 		*/

#define SYSMOD				0x0B		/* Read Only*/
#define INT_SOURCE			0x0C		/* Read Only*/
#define WHO_AM_I			0x0D		/* Read Only*/

#define XYZ_DATA_CFG		0x0E		/* R/W 		*/
#define HP_FILTER_CUTOFF	0x0F		/* R/W 		*/
#define PL_STATUS			0x10		/* Read Only*/
#define PL_CFG				0x11		/* R/W 		*/
#define PL_COUNT			0x12		/* R/W 		*/
#define PL_BF_ZCOMP			0x13		/* R/W 		*/
#define P_L_THS_REG			0x14		/* R/W 		*/

#define FF_MT_CFG			0x15		/* R/W 		*/
#define FF_MT_SRC			0x16		/* Read Only*/
#define FF_MT_THS			0x17		/* R/W 		*/
#define FF_MT_COUNT			0x18		/* R/W 		*/

#define TRANSIENT_CFG		0x1D		/* R/W 		*/
#define TRANSIENT_SRC		0x1E		/* Read Only*/
#define TRANSIENT_THS		0x1F		/* R/W 		*/
#define TRANSIENT_COUNT		0x20		/* R/W 		*/

#define PULSE_CFG			0x21		/* R/W 		*/
#define PULSE_SRC			0x22		/* Read Only*/
#define PULSE_THSX			0x23		/* R/W 		*/
#define PULSE_THSY			0x24		/* R/W 		*/
#define PULSE_THSZ			0x25		/* R/W 		*/
#define PULSE_TMLT			0x26		/* R/W 		*/
#define PULSE_LTCY			0x27		/* R/W 		*/
#define PULSE_WIND			0x28		/* R/W 		*/
#define ASPL_COUNT			0x29		/* R/W 		*/

#define	CTRL_REG1			0x2A		/* R/W 		*/
#define CTRL_REG2			0x2B		/* R/W 		*/
#define CTRL_REG3			0x2C		/* R/W 		*/
#define CTRL_REG4			0x2D		/* R/W 		*/
#define CTRL_REG5			0x2E		/* R/W 		*/

#define OFF_X				0x2F		/* R/W 		*/
#define OFF_Y				0x30		/* R/W 		*/
#define OFF_Z				0x31		/* R/W 		*/


//#define MMA8450_ID			0xc6
#define MMA8451_ID			0x1a
//#define MMA8452_ID			0x2a
//#define MMA8453_ID			0x3a

struct mma8451q_platform_data {
        int int1;
        int int2;
};

#endif  /* __L3G4200D_H__ */
