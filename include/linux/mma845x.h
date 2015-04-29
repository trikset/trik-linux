
#ifndef __MMA845X_H__
#define __MMA845X_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define MMA845X_NAME   "mma845x"
    

#define MMA845X_IOCTL_BASE 'a'
/** The following define the IOCTL command values via the ioctl macros */
#define MMA845X_IOCTL_SET_STATE			_IOW(MMA845X_IOCTL_BASE, 0, int)
#define MMA845X_IOCTL_GET_STATE			_IOR(MMA845X_IOCTL_BASE, 1, int)
#define MMA845X_IOCTL_SET_MODEG			_IOW(MMA845X_IOCTL_BASE, 2, int)
#define MMA845X_IOCTL_GET_MODEG			_IOR(MMA845X_IOCTL_BASE, 3, int)
#define MMA845X_IOCTL_SET_SAMPLE_RATE	_IOW(MMA845X_IOCTL_BASE, 4, int)
#define MMA845X_IOCTL_GET_SAMPLE_RATE	_IOR(MMA845X_IOCTL_BASE, 5, int)

#endif  /* __MMA845X_H__ */
