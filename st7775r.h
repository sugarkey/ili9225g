#ifndef ST7775R_H
#define ST7775R_H

#include <linux/types.h>

/* IOCTL commands */

#define ST7775R_IOC_MAGIC			's'

/* Read / Write ST7775R device default max speed hz */
#define ST7775R_IOC_RD_MAX_SPEED_HZ		_IOR(ST7775R_IOC_MAGIC, 0, __u32)
#define ST7775R_IOC_WR_MAX_SPEED_HZ		_IOW(ST7775R_IOC_MAGIC, 0, __u32)

#define ST7775R_IOC_WR_BUFFER_SIZE		_IOW(ST7775R_IOC_MAGIC, 1, __u32)



#endif 

