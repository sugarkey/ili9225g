#ifndef ST7775R_H
#define ST7775R_H

#include <linux/types.h>

/* IOCTL commands */

#define ST7775R_IOC_MAGIC			's'

#define ST7775R_IOC_WR_ENTER_STANDBY		_IOW(ST7775R_IOC_MAGIC, 0, __u8)
#define ST7775R_IOC_WR_EXIT_STANDBY		_IOW(ST7775R_IOC_MAGIC, 1, __u8)

#define ST7775R_IOC_WR_BUFFER_SIZE		_IOW(ST7775R_IOC_MAGIC, 2, __u32)

#define ST7775R_IOC_BACKLIGHT_CONTROL	_IOW(ST7775R_IOC_MAGIC, 3, __u32)




#endif 

