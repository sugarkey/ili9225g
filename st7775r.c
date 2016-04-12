#include <linux/init.h>
#include <linux/types.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/major.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/rcupdate.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <mach/camera.h>
#include <soc/gpio.h>

static struct spi_device	*spi;
static unsigned char		*buffer;


struct st7775r_board_info
{
    int reset_gpio;
    int rs_gpio;
};

#define SPIDEV_MAJOR			153	/* assigned */

static struct class *st7775r_class;

struct st7775r_board_info st7775r_board = {
    .reset_gpio = GPIO_PB(17),
    .rs_gpio = GPIO_PB(18),
//    .reset_gpio = GPIO_PA(4),
//    .rs_gpio = GPIO_PA(2),

};

#define RESET(a) do{gpio_direction_output(st7775r_board.reset_gpio, a);}while(0)
#define RS(a) do{gpio_direction_output(st7775r_board.rs_gpio, a);}while(0)

static int write_command(unsigned char value_l,unsigned char value_h)
{
    unsigned short data;
    int ret;
    struct spi_message msg;
    struct spi_transfer t[] = {
        {
            .tx_buf = &data,
            .len = 2,
//          .cs_change = 1,
        }
    };

    RS(0);
    data = value_l|(value_h<<8);
	spi_message_init(&msg);
	spi_message_add_tail(&t[0], &msg);
	ret = spi_sync(spi, &msg);    
    if (ret < 0)
    {
       return -1;
    }
    return 0;
}

static int write_data(unsigned char value_l,unsigned char value_h)
{
    unsigned short data;
    int ret;
    struct spi_message msg;
    struct spi_transfer t[] = {
        {
            .tx_buf = &data,
            .len = 2,
    //          .cs_change = 1,
        }
    };

    RS(1);
    data = value_l|(value_h<<8);
	spi_message_init(&msg);
	spi_message_add_tail(&t[0], &msg);
	ret = spi_sync(spi, &msg);    
    if (ret < 0)
    {
       return -1;
    }
    return 0;
}

static int st7775r_write_data(unsigned char *data,int len)
{
    RS(1);
    return 0;    
}

static void inti_st7775r(void)
{
    printk("inti_st7775r\n");        
    RESET(1);                                                             
    mdelay(1);                                                          
    RESET(0);                                                             
    mdelay(10);                                                         
    RESET(1);                                                            
    mdelay(200);                                                        
    //************* Start Initial Sequence **********//                
    write_command(0x00,0x01);                                         
    write_data(0x01,0x1C);//01,1c,                                    
                                                                       
    write_command(0x00,0x02);                                         
    write_data(0x01,0x00);                                             
                                                                       
    write_command(0x00,0x03);                                         
    write_data(0x10,0x30); //10,38转为横屏                             
                                                                       
    write_command(0x00,0x08);                                         
    write_data(0x08,0x08);                                             
                                                                       
    write_command(0x00,0x0c);                                         
    write_data(0x00,0x00);                                             
                                                                       
    write_command(0x00,0x0f);                                         
    write_data(0x00,0x01);                                             
                                                                       
    write_command(0x00,0x20);   // Set GRAM Address (GRAM地址选择）   
    write_data(0x00,0x00);                                             
                                                                       
    write_command(0x00,0x21);  // Set GRAM Address                    
    write_data(0x00,0x00);                                             
                                                                       
    //*************Power On sequence ****************//                
    write_command(0x00,0x10);                                         
    write_data(0x00,0x00);                                             
                                                                       
    write_command(0x00,0x11);                                         
    write_data(0x10,0x00);                                             
    mdelay(100);                                                        
    //------------------ Set GRAM area ---------------------------//   
    write_command(0x00,0x30);                                         
    write_data(0x00,0x00);                                             
                                                                       
    write_command(0x00,0x31);                                         
    write_data(0x00,0xdb);                                             
                                                                       
    write_command(0x00,0x32);                                         
    write_data(0x00,0x00);                                             
                                                                       
    write_command(0x00,0x33);                                         
    write_data(0x00,0x00);                                             
                                                                       
    write_command(0x00,0x34);                                         
    write_data(0x00,0xdb);                                             
                                                                       
    write_command(0x00,0x35);                                         
    write_data(0x00,0x00);                                             
                                                                       
    write_command(0x00,0x36);                                         
    write_data(0x00,0xaf);                                             
                                                                       
    write_command(0x00,0x37);                                         
    write_data(0x00,0x00);                                             
                                                                       
    write_command(0x00,0x38);                                         
    write_data(0x00,0xdb);                                             
                                                                       
    write_command(0x00,0x39);                                         
    write_data(0x00,0x00);                                             
    
    mdelay(10);                                                         
    write_command(0x00,0xff);                                         
    write_data(0x00,0x03);                                             
                                                                       
    // ----------- Adjust the Gamma  Curve ----------//                 
    write_command(0x00,0x50);                                         
    write_data(0x02,0x03);                                             
                                                                       
    write_command(0x00,0x051);                                        
    write_data(0x0a,0x09);                                             
                                                                       
    write_command(0x00,0x52);                                         
    write_data(0x00,0x05);                                             
                                                                       
    write_command(0x00,0x53);                                         
    write_data(0x10,0x21);                                             
                                                                       
    write_command(0x00,0x54);                                         
    write_data(0x06,0x02);                                             
                                                                       
    write_command(0x00,0x55);                                         
    write_data(0x00,0x03);                                             
                                                                       
    write_command(0x00,0x56);                                         
    write_data(0x07,0x03);                                             
                                                                       
    write_command(0x00,0x57);                                         
    write_data(0x05,0x07);                                             
                                                                       
    write_command(0x00,0x58);                                         
    write_data(0x10,0x21);                                             
                                                                       
    write_command(0x00,0x59);                                         
    write_data(0x07,0x03);                                             
                                                                       
    //***************************************                          
    write_command(0x00,0xB0); //VCOM                                  
    write_data(0x25,0x01); //高位调水波纹                             
                                                                       
    //********************************************************         
    write_command(0x00,0xFF);                                         
    write_data(0x00,0x00);                                             
                                                                       
    write_command(0x00,0x07);                                         
    write_data(0x10,0x17);                                            
    
    mdelay(200);                                                      
    write_command(0x00,0x22);                                         
    
}


static int st7775r_open(struct inode *inode,struct file *file)
{
    printk("st7775r_open\n");
    return 0;    
}
static int st7775r_read(struct file *file, char __user *user_buf, size_t size, loff_t *ppos)
{
    unsigned char data[3]="100";
    printk("st7775r_read\n");
    copy_to_user(user_buf,&data,size);
    return 0;
}
static int st7775r_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{


//    copy_from_user(buffer,buf,count);
    return 0;
}

static long st7775r_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    switch(cmd)
    {
        case 1:
            printk("1 args's %ld\n",arg);
            break;
        case 2:            
            printk("2 args's %ld\n",arg);
            break;
        default:
            printk("cmd:%x,arg=%ld",cmd,arg);        
    }
    
    return 0;
}

static const struct file_operations st7775r_fops =
{                   
    .owner = THIS_MODULE,
    .open  = st7775r_open,
    .read  = st7775r_read,
    .write = st7775r_write,
    .unlocked_ioctl = st7775r_ioctl,
};
static struct cdev *st7775r_cdev;
struct device *class_dev;

static int st7775r_spidev_probe(struct spi_device *spi_dev)
{
	/* Initialize the driver data */
    printk("st7775r_spidev_probe\n");
	spi = spi_dev;
    inti_st7775r();    
	return 0;
}

static int st7775r_spidev_remove(struct spi_device *spi)
{
    spi = NULL;
	return 0;
}


static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =		"spidev",
		.owner =	THIS_MODULE,
	},
	.probe = st7775r_spidev_probe,
	.remove = st7775r_spidev_remove,
};


static int __init st7775r_init(void)
{
    int ret,status;
    dev_t hell_devid;
    
    ret = gpio_request(st7775r_board.reset_gpio,"st7775r_reset");       
    if(ret){                                                         
          printk("gpio requrest fail %d\n",st7775r_board.reset_gpio);
    }                                                                 

    ret = gpio_request(st7775r_board.rs_gpio,"st7775r_rs");       
    if(ret){                                                         
          printk("gpio requrest fail %d\n",st7775r_board.rs_gpio);
    }                                                                 
    
    alloc_chrdev_region(&hell_devid, 0, 32768, "st7775r");
#if 1
    st7775r_cdev = cdev_alloc();
    cdev_init(st7775r_cdev, &st7775r_fops);
    st7775r_cdev->owner = THIS_MODULE;
    cdev_add(st7775r_cdev, hell_devid, 1);  //
#else    
    register_chrdev(231, "st7775r", &st7775r_fops);
#endif
    st7775r_class=class_create(THIS_MODULE, "st7775r");
    class_dev=device_create(st7775r_class, NULL, hell_devid, NULL, "st7775r");
    
	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0) {
		unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
	}

    return 0;
}
static void __exit st7775r_exit(void)
{
    printk("st7775r_exit\n");    
    
    gpio_free(st7775r_board.rs_gpio);
    gpio_free(st7775r_board.reset_gpio);
    
  	spi_unregister_driver(&spidev_spi_driver);
#if 1    
    cdev_del(st7775r_cdev);
#else
    //    unregister_chrdev(231, "st7775r");
#endif
    device_unregister(class_dev);
    class_destroy(st7775r_class);
}
module_init(st7775r_init);
module_exit(st7775r_exit);
MODULE_LICENSE("GPL");
