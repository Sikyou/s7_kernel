#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include "ISSP_Routines.h"

/*Macro definition*/
#define DEV_NAME "cypress_firmware"

/*IOCTL command*/
#define CYPRESS_MAGIC 'D'
#define IOCTL_FIRMWARE _IO(CYPRESS_MAGIC,0)

/*Macro switch*/
#define DEBUG_CYPRESS_FIRMWARE
#ifdef  DEBUG_CYPRESS_FIRMWARE
#define CYPRESS_DMSG(format,args...) printk( KERN_INFO "[%s] (line:%u)" format "\n", \
		__FUNCTION__,__LINE__,##args)
#else
#define CYPRESS_DMSG(format,args...)
#endif
#define CYPRESS_ERRMSG(format,args...) printk( KERN_INFO "[%s] (line:%u)" format "\n", \
		__FUNCTION__,__LINE__,##args)

/* type definition */
struct cypress_firmware_driver_struct
{
	struct miscdevice dev;
	struct file_operations fops;
};

/*static varialbe definition*/
static struct cypress_firmware_driver_struct firmware_driver;


static int cypress_firmware_open(struct inode* inode, struct file* file)
{
	return 0;
}	

static int cypress_firmware_close(struct inode* inode, struct file* file)
{
	return 0;
}	

static int cypress_firmware_ioctl(struct inode* inode, struct file* file, unsigned int cmd,
	                          unsigned long param)
{	
	int ret = 0;
	switch(cmd)
	{
		case IOCTL_FIRMWARE:
			CYPRESS_DMSG("------IOCTL_FIRMWARE----\n");
			if( IsspProgram() != 0 )
			{
				CYPRESS_ERRMSG("IsspProgram error!");
				ret = -1;
			}
			break;
	}
	return ret;
}

static int __init
cypress_firmware_init(void)
{
	int err = -1;
	firmware_driver.dev.name = DEV_NAME;
	firmware_driver.dev.minor = MISC_DYNAMIC_MINOR;
	firmware_driver.fops.ioctl = cypress_firmware_ioctl;
	firmware_driver.fops.open = cypress_firmware_open;
	firmware_driver.fops.release = cypress_firmware_close;
	firmware_driver.dev.fops = &firmware_driver.fops;
	if(err = misc_register(&firmware_driver.dev))
	{
		CYPRESS_ERRMSG("misc_register failed \n");
		return -1;
	}
	return 0;
}

static void __exit
cypress_firmware_exit(void)
{
	misc_deregister(&firmware_driver.dev);
}

module_init(cypress_firmware_init);
module_exit(cypress_firmware_init);

MODULE_DESCRIPTION("cypress download firmware driver");
MODULE_AUTHOR("nielimin/00164272");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");




