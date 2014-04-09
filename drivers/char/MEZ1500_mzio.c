#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <mach/regs-gpio.h>
#include <mach/hardware.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/unistd.h>
#include "MEZ1500_mzio.h"

#undef DEBUG
#define DEBUG
#ifdef DEBUG
#define DPRINTK(x...) {printk(__FUNCTION__"(%d): ",__LINE__);printk(##x);}
#else
#define DPRINTK(x...) (void)(0)
#endif

#define DEVICE_NAME "mzio"




static unsigned int GPIO_table [] = {
	S3C2410_GPD(0), 	// MZIO_MOD_RESET
	S3C2410_GPD(1),		// MZIO_MOD_PWR
	
	// Port J, CAMIF
	S3C2410_GPJ(0),		// MZIO_CAMIF_DAT0
	S3C2410_GPJ(1),		// MZIO_CAMIF_DAT1
	S3C2410_GPJ(2),		// MZIO_CAMIF_DAT2
	S3C2410_GPJ(3),		// MZIO_CAMIF_DAT3
	S3C2410_GPJ(4),		// MZIO_CAMIF_DAT4
	S3C2410_GPJ(5),		// MZIO_CAMIF_DAT5
	S3C2410_GPJ(6),		// MZIO_CAMIF_DAT6
	S3C2410_GPJ(7),		// MZIO_CAMIF_DAT7          
	                     
	// Add more MZIO gpios here
	0
};




//-----------------------------------------------------------------------------
//
// Create a compile date and time stamp
#define COMPILE_HOUR (((__TIME__[0]-'0')*10) + (__TIME__[1]-'0'))
#define COMPILE_MIN	 (((__TIME__[3]-'0')*10) + (__TIME__[4]-'0'))
#define COMPILE_SEC	 (((__TIME__[6]-'0')*10) + (__TIME__[7]-'0'))

#define YEAR ((((__DATE__ [7]-'0')*10+(__DATE__ [8]-'0'))*10+ \
(__DATE__ [9]-'0'))*10+(__DATE__ [10]-'0'))

/* Month: 0 - 11 */
#define MONTH ( __DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? 0 : 5) \
	            : __DATE__ [2] == 'b' ? 1 \
              : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 2 : 3) \
              : __DATE__ [2] == 'y' ? 4 \
              : __DATE__ [2] == 'l' ? 6 \
              : __DATE__ [2] == 'g' ? 7 \
              : __DATE__ [2] == 'p' ? 8 \
              : __DATE__ [2] == 't' ? 9 \
              : __DATE__ [2] == 'v' ? 10 : 11)

#define DAY ((__DATE__ [4]==' ' ? 0 : __DATE__ [4]-'0')*10+(__DATE__[5]-'0'))

static int GetCompileHour  (void){int hour  = COMPILE_HOUR;return(hour);}
static int GetCompileMinute(void){int minute= COMPILE_MIN; return(minute);}
static int GetCompileYear  (void){int year  = YEAR;        return(year);}
static int GetCompileMonth (void){int month = MONTH;       return(month + 1);}
static int GetCompileDay   (void){int day   = DAY;         return((char)day);}
//-----------------------------------------------------------------------------


static int sbc2440_mzio_ioctl(
	struct inode *inode, 
	struct file *file, 
	unsigned int cmd, 
	unsigned long arg)
{	
	printk("cmd=0x%x\n", cmd);	
	printk("arg=0x%lx\n", arg);	

	switch(cmd)
	{
		case MZIO_GPIO_SET_HIGH:
			//if (arg < sizeof(GPIO_table))	
			{			
				printk("MZIO_GPIO_SET_HIGH\n");	
				s3c2410_gpio_setpin(GPIO_table[arg], 1);
			}
			return 0;

		case MZIO_GPIO_SET_LOW:
			//if (arg < sizeof(GPIO_table))	
			{			
				printk("MZIO_GPIO_SET_LOW\n");	
				s3c2410_gpio_setpin(GPIO_table[arg], 0);
			}
			return 0;

		case MZIO_GPIO_SET_DIR_OUT:
//			if (arg < sizeof(GPIO_table))	
			{			
				printk("MZIO_GPIO_SET_DIR_OUT\n");	
				s3c2410_gpio_cfgpin(GPIO_table[arg], S3C2410_GPIO_OUTPUT);
			}
			return 0;

		case MZIO_GPIO_SET_DIR_IN:
			//if (arg < sizeof(GPIO_table))	
			{			
				printk("MZIO_GPIO_SET_DIR_IN\n");	
				s3c2410_gpio_cfgpin(GPIO_table[arg], S3C2410_GPIO_INPUT);
			}
			return 0;

		case MZIO_GPIO_SET_PU_OFF:
			//if (arg < sizeof(GPIO_table))	
			{			
				printk("MZIO_GPIO_SET_PU_OFF\n");	
				s3c2410_gpio_pullup(GPIO_table[arg], 1);
			}
			return 0;

		case MZIO_GPIO_SET_PU_ON:
			//if (arg < sizeof(GPIO_table))	
			{						
				printk("MZIO_GPIO_SET_PU_ON\n");	
				s3c2410_gpio_pullup(GPIO_table[arg], 0);
			}
			return 0;

		case MZIO_GPIO_GET:
			//if (arg < sizeof(GPIO_table))	
			{			
				printk("MZIO_GPIO_GET\n");	
				s3c2410_gpio_cfgpin(GPIO_table[arg], S3C2410_GPIO_INPUT);
			}
			return s3c2410_gpio_getpin(GPIO_table[arg]);

		default:
			return -EINVAL;
	}
}

static int sbc2440_mzio_read(struct file *filp, char *buffer, size_t count, loff_t *ppos)
{
	char str[20];
	int value;
	size_t len;
	
	s3c2410_gpio_cfgpin(S3C2410_GPD(0), S3C2410_GPIO_INPUT);	
	value = s3c2410_gpio_getpin(S3C2410_GPD(0));
	
	len = sprintf(str, "%d\n", value);
	if (count >= len) {
		int r = copy_to_user(buffer, str, len);
		return r ? r : len;
	} else {
		return -EINVAL;
	}
}


static struct file_operations dev_fops = {
	.owner	=	THIS_MODULE,
	.ioctl	=	sbc2440_mzio_ioctl,
	.read   = sbc2440_mzio_read,
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &dev_fops,
};

static int __init dev_init(void)
{
	int ret;

	ret = misc_register(&misc);

	printk(DEVICE_NAME"\tversion %d%02d%02d%02d%02d\tinitialized\n", GetCompileYear(),
	GetCompileMonth(), GetCompileDay(), GetCompileHour(), GetCompileMinute());
	
	return ret;
}

static void __exit dev_exit(void)
{
	printk(DEVICE_NAME"\tgoodbye!\n");
	misc_deregister(&misc);
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aceeca Inc.");
