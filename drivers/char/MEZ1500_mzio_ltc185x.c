#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <mach/hardware.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/pci.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>	// copy_to_user
//#include <asm/unistd.h>  
//#include <linux/delay.h>
//#include <asm/irq.h>
//#include <linux/mm.h>
//#include <linux/errno.h>
//#include <linux/list.h>
//#include <asm/atomic.h>
//#include <linux/spi/spi.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <plat/regs-timer.h>
#include <plat/regs-adc.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/irq.h>



#include "MEZ1500_mzio.h"
#include "MEZ1500_mzio_ltc185x.h"
#include "MEZ1500_mzio_ltc185x_prv.h"


#undef DEBUG
#define DEBUG
#ifdef DEBUG
#define DPRINTK(x...) {printk(__FUNCTION__"(%d): ",__LINE__);printk(##x);}
#else
#define DPRINTK(x...) (void)(0)
#endif

#define DEVICE_NAME "ltc185x"

extern void hello_export(void);

typedef struct {
  int delay;
} LTC185x_DEV;
static LTC185x_DEV LTC185xDev;

int gIntCount = 0;


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


// Timer interrupt handler
static irqreturn_t TimerINTHandler(int irq,void *TimDev)
{    
	unsigned long temp32;

	// Set for LED to toggle
	temp32 = __raw_readl(S3C2440_GPJDAT);
	temp32 &= ~(bDAT_CRNT_CN3_EN_N_CAM_DATA2);
	__raw_writel(temp32, S3C2440_GPJDAT);
	temp32 |= bDAT_CRNT_CN3_EN_N_CAM_DATA2;
	__raw_writel(temp32, S3C2440_GPJDAT);

  gIntCount++;

  if (gIntCount >= 1000)
  {
		printk("Beep=%d\n", gIntCount);	
		gIntCount = 0;
	}

  
  
  return IRQ_HANDLED;
}

static struct irqaction s3c2410_timer_irq = {
	.name		= "LTC185x tick timer",
	.flags		= IRQF_TIMER,
	.handler	= TimerINTHandler,
};


static int sbc2440_mzio_ltc1857_ioctl(
	struct inode *inode, 
	struct file *file, 
	unsigned int cmd, 
	unsigned long arg)
{	
	//printk("cmd=%d\n", cmd);	
	//printk("arg=%ld,%d\n", arg, (unsigned int) arg);	

	switch(cmd)
	{
		case MZIO_LTC1857_INIT:
			printk("LTC1857: Init the ltc185x module\n");	
/*
			{
				struct spi_device spi;
//				struct spi_device *spi = to_spi_device(dev);
				u8 txbuf[1], rxbuf[1];
				unsigned long reg;
				int ret;
				
				txbuf[0] = 0xAA;
				ret = spi_write(&spi, txbuf, len + 1);
				printk("  ret=%d, txbuf=%d,rxbuf=%d\n", ret, txbuf[0], rxbuf[0]);	

			}
*/
			return 0;

		default:
			return -EINVAL;
	}
}


static int sbc2440_mzio_ltc1857_read(struct file *filp, char *buffer, size_t count, loff_t *ppos)
{
	{
		char 			str[20];
		size_t 		len;
	  unsigned 	TimerCNTB = readl(S3C2410_TCNTB(2));

		len = sprintf(str, "TimerCNTB=%d\n", TimerCNTB);
		if (count >= len) {
			int r = copy_to_user(buffer, str, len);
			return r ? r : len;
		} else {
			return -EINVAL;
		}			
	}
	
	return 0;	
}



static struct file_operations dev_fops = {
	.owner	=	THIS_MODULE,
	.ioctl	=	sbc2440_mzio_ltc1857_ioctl,
	.read 	= sbc2440_mzio_ltc1857_read,
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &dev_fops,
};

static int __init dev_init(void)
{
	int ret;
	
	printk(DEVICE_NAME"\tversion %d%02d%02d%02d%02d\tinitialized\n", GetCompileYear(),
	GetCompileMonth(), GetCompileDay(), GetCompileHour(), GetCompileMinute());
	
//	hello_export();

	{
	  unsigned TimerControl;
	  unsigned TimerCfg0;
	  unsigned TimerCfg1;
	  unsigned TimerCNTB;
	  unsigned TimerCMPB;
	  static struct clk *timerclk;
		unsigned long pclk;
		unsigned long	temp32;

	  gIntCount = 0;
	  
	  TimerCfg0 		=	readl(S3C2410_TCFG0);
	  TimerCfg1 		=	readl(S3C2410_TCFG1);
	  TimerControl 	= readl(S3C2410_TCON);
		printk("TimerCfg0=0x%x\n", TimerCfg0);
		printk("TimerCfg1=0x%x\n", TimerCfg1);
		printk("TimerControl=0x%x\n", TimerControl);
		
		timerclk = clk_get(NULL, "timers");
		pclk = clk_get_rate(timerclk);
		printk("pclk=0x%lx\n", pclk);

/*
TimerCfg0=0x204
TimerCfg1=0x10
TimerControl=0x500000
pclk=0x30479e8
*/

		// Counter and compare registers  
	  TimerCNTB = readl(S3C2410_TCNTB(2));
	  TimerCMPB = readl(S3C2410_TCMPB(2));
	  
	  temp32 = pclk / (((TimerCfg0 & 0x0000FF00) >> 8) + 1) / 2;   // pclk / (prescalar + 1) / div(=2)
		printk("temp32=%ld\n", temp32);
	  TimerCNTB = temp32/1000;  // 1ms 
	  TimerCMPB = temp32/1000;
	  
	  writel(TimerCNTB, S3C2410_TCNTB(2));
	  writel(TimerCMPB, S3C2410_TCMPB(2));
	  
	  // Timer control
	  TimerControl |= S3C2410_TCON_T2RELOAD;
	  TimerControl |= S3C2410_TCON_T2MANUALUPD;
	  //TimerControl &= ~S3C2410_TCON_T2INVERT;
	  //TimerControl |= S3C2410_TCON_T2START;
	  
	  
	  writel(TimerControl, S3C2410_TCON);
	  	  
	  // Start the timer	  
	  TimerControl &= ~S3C2410_TCON_T2MANUALUPD;
	  TimerControl |= S3C2410_TCON_T2START;  
	  writel(TimerControl, S3C2410_TCON); 	  

	  TimerControl = readl(S3C2410_TCON);
		printk("TimerControl=0x%x\n", TimerControl);

	  gIntCount = 0;
	  
	  
	  // Setup PortJ
		__raw_writel(bGPCON_CAM_init,S3C2440_GPJCON);
		__raw_writel(bGPDAT_CAM_init,S3C2440_GPJDAT);
		__raw_writel(bGPUP_CAM_init,S3C2440_GPJUP);
		
		// Set for LED to toggle, init off
		temp32 = __raw_readl(S3C2440_GPJDAT);
		temp32 |= bDAT_CRNT_CN3_EN_N_CAM_DATA2;
		temp32 &= ~(bDAT_CLIM_EN_N_CAM_DATA3);
		__raw_writel(temp32, S3C2440_GPJDAT);
	}
	setup_irq(IRQ_TIMER2, &s3c2410_timer_irq);
	
	ret = misc_register(&misc);

	return ret;
}

static void __exit dev_exit(void)
{
	printk(DEVICE_NAME"\tgoodbye!\n");
	remove_irq(IRQ_TIMER2, &s3c2410_timer_irq);
	misc_deregister(&misc);
}


module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aceeca Inc.");
