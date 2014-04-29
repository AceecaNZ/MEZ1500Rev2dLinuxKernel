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

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <plat/regs-timer.h>
#include <plat/regs-adc.h>
#include <plat/regs-spi.h>
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

static LTC185x_DEV gLTC185x;

unsigned long	gIntCount1ms 		= 0;
unsigned long	gIntCount10ms 	= 0;
unsigned long	gIntCount100ms 	= 0;
unsigned long	gIntCount1000ms	= 0;
unsigned int 	gGPJDAT;


// Timer interrupt handler
static irqreturn_t TimerINTHandler(int irq,void *TimDev)
{    

  gIntCount1ms++;
  gIntCount10ms++;
  gIntCount100ms++;
  gIntCount1000ms++;

  if (gIntCount1ms >= Timer1ms)
  {
//		printk("Beep=%d\n", gIntCount);	
		gIntCount1ms = 0;

		// Set for LED to toggle
/*
		gGPJDAT = __raw_readl(S3C2440_GPJDAT);
		if (gGPJDAT & bDAT_CRNT_CN3_EN_N_CAM_DATA2)
		{
			gGPJDAT	&= ~(bDAT_CRNT_CN3_EN_N_CAM_DATA2);
		}
		else
		{
			gGPJDAT	|= bDAT_CRNT_CN3_EN_N_CAM_DATA2;		
		}
		gGPJDAT	&= ~(bDAT_CRNT_CN3_EN_N_CAM_DATA2);
		__raw_writel(gGPJDAT, S3C2440_GPJDAT);
		gGPJDAT	|= bDAT_CRNT_CN3_EN_N_CAM_DATA2;		
		__raw_writel(gGPJDAT, S3C2440_GPJDAT);
*/
	}


  if (gIntCount10ms >= Timer10ms)
  {
		gIntCount10ms = 0;
	}  
  
  if (gIntCount100ms >= Timer100ms)
  {
		gIntCount100ms = 0;
		// Set for LED to toggle
		gGPJDAT = __raw_readl(S3C2440_GPJDAT);
		gGPJDAT	&= ~(bDAT_CRNT_CN3_EN_N_CAM_DATA2);
		__raw_writel(gGPJDAT, S3C2440_GPJDAT);
		gGPJDAT	|= bDAT_CRNT_CN3_EN_N_CAM_DATA2;		
		__raw_writel(gGPJDAT, S3C2440_GPJDAT);
	}  

  if (gIntCount1000ms >= Timer1000ms)
  {
		printk("ISR:\n");	
		gIntCount1000ms = 0;
	}  

  return IRQ_HANDLED;
}

static struct irqaction s3c2410_timer_irq = {
	.name		= "LTC185x tick timer",
	.flags		= IRQF_TIMER,
	.handler	= TimerINTHandler,
};


static int sbc2440_mzio_LTC185x_ioctl(
	struct inode *inode, 
	struct file *file, 
	unsigned int cmd, 
	unsigned long arg)
{	
	unsigned long temp32;

	switch(cmd)
	{
		// -----------------------------------------------------
		// Init routines
		// -----------------------------------------------------		
		case MZIO_LTC185x_INIT:					
			printk("LTC185x: Init++\n");	
		
			// Turn off the SPI clock
			temp32 = readl(S3C2410_CLKCON);
			temp32 &= ~(S3C2410_CLKCON_SPI);
			writel(temp32,S3C2410_CLKCON);

			// Turn off the MZIO 5V
			s3c2410_gpio_setpin(S3C2410_GPC(8), 1);

			// Setup GPIO
		  // Setup PortJ
			writel(bGPCON_CAM_init,S3C2440_GPJCON);
			writel(bGPDAT_CAM_init,S3C2440_GPJDAT);
			writel(bGPUP_CAM_init,S3C2440_GPJUP);				

			// Setup SPI lines as dedicated
			writel(bmaskGPECON_SPI_INIT, S3C2410_GPECON);
	
			// SPI clock turn on	
			temp32 = readl(S3C2410_CLKCON);
			temp32 |= ~(S3C2410_CLKCON_SPI);
			writel(temp32,S3C2410_CLKCON);

			// Power up the 5V, to power the CLIM
			s3c2410_gpio_setpin(S3C2410_GPC(8), 0);
		
			// Setup SPICON0 register
			// Polling mode, CLK enabled, Master, pol=pha=0, no garbage mode
			writel(bSPCONx_SMOD_Polling|bSPCONx_ENSCK|bSPCONx_MSTR, S3C2410_SPCON);
		
			// Set prescaler value, 
			// Note: LTC185x only supports up to 20MHz tops, closest configuration with PCLK=50MHz, is for 12.5MHz.  baud=PCLK/2/(prescaler+1)	 
			writel(2, S3C2410_SPPRE);
			printk("LTC185x: Init--\n");	
			return 0;

		case MZIO_LTC185x_DEINIT:
			printk("LTC185x: Deinit++\n");	

			// Turn off the SPI clock
			temp32 = readl(S3C2410_CLKCON);
			temp32 &= ~(S3C2410_CLKCON_SPI);
			writel(temp32,S3C2410_CLKCON);

			// Set 5V to enabled once again
			s3c2410_gpio_setpin(S3C2410_GPC(8), 0);

			// Deinit the SPI registers	
			writel(0, S3C2410_SPCON);

		  // Setup PortJ
			writel(bGPCON_CAM_init,S3C2440_GPJCON);
			writel(bGPDAT_CAM_init,S3C2440_GPJDAT);
			writel(bGPUP_CAM_init, S3C2440_GPJUP);				

			// Turn off the MZIO 5V
			s3c2410_gpio_setpin(S3C2410_GPC(8), 1);
			
			// Set SPI lines as output low
			writel(bmaskGPECON_SPI_DEINIT, S3C2410_GPECON);
			temp32 = readl(S3C2410_GPEDAT);
			temp32 &= ~(bGPEDAT_MZIO_SPIMISO|bGPEDAT_MZIO_SPIMOSI|bGPEDAT_MZIO_SPICLK);
			writel(temp32,S3C2410_GPEDAT);
				
			printk("LTC185x: Deinit--\n");	
			return 0;


		// -----------------------------------------------------
		// ADC Channel setup routines
		// -----------------------------------------------------
		case MZIO_LTC185x_SETUP_CH0SE:
			if (arg & LTC185x_ChSetup_Enabled) 	gLTC185x.ChSelect |= Ch0Select;
			else 																gLTC185x.ChSelect &= ~(Ch0Select);
							
			gLTC185x.Ch0Ctrl = 	0;				
			gLTC185x.Ch0Ctrl |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT0;						

			printk("LTC185x: Setup CH0SE ctrl=0x%x\n", gLTC185x.Ch0Ctrl);	
			return 0;

		case MZIO_LTC185x_SETUP_CH1SE:
			if (arg & LTC185x_ChSetup_Enabled) 	gLTC185x.ChSelect |= Ch1Select;
			else 																gLTC185x.ChSelect &= ~(Ch1Select);
			
			gLTC185x.Ch1Ctrl = 	0;				
			gLTC185x.Ch1Ctrl |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT1;						

			printk("LTC185x: Setup CH1SE ctrl=0x%x\n", gLTC185x.Ch1Ctrl);	
			return 0;

		case MZIO_LTC185x_SETUP_CH2SE:
			if (arg & LTC185x_ChSetup_Enabled) 	gLTC185x.ChSelect |= Ch2Select;
			else 																gLTC185x.ChSelect &= ~(Ch2Select);
			
			gLTC185x.Ch2Ctrl = 	0;				
			gLTC185x.Ch2Ctrl |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT2;						

			printk("LTC185x: Setup CH2SE ctrl=0x%x\n", gLTC185x.Ch2Ctrl);	
			return 0;

		case MZIO_LTC185x_SETUP_CH3SE:
			if (arg & LTC185x_ChSetup_Enabled) 	gLTC185x.ChSelect |= Ch3Select;
			else 																gLTC185x.ChSelect &= ~(Ch3Select);
			
			gLTC185x.Ch3Ctrl = 	0;				
			gLTC185x.Ch3Ctrl |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT3;						

			printk("LTC185x: Setup CH3SE ctrl=0x%x\n", gLTC185x.Ch3Ctrl);	
			return 0;

		case MZIO_LTC185x_SETUP_CH4SE:
			if (arg & LTC185x_ChSetup_Enabled) 	gLTC185x.ChSelect |= Ch4Select;
			else 																gLTC185x.ChSelect &= ~(Ch4Select);
			
			gLTC185x.Ch4Ctrl = 	0;				
			gLTC185x.Ch4Ctrl |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT4;						

			printk("LTC185x: Setup CH4SE ctrl=0x%x\n", gLTC185x.Ch4Ctrl);	
			return 0;

		case MZIO_LTC185x_SETUP_CH5SE:
			if (arg & LTC185x_ChSetup_Enabled) 	gLTC185x.ChSelect |= Ch5Select;
			else 																gLTC185x.ChSelect &= ~(Ch5Select);
			
			gLTC185x.Ch5Ctrl = 	0;				
			gLTC185x.Ch5Ctrl |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT5;						

			printk("LTC185x: Setup CH5SE ctrl=0x%x\n", gLTC185x.Ch5Ctrl);	
			return 0;

		case MZIO_LTC185x_SETUP_CH6SE:
			if (arg & LTC185x_ChSetup_Enabled) 	gLTC185x.ChSelect |= Ch6Select;
			else 																gLTC185x.ChSelect &= ~(Ch6Select);
			
			gLTC185x.Ch6Ctrl = 	0;				
			gLTC185x.Ch6Ctrl |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT6;						

			printk("LTC185x: Setup CH6SE ctrl=0x%x\n", gLTC185x.Ch6Ctrl);	
			return 0;

		case MZIO_LTC185x_SETUP_CH7SE:
			if (arg & LTC185x_ChSetup_Enabled) 	gLTC185x.ChSelect |= Ch7Select;
			else 																gLTC185x.ChSelect &= ~(Ch7Select);
			
			gLTC185x.Ch7Ctrl = 	0;				
			gLTC185x.Ch7Ctrl |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT7;						

			printk("LTC185x: Setup CH7SE ctrl=0x%x\n", gLTC185x.Ch7Ctrl);	
			return 0;

		case MZIO_LTC185x_SETUP_CH01DE:
			if (arg & LTC185x_ChSetup_Enabled) 	gLTC185x.ChSelect |= Ch01Select;
			else 																gLTC185x.ChSelect &= ~(Ch01Select);
			
			gLTC185x.Ch01Ctrl = 0;
			gLTC185x.Ch01Ctrl |= (arg & 0xFF) | ADC_SINGLE_ENDED_INPUT01;

			printk("LTC185x: Setup CH01DE ctrl=0x%x\n", gLTC185x.Ch01Ctrl);	
			return 0;

		case MZIO_LTC185x_SETUP_CH23DE:
			if (arg & LTC185x_ChSetup_Enabled) 	gLTC185x.ChSelect |= Ch23Select;
			else 																gLTC185x.ChSelect &= ~(Ch23Select);
			
			gLTC185x.Ch23Ctrl = 0;
			gLTC185x.Ch23Ctrl |= (arg & 0xFF) | ADC_SINGLE_ENDED_INPUT23;

			printk("LTC185x: Setup CH23DE ctrl=0x%x\n", gLTC185x.Ch23Ctrl);	
			return 0;

		case MZIO_LTC185x_SETUP_CH45DE:
			if (arg & LTC185x_ChSetup_Enabled) 	gLTC185x.ChSelect |= Ch45Select;
			else 																gLTC185x.ChSelect &= ~(Ch45Select);
			
			gLTC185x.Ch45Ctrl = 0;
			gLTC185x.Ch45Ctrl |= (arg & 0xFF) | ADC_SINGLE_ENDED_INPUT45;

			printk("LTC185x: Setup CH45DE ctrl=0x%x\n", gLTC185x.Ch45Ctrl);	
			return 0;

		case MZIO_LTC185x_SETUP_CH67DE:
			if (arg & LTC185x_ChSetup_Enabled) 	gLTC185x.ChSelect |= Ch67Select;
			else 																gLTC185x.ChSelect &= ~(Ch67Select);
			
			gLTC185x.Ch67Ctrl = 0;
			gLTC185x.Ch67Ctrl |= (arg & 0xFF) | ADC_SINGLE_ENDED_INPUT67;

			printk("LTC185x: Setup CH67DE ctrl=0x%x\n", gLTC185x.Ch67Ctrl);	
			return 0;


		// -----------------------------------------------------
		// 5V power control
		// -----------------------------------------------------
		case MZIO_LTC185x_5VCn1_Enable:
			if (arg) 
			{
				gGPJDAT = readl(S3C2440_GPJDAT);
				gGPJDAT	&= ~(bDAT_CRNT_CN1_EN_N_CAM_DATA0|bDAT_CLIM_EN_N_CAM_DATA3);
				writel(gGPJDAT, S3C2440_GPJDAT);		
				printk("LTC185x: 5V CN1 enabled\n");	
			} else
			{
				gGPJDAT = readl(S3C2440_GPJDAT);
				gGPJDAT	|= bDAT_CRNT_CN1_EN_N_CAM_DATA0;
				
				// Turn off CLIM if all lines are now off
				if (!(gGPJDAT & bDAT_CRNT_CN2_EN_N_CAM_DATA1) && !(gGPJDAT & bDAT_CRNT_CN3_EN_N_CAM_DATA2) )
					gGPJDAT	|= bDAT_CLIM_EN_N_CAM_DATA3;
				
				writel(gGPJDAT, S3C2440_GPJDAT);						
				printk("LTC185x: 5V CN1 disabled\n");	
			}			
			return 0;

		case MZIO_LTC185x_5VCn2_Enable:
			if (arg) 
			{
				gGPJDAT = readl(S3C2440_GPJDAT);
				gGPJDAT	&= ~(bDAT_CRNT_CN2_EN_N_CAM_DATA1|bDAT_CLIM_EN_N_CAM_DATA3);
				writel(gGPJDAT, S3C2440_GPJDAT);		
				printk("LTC185x: 5V CN1 enabled\n");	
			} else
			{
				gGPJDAT = readl(S3C2440_GPJDAT);
				gGPJDAT	|= bDAT_CRNT_CN2_EN_N_CAM_DATA1;

				// Turn off CLIM if all lines are now off
				if (!(gGPJDAT & bDAT_CRNT_CN1_EN_N_CAM_DATA0) && !(gGPJDAT & bDAT_CRNT_CN3_EN_N_CAM_DATA2) )
					gGPJDAT	|= bDAT_CLIM_EN_N_CAM_DATA3;

				writel(gGPJDAT, S3C2440_GPJDAT);						
				printk("LTC185x: 5V CN2 disabled\n");	
			}
			return 0;

		case MZIO_LTC185x_5VCn3_Enable:
			if (arg) 
			{
				gGPJDAT = readl(S3C2440_GPJDAT);
				gGPJDAT	&= ~(bDAT_CRNT_CN3_EN_N_CAM_DATA2|bDAT_CLIM_EN_N_CAM_DATA3);
				writel(gGPJDAT, S3C2440_GPJDAT);		
				printk("LTC185x: 5V CN3 enabled\n");	
			} else
			{
				gGPJDAT = readl(S3C2440_GPJDAT);
				gGPJDAT	|= bDAT_CRNT_CN3_EN_N_CAM_DATA2;

				// Turn off CLIM if all lines are now off
				if (!(gGPJDAT & bDAT_CRNT_CN1_EN_N_CAM_DATA0) && !(gGPJDAT & bDAT_CRNT_CN2_EN_N_CAM_DATA1) )
					gGPJDAT	|= bDAT_CLIM_EN_N_CAM_DATA3;

				writel(gGPJDAT, S3C2440_GPJDAT);						
				printk("LTC185x: 5V CN3 disabled\n");	
			}
			return 0;

	
		// -----------------------------------------------------
		// Start/stop sampling
		// -----------------------------------------------------
		case MZIO_LTC185x_START:
			{
			  unsigned TimerControl;
			  unsigned TimerCfg0;
			  unsigned TimerCfg1;
			  unsigned TimerCNTB;
			  unsigned TimerCMPB;
			  static struct clk *timerclk;
				unsigned long pclk;
				unsigned long	temp32;
			  
			  TimerCfg0 		=	readl(S3C2410_TCFG0);
			  TimerCfg1 		=	readl(S3C2410_TCFG1);
			  TimerControl 	= readl(S3C2410_TCON);
				printk("TimerCfg0=0x%x\n", TimerCfg0);
				printk("TimerCfg1=0x%x\n", TimerCfg1);
				printk("TimerControl=0x%x\n", TimerControl);
				
				timerclk = clk_get(NULL, "timers");
				pclk = clk_get_rate(timerclk);
				printk("pclk=0x%lx\n", pclk);
				
				// Counter and compare registers  
			  TimerCNTB = readl(S3C2410_TCNTB(2));
			  TimerCMPB = readl(S3C2410_TCMPB(2));
			  
			  temp32 = pclk / (((TimerCfg0 & 0x0000FF00) >> 8) + 1) / 2;   // pclk / (prescalar + 1) / div(=2)
				printk("temp32=%ld\n", temp32);
			  TimerCNTB = temp32/TimerFreq;
			  TimerCMPB = temp32/TimerFreq;
			  
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
	
				// Setup the IRQ
				setup_irq(IRQ_TIMER2, &s3c2410_timer_irq);		
			}
			printk("LTC185x: Start\n");	
			return 0;

		case MZIO_LTC185x_STOP:
			{
				unsigned long TimerControl;
				
				remove_irq(IRQ_TIMER2, &s3c2410_timer_irq);
				
				// Stop the timer
			  TimerControl 	= readl(S3C2410_TCON);
 			  TimerControl &= ~S3C2410_TCON_T2RELOAD;
			  TimerControl &= ~S3C2410_TCON_T2MANUALUPD;
			  TimerControl &= ~S3C2410_TCON_T2INVERT;
			  TimerControl &= ~S3C2410_TCON_T2START;
			  writel(TimerControl, S3C2410_TCON);
			}
			printk("LTC185x: Stop\n");	
			return 0;

		default:
			return -EINVAL;
	}
}

/*
static int sbc2440_mzio_LTC185x_read(struct file *filp, char *buffer, size_t count, loff_t *ppos)
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
*/


static struct file_operations dev_fops = {
	.owner	=	THIS_MODULE,
	.ioctl	=	sbc2440_mzio_LTC185x_ioctl,
//	.read 	= sbc2440_mzio_LTC185x_read,
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
