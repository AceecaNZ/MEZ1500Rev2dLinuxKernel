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
unsigned long gIntCount1000ms=0;
unsigned int 	gGPJDAT;

// ------------------------------------------------------------------
// Private routines
// ------------------------------------------------------------------
// Function for sending data out via SPI master, returns the contents in the receive register after sending
uint8_t PrvSPISendReceiveData(uint8_t data)
{
//	volatile HwrS3C2440_SPI_RegPtr 	spiP = (HwrS3C2440_SPI_RegPtr)hwrS3C2440_SPI_Base;

	// Send a byte
	writel(data, S3C2410_SPTDAT);

	// Wait until full 8 bits are sent
	while (!(readl(S3C2410_SPSTA)& S3C2410_SPSTA_READY)) continue;

	return readl(S3C2410_SPRDAT);
}




// ------------------------------------------------------------------
// Driver routines
// ------------------------------------------------------------------
// Timer interrupt handler
/*
	Note: The tconv conversion time is 5us max, which is half of the time between ISR IRQs.
	If we wait around for this, the overhead will likely kill the processor at 10us sampling
	periods. As such we need to stagger the sampling and reading between consequtive timer
	IRQs, so that a minimum amount of time is spent in the ISR itself. This will retain timing
	coherence and not bog down the processor. 
*/
static irqreturn_t TimerINTHandler(int irq,void *TimDev)
{
	unsigned char	ChSelected;
	unsigned char	sampleH, sampleL;
	unsigned int	sampleValue;
	int						Ch;
	
	// Skip the IRQ handling if we need to
	if (gLTC185x.SkipIRQ) goto exit;
	if (gLTC185x.InIRQ)		
	{
		// Reentrant code, shouldn't happen
		printk("Reentrant ISR issue!\n");
		goto exit;
	}
	
	// Mark that we are in the IRQ now
	gLTC185x.InIRQ = 1;
	gIntCount1000ms++;
	
	// Update all the counters for enabled channels
	for (Ch=Ch0; Ch<=Ch67; Ch++) {
		if (gLTC185x.ChData[Ch].enabled) {
			if (gLTC185x.ChData[Ch].trig) {
				if (gLTC185x.ChData[Ch].count) gLTC185x.ChData[Ch].count--;
				else {	
					// Add this channel to the sequencer
					*(gLTC185x.seqP++) = Ch;
		
					// Wrap around the sequencing pointer if necessary
					if (gLTC185x.seqP > &gLTC185x.Sequence[ChMax]) gLTC185x.seqP = gLTC185x.Sequence;
		
					// Reset the counter
					gLTC185x.ChData[Ch].count = gLTC185x.ChData[Ch].trig;
		
					printk("Ch=%d triggered\n", Ch);
					break;
				}
			}
		}
	}

	// ---------------------------------------------
	// Write new data to ADC for conversion
	// ---------------------------------------------
	ChSelected = *gLTC185x.wrP;
	*gLTC185x.wrP = 0;
	
	// Increment and wrap around the write pointer if necessary
	if (gLTC185x.wrP > &gLTC185x.Sequence[ChMax]) gLTC185x.wrP = gLTC185x.Sequence;
	else gLTC185x.wrP++;


	gGPJDAT = readl(S3C2440_GPJDAT);

	// Assert RD line low, set CONVST to low as well
	gGPJDAT &= ~(bCON_ADC_RD_EN_N_CAM_DATA5|bCON_ADC_CNV_START_CAM_DATA6);
	writel(bGPDAT_CAM_init,S3C2440_GPJDAT);
		 	
	// Send the SPI control code 	
	// Send control byte
	sampleH = PrvSPISendReceiveData(gLTC185x.ChData[ChSelected].control); 		
	// Sending dummy
	sampleL = PrvSPISendReceiveData(0xFF);	  																

	// CONVST high, trigger conversion, also disable read enable
	gGPJDAT |= (bCON_ADC_CNV_START_CAM_DATA6|bCON_ADC_RD_EN_N_CAM_DATA5);												
	writel(bGPDAT_CAM_init,S3C2440_GPJDAT);


	// ---------------------------------------------
	// Process the read data from ADC
	// ---------------------------------------------
	ChSelected = *gLTC185x.rdP;
	*gLTC185x.rdP = 0;
	
	// Increment and wrap around the read pointer if necessary
	if (gLTC185x.rdP > &gLTC185x.Sequence[ChMax]) gLTC185x.rdP = gLTC185x.Sequence;
	else gLTC185x.rdP++;
	
	sampleValue = (sampleL << 8) | sampleH;

	// For now just print out the value to the log	
	printk("Ch%d=0x%x\n", ChSelected, sampleValue);

  if (gIntCount1000ms >= Timer1000ms*5)
  {
		printk("Beep:\n");
		gIntCount1000ms = 0;
	}
exit:
	gLTC185x.InIRQ = 0;
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
			
			// Init arrays
			memset(gLTC185x.Sequence, 0, sizeof(gLTC185x.Sequence));
			gLTC185x.wrP = gLTC185x.Sequence;
			gLTC185x.rdP = gLTC185x.Sequence;			
			gLTC185x.seqP = gLTC185x.Sequence;			
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
		case MZIO_LTC185x_CH0SE_SETUP:
			gLTC185x.ChData[Ch0].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch0].control = 	0;
			gLTC185x.ChData[Ch0].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT0;
			return 0;

		case MZIO_LTC185x_CH0SE_SET_PERIOD:
			gLTC185x.ChData[Ch0].count 	= 	arg;
			gLTC185x.ChData[Ch0].trig = 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH1SE_SETUP:
			gLTC185x.ChData[Ch1].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch1].control = 	0;
			gLTC185x.ChData[Ch1].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT1;
			return 0;

		case MZIO_LTC185x_CH1SE_SET_PERIOD:
			gLTC185x.ChData[Ch1].count 	= 	arg;
			gLTC185x.ChData[Ch1].trig = 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH2SE_SETUP:
			gLTC185x.ChData[Ch2].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch2].control = 	0;
			gLTC185x.ChData[Ch2].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT2;
			return 0;

		case MZIO_LTC185x_CH2SE_SET_PERIOD:
			gLTC185x.ChData[Ch2].count 	= 	arg;
			gLTC185x.ChData[Ch2].trig = 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH3SE_SETUP:
			gLTC185x.ChData[Ch3].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch3].control = 	arg;
			gLTC185x.ChData[Ch3].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT3;
			return 0;

		case MZIO_LTC185x_CH3SE_SET_PERIOD:
			gLTC185x.ChData[Ch3].count 	= 	arg;
			gLTC185x.ChData[Ch3].trig = 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH4SE_SETUP:
			gLTC185x.ChData[Ch4].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch4].control = 	arg;
			gLTC185x.ChData[Ch4].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT4;
			return 0;

		case MZIO_LTC185x_CH4SE_SET_PERIOD:
			gLTC185x.ChData[Ch4].count 	= 	arg;
			gLTC185x.ChData[Ch4].trig = 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH5SE_SETUP:
			gLTC185x.ChData[Ch5].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch5].control = 	arg;
			gLTC185x.ChData[Ch5].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT5;
			return 0;

		case MZIO_LTC185x_CH5SE_SET_PERIOD:
			gLTC185x.ChData[Ch6].count 	= 	arg;
			gLTC185x.ChData[Ch6].trig = 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH6SE_SETUP:
			gLTC185x.ChData[Ch6].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch6].control = 	arg;
			gLTC185x.ChData[Ch6].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT6;
			return 0;

		case MZIO_LTC185x_CH6SE_SET_PERIOD:
			gLTC185x.ChData[Ch6].count 		= 	arg;
			gLTC185x.ChData[Ch6].trig 	= 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH7SE_SETUP:
			gLTC185x.ChData[Ch7].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch7].control = 	0;
			gLTC185x.ChData[Ch7].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT7;
			return 0;

		case MZIO_LTC185x_CH7SE_SET_PERIOD:
			gLTC185x.ChData[Ch7].count 		= 	arg;
			gLTC185x.ChData[Ch7].trig 	= 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH01DE_SETUP:
			gLTC185x.ChData[Ch01].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch01].control = 	arg;
			gLTC185x.ChData[Ch01].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT01;
			return 0;

		case MZIO_LTC185x_CH01SE_SET_PERIOD:
			gLTC185x.ChData[Ch01].count 	= 	arg;
			gLTC185x.ChData[Ch01].trig 	= 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH23DE_SETUP:
			gLTC185x.ChData[Ch23].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch23].control = 	arg;
			gLTC185x.ChData[Ch23].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT23;
			return 0;

		case MZIO_LTC185x_CH23SE_SET_PERIOD:
			gLTC185x.ChData[Ch23].count 	= 	arg;
			gLTC185x.ChData[Ch23].trig 	= 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH45DE_SETUP:
			gLTC185x.ChData[Ch45].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch45].control = 	arg;
			gLTC185x.ChData[Ch45].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT45;
			return 0;

		case MZIO_LTC185x_CH45SE_SET_PERIOD:
			gLTC185x.ChData[Ch45].count 	= 	arg;
			gLTC185x.ChData[Ch45].trig 	= 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH67DE_SETUP:
			gLTC185x.ChData[Ch67].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch67].control = 	arg;
			gLTC185x.ChData[Ch67].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT67;
			return 0;

		case MZIO_LTC185x_CH67SE_SET_PERIOD:
			gLTC185x.ChData[Ch67].count 	= 	arg;
			gLTC185x.ChData[Ch67].trig 	= 	arg;
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
