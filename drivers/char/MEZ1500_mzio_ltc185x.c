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
	unsigned char	control;
	
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
/*
	// Channel processing
	
	// Read the current ADC value and attribute its value to the correct buffer
	
	// Get to next write channel 
	while (gLTC185x.ChSelect && !(gLTC185x.ChSelect & gLTC185x.CurrWrCh))
	{ 
		gLTC185x.CurrWrCh << 1;
		
		// Wrap around condition
		if (gLTC185x.CurrWrCh == ChLastSelect) gLTC185x.CurrWrCh = Ch0Select;
	}
	
	// Set the read channel to the current write channel
	gLTC185x.CurrRdCh = gLTC185x.CurrWrCh
	
	// Write the control byte
	if 			(gLTC185x.CurrWrCh == Ch0Select) 	control = gLTC185x.Ch0Ctrl;  
	else if (gLTC185x.CurrWrCh == Ch1Select)  control = gLTC185x.Ch1Ctrl;  
	else if (gLTC185x.CurrWrCh == Ch2Select)  control = gLTC185x.Ch2Ctrl;  
	else if (gLTC185x.CurrWrCh == Ch3Select)  control = gLTC185x.Ch3Ctrl;  
	else if (gLTC185x.CurrWrCh == Ch4Select)  control = gLTC185x.Ch4Ctrl;  
	else if (gLTC185x.CurrWrCh == Ch5Select)  control = gLTC185x.Ch5Ctrl;  
	else if (gLTC185x.CurrWrCh == Ch6Select)  control = gLTC185x.Ch6Ctrl;  
	else if (gLTC185x.CurrWrCh == Ch7Select)  control = gLTC185x.Ch7Ctrl;  
	else if (gLTC185x.CurrWrCh == Ch01Select) control = gLTC185x.Ch10Ctrl;
	else if (gLTC185x.CurrWrCh == Ch23Select) control = gLTC185x.Ch23Ctrl;
	else if (gLTC185x.CurrWrCh == Ch45Select) control = gLTC185x.Ch45Ctrl;
	else if (gLTC185x.CurrWrCh == Ch67Select) control = gLTC185x.Ch67Ctrl;
			
	if (gLTC185x.ChSelect & Ch0Select)
	{
		// Channel is selected 
	  if (gLTC185x.Ch0Thresh)
	  {
	  	// The counter threshold is > 0
	  	if (gLTC185x.Ch0Count++ >= gLTC185x.Ch0Thresh) 
	  	{
	  		// The counter threshold has been reached
				if (gLTC185x.CurrRdCh & Ch0Select)
				{
					// We need to read the ADC value
					gLTC185x.CurrRdCh 
				}

	  		// Count has reached the threshold, we sample this channel
				gLTC185x.Ch0Count = 0;

				gGPJDAT = readl(S3C2440_GPJDAT);

				// Assert RD line
				gGPJDAT &= ~(bCON_ADC_RD_EN_N_CAM_DATA5);
				writel(bGPDAT_CAM_init,S3C2440_GPJDAT);
			
				// Wakeup from sleep mode	
//				gGPJDAT &= ~(bCON_ADC_CNV_START_CAM_DATA6);
//				writel(bGPDAT_CAM_init,S3C2440_GPJDAT);

//				gGPJDAT |= bCON_ADC_CNV_START_CAM_DATA6;
//				writel(bGPDAT_CAM_init,S3C2440_GPJDAT);

				gGPJDAT &= ~(bCON_ADC_CNV_START_CAM_DATA6);
				writel(bGPDAT_CAM_init,S3C2440_GPJDAT);
			
				// Wait until BUSYn is deasserted
//				while (BUSY_IS_ASSERTED && timeout--) continue;
			
//				ADC_CONVST_LOW;
				 	
				// Send the SPI control code 	
				PrvSPISendReceiveData(gLTC185x.Ch0Ctrl); 		// Send control byte
				PrvSPISendReceiveData(0xFF);	  						// Sending dummy

				// Need delay?

				gGPJDAT |= bCON_ADC_CNV_START_CAM_DATA6;		// Trigger conversion
				writel(bGPDAT_CAM_init,S3C2440_GPJDAT);

				while (BUSY_IS_DEASSERTED && timeout--) continue;
				while (BUSY_IS_ASSERTED && timeout--) continue;
						
				// Grab 16-bit word and send ADC to sleep again
				sampleValueH	= PrvSPISendReceiveData(control);
				sampleValueL 	= PrvSPISendReceiveData(0xFF);
				
				if (ADCgP->chControl & LittleEndian)
				{
					// Make sample LittleEndian
					sampleValue = (sampleValueH << 8) | sampleValueL;
				}
				else
				{
					// Default is BigEndian
					sampleValue = (sampleValueL << 8) | sampleValueH;
				}	
			
				ADC_RD_OFF;	
				
				ADCgP->sampleCount++;				
			}
		}
		}
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
*/
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
			gLTC185x.ChData[Ch0Select].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch0Select].control = 	0;
			gLTC185x.ChData[Ch0Select].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT0;
			return 0;

		case MZIO_LTC185x_CH0SE_SET_PERIOD:
			gLTC185x.ChData[Ch0Select].count 	= 	0;
			gLTC185x.ChData[Ch0Select].thresh = 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH1SE_SETUP:
			gLTC185x.ChData[Ch1Select].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch1Select].control = 	0;
			gLTC185x.ChData[Ch1Select].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT1;
			return 0;

		case MZIO_LTC185x_CH1SE_SET_PERIOD:
			gLTC185x.ChData[Ch1Select].count 	= 	0;
			gLTC185x.ChData[Ch1Select].thresh = 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH2SE_SETUP:
			gLTC185x.ChData[Ch2Select].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch2Select].control = 	0;
			gLTC185x.ChData[Ch2Select].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT2;
			return 0;

		case MZIO_LTC185x_CH2SE_SET_PERIOD:
			gLTC185x.ChData[Ch2Select].count 	= 	0;
			gLTC185x.ChData[Ch2Select].thresh = 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH3SE_SETUP:
			gLTC185x.ChData[Ch3Select].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch3Select].control = 	0;
			gLTC185x.ChData[Ch3Select].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT3;
			return 0;

		case MZIO_LTC185x_CH3SE_SET_PERIOD:
			gLTC185x.ChData[Ch3Select].count 	= 	0;
			gLTC185x.ChData[Ch3Select].thresh = 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH4SE_SETUP:
			gLTC185x.ChData[Ch4Select].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch4Select].control = 	0;
			gLTC185x.ChData[Ch4Select].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT4;
			return 0;

		case MZIO_LTC185x_CH4SE_SET_PERIOD:
			gLTC185x.ChData[Ch4Select].count 	= 	0;
			gLTC185x.ChData[Ch4Select].thresh = 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH5SE_SETUP:
			gLTC185x.ChData[Ch5Select].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch5Select].control = 	0;
			gLTC185x.ChData[Ch5Select].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT5;
			return 0;

		case MZIO_LTC185x_CH5SE_SET_PERIOD:
			gLTC185x.ChData[Ch6Select].count 	= 	0;
			gLTC185x.ChData[Ch6Select].thresh = 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH6SE_SETUP:
			gLTC185x.ChData[Ch6Select].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch6Select].control = 	0;
			gLTC185x.ChData[Ch6Select].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT6;
			return 0;

		case MZIO_LTC185x_CH6SE_SET_PERIOD:
			gLTC185x.ChData[Ch6Select].count 		= 	0;
			gLTC185x.ChData[Ch6Select].thresh 	= 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH7SE_SETUP:
			gLTC185x.ChData[Ch7Select].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch7Select].control = 	0;
			gLTC185x.ChData[Ch7Select].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT7;
			return 0;

		case MZIO_LTC185x_CH7SE_SET_PERIOD:
			gLTC185x.ChData[Ch7Select].count 		= 	0;
			gLTC185x.ChData[Ch7Select].thresh 	= 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH01DE_SETUP:
			gLTC185x.ChData[Ch01Select].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch01Select].control = 	0;
			gLTC185x.ChData[Ch01Select].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT01;
			return 0;

		case MZIO_LTC185x_CH01SE_SET_PERIOD:
			gLTC185x.ChData[Ch01Select].count 	= 	0;
			gLTC185x.ChData[Ch01Select].thresh 	= 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH23DE_SETUP:
			gLTC185x.ChData[Ch23Select].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch23Select].control = 	0;
			gLTC185x.ChData[Ch23Select].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT23;
			return 0;

		case MZIO_LTC185x_CH23SE_SET_PERIOD:
			gLTC185x.ChData[Ch23Select].count 	= 	0;
			gLTC185x.ChData[Ch23Select].thresh 	= 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH45DE_SETUP:
			gLTC185x.ChData[Ch45Select].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch45Select].control = 	0;
			gLTC185x.ChData[Ch45Select].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT45;
			return 0;

		case MZIO_LTC185x_CH45SE_SET_PERIOD:
			gLTC185x.ChData[Ch45Select].count 	= 	0;
			gLTC185x.ChData[Ch45Select].thresh 	= 	arg;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH67DE_SETUP:
			gLTC185x.ChData[Ch67Select].enabled = 	(arg & LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch67Select].control = 	0;
			gLTC185x.ChData[Ch67Select].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT67;
			return 0;

		case MZIO_LTC185x_CH67SE_SET_PERIOD:
			gLTC185x.ChData[Ch67Select].count 	= 	0;
			gLTC185x.ChData[Ch67Select].thresh 	= 	arg;
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
