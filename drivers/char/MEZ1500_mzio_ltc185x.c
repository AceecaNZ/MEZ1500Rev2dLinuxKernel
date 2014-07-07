#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/pci.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>	// for mmap stuff
#include <asm/uaccess.h>		// copy_to_user
//#include <linux/delay.h>
//#include <asm/irq.h>
//#include <linux/errno.h>
//#include <linux/list.h>
//#include <asm/atomic.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <mach/map.h>
#include <mach/hardware.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <plat/regs-timer.h>
#include <plat/regs-adc.h>
#include <plat/regs-spi.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include "MEZ1500_mzio.h"
#include "MEZ1500_mzio_ltc185x.h"
#include "MEZ1500_mzio_ltc185x_prv.h"


#define SPI_MISO			S3C2410_GPE(11)
#define SPI_MOSI			S3C2410_GPE(12)
#define SPI_CLK				S3C2410_GPE(13)
                                          
#define MZIO_5V_ENn		S3C2410_GPC(8)

static void __iomem *base_addr_SPI;
#define SPCON      (*(volatile unsigned long *)(base_addr_SPI + S3C2410_SPCON	))//SPI control
#define SPPRE      (*(volatile unsigned long *)(base_addr_SPI + S3C2410_SPPRE	))//SPI prescaler
#define SPSTA      (*(volatile unsigned long *)(base_addr_SPI + S3C2410_SPSTA	))//SPI status
#define SPTDAT     (*(volatile unsigned long *)(base_addr_SPI + S3C2410_SPTDAT))//SPI transmit data
#define SPRDAT     (*(volatile unsigned long *)(base_addr_SPI + S3C2410_SPRDAT))//SPI receive data

static void __iomem *base_addr_CLK;
//#define CLKCON     (*(volatile unsigned long *)(base_addr_CLK + S3C2410_CLKCON))	//clock control
#define CLKCON     (*(volatile unsigned long *)(S3C2410_CLKCON))	//clock control

static void __iomem *base_addr_GPIO;
#define GPJCON 		(*(volatile unsigned long *)(base_addr_GPIO + S3C2440_GPJCON))	//	port J control
#define GPJDAT 		(*(volatile unsigned long *)(base_addr_GPIO + S3C2440_GPJDAT))	//	port J data


#undef DEBUG
#define DEBUG
#ifdef DEBUG
#define DPRINTK(x...) {printk(__FUNCTION__"(%d): ",__LINE__);printk(##x);}
#else
#define DPRINTK(x...) (void)(0)
#endif

#define DEVICE_NAME "ltc185x"

#define EndianSwap16(n)	(((((unsigned int) n) << 8) & 0xFF00) | \
                         ((((unsigned int) n) >> 8) & 0x00FF))


#define EndianSwap32(n)	(((((unsigned long) n) << 24) & 0xFF000000) |	\
                         ((((unsigned long) n) <<  8) & 0x00FF0000) |	\
                         ((((unsigned long) n) >>  8) & 0x0000FF00) |	\
                         ((((unsigned long) n) >> 24) & 0x000000FF))

static LTC185x_DEV gLTC185x;
unsigned long gGPJDAT;
unsigned int 	gTimerReloadValue;



// ------------------------------------------------------------------
// TODO List:
// - ensure concurrency of variables changed in ISR is not broken, numSamples particularly
// ------------------------------------------------------------------


// ------------------------------------------------------------------
// Private routines
// ------------------------------------------------------------------
// Function for sending data out via SPI master, returns the contents in the receive register after sending
/*
static uint8_t PrvSPISendReceiveData(uint8_t data)
{
	// Send a byte
	SPTDAT = data;

	// Wait until full 8 bits are sent
	while (!SPSTA& S3C2410_SPSTA_READY) continue;

	return SPRDAT;
}
*/
static void PrvStartTimer(void)
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

	// Calculating timer input clock frequency
  temp32 = pclk / (((TimerCfg0 & 0x0000FF00) >> 8) + 1) / 2;   // pclk / (prescalar + 1) / divider value (=2)
  // Note: Prescalar is set by system to be 2
  // Divider for mux2 is set at 1/2, divider value is 2
  
	printk("Freq=%ld\n", temp32);
  TimerCNTB = temp32/TimerIntFreq;
  TimerCMPB = temp32/TimerIntFreq;

	printk("TimerCNTB=%d\n", TimerCNTB);
	printk("TimerCMPB=%d\n", TimerCMPB);
	
	gTimerReloadValue = TimerCNTB;

  writel(TimerCNTB, S3C2410_TCNTB(2));
  writel(TimerCMPB, S3C2410_TCMPB(2));

	// Setup the IRQ
	if(setup_irq(IRQ_TIMER2, &s3c2410_timer_irq) == 0)
		s3c2410_timer_irq.dev_id = &s3c2410_timer_irq;

  // Timer control
  TimerControl |= S3C2410_TCON_T2RELOAD;
  TimerControl |= S3C2410_TCON_T2MANUALUPD;
  //TimerControl &= ~S3C2410_TCON_T2INVERT;
  //TimerControl |= S3C2410_TCON_T2START;

  writel(TimerControl, S3C2410_TCON);

  // Start the timer
//  TimerControl &= ~(S3C2410_TCON_T2RELOAD);
  TimerControl &= ~(S3C2410_TCON_T2MANUALUPD);
  TimerControl |= S3C2410_TCON_T2START;
  writel(TimerControl, S3C2410_TCON);
//  writel(gTimerReloadValue, S3C2410_TCNTB(2));

  TimerControl = readl(S3C2410_TCON);
	printk("TimerControl=0x%x\n", TimerControl);

}

static void PrvStopTimer(void)
{
	unsigned long TimerControl;

	if(s3c2410_timer_irq.dev_id)
	{
		remove_irq(IRQ_TIMER2, &s3c2410_timer_irq);
		s3c2410_timer_irq.dev_id = 0;
	}

	// Stop the timer
  TimerControl 	= readl(S3C2410_TCON);
  TimerControl &= ~S3C2410_TCON_T2RELOAD;
  TimerControl &= ~S3C2410_TCON_T2MANUALUPD;
  TimerControl &= ~S3C2410_TCON_T2INVERT;
  TimerControl &= ~S3C2410_TCON_T2START;
  writel(TimerControl, S3C2410_TCON);
}

static void PrvLTC185xInit(void)
{
	printk("LTC185x: Init++\n");
	
	// Turn off the SPI clock
	CLKCON &= ~(S3C2410_CLKCON_SPI);
	
	// Turn off the MZIO 5V
	s3c2410_gpio_setpin(MZIO_5V_ENn, 1);
	
	// Setup GPIO
	// Setup PortJ
	writel(bGPCON_CAM_init,S3C2440_GPJCON);
	writel(bGPDAT_CAM_init,S3C2440_GPJDAT);
	writel(bGPUP_CAM_init,S3C2440_GPJUP);
	
	// Setup SPI lines as dedicated
	s3c2410_gpio_cfgpin(SPI_MISO, S3C2410_GPIO_SFN2);			
	s3c2410_gpio_cfgpin(SPI_MOSI	, S3C2410_GPIO_SFN2);			
	s3c2410_gpio_cfgpin(SPI_CLK	, S3C2410_GPIO_SFN2);	
	
	// SPI clock turn on
	CLKCON |= S3C2410_CLKCON_SPI;
	
	// Power up the 5V, to power the CLIM
	s3c2410_gpio_setpin(MZIO_5V_ENn, 0);
	
	// Setup SPICON0 register
	// Polling mode, CLK enabled, Master, pol=pha=0, no garbage mode
	SPCON = bSPCONx_SMOD_Polling|bSPCONx_ENSCK|bSPCONx_MSTR;
	
	// Set prescaler value,
	// Note: LTC185x only supports up to 20MHz tops, closest configuration with PCLK=50MHz, is for 12.5MHz.  baud=PCLK/2/(prescaler+1)
	SPPRE=2;
	
	printk("LTC185x: Init--\n");	
}


static void PrvLTC185xDeinit(void)
{
	printk("LTC185x: Deinit++\n");

	// Stop the timer
	PrvStopTimer();

	// Turn off the SPI clock
	CLKCON &= ~(S3C2410_CLKCON_SPI);

	// Set 5V to enabled once again
	s3c2410_gpio_setpin(MZIO_5V_ENn, 0);

	// Deinit the SPI registers
	SPCON = 0;

  // Setup PortJ
	writel(bGPCON_CAM_init,S3C2440_GPJCON);
	writel(bGPDAT_CAM_init,S3C2440_GPJDAT);
	writel(bGPUP_CAM_init, S3C2440_GPJUP);

	// Turn off the MZIO 5V
	s3c2410_gpio_setpin(MZIO_5V_ENn, 1);

	// Set SPI lines as output low
	s3c2410_gpio_cfgpin(SPI_MISO, S3C2410_GPIO_OUTPUT);			
	s3c2410_gpio_cfgpin(SPI_MOSI	, S3C2410_GPIO_OUTPUT);			
	s3c2410_gpio_cfgpin(SPI_CLK	, S3C2410_GPIO_OUTPUT);	
			
	s3c2410_gpio_setpin(SPI_MISO, 0);
	s3c2410_gpio_setpin(SPI_MOSI	, 0);
	s3c2410_gpio_setpin(SPI_CLK	, 0);
	
	printk("LTC185x: Deinit--\n");	
}

static int PrvSetChannelConfig(unsigned int arg)
{
	ChConfigDataType ChConfig;
	unsigned long offset;
	
	// Get data from user space
	if (copy_from_user(&ChConfig, (void __user*)arg, sizeof(ChConfigDataType))) return -EFAULT;							

	// Check that periodUSecs is non-zero
	if (ChConfig.config && !ChConfig.periodUSecs) return -EFAULT;

	gLTC185x.ChData[ChConfig.ch].enabled 			= 	(ChConfig.config && LTC185x_ChSetup_Enabled);
	gLTC185x.ChData[ChConfig.ch].control 			= 	0;
	gLTC185x.ChData[ChConfig.ch].control 			|=	((ChConfig.config >> 16) & 0xFF);
	
	if (gLTC185x.ChData[ChConfig.ch].enabled)
	{
		switch(ChConfig.ch)
		{
			case Chn0:
				gLTC185x.ChData[ChConfig.ch].control 	|=	ADC_SINGLE_ENDED_INPUT0;
				break;
	
			case Chn1:
				gLTC185x.ChData[ChConfig.ch].control 	|=	ADC_SINGLE_ENDED_INPUT0;
				break;
	
			case Chn2:
				gLTC185x.ChData[ChConfig.ch].control 	|=	ADC_SINGLE_ENDED_INPUT2;
				break;
	
			case Chn3:
				gLTC185x.ChData[ChConfig.ch].control 	|=	ADC_SINGLE_ENDED_INPUT3;
				break;
	
			case Chn4:
				gLTC185x.ChData[ChConfig.ch].control 	|=	ADC_SINGLE_ENDED_INPUT4;
				break;
	
			case Chn5:
				gLTC185x.ChData[ChConfig.ch].control 	|=	ADC_SINGLE_ENDED_INPUT5;
				break;
	
			case Chn6:
				gLTC185x.ChData[ChConfig.ch].control 	|=	ADC_SINGLE_ENDED_INPUT6;
				break;
	
			case Chn7:
				gLTC185x.ChData[ChConfig.ch].control 	|=	ADC_SINGLE_ENDED_INPUT7;
				break;
	
			case Chn01:
				gLTC185x.ChData[ChConfig.ch].control 	|=	ADC_DIFFERENTIAL_EVEN_INPUT01;
				break;
	
			case Chn23:
				gLTC185x.ChData[ChConfig.ch].control 	|=	ADC_DIFFERENTIAL_EVEN_INPUT23;
				break;
	
			case Chn45:
				gLTC185x.ChData[ChConfig.ch].control 	|=	ADC_DIFFERENTIAL_EVEN_INPUT45;
				break;
	
			case Chn67:
				gLTC185x.ChData[ChConfig.ch].control 	|=	ADC_DIFFERENTIAL_EVEN_INPUT67;
				break;
			
		}
	}
		
	offset = (ChConfig.ch * ChSampleSize);
	gLTC185x.ChData[ChConfig.ch].bufferStart  = (unsigned short*) gLTC185x.Buf + offset;
	gLTC185x.ChData[ChConfig.ch].wrP 					= gLTC185x.ChData[ChConfig.ch].bufferStart;
	gLTC185x.ChData[ChConfig.ch].rdP 					= gLTC185x.ChData[ChConfig.ch].wrP;
	gLTC185x.ChData[ChConfig.ch].bufferSize		= ChSampleSize;
	gLTC185x.ChData[ChConfig.ch].bufferEnd		= gLTC185x.ChData[ChConfig.ch].bufferStart + gLTC185x.ChData[ChConfig.ch].bufferSize;

	// Set min and max sampling period
	if (ChConfig.periodUSecs > TimerCount1HrInUs) 
		ChConfig.periodUSecs = TimerCount1HrInUs;

	if (ChConfig.periodUSecs != 0 && ChConfig.periodUSecs < TimerIntUsecs)
		ChConfig.periodUSecs = TimerIntUsecs;


	// Note: Triggering one interrupt earlier to account for zero-to-trigger latency
	gLTC185x.ChData[ChConfig.ch].trigUSecs	= 	(ChConfig.periodUSecs / TimerIntUsecs);	
	gLTC185x.ChData[ChConfig.ch].countUSecs		= 	gLTC185x.ChData[ChConfig.ch].trigUSecs;

	gLTC185x.ChData[ChConfig.ch].trigHours		= 	ChConfig.periodHours;
	gLTC185x.ChData[ChConfig.ch].countHours		= 	gLTC185x.ChData[ChConfig.ch].trigHours;
	
	
	printk("LTC185x: Ch%d\n config=0x%x\n control=0x%x\n buf={0x%lx 0x%lx}\n %d samples\n periodUSecs=%luus countUSecs=%lu countHours=%lu\n",
		ChConfig.ch,
		ChConfig.config, 
		gLTC185x.ChData[ChConfig.ch].control,
		(unsigned long) gLTC185x.ChData[ChConfig.ch].bufferStart,
		(unsigned long) gLTC185x.ChData[ChConfig.ch].bufferEnd,
		gLTC185x.ChData[ChConfig.ch].bufferSize,
		ChConfig.periodUSecs,
		gLTC185x.ChData[ChConfig.ch].countUSecs,
		gLTC185x.ChData[ChConfig.ch].countHours
		);		
	return 0;	
}

static int PrvReadData(unsigned int arg)
{
	unsigned int 		numSamplesToRead;				
	unsigned int 		remainSamples;
	unsigned long		numBytes;
	unsigned long		retVal;
	ReadBufferData 	RdBufDat;

	// Get data from user space
	if (copy_from_user(&RdBufDat, (void __user*)arg, sizeof(ReadBufferData))) return -EFAULT;							

	printk("PrvReadData Ch%d rdP=0x%lx\n", 
		RdBufDat.ch,
		(unsigned long) gLTC185x.ChData[RdBufDat.ch].rdP
		);

	// Check for an overun, set the user space variable to indicate this
	if (gLTC185x.ChData[RdBufDat.ch].numSamples > ChSampleSize)
	{
		printk("Overun detected!!\n");

		if (RdBufDat.overun)
			if (put_user(1, RdBufDat.overun)) return -EFAULT;
	}

	// If read and write pointers are equal no new data has come, return 0
	if (gLTC185x.ChData[RdBufDat.ch].rdP == gLTC185x.ChData[RdBufDat.ch].wrP) 
	{
		printk("No data\n");
		return 0;
	}

	// Check if write pointer has wrapped around, read pointer will then be ahead
	if (gLTC185x.ChData[RdBufDat.ch].rdP > gLTC185x.ChData[RdBufDat.ch].wrP) 
	{						
		// Write pointer has wrapped around
		numSamplesToRead = gLTC185x.ChData[RdBufDat.ch].bufferEnd - gLTC185x.ChData[RdBufDat.ch].rdP;												
		remainSamples = gLTC185x.ChData[RdBufDat.ch].wrP - gLTC185x.ChData[RdBufDat.ch].bufferStart;	

		printk("rdP>wrP numSamplesToRead=%d remainSamples=%d userBuf=0x%lx samplesRequested=%d\n", 
			numSamplesToRead, 
			remainSamples, 
			(unsigned long) RdBufDat.buf,
			RdBufDat.numSamples
			);

		// If there is no user buffer, return the number of samples available
		if (!RdBufDat.buf) 
			return (numSamplesToRead + remainSamples);

		// Check how many bytes to transfer
		if ((numSamplesToRead + remainSamples) < RdBufDat.numSamples)
		{
			printk("... ALL\n");

			// Transfer all the samples
			// Copy to user buf
			numBytes = numSamplesToRead * sampleSize;
			if (copy_to_user(RdBufDat.buf ,gLTC185x.ChData[RdBufDat.ch].rdP, numBytes)) 
				return -EFAULT;							

			// Decrement the sample count
			gLTC185x.ChData[RdBufDat.ch].numSamples -= numSamplesToRead;
									
			// Copy remainder to user buf
			numBytes = remainSamples * sampleSize;
			if (copy_to_user(&RdBufDat.buf[numSamplesToRead],gLTC185x.ChData[RdBufDat.ch].bufferStart, numBytes)) 
				return -EFAULT;							

			// Decrement the sample count
			gLTC185x.ChData[RdBufDat.ch].numSamples -= remainSamples;
										
			// Update read pointer
			gLTC185x.ChData[RdBufDat.ch].rdP = &gLTC185x.ChData[RdBufDat.ch].bufferStart[remainSamples];

			printk("rdP=0x%lx %d bytes transferred\n", 
				(unsigned long) gLTC185x.ChData[RdBufDat.ch].rdP,
				(numSamplesToRead + remainSamples) * sampleSize
				);		

		}
		else
		{					
			printk("... PART ");
	
			// Copy a portion of the sample data
			if (numSamplesToRead < RdBufDat.numSamples)
			{
				printk("TOP\n");

				// Copy to user buf
				numBytes = numSamplesToRead * sampleSize;
				if (copy_to_user(RdBufDat.buf, gLTC185x.ChData[RdBufDat.ch].rdP, numBytes)) 
					return -EFAULT;							

				// Decrement the sample count
				gLTC185x.ChData[RdBufDat.ch].numSamples -= numSamplesToRead;
				
				// Calculate the remaining samples to transfer
				remainSamples = RdBufDat.numSamples - numSamplesToRead;

				printk("BOTTOM\n");

				// Copy remainder to user buf
				numBytes = remainSamples * sampleSize;
				if (copy_to_user(&RdBufDat.buf[numSamplesToRead], gLTC185x.ChData[RdBufDat.ch].bufferStart, numBytes)) 
					return -EFAULT;							

				// Decrement the sample count
				gLTC185x.ChData[RdBufDat.ch].numSamples -= remainSamples;

				// Update read pointer
				gLTC185x.ChData[RdBufDat.ch].rdP = &gLTC185x.ChData[RdBufDat.ch].bufferStart[remainSamples];

				printk("rdP=0x%lx %d bytes transferred\n", 
					(unsigned long) gLTC185x.ChData[RdBufDat.ch].rdP,
					(numSamplesToRead + remainSamples) * sampleSize
					);		
			}
			else
			{
				printk("... TOP ONLY\n");

				// Only read the number of samples requested
				numSamplesToRead = RdBufDat.numSamples;

				// Copy to user buf
				numBytes = numSamplesToRead * sampleSize;
				if (copy_to_user(&RdBufDat.buf[0], gLTC185x.ChData[RdBufDat.ch].rdP, numBytes)) 
					return -EFAULT;							

				// Decrement the sample count
				gLTC185x.ChData[RdBufDat.ch].numSamples -= numSamplesToRead;

				// Update read pointer
				gLTC185x.ChData[RdBufDat.ch].rdP += numSamplesToRead * sampleSize;

				printk("rdP=0x%lx %d bytes transferred\n", 
					(unsigned long) gLTC185x.ChData[RdBufDat.ch].rdP,
					(numSamplesToRead + remainSamples) * sampleSize
					);		
			}
		}
	} 
	else // gLTC185x.ChData[ch].rdP < gLTC185x.ChData[ch].wrP)
	{
		numSamplesToRead = gLTC185x.ChData[RdBufDat.ch].numSamples;

		printk("rdP<wrP numSamplesToRead=%d userBuf=0x%lx\n", 
			numSamplesToRead, 
			(unsigned long) RdBufDat.buf);

		// If there is no user buffer, return the number of samples available
		if (!RdBufDat.buf) 
		{
			printk("No buffer!\n");
			return numSamplesToRead;
		}

		// Check to see how many samples to return
		if (numSamplesToRead > RdBufDat.numSamples)
			numSamplesToRead = RdBufDat.numSamples;

		numBytes = numSamplesToRead * sampleSize;
		retVal = copy_to_user(RdBufDat.buf, gLTC185x.ChData[RdBufDat.ch].rdP, numBytes);
		if (retVal) 
		{
			printk("Copy to user failed 0x%lx->0x%lx %ld bytes, retVal=%ld\n", 
				(unsigned long) gLTC185x.ChData[RdBufDat.ch].rdP, 
				(unsigned long) RdBufDat.buf, 
				numBytes,
				retVal
				);
							
			return -EFAULT;							
		}
				
		// Decrement the sample count
		gLTC185x.ChData[RdBufDat.ch].numSamples -= (numSamplesToRead);

		// Update read pointer
		gLTC185x.ChData[RdBufDat.ch].rdP += numSamplesToRead;

		printk("   rdP=0x%lx %ld bytes transferred\n", 
			(unsigned long) gLTC185x.ChData[RdBufDat.ch].rdP, 
			numBytes
		);		
		
		return numSamplesToRead;
	}
	
	return 0;	
}



// ------------------------------------------------------------------
// Driver routines
// ------------------------------------------------------------------
// Read handler
//void do_read_tasklet(unsigned long unused)
//{
//	printk("reading::\n");
//
//	// Only doing Ch0 for now
//	if (gLTC185x.rdCh == Ch0)
//	{
//		unsigned int numSamples;
//
//		printk("Ch%d %x to 0x%lx\n", gLTC185x.rdCh, gLTC185x.sampleValue, (unsigned long) gLTC185x.ChData[Ch0].wrP);
//		
//		// Write sample to user space
//		put_user(gLTC185x.sampleValue,	gLTC185x.ChData[Ch0].wrP);
//		
//		// Wrap pointer around if necessary 
//		gLTC185x.ChData[Ch0].wrP++;
//		if (gLTC185x.ChData[Ch0].wrP > gLTC185x.ChData[Ch0].bufferEnd) 
//		{
//				gLTC185x.ChData[Ch0].wrP = gLTC185x.ChData[Ch0].bufferStart;
//				printk("     wrap\n");
//		}
//			
//		// Increment number of samples (note: this might be changed in user space, hence read-modify-write
//		// TODO: potential race condition here!!
//		if (get_user(numSamples, gLTC185x.ChData[Ch0].numSamplesP) == 0);
//		{
//			printk("numSamples=%d 0x%lx\n", numSamples, (unsigned long)gLTC185x.ChData[Ch0].numSamplesP);
//			numSamples++;
//			if (put_user(numSamples, gLTC185x.ChData[Ch0].numSamplesP))
//			{
//				printk("error writing numSamples\n");					
//			}
//
//			if (copy_to_user(&numSamples, gLTC185x.ChData[Ch0].numSamplesP, sizeof(numSamples)))
//			{
//				printk("error copy_to_user\n");											
//			}
//		}
//	}			
//}
//DECLARE_TASKLET(read_tasklet, do_read_tasklet, 0);

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
	unsigned char	sampleH, sampleL;
	unsigned int	sampleValue = 0;
	int						Ch;
	int						loopCount;
	unsigned char	triggered;
	unsigned char	takeReading;

	// Debug toggle
	gGPJDAT = readl(S3C2440_GPJDAT);
	gGPJDAT &= ~(bDAT_CLIM_EN_N_CAM_DATA3);
	writel(gGPJDAT,S3C2440_GPJDAT);

	
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

	// Check and update ISR counter
	loopCount = ChnMax;
	Ch = gLTC185x.wrCh;	
	takeReading = 0;
	triggered = 0;
	do  
	{			
		// Increment and wrap around the channel counter
		Ch++;
		if (Ch>ChnMax) Ch=0;
			
		if (gLTC185x.ChData[Ch].enabled) 
		{
			if (gLTC185x.ChData[Ch].countUSecs) 
			{
				gLTC185x.ChData[Ch].countUSecs--;
				
				// Check to see if we have hours to decrement, if we don't then we know that the counter is now up
				// and will sample on the next timer interrupt
				if (gLTC185x.ChData[Ch].countUSecs == 0)
				{
					// If we have no countUSecs remaining, check countHours
					if (gLTC185x.ChData[Ch].countHours--)
					{						
						// If we still have countHours, we reload the usecs count register with another hour
						gLTC185x.ChData[Ch].countUSecs = TimerCount1HrInUs/TimerIntUsecs;							
						printk("INT:countUSecs=%lu  countHours=%lu\n", gLTC185x.ChData[Ch].countUSecs, gLTC185x.ChData[Ch].countHours);
					}
				}
			} 
			else 
			{	
				if (!triggered)
				{						
					// Update the write channel and read channels
					gLTC185x.rdCh = gLTC185x.wrCh;
					gLTC185x.wrCh = Ch;
		
					// Reset the Channel countUSecs and countHours
					// Note: Trigger happens after it has reached zero, not exactly on zero
					if (gLTC185x.ChData[Ch].trigUSecs)
					{
						gLTC185x.ChData[Ch].countUSecs = gLTC185x.ChData[Ch].trigUSecs; 
						gLTC185x.ChData[Ch].countHours = gLTC185x.ChData[Ch].trigHours;
//						printk("INT1:countUSecs=%lu  countHours=%lu!!!\n", gLTC185x.ChData[Ch].countUSecs, gLTC185x.ChData[Ch].countHours);
					} 
					else
					{
						if (gLTC185x.ChData[Ch].trigHours)
						{ 	
							// If trigHours is non-zero, we reset the countHours to trigHours-1, and set countUSecs to 1 hour
							gLTC185x.ChData[Ch].countHours = gLTC185x.ChData[Ch].trigHours-1;
							gLTC185x.ChData[Ch].countUSecs = TimerCount1HrInUs/TimerIntUsecs; 
							printk("INT2:countUSecs=%lu  countHours=%lu!!!\n", gLTC185x.ChData[Ch].countUSecs, gLTC185x.ChData[Ch].countHours);
						}
						else
						{
							// If trigHours is zero, we ensure that countHours is also zero
							gLTC185x.ChData[Ch].countHours = 0; 
							gLTC185x.ChData[Ch].countUSecs = 0; 
							printk("INT3:countUSecs=%lu  countHours=%lu!!!\n", gLTC185x.ChData[Ch].countUSecs, gLTC185x.ChData[Ch].countHours);
						}
					}
							
					// Only 1 channel triggers at a time, if there is more than one channel that needs to sample
					// the next channel to sample occurs at the next timer interrupt, and hence offsets future
					// trigger event for that channel against other simultaneous triggers (unless there are odd
					// sampling period values).
					triggered = 1;
					gLTC185x.readCnt++;
				}
			}
		}
	} while (loopCount--);
	
//		for (Ch=0; Ch<ChMax; Ch++)
//		{
//			if (gLTC185x.ChData[Ch].enabled) 
//				printk("Ch%d=%ld ", Ch, gLTC185x.ChData[Ch].count);
//		}
//		printk("\n");

	// Exit if we've got nothing more to do.	
	if (triggered)
	{			
		// ---------------------------------------------
		// Write new data to ADC for conversion
		// ---------------------------------------------

//		printk("Ch%d write %x\n", gLTC185x.wrCh, gLTC185x.ChData[gLTC185x.wrCh].control);								
			
		// Assert RD line low, set CONVST to low as well
		gGPJDAT = readl(S3C2440_GPJDAT);
		gGPJDAT &= ~(bDAT_ADC_CNV_START_CAM_DATA6|bDAT_ADC_RD_EN_N_CAM_DATA5);
		writel(gGPJDAT,S3C2440_GPJDAT);
			 	
		// Send the SPI control code 	
		// Send control byte
		SPTDAT = gLTC185x.ChData[gLTC185x.wrCh].control;
//		printk("SPTDAT 0x%lx\n", (unsigned long)&SPTDAT);

		// Wait until full 8 bits are sent
		while (!(SPSTA & S3C2410_SPSTA_READY)) continue;
		sampleH = SPRDAT;
	
//		printk("SPRDAT 0x%lx = 0x%xH\n", (unsigned long)&SPRDAT, sampleH);

				
		SPTDAT = 0xFF;
		while (!(SPSTA & S3C2410_SPSTA_READY)) continue;
		sampleL = SPRDAT;

//		printk("SPRDAT 0x%lx = 0x%xL\n", (unsigned long)&SPRDAT, sampleL);

	
		// CONVST high, trigger conversion, also disable read enable
		gGPJDAT |= (bDAT_ADC_CNV_START_CAM_DATA6|bDAT_ADC_RD_EN_N_CAM_DATA5);												
		writel(gGPJDAT,S3C2440_GPJDAT);	
	
		// ---------------------------------------------
		// Process the read data from ADC
		// ---------------------------------------------		
		if (gLTC185x.skipRead == 0)
		{
			if (gLTC185x.readCnt > 0)
			{								
				// Decrement the read count
				gLTC185x.readCnt--;
	
				takeReading = 1;
			}
		} else
		{
			gLTC185x.skipRead = 0;
		}
	}
	// Special case, need to do one extra read even if not triggered to grab the last read out of the ADC
	else 
	{
		if (gLTC185x.readCnt > 0) 
		{
			// ---------------------------------------------
			// Process the read data from ADC
			// ---------------------------------------------		
		
			// Assert RD line low, set CONVST to low as well
			gGPJDAT = readl(S3C2440_GPJDAT);
			gGPJDAT &= ~(bDAT_ADC_RD_EN_N_CAM_DATA5);
			writel(gGPJDAT,S3C2440_GPJDAT);
				 	
			// Send the SPI control code 	
			// Send control byte
			SPTDAT = gLTC185x.ChData[gLTC185x.wrCh].control;
			// Wait until full 8 bits are sent
			while (!(SPSTA & S3C2410_SPSTA_READY)) continue;
			sampleH = SPRDAT;

			SPTDAT = 0xFF;
			while (!(SPSTA & S3C2410_SPSTA_READY)) continue;
			sampleL = SPRDAT;
		
			// Disable read enable
			gGPJDAT |= (bDAT_ADC_RD_EN_N_CAM_DATA5);												
			writel(gGPJDAT,S3C2440_GPJDAT);
				
			// Reset the read count
			gLTC185x.readCnt 	= ReadCntStart;
			
			// Ensure that the next read is skipped
			gLTC185x.skipRead = 1;
		
			// Note: If we have reached here, the final read is for the last channel written
			gLTC185x.rdCh = gLTC185x.wrCh;
			
			takeReading = 1;			
		}
	}
	
	if (takeReading)
	{
		sampleValue = (sampleH << 8) | sampleL;

		// Write the sample value to a temporary buffer that will be handled by the tasklet	
		gLTC185x.sampleValue = sampleValue;

//		tasklet_schedule(&read_tasklet);


		*gLTC185x.ChData[gLTC185x.rdCh].wrP++ = sampleValue;
		gLTC185x.ChData[gLTC185x.rdCh].numSamples++;

		printk("sH=0x%x\n", sampleH);
		printk("sL=0x%x\n", sampleL);
		printk("s=0x%x\n", sampleValue);

		// Record the sample in the appropriate buffer
//		printk("Ch%d wrP(0x%lx)=0x%x, %d samples\n", 
//			gLTC185x.rdCh,
//			(unsigned long) gLTC185x.ChData[gLTC185x.rdCh].wrP, 
//			*(gLTC185x.ChData[gLTC185x.rdCh].wrP-sampleSize), 
//			gLTC185x.ChData[gLTC185x.rdCh].numSamples
//			);

		// Wrap around if necessary
		if (gLTC185x.ChData[gLTC185x.rdCh].wrP >= gLTC185x.ChData[gLTC185x.rdCh].bufferEnd)
		{
//			printk("wrP wrapped\n"); 			
			gLTC185x.ChData[gLTC185x.rdCh].wrP = gLTC185x.ChData[gLTC185x.rdCh].bufferStart;
		}
	}
	
exit:
	gLTC185x.InIRQ = 0;

	// Debug toggle
	gGPJDAT |= bDAT_CLIM_EN_N_CAM_DATA3;
	writel(gGPJDAT,S3C2440_GPJDAT);

  return IRQ_HANDLED;
}






static int sbc2440_mzio_LTC185x_ioctl(
	struct inode *inode,
	struct file *file,
	unsigned int cmd,
	unsigned long arg)
{
	
	// Are we turned on already?
	if (gLTC185x.IsOn)
	{
		// If sampling is turned on, only allow access to STOP and READ

		if (cmd == MZIO_LTC185x_STOP)
		{
				printk("LTC185x: Stop++\n");
				
				// Wait for IRQ activity to stop first
				while (gLTC185x.InIRQ) continue;
				
				// Disable the timer
				PrvStopTimer();
	
				// Mark the device is now off
				gLTC185x.IsOn 		= 0;
		    
				printk("LTC185x: Stop--\n");
				return 0;
		}
		
		if (!MZIO_LTC185x_READ_BUFFER) 
			return -EFAULT;
	}

	switch(cmd)
	{			
		// -----------------------------------------------------
		// Init routines
		// -----------------------------------------------------
		case MZIO_LTC185x_INIT:
			PrvLTC185xInit();
			return 0;

		case MZIO_LTC185x_DEINIT:
			PrvLTC185xDeinit();
			return 0;

		// -----------------------------------------------------
		// Start sampling
		// -----------------------------------------------------
		case MZIO_LTC185x_START:
			printk("LTC185x: Start++\n");
	
			// Allocate start values
			gLTC185x.wrCh			=	ChnMax;
			gLTC185x.readCnt	=	ReadCntStart;
			gLTC185x.skipRead	=	1;
			gLTC185x.IsOn 		= 1;

			// Zero the sample buffer
			memset(gLTC185x.Buf, 0, sizeof(BufData));

			// Reset all channel buffers
			{
				int i;
				for (i=Chn0; i<=ChnMax; i++)
				{
					gLTC185x.ChData[i].wrP 					= gLTC185x.ChData[i].bufferStart;
					gLTC185x.ChData[i].rdP 					= gLTC185x.ChData[i].wrP;
					gLTC185x.ChData[i].numSamples 	= 0;
				}
			}
			PrvStartTimer();
			
			printk("LTC185x: Start--\n");
			return 0;	


		// -----------------------------------------------------
		// ADC read buffer
		// -----------------------------------------------------
		case MZIO_LTC185x_READ_BUFFER:
			return PrvReadData(arg);

		// -----------------------------------------------------
		// ADC Channel setup routine
		// -----------------------------------------------------
		case MZIO_LTC185x_CHANNEL_SETUP:	
			return PrvSetChannelConfig(arg);

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

//---------------------------------------------------------------------------------------------
//	Init routines
//---------------------------------------------------------------------------------------------
static int __init dev_init(void)
{
	int ret;

	printk(DEVICE_NAME"\tversion %d%02d%02d%02d%02d\tinitialized\n", GetCompileYear(),
	GetCompileMonth(), GetCompileDay(), GetCompileHour(), GetCompileMinute());
	
	// Zero the globals
	memset(&gLTC185x, 0, sizeof(LTC185x_DEV));

	// Allocate memory
	gLTC185x.Buf = kmalloc(sizeof(BufData), GFP_KERNEL);
	if (!gLTC185x.Buf) 
	{
		printk("Error allocating memory\n");
		return -ENOMEM;
	}
	printk("Allocate sample buffer %d bytes at 0x%lx\n", 
		sizeof(BufData), 
		(unsigned long) gLTC185x.Buf
		);


	base_addr_SPI = ioremap(S3C2410_PA_SPI,0x20);
	if (base_addr_SPI == NULL) {
		printk(KERN_ERR "Failed to remap register block\n");
		return -ENOMEM;
	}

	base_addr_CLK = ioremap(S3C2410_PA_CLKPWR,0x20);
	if (base_addr_CLK == NULL) {
		printk(KERN_ERR "Failed to remap register block\n");
		return -ENOMEM;
	}

	base_addr_GPIO = ioremap(S3C2410_PA_GPIO,0x20);
	if (base_addr_GPIO == NULL) {
		printk(KERN_ERR "Failed to remap register block\n");
		return -ENOMEM;
	}

	ret = misc_register(&misc);

	return ret;
}

static void __exit dev_exit(void)
{
	printk(DEVICE_NAME"\tgoodbye!\n");

	if(s3c2410_timer_irq.dev_id)
	{
		remove_irq(IRQ_TIMER2, &s3c2410_timer_irq);
		s3c2410_timer_irq.dev_id = 0;
	}

	// Free the allocated memory
	if (gLTC185x.Buf) kfree(gLTC185x.Buf);
	
	misc_deregister(&misc);
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aceeca Inc.");
