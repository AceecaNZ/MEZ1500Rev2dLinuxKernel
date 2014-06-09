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
unsigned long gIntCount1000ms=0;
unsigned long gGPJDAT;
unsigned int 	gTimerReloadValue;



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

  temp32 = pclk / (((TimerCfg0 & 0x0000FF00) >> 8) + 1) / 2;   // pclk / (prescalar + 1) / divider value (=2)
  // Note: Prescalar is set by system to be 2
  // Divider for mux2 is set at 1/2, divider value is 2
  
	printk("Freq=%ld\n", temp32);
  TimerCNTB = temp32/TimerFreq;
  TimerCMPB = temp32/TimerFreq;

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


static int PrvReadData(int ch, ReadBufferData* RdBufDat)
{
	unsigned int 		numSamplesToRead;				
	unsigned int 		remainSamples;
	unsigned long		numBytes;
	unsigned long		retVal;

	printk("PrvReadData\n");

	// Check for an overun, set the user space variable to indicate this
	if (gLTC185x.ChData[ch].numSamples > ChSampleSize)
	{
		printk("Overun detected!!\n");

		if (RdBufDat->overun)
			if (put_user(1, RdBufDat->overun)) return -EFAULT;
	}

	// If read and write pointers are equal no new data has come, return 0
	if (gLTC185x.ChData[ch].rdP == gLTC185x.ChData[ch].wrP) 
	{
		printk("No data\n");
		return 0;
	}

	// Check if write pointer has wrapped around, read pointer will then be ahead
	if (gLTC185x.ChData[ch].rdP > gLTC185x.ChData[ch].wrP) 
	{						
		// Write pointer has wrapped around
		numSamplesToRead = gLTC185x.ChData[ch].bufferEnd - gLTC185x.ChData[ch].rdP;												
		remainSamples = gLTC185x.ChData[ch].wrP - gLTC185x.ChData[ch].bufferStart;	

		printk("rdP>wrP numSamplesToRead=%d remainSamples=%d userBuf=0x%lx samplesRequested=%d\n", 
			numSamplesToRead, 
			remainSamples, 
			(unsigned long) RdBufDat->buf,
			RdBufDat->numSamples
			);

		// If there is no user buffer, return the number of samples available
		if (!RdBufDat->buf) 
			return (numSamplesToRead + remainSamples);

		// Check how many bytes to transfer
		if ((numSamplesToRead + remainSamples) < RdBufDat->numSamples)
		{
			printk("... ALL\n");

			// Transfer all the samples
			// Copy to user buf
			numBytes = numSamplesToRead * sampleSize;
			if (copy_to_user(gLTC185x.ChData[ch].rdP, &RdBufDat->buf[0], numBytes)) 
				return -EFAULT;							

			// Decrement the sample count
			gLTC185x.ChData[ch].numSamples -= numSamplesToRead;
									
			// Copy remainder to user buf
			numBytes = remainSamples * sampleSize;
			if (copy_to_user(gLTC185x.ChData[ch].bufferStart, &RdBufDat->buf[numSamplesToRead], numBytes)) 
				return -EFAULT;							

			// Decrement the sample count
			gLTC185x.ChData[ch].numSamples -= remainSamples;
										
			// Update read pointer
			gLTC185x.ChData[ch].rdP = gLTC185x.ChData[ch].bufferStart + numBytes;

			printk("rdP=0x%lx\n", (unsigned long) gLTC185x.ChData[ch].rdP);

		}
		else
		{					
			printk("... PART\n");
	
			// Copy a portion of the sample data
			if (numSamplesToRead < RdBufDat->numSamples)
			{
				printk("... TOP\n");

				// Copy to user buf
				numBytes = numSamplesToRead * sampleSize;
				if (copy_to_user(gLTC185x.ChData[ch].rdP, &RdBufDat->buf[0], numBytes)) 
					return -EFAULT;							

				// Decrement the sample count
				gLTC185x.ChData[ch].numSamples -= numSamplesToRead;
				
				// Calculate the remaining samples to transfer
				remainSamples = RdBufDat->numSamples - numSamplesToRead;

				printk("... BOTTOM\n");

				// Copy remainder to user buf
				numBytes = remainSamples * sampleSize;
				if (copy_to_user(gLTC185x.ChData[ch].bufferStart, &RdBufDat->buf[numSamplesToRead], numBytes)) 
					return -EFAULT;							

				// Decrement the sample count
				gLTC185x.ChData[ch].numSamples -= remainSamples;

				// Update read pointer
				gLTC185x.ChData[ch].rdP = &gLTC185x.ChData[ch].bufferStart[remainSamples];

				printk("rdP=0x%lx\n", (unsigned long) gLTC185x.ChData[ch].rdP);		
			}
			else
			{
				printk("... TOP ONLY\n");

				// Only read the number of samples requested
				numSamplesToRead = RdBufDat->numSamples;

				// Copy to user buf
				numBytes = numSamplesToRead * sampleSize;
				if (copy_to_user(gLTC185x.ChData[ch].rdP, &RdBufDat->buf[0], numBytes)) 
					return -EFAULT;							

				// Decrement the sample count
				gLTC185x.ChData[ch].numSamples -= numSamplesToRead;

				// Update read pointer
				gLTC185x.ChData[ch].rdP += numSamplesToRead * sampleSize;

				printk("rdP=0x%lx\n", (unsigned long) gLTC185x.ChData[ch].rdP);		
			}
		}
	} 
	else // gLTC185x.ChData[ch].rdP < gLTC185x.ChData[ch].wrP)
	{
		numSamplesToRead = gLTC185x.ChData[ch].wrP - gLTC185x.ChData[ch].rdP;

		printk("rdP<wrP numSamplesToRead=%d userBuf=0x%lx\n", 
			numSamplesToRead, 
			(unsigned long) RdBufDat->buf);
		
		// If there is no user buffer, return the number of samples available
		if (!RdBufDat->buf) 
		{
			printk("No buffer!\n");
			return numSamplesToRead;
		}

		// Check to see how many samples to return
		if (numSamplesToRead > RdBufDat->numSamples)
			numSamplesToRead = RdBufDat->numSamples;

		numBytes = numSamplesToRead * sampleSize;
		if (access_ok(VERIFY_WRITE, RdBufDat->buf, numBytes))
		{
			unsigned short tempBuf[5] = {1,2,3,4,5};
			retVal = copy_to_user(tempBuf, RdBufDat->buf, sizeof(tempBuf));
//			retVal = copy_to_user(gLTC185x.ChData[ch].rdP, RdBufDat->buf, numBytes);
			if (retVal) 
			{
				printk("Copy to user failed 0x%lx->0x%lx %ld bytes, retVal=%ld\n", 
					(unsigned long) gLTC185x.ChData[ch].rdP, 
					(unsigned long) RdBufDat->buf, 
					numBytes,
					retVal
					);
								
				return -EFAULT;							
			}
		}
		else
		{
			printk("Can't write to user buf 0x%lx\n", (unsigned long) RdBufDat->buf); 			
		}
				
		// Decrement the sample count
		gLTC185x.ChData[ch].numSamples -= (numSamplesToRead);

		// Update read pointer
		gLTC185x.ChData[ch].rdP += numBytes;

		printk("   rdP=0x%lx\n", (unsigned long) gLTC185x.ChData[ch].rdP);		
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
//			if (copy_to_user(gLTC185x.ChData[Ch0].numSamplesP, &numSamples, sizeof(numSamples)))
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
	loopCount = ChMax;
	Ch = gLTC185x.wrCh;	
	takeReading = 0;
	triggered = 0;
	do  
	{			
		// Increment and wrap around the channel counter
		Ch++;
		if (Ch>ChMax) Ch=0;
			
		if (gLTC185x.ChData[Ch].enabled) {
			if (gLTC185x.ChData[Ch].trig) {
				if (gLTC185x.ChData[Ch].count) 
				{
					gLTC185x.ChData[Ch].count--;
				} else {	
					if (!triggered)
					{						
						// Update the write channel and read channels
						gLTC185x.rdCh = gLTC185x.wrCh;
						gLTC185x.wrCh = Ch;
			
						// Reset the Channel counter
						// Note: Trigger happens after it has reached zero, not exactly on zero
						gLTC185x.ChData[Ch].count = gLTC185x.ChData[Ch].trig - 1; 
			
						triggered = 1;
						gLTC185x.readCnt++;
					}
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

//			printk("Ch%d write\n", gLTC185x.wrCh);								
			
		// Assert RD line low, set CONVST to low as well
		gGPJDAT = readl(S3C2440_GPJDAT);
		gGPJDAT &= ~(bDAT_ADC_CNV_START_CAM_DATA6|bDAT_ADC_RD_EN_N_CAM_DATA5);
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
				// Assemble the sample value if we need to do a read
				sampleValue = (sampleL << 8) | sampleH;
				
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
//			sampleH = PrvSPISendReceiveData(0xFF); 		
			// Sending dummy
//			sampleL = PrvSPISendReceiveData(0xFF);	  																
			SPTDAT = gLTC185x.ChData[gLTC185x.wrCh].control;
			// Wait until full 8 bits are sent
			while (!(SPSTA & S3C2410_SPSTA_READY)) continue;
			sampleH = SPRDAT;
	
			SPTDAT = 0xFF;
			while (!(SPSTA & S3C2410_SPSTA_READY)) continue;
			sampleL = SPRDAT;

		
			// CONVST high, trigger conversion, also disable read enable
			gGPJDAT |= (bDAT_ADC_RD_EN_N_CAM_DATA5);												
			writel(gGPJDAT,S3C2440_GPJDAT);
				
			sampleValue = (sampleL << 8) | sampleH;
	
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
		// Write the sample value to a temporary buffer that will be handled by the tasklet	
		gLTC185x.sampleValue = sampleValue;

//		tasklet_schedule(&read_tasklet);

		// Record the sample in the appropriate buffer
		printk("takeReading wrP 0x%lx %x numsam=%d\n", 
			(unsigned long) gLTC185x.ChData[gLTC185x.rdCh].wrP, 
			sampleValue, 
			gLTC185x.ChData[gLTC185x.rdCh].numSamples
			);

		*gLTC185x.ChData[gLTC185x.rdCh].wrP++ = sampleValue;
		gLTC185x.ChData[gLTC185x.rdCh].numSamples++;

		// Wrap around if necessary
		if (gLTC185x.ChData[gLTC185x.rdCh].wrP >= gLTC185x.ChData[gLTC185x.rdCh].bufferEnd)
			gLTC185x.ChData[gLTC185x.rdCh].wrP = gLTC185x.ChData[gLTC185x.rdCh].bufferStart;
	}
	
exit:
  if (gIntCount1000ms-- == 0)
  {
//		printk("Beep\n");
//		gIntCount1000ms = Timer1000ms*5;
	}

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
		if (!MZIO_LTC185x_CH0SE_READ_BUFFER) 
			return -EFAULT;
	}

	switch(cmd)
	{			
		// -----------------------------------------------------
		// Init routines
		// -----------------------------------------------------
		case MZIO_LTC185x_INIT:
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
			SPPRE=1;
			
			printk("LTC185x: Init--\n");
			return 0;

		case MZIO_LTC185x_DEINIT:
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
			return 0;

		// -----------------------------------------------------
		// Start sampling
		// -----------------------------------------------------
		case MZIO_LTC185x_START:
			printk("LTC185x: Start++\n");
	
			// Allocate start values
			gLTC185x.wrCh			=	ChMax;
			gLTC185x.readCnt	=	ReadCntStart;
			gLTC185x.skipRead	=	1;
			gIntCount1000ms 	= Timer1000ms*5;
			gLTC185x.IsOn 		= 1;

			// Zero the sample buffer
			memset(gLTC185x.Buf, 0, sizeof(BufData));

//			PrvStartTimer();
			
			printk("LTC185x: Start--\n");
			return 0;	

		// -----------------------------------------------------
		// ADC Channel setup routines
		// -----------------------------------------------------
		case MZIO_LTC185x_CH0SE_SETUP:	
			gLTC185x.ChData[Ch0].enabled = 	(arg && LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch0].control = 	0;
			gLTC185x.ChData[Ch0].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT0;

			gLTC185x.ChData[Ch0].bufferStart  = gLTC185x.Buf->Ch0Buf;
			gLTC185x.ChData[Ch0].wrP 					= gLTC185x.ChData[Ch0].bufferStart;
			gLTC185x.ChData[Ch0].rdP 					= gLTC185x.ChData[Ch0].wrP;
			gLTC185x.ChData[Ch0].bufferSize		= ChSampleSize * sampleSize;
			gLTC185x.ChData[Ch0].bufferEnd		= gLTC185x.ChData[Ch0].bufferStart + gLTC185x.ChData[Ch0].bufferSize;
			
			printk("LTC185x: CH0SE_SETUP arg=0x%lx enabled=%d .control=0x%x buf={0x%lx 0x%lx} sz=%d bytes\n",
				arg, 
				gLTC185x.ChData[Ch0].enabled, 
				gLTC185x.ChData[Ch0].control,
				(unsigned long) gLTC185x.ChData[Ch0].bufferStart,
				(unsigned long) gLTC185x.ChData[Ch0].bufferEnd,
				gLTC185x.ChData[Ch0].bufferSize
				);
			return 0;

		case MZIO_LTC185x_CH0SE_SET_PERIOD:
			gLTC185x.ChData[Ch0].trig 	= 	arg / Timer1uSDivideRatio;
			gLTC185x.ChData[Ch0].count 	= 	gLTC185x.ChData[Ch0].trig;
			printk("LTC185x: CH0SE_SET_PERIOD .count=%ld .trig=%ld\n", 
				gLTC185x.ChData[Ch0].count,
				gLTC185x.ChData[Ch0].trig);
			return 0;

		case MZIO_LTC185x_CH0SE_READ_BUFFER:
			{
				// Get data from user space
				ReadBufferData 	RdBufDat;

				printk("CH0SE_READ_BUFFER\n");
				if (copy_from_user(&RdBufDat, (void __user*)arg, sizeof(ReadBufferData))) return -EFAULT;							
					
				return PrvReadData(Ch0, &RdBufDat);
			}
			return 0;


		// -----------------------------------------------------
		case MZIO_LTC185x_CH1SE_SETUP:	
			gLTC185x.ChData[Ch1].enabled = 	(arg && LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch1].control = 	0;
			gLTC185x.ChData[Ch1].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT1;
			return 0;

		case MZIO_LTC185x_CH1SE_SET_PERIOD:	
			gLTC185x.ChData[Ch1].trig 	= 	arg / Timer1uSDivideRatio;
			gLTC185x.ChData[Ch1].count 	= 	gLTC185x.ChData[Ch0].trig;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH2SE_SETUP:	
			gLTC185x.ChData[Ch2].enabled = 	(arg && LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch2].control = 	0;
			gLTC185x.ChData[Ch2].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT2;
			return 0;

		case MZIO_LTC185x_CH2SE_SET_PERIOD:	
			gLTC185x.ChData[Ch2].trig 	= 	arg / Timer1uSDivideRatio;
			gLTC185x.ChData[Ch2].count 	= 	gLTC185x.ChData[Ch0].trig;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH3SE_SETUP:	
			gLTC185x.ChData[Ch3].enabled = 	(arg && LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch3].control = 	arg;
			gLTC185x.ChData[Ch3].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT3;
			return 0;

		case MZIO_LTC185x_CH3SE_SET_PERIOD:	
			gLTC185x.ChData[Ch3].trig 	= 	arg / Timer1uSDivideRatio;
			gLTC185x.ChData[Ch3].count 	= 	gLTC185x.ChData[Ch0].trig;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH4SE_SETUP:	
			gLTC185x.ChData[Ch4].enabled = 	(arg && LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch4].control = 	arg;
			gLTC185x.ChData[Ch4].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT4;
			return 0;

		case MZIO_LTC185x_CH4SE_SET_PERIOD:	
			gLTC185x.ChData[Ch4].trig 	= 	arg / Timer1uSDivideRatio;
			gLTC185x.ChData[Ch4].count 	= 	gLTC185x.ChData[Ch0].trig;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH5SE_SETUP:	
			gLTC185x.ChData[Ch5].enabled = 	(arg && LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch5].control = 	arg;
			gLTC185x.ChData[Ch5].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT5;
			return 0;

		case MZIO_LTC185x_CH5SE_SET_PERIOD:
			gLTC185x.ChData[Ch5].trig 	= 	arg / Timer1uSDivideRatio;
			gLTC185x.ChData[Ch5].count 	= 	gLTC185x.ChData[Ch0].trig;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH6SE_SETUP:	
			gLTC185x.ChData[Ch6].enabled = 	(arg && LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch6].control = 	arg;
			gLTC185x.ChData[Ch6].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT6;
			return 0;

		case MZIO_LTC185x_CH6SE_SET_PERIOD:	
			gLTC185x.ChData[Ch6].trig 	= 	arg / Timer1uSDivideRatio;
			gLTC185x.ChData[Ch6].count 	= 	gLTC185x.ChData[Ch0].trig;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH7SE_SETUP:	
			gLTC185x.ChData[Ch7].enabled = 	(arg && LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch7].control = 	0;
			gLTC185x.ChData[Ch7].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT7;
			return 0;

		case MZIO_LTC185x_CH7SE_SET_PERIOD:	
			gLTC185x.ChData[Ch7].trig 	= 	arg / Timer1uSDivideRatio;
			gLTC185x.ChData[Ch7].count 	= 	gLTC185x.ChData[Ch0].trig;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH01DE_SETUP:	
			gLTC185x.ChData[Ch01].enabled = 	(arg && LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch01].control = 	arg;
			gLTC185x.ChData[Ch01].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT01;
			return 0;

		case MZIO_LTC185x_CH01SE_SET_PERIOD:	
			gLTC185x.ChData[Ch01].trig 	= 	arg / Timer1uSDivideRatio;
			gLTC185x.ChData[Ch01].count 	= 	gLTC185x.ChData[Ch0].trig;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH23DE_SETUP:	
			gLTC185x.ChData[Ch23].enabled = 	(arg && LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch23].control = 	arg;
			gLTC185x.ChData[Ch23].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT23;
			return 0;

		case MZIO_LTC185x_CH23SE_SET_PERIOD:	
			gLTC185x.ChData[Ch23].trig 	= 	arg / Timer1uSDivideRatio;
			gLTC185x.ChData[Ch23].count 	= 	gLTC185x.ChData[Ch0].trig;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH45DE_SETUP:	
			gLTC185x.ChData[Ch45].enabled = 	(arg && LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch45].control = 	arg;
			gLTC185x.ChData[Ch45].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT45;
			return 0;

		case MZIO_LTC185x_CH45SE_SET_PERIOD:	
			gLTC185x.ChData[Ch45].trig 	= 	arg / Timer1uSDivideRatio;
			gLTC185x.ChData[Ch45].count 	= 	gLTC185x.ChData[Ch0].trig;
			return 0;

		// -----------------------------------------------------
		case MZIO_LTC185x_CH67DE_SETUP:	
			gLTC185x.ChData[Ch67].enabled = 	(arg && LTC185x_ChSetup_Enabled);
			gLTC185x.ChData[Ch67].control = 	arg;
			gLTC185x.ChData[Ch67].control |=	(arg & 0xFF) | ADC_SINGLE_ENDED_INPUT67;
			return 0;

		case MZIO_LTC185x_CH67SE_SET_PERIOD:	
			gLTC185x.ChData[Ch67].trig 	= 	arg / Timer1uSDivideRatio;
			gLTC185x.ChData[Ch67].count 	= 	gLTC185x.ChData[Ch0].trig;
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

		
		case MZIO_LTC185x_READ:
			printk("LTC185x: Read to 0x%lx\n", arg);
			
			// Wait for data to be ready
//			wait_eventinterruptible(wq, 0);

//			do_read();
			
/*			
			{
//				int tempBuf[5] = {1,2,3,4,5};
				
//				if (!access_ok(VERIFY_WRITE, (void __user *)arg, 5)) return -EFAULT;
					
//				return put_user(SPPRE,	(int *)arg);
	//			if (copy_to_user((void *)arg, &SPPRE, sizeof(int))) return -EFAULT;
				
			}
*/			
			return 0;

		case MZIO_LTC185x_SET_READ_FILENAME:
			// If system is running, return error
			
			// Create the file
			
			// Create the mmap
			
			
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
