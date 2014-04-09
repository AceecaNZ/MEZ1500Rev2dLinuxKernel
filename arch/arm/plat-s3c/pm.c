/* linux/arch/arm/plat-s3c/pm.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2004,2006,2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * S3C common power management (suspend to ram) support.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/serial_core.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include <mach/map.h>

#include <plat/regs-serial.h>
#include <mach/regs-clock.h>
#include <mach/regs-irq.h>
#include <asm/irq.h>

#include <plat/pm.h>
#include <plat/pm-core.h>

// 2011-03-08 SV: Added for s3c_pm_enter debug
#include <linux/gpio.h>
#include <linux/irq.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>

// 2011-04-13 SV: Added
#include <plat/regs-adc.h>
static unsigned long m_adc_regs[3];


/* for external use */

unsigned long s3c_pm_flags;


/* Debug code:
 *
 * This code supports debug output to the low level UARTs for use on
 * resume before the console layer is available.
*/

#ifdef CONFIG_S3C2410_PM_DEBUG
extern void printascii(const char *);

void s3c_pm_dbg(const char *fmt, ...)
{
	va_list va;
	char buff[256];

	va_start(va, fmt);
	vsprintf(buff, fmt, va);
	va_end(va);

	printascii(buff);
}

static inline void s3c_pm_debug_init(void)
{
	/* restart uart clocks so we can use them to output */
	s3c_pm_debug_init_uart();
}

#else
#define s3c_pm_debug_init() do { } while(0)

#endif /* CONFIG_S3C2410_PM_DEBUG */

/* Save the UART configurations if we are configured for debug. */

unsigned char pm_uart_udivslot;

#ifdef CONFIG_S3C2410_PM_DEBUG

struct pm_uart_save uart_save[CONFIG_SERIAL_SAMSUNG_UARTS];

static void s3c_pm_save_uart(unsigned int uart, struct pm_uart_save *save)
{
	void __iomem *regs = S3C_VA_UARTx(uart);

	save->ulcon = __raw_readl(regs + S3C2410_ULCON);
	save->ucon = __raw_readl(regs + S3C2410_UCON);
	save->ufcon = __raw_readl(regs + S3C2410_UFCON);
	save->umcon = __raw_readl(regs + S3C2410_UMCON);
	save->ubrdiv = __raw_readl(regs + S3C2410_UBRDIV);

	if (pm_uart_udivslot)
		save->udivslot = __raw_readl(regs + S3C2443_DIVSLOT);

	S3C_PMDBG("UART[%d]: ULCON=%04x, UCON=%04x, UFCON=%04x, UBRDIV=%04x\n",
		  uart, save->ulcon, save->ucon, save->ufcon, save->ubrdiv);
}

static void s3c_pm_save_uarts(void)
{
	struct pm_uart_save *save = uart_save;
	unsigned int uart;

	for (uart = 0; uart < CONFIG_SERIAL_SAMSUNG_UARTS; uart++, save++)
		s3c_pm_save_uart(uart, save);
}

static void s3c_pm_restore_uart(unsigned int uart, struct pm_uart_save *save)
{
	void __iomem *regs = S3C_VA_UARTx(uart);

	s3c_pm_arch_update_uart(regs, save);

	__raw_writel(save->ulcon, regs + S3C2410_ULCON);
	__raw_writel(save->ucon,  regs + S3C2410_UCON);
	__raw_writel(save->ufcon, regs + S3C2410_UFCON);
	__raw_writel(save->umcon, regs + S3C2410_UMCON);
	__raw_writel(save->ubrdiv, regs + S3C2410_UBRDIV);

	if (pm_uart_udivslot)
		__raw_writel(save->udivslot, regs + S3C2443_DIVSLOT);
}

static void s3c_pm_restore_uarts(void)
{
	struct pm_uart_save *save = uart_save;
	unsigned int uart;

	for (uart = 0; uart < CONFIG_SERIAL_SAMSUNG_UARTS; uart++, save++)
		s3c_pm_restore_uart(uart, save);
}
#else
static void s3c_pm_save_uarts(void) { }
static void s3c_pm_restore_uarts(void) { }
#endif

/* The IRQ ext-int code goes here, it is too small to currently bother
 * with its own file. */

unsigned long s3c_irqwake_intmask	= 0xffffffffL;
unsigned long s3c_irqwake_eintmask	= 0xffffffffL;

int s3c_irqext_wake(unsigned int irqno, unsigned int state)
{
	unsigned long bit = 1L << IRQ_EINT_BIT(irqno);

	if (!(s3c_irqwake_eintallow & bit))
		return -ENOENT;

	printk(KERN_INFO "wake %s for irq %d\n",
	       state ? "enabled" : "disabled", irqno);

	if (!state)
		s3c_irqwake_eintmask |= bit;
	else
		s3c_irqwake_eintmask &= ~bit;

	return 0;
}

/* helper functions to save and restore register state */

/**
 * s3c_pm_do_save() - save a set of registers for restoration on resume.
 * @ptr: Pointer to an array of registers.
 * @count: Size of the ptr array.
 *
 * Run through the list of registers given, saving their contents in the
 * array for later restoration when we wakeup.
 */
void s3c_pm_do_save(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		ptr->val = __raw_readl(ptr->reg);
		S3C_PMDBG("saved %p value %08lx\n", ptr->reg, ptr->val);
	}
}

/**
 * s3c_pm_do_restore() - restore register values from the save list.
 * @ptr: Pointer to an array of registers.
 * @count: Size of the ptr array.
 *
 * Restore the register values saved from s3c_pm_do_save().
 *
 * Note, we do not use S3C_PMDBG() in here, as the system may not have
 * restore the UARTs state yet
*/

void s3c_pm_do_restore(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		printk(KERN_DEBUG "restore %p (restore %08lx, was %08x)\n",
		       ptr->reg, ptr->val, __raw_readl(ptr->reg));

		__raw_writel(ptr->val, ptr->reg);
	}
}

/**
 * s3c_pm_do_restore_core() - early restore register values from save list.
 *
 * This is similar to s3c_pm_do_restore() except we try and minimise the
 * side effects of the function in case registers that hardware might need
 * to work has been restored.
 *
 * WARNING: Do not put any debug in here that may effect memory or use
 * peripherals, as things may be changing!
*/

void s3c_pm_do_restore_core(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++)
		__raw_writel(ptr->val, ptr->reg);
}

/* s3c2410_pm_show_resume_irqs
 *
 * print any IRQs asserted at resume time (ie, we woke from)
*/
static void s3c_pm_show_resume_irqs(int start, unsigned long which,
				    unsigned long mask)
{
	int i;

	which &= ~mask;

	for (i = 0; i <= 31; i++) {
		if (which & (1L<<i)) {
			S3C_PMDBG("IRQ %d asserted at resume\n", start+i);
		}
	}
}

// 2011-04-13 SV: Added
static void s3c_pm_save_adc_regs(void)
{
	static void __iomem *base_addr;	
	base_addr=ioremap(S3C2410_PA_ADC,0x20);
		
	m_adc_regs[0] = __raw_readl(base_addr + S3C2410_ADCCON);
	m_adc_regs[1] = __raw_readl(base_addr + S3C2410_ADCDLY);
	m_adc_regs[2] = __raw_readl(base_addr + S3C2410_ADCTSC);	
	
	iounmap(base_addr);
}

// 2011-04-13 SV: Added
static void s3c_pm_restore_adc_regs(void)
{
	static void __iomem *base_addr;	
	base_addr=ioremap(S3C2410_PA_ADC,0x20);

	__raw_writel(m_adc_regs[0], base_addr + S3C2410_ADCCON);
	__raw_writel(m_adc_regs[1], base_addr + S3C2410_ADCDLY);
	__raw_writel(m_adc_regs[2], base_addr + S3C2410_ADCTSC);		
	
	iounmap(base_addr);
}



// 2011-03-?? SV: Added
static void s3c_pm_safe_gpios(void)
{
	S3C_PMDBG("s3c_pm_safe_gpios()\n");

// Port A
//Ports  : GPA22 GPA21   GPA20 GPA19 GPA18 GPA17 GPA16     GPA15         GPA14      GPA13   GPA12  GPA11  GPA10  GPA9   GPA8   GPA7   GPA6   GPA5   GPA4   GPA3   GPA2   GPA1   GPA0
//Signal : nFCE  nRSTOUT nFRE  nFWE  ALE   CLE   AUDIO_ENn KP_MATRIX_CSn VIBMTR_ENn CHG_ENn L3MODE ADDR26 ADDR25 ADDR24 ADDR23 ADDR22 ADDR21 ADDR20 ADDR19 ADDR18 ADDR17 ADDR16 ADDR0
//Binary : 1     1       1     1     1     1     0         1             0          0       0      1      1      1      1      1      1      1      1      1      1      1      1
	__raw_writel(__raw_readl(S3C2410_GPADAT) | ((1<<16) | (1<<14)), S3C2410_GPADAT);
	__raw_writel(0x7E8FFF, S3C2410_GPACON);

// Port B
// Ports  : GPB10       GPB9       GPB8         GPB7       GPB6      GPB5          GPB4    GPB3   GPB2             GPB1          GPB0
// Signal : CHG_PWR_GDn CHG_STAT2n 5V0_SMPS_ENn CHG_STAT1n PSU_PWRDN WLESS_5V0_ENn L3CLOCK L3DATA MZIO_SPI_DRN_PWM BUZZER_PWM_EN BL_EN_PWM
// Setting: IN          IN         OUT          IN         OUT       x           	 OUT     OUT    x              	 OUT           OUT 
// Binary : 00          00         01           00         01        xx            01      01     xx               01            01  
// PU_OFF : 0           0          1            0          1         1             1       1      1                1             1
// DATA   : 0           0          0            0          0         1             0       0      0                0             0 
	__raw_writel(0x020, 	S3C2410_GPBDAT);
	__raw_writel(0x11555, S3C2410_GPBCON);
	__raw_writel(0x17F, 	S3C2410_GPBUP);

// Port C
// Ports  : GPC15 GPC14 GPC13 GPC12 GPC11 GPC10 GPC9      GPC8      GPC7      GPC6        GPC5    GPC4    GPC3      GPC2      GPC1     GPC0
// Signal : VD7   VD6   VD5   VD4   VD3   VD2   5V_CR_ENn 5V_MZ_ENn BT_RESETn WIFI_RESETn WIFI_CS WIFI_EN LCD_VSYNC LCD_HYSNC LCD_PCLK Wless_BT_EN
// Setting: OUT   OUT   OUT   OUT   OUT   OUT   OUT       OUT       IN        IN          OUT     OUT     OUT       OUT       OUT      OUT
// Binary : 01    01    01    01    01    01    01        01        00        00          01      01      01        01        01       01
// PU_OFF : 1     1     1     1     1     1     1         1         1         1           1       1       1         1         1        1
// DATA   : 0     0     0     0     0     0     1         1         0         0           0       0       0         0         0        0
	__raw_writel(0x0300, 			S3C2410_GPCDAT);
	__raw_writel(0x55550555, 	S3C2410_GPCCON);
	__raw_writel(0xFFFF, 			S3C2410_GPCUP);

// Port D
// Ports  : GPD15 GPD14 GPD13 GPD12 GPD11 GPD10 GPD9     GPD8         GPD7 GPD6 GPD5 GPD4 GPD3 GPD2 GPD1         GPD0
// Signal : VD23  VD22  VD21  VD20  VD19  VD18  RS232_EN MZIO_IrDAENn VD15 VD14 VD13 VD12 VD11 VD10 MZIO_MOD_PWR MZIO_RESET
// Setting: OUT   OUT   OUT   OUT   OUT   OUT   OUT      OUT          OUT  OUT  OUT  OUT  OUT  OUT  OUT          OUT
// Binary : 01    01    01    01    01    01    01       01           01   01   01   01   01   01   01           01
// PU_OFF : 1     1     1     1     1     1     1        1            1    1    1    1    1    1    1            1
// DATA   : 0     0     0     0     0     0     0        1            0    0    0    0    0    0    0            0
	__raw_writel(0x0100, 			S3C2410_GPDDAT);
	__raw_writel(0x55555555, 	S3C2410_GPDCON);
	__raw_writel(0xFFFF, 			S3C2410_GPDUP);

// Port E
// Ports  : GPE15  GPE14  GPE13      GPE12       GPE11       GPE10   GPE9    GPE8    GPE7    GPE6  GPE5  GPE4   GPE3   GPE2  GPE1    GPE0    
// Signal : IICSDA IICSCL MZ_SPI_CLK MZ_SPI_MOSI MZ_SPI_MISO SDDATA3 SDDATA2 SDDATA1 SDDATA0 SDCMD SDCLK I2SSDO I2SSDI CDCLK I2SSCLK I2SLRCK
// Setting: IN     IN     OUT        OUT         IN          OUT     OUT     OUT     OUT     OUT   OUT   OUT    IN     OUT   OUT     OUT
// Binary : 00     00     01         01          00          01      01      01      01      01    01    01     00     01    01      01
// PU_OFF : 1      1      1          1           1           1       1       1       1       1     1     1      1      1     1       1 
// DATA   : 1      1      0          0           0           0       0       0       0       0     0     0      0      0     0       0
	__raw_writel(0xC000, 			S3C2410_GPEDAT);
	__raw_writel(0x5000115, 	S3C2410_GPECON);	// SV: Setting SD lines as input because we are keeping the card powered!
//	__raw_writel(0x05155515, 	S3C2410_GPECON);
	__raw_writel(0xFFFF, 			S3C2410_GPEUP);

// Port F
// Ports  : GPF7          GPF6            GPF5       GPF4     GPF3           GPF2           GPF1         GPF0
// Signal : MZ_RDY_IREQ_N KP_IREQN_WAKEUP WIFI_IREQn BT_IREQn KP_SideButtonR KP_SideButtonL KP_BLIGHTKEY KP_POWERKEY
// Setting: x         		IN              x      	 	 x				IN             IN             IN           EINT0
// Binary : xx            00              xx         xx       00             00             00           10
// PU_OFF : 1             1               x          x        1              1              1            1
// DATA   : 0             0               0          0        0              0              0            0
	__raw_writel(0xC000, 	S3C2410_GPFDAT);
	__raw_writel(0x0002,	S3C2410_GPFCON);
	__raw_writel(0xFF, 		S3C2410_GPFUP);

// Port G
// ** WARNING ** GPG[15:13] must be inputs for NAND mode
// Ports  : GPG15 GPG14 GPG13 GPG12   GPG11          GPG10   GPG9    GPG8   GPG7     GPG6      GPG5      GPG4   GPG3          GPG2      GPG1   GPG0    
// Signal : x     x     x     BATT_SW MZ_SPI_DRN_PWM BT_CTSn BT_RTSn SD_CDn WIFI_CLK WIFI_MOSI WIFI_MISO LCD_ON CRADLE_WAKEUP MZ_SPI_SS SD_ENn CRADLE_HOTSYNCINT
// Setting: IN    IN    IN    IN	    IN             IN      OUT     IN     OUT      OUT       OUT       OUT    EINT11        OUT       OUT	   EINT8   
// Binary : 00    00    00    00      00             00      01      00     01       01        01        01     10            01        01     10
// PU_OFF : 1     1     1     1       1              1       1       1      1        1         1         1      1             1         1      1
// DATA   : 0     0     0     0       0              0       0       0      0        0         0         0      0             0         1      0
__raw_writel(0x0000, 			S3C2410_GPGDAT);	// SV: Keep SD card powered!
//	__raw_writel(0x0002, 			S3C2410_GPGDAT);
	__raw_writel(0x00045596, 	S3C2410_GPGCON);
	__raw_writel(0xFFFF, 			S3C2410_GPGUP);

// Port H
// Ports  : GPH10       GPH9       GPH8  GPH7 GPH6 GPH5    GPH4    GPH3 GPH2 GPH1 GPH0 
// Signal : USB_HOST_EN 3V3EXT_ENn SD_WP RXD2 TXD2 BT_RXD1 BT_TXD1 RXD0 TXD0 RTS0 CTS0
// Setting: OUT         OUT        IN    IN   OUT  IN      OUT     IN   OUT  OUT  IN
// Binary : 01          01         00    00   01   00      01      00   01   01   00
// PU_OFF : 1           1          1     1    1    1       1       1    1    1    1
// DATA   : 1           1          0     0    0    0       0       0    0    0    0
// Note: USB_HOST_EN is high as a USB electrical disconnect
	__raw_writel(0x0600, 		S3C2410_GPHDAT);
	__raw_writel(0x14A114, 	S3C2410_GPHCON);	// SV: Keep the debug UART alive!
//	__raw_writel(0x141114, 	S3C2410_GPHCON);
	__raw_writel(0x7FF,			S3C2410_GPHUP);
	
// Port J
// Ports  : GPJ12   GPJ11       GPJ10	   GPJ9      GPJ8     GPJ7      GPJ6      GPJ5      GPJ4      GPJ3      GPJ2      GPJ1      GPJ0
// Signal : CAM_RST CAM_CLK_OUT CAM_HREF CAM_VSYNC CAM_PCLK CAM_DATA7 CAM_DATA6 CAM_DATA5 CAM_DATA4 CAM_DATA3 CAM_DATA2 CAM_DATA1 CAM_DATA0
// Setting: OUT     OUT         OUT      OUT       OUT      OUT       OUT       OUT       OUT       OUT       OUT       OUT       OUT
// Binary : 01      01          01       01        01       01        01        01        01        01        01        01        01
// PU_OFF : 1       1           1        1         1        1         1         1         1         1         1         1         1
// DATA   : 0       0           0        0         0        0         0         0         0         0         0         0         0
	__raw_writel(0x0000, 		S3C2440_GPJDAT);
	__raw_writel(0x1555555, S3C2440_GPJCON);
	__raw_writel(0x1FFF,		S3C2440_GPJUP);
}



void (*pm_cpu_prep)(void);
void (*pm_cpu_sleep)(void);

#define any_allowed(mask, allow) (((mask) & (allow)) != (allow))

/* s3c_pm_enter
 *
 * central control for sleep/resume process
*/

static int s3c_pm_enter(suspend_state_t state)
{
	static unsigned long regs_save[16];

	/* ensure the debug is initialised (if enabled) */

	s3c_pm_debug_init();

	S3C_PMDBG("%s(%d)\n", __func__, state);

	if (pm_cpu_prep == NULL || pm_cpu_sleep == NULL) {
		printk(KERN_ERR "%s: error: no cpu sleep function\n", __func__);
		return -EINVAL;
	}

	/* check if we have anything to wake-up with... bad things seem
	 * to happen if you suspend with no wakeup (system will often
	 * require a full power-cycle)
	*/

	if (!any_allowed(s3c_irqwake_intmask, s3c_irqwake_intallow) &&
	    !any_allowed(s3c_irqwake_eintmask, s3c_irqwake_eintallow)) {
		printk(KERN_ERR "%s: No wake-up sources!\n", __func__);
		printk(KERN_ERR "%s: Aborting sleep\n", __func__);
		return -EINVAL;
	}

	/* store the physical address of the register recovery block */

	s3c_sleep_save_phys = virt_to_phys(regs_save);

	S3C_PMDBG("s3c_sleep_save_phys=0x%08lx\n", s3c_sleep_save_phys);

	/* save all necessary core registers not covered by the drivers */

	s3c_pm_save_gpios();


// 2011-03-23 SV: Added for safe & lowest current during sleep.
	s3c_pm_safe_gpios();

	
	s3c_pm_save_uarts();
	
	s3c_pm_save_adc_regs();
	
	s3c_pm_save_core();

	/* set the irq configuration for wake */

	s3c_pm_configure_extint();

	S3C_PMDBG("sleep: irq wakeup masks: %08lx,%08lx\n",
	    s3c_irqwake_intmask, s3c_irqwake_eintmask);

	s3c_pm_arch_prepare_irqs();
	
	/* call cpu specific preparation */

	pm_cpu_prep();

	/* flush cache back to ram */

	flush_cache_all();

	s3c_pm_check_store();

	/* send the cpu to sleep... */

	s3c_pm_arch_stop_clocks();

	/* s3c_cpu_save will also act as our return point from when
	 * we resume as it saves its own register state and restores it
	 * during the resume.  */
	s3c_cpu_save(regs_save);


	/* restore the cpu state using the kernel's cpu init code. */
	cpu_init();

	/* restore the system state */
	s3c_pm_restore_core();
	s3c_pm_restore_uarts();
	s3c_pm_restore_gpios();

	s3c_pm_debug_init();

	/* check what irq (if any) restored the system */

	s3c_pm_arch_show_resume_irqs();

	s3c_pm_restore_adc_regs();

	S3C_PMDBG("%s: post sleep, preparing to return\n", __func__);

	/* LEDs should now be 1110 */
//	s3c_pm_debug_smdkled(1 << 1, 0);

	s3c_pm_check_restore();

	/* ok, let's return from sleep */

	S3C_PMDBG("S3C PM Resume (post-restore)\n");
	return 0;
}

/* callback from assembly code */
void s3c_pm_cb_flushcache(void)
{
	flush_cache_all();
}

static int s3c_pm_prepare(void)
{
	/* prepare check area if configured */

	s3c_pm_check_prepare();
	return 0;
}

static void s3c_pm_finish(void)
{
	s3c_pm_check_cleanup();
}

static struct platform_suspend_ops s3c_pm_ops = {
	.enter		= s3c_pm_enter,
	.prepare	= s3c_pm_prepare,
	.finish		= s3c_pm_finish,
	.valid		= suspend_valid_only_mem,
};

/* s3c_pm_init
 *
 * Attach the power management functions. This should be called
 * from the board specific initialisation if the board supports
 * it.
*/

int __init s3c_pm_init(void)
{
	printk("S3C Power Management, Copyright 2004 Simtec Electronics\n");

	suspend_set_ops(&s3c_pm_ops);
	return 0;
}
