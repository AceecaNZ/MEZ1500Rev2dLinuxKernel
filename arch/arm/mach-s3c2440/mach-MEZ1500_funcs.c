
// 2011-04-14 SV: Added

#include <linux/interrupt.h>
#include <linux/apm-emulation.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/kmod.h>
#include <linux/timer.h>

#include <mach/mach-MEZ1500_funcs.h>
#include <mach/regs-gpio.h>

#include <plat/regs-timer.h>
#include <plat/regs-adc.h>

#define MEZ1500_BOOTING				0
#define MEZ1500_SUSPEND   		1
#define MEZ1500_RESUME  			2
#define MEZ1500_READY					3

#define BTN_TIMER_100mS	(HZ / 10)
#define BTN_LONG_PRESS_TIME		30		// value * 100mS

static unsigned int	 m_pwr_btn_irq_timer_en = 0;
static unsigned long m_pwr_btn_irq_timer_count = 0;

static struct timer_list m_pwr_btn_timer;

static char * pwr_btn_shutdown_argv[] = { "/sbin/poweroff", NULL };

int m_mez1500_state = MEZ1500_BOOTING;

long m_bl_value = 100;

static void mez1500_set_bl(long value)
{ 
	unsigned long tcon;
	unsigned long tcnt;
	unsigned long tcfg1;
	unsigned long tcfg0;

	struct clk *clk_p;
	unsigned long pclk;
	
	if (value == 0)
	{
		s3c2410_gpio_setpin(S3C2410_GPB(0), 0);
		s3c2410_gpio_cfgpin(S3C2410_GPB(0), S3C2410_GPIO_OUTPUT);
		tcon = __raw_readl(S3C2410_TCON);
		tcon &= ~(0xF);
		__raw_writel(tcon, S3C2410_TCON);
		return;
	}

	if (value == 100)
	{
		s3c2410_gpio_setpin(S3C2410_GPB(0), 1);
		s3c2410_gpio_cfgpin(S3C2410_GPB(0), S3C2410_GPIO_OUTPUT);
		tcon = __raw_readl(S3C2410_TCON);
		tcon &= ~(0xF);
		__raw_writel(tcon, S3C2410_TCON);
		return;
	}

	//set GPB0 as tout0, pwm output
	s3c2410_gpio_cfgpin(S3C2410_GPB(0), S3C2410_GPB0_TOUT0);

	tcon = __raw_readl(S3C2410_TCON);
	tcfg1 = __raw_readl(S3C2410_TCFG1);
	tcfg0 = __raw_readl(S3C2410_TCFG0);

	//prescaler = 5
	tcfg0 &= ~S3C2410_TCFG_PRESCALER0_MASK;
	tcfg0 |= (5 - 1); 

	//mux = 1/2
	tcfg1 &= ~S3C2410_TCFG1_MUX0_MASK;
	tcfg1 |= S3C2410_TCFG1_MUX0_DIV2;

	__raw_writel(tcfg1, S3C2410_TCFG1);
	__raw_writel(tcfg0, S3C2410_TCFG0);

	clk_p = clk_get(NULL, "pclk");
	pclk  = clk_get_rate(clk_p);
	
	tcnt  = (pclk/5/2)/50000;			// 50khz
	
	__raw_writel(tcnt,  S3C2410_TCNTB(0));
	__raw_writel(value, S3C2410_TCMPB(0));
				
	tcon &= ~0x1f;
	tcon |= 0xb;		//disable deadzone, auto-reload, inv-off, update TCNTB0&TCMPB0, start timer 0
	__raw_writel(tcon, S3C2410_TCON);
	
	tcon &= ~2;			//clear manual update bit
	__raw_writel(tcon, S3C2410_TCON);
} 

void pwr_btn_timer_irq(unsigned long arg)
{
	m_pwr_btn_irq_timer_count++;	

	if((s3c2410_gpio_getpin(S3C2410_GPF(0)) == 0) && m_pwr_btn_irq_timer_count < BTN_LONG_PRESS_TIME)
	{
		//printk("Short press\n");
		m_pwr_btn_irq_timer_en = 0;
		m_mez1500_state = MEZ1500_SUSPEND;
		apm_queue_event(APM_USER_SUSPEND);
		return;
	}
	
	if((s3c2410_gpio_getpin(S3C2410_GPF(0)) == 1) && m_pwr_btn_irq_timer_count > BTN_LONG_PRESS_TIME)
	{
		//printk("Long press\n");
		mez1500_set_bl(0);
		m_pwr_btn_irq_timer_en = 0;
		call_usermodehelper(pwr_btn_shutdown_argv[0], pwr_btn_shutdown_argv, NULL, UMH_NO_WAIT);
		return;
	}

	if(s3c2410_gpio_getpin(S3C2410_GPF(0)) == 1)
	{
		m_pwr_btn_timer.expires = jiffies + BTN_TIMER_100mS;
		add_timer (&m_pwr_btn_timer);
	}
}

static irqreturn_t pwr_btn_irq(int irq, void *pw)
{
	//printk("+++pwr_btn_irq\n");

	switch(m_mez1500_state)
	{
		case MEZ1500_BOOTING:
			m_mez1500_state = MEZ1500_READY;
			break;
			
		case MEZ1500_READY:
			if(m_pwr_btn_irq_timer_en == 0)
			{
				// Init btn timer for debounce etc
				m_pwr_btn_timer.expires = jiffies + BTN_TIMER_100mS;
				add_timer (&m_pwr_btn_timer);
		    m_pwr_btn_irq_timer_en = 1;
		    m_pwr_btn_irq_timer_count = 0;
			}
			break;
	}

	//printk("---pwr_btn_irq\n\n");
	
	return IRQ_HANDLED;
}

static irqreturn_t bl_btn_irq(int irq, void *pw)
{
	//printk("\n+++bl_btn_irq\n");

	switch(m_bl_value)
	{
		case 100: m_bl_value = 80; break;
		case  80: m_bl_value = 60; break;
		case  60: m_bl_value = 40; break;
		case  40: m_bl_value = 20; break;
		case  20: m_bl_value = 10; break;												
		case  10: m_bl_value =  5; break;
		default: 	m_bl_value =100; break;
	}

	mez1500_set_bl(m_bl_value);

	//printk("---bl_btn_irq m_bl_value = %d\n\n", (int)m_bl_value);
	
	return IRQ_HANDLED;
}

static void mez1500_init_irqs(void)
{
// Power on/off btn	
	request_irq(IRQ_EINT0, pwr_btn_irq, IRQF_TRIGGER_RISING, "pwr-button-irq-eint0", NULL);
	enable_irq_wake(IRQ_EINT0);

// Backlight btn	
	request_irq(IRQ_EINT1, bl_btn_irq, IRQF_TRIGGER_RISING, "bl-button-irq-eint0", NULL);
}

void mez1500_suspend(void)
{
	mez1500_set_bl(0);
}

void mez1500_resume(void)
{
	mez1500_set_bl(m_bl_value);
	m_mez1500_state = MEZ1500_READY;
}

/*
static void Delay(int cnt)
{
	volatile int i;

	for(;cnt>0;cnt--)
	for(i=0;i<150;i++);
}
*/
/*
static unsigned int read_adc_ch(int ch)
{
	unsigned long con;
	
	// Turn on the ADC prescaler, leave on default maximum prescale value
	con = __raw_readl(S3C2410_ADCCON);
	con |= (0x1 << 14);
	__raw_writel(con, S3C2410_ADCCON);
	
	Delay(3);

	// Take ADC off standby mode
	con = __raw_readl(S3C2410_ADCCON);	
	con &= ~(0x1 << 2);
	__raw_writel(con, S3C2410_ADCCON);

	// Configure to read ADC channel
	con = __raw_readl(S3C2410_ADCCON);	
	con &= ~(0x7 << 3);
	con |= (ch << 3);
	__raw_writel(con, S3C2410_ADCCON);
		
	// Start the ADC conversion
	con = __raw_readl(S3C2410_ADCCON);	
	con |= 0x1;
	__raw_writel(con, S3C2410_ADCCON);
		
	// Wait for the valid value
	while (!(__raw_readl(S3C2410_ADCCON) & 0x8000)) 	
	{	
		continue;
	}
	
	return (__raw_readl(S3C2410_ADCDAT0) & 0x3FF);
}
*/

static void mez1500_apm_get_power_status(struct apm_power_info *info)
{
	//printk("\n++read_adc_ch(0) = 0x%X", read_adc_ch(0));
}

void mez1500_init(void)
{
	init_timer (&m_pwr_btn_timer);
	m_pwr_btn_timer.function = pwr_btn_timer_irq;
	m_pwr_btn_timer.data = 0;
	
	mez1500_set_bl(m_bl_value);
	mez1500_init_irqs();	
	
	m_mez1500_state = MEZ1500_READY;

	apm_get_power_status = mez1500_apm_get_power_status;
}
