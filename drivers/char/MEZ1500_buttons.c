#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <mach/regs-gpio.h>
#include <mach/hardware.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/gpio.h>

#define DEVICE_NAME     "buttons"

struct button_irq_desc {
    int irq;
    int pin;
    int pin_setting;
    int number;
    char *name;	
};

// 2011-01-22 SV: Adjusted for MEZ1500Rev2c
static struct button_irq_desc button_irqs [] = {
//    {IRQ_EINT0 , S3C2410_GPF(0),  S3C2410_GPF0_EINT0  , 0, "KEY0"},		// KP_POWERKEY
//    {IRQ_EINT1 , S3C2410_GPF(1),  S3C2410_GPF1_EINT1  , 1, "KEY1"},		// KP_BLIGHTKEY
    {IRQ_EINT2 , S3C2410_GPF(2),  S3C2410_GPF2_EINT2  , 2, "KEY2"},		// KP_SideButtonL
    {IRQ_EINT3 , S3C2410_GPF(3),  S3C2410_GPF3_EINT3  , 3, "KEY3"},		// KP_SideButtonR
    {IRQ_EINT6 , S3C2410_GPF(6),  S3C2410_GPF6_EINT6  , 4, "KEY4"},		// KP_IREQN_WAKEUP
    {        -1,            -1,                 -1    , 5, "KEY5"},   // Undefined 
};

static volatile char key_values [] = {'0', '0', '0', '0', '0', '0'};

static DECLARE_WAIT_QUEUE_HEAD(button_waitq);

static volatile int ev_press = 0;

// 2011-02-22 SV: Adjusted for MEZ1500Rev2c
static irqreturn_t buttons_interrupt(int irq, void *dev_id)
{
    struct button_irq_desc *button_irqs = (struct button_irq_desc *)dev_id;
    int pressed;
    
    printk("++irqreturn_t buttons_interrupt\n");

    //udelay(0);
    pressed = s3c2410_gpio_getpin(button_irqs->pin);

    if (pressed != (key_values[button_irqs->number] & '0'))
    { // Changed

			key_values[button_irqs->number] = '0' + pressed;
    	printk("%s\n", button_irqs->name);
	
      ev_press = 1;
      wake_up_interruptible(&button_waitq);
    }
    
    printk("--irqreturn_t buttons_interrupt\n");
    
    return IRQ_RETVAL(IRQ_HANDLED);
}



static int s3c24xx_buttons_open(struct inode *inode, struct file *file)
{
    int i;
    int err = 0;
    
		printk (DEVICE_NAME"\ts3c24xx_buttons_open\n");

    for (i = 0; i < sizeof(button_irqs)/sizeof(button_irqs[0]); i++)
    {
			if (button_irqs[i].irq < 0)
			{
				continue;
			}

			printk ("Requesting IRQ\n");

      err = request_irq(button_irqs[i].irq, buttons_interrupt, IRQ_TYPE_EDGE_BOTH, 
                        button_irqs[i].name, (void *)&button_irqs[i]);
      if (err)
      {
				printk ("error=%x\n", err);
        break;
      }
    }

    if (err)
    {
			printk ("Err\n");

    	i--;
      for (; i >= 0; i--)
      {
	    	if (button_irqs[i].irq < 0)
	    	{
					continue;
	    	}
	    	
	    	disable_irq(button_irqs[i].irq);
        free_irq(button_irqs[i].irq, (void *)&button_irqs[i]);
        
			}
        return -EBUSY;
    }

    ev_press = 1;

		printk ("Returning\n");
    
    return 0;
}


static int s3c24xx_buttons_close(struct inode *inode, struct file *file)
{
    int i;
    
		printk (DEVICE_NAME"\ts3c24xx_buttons_close\n");

    for (i = 0; i < sizeof(button_irqs)/sizeof(button_irqs[0]); i++)
    {
			if (button_irqs[i].irq < 0)
			{
	    	continue;
			}
		
			free_irq(button_irqs[i].irq, (void *)&button_irqs[i]);
    }

    return 0;
}


static int s3c24xx_buttons_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
    unsigned long err;

		printk (DEVICE_NAME"\ts3c24xx_buttons_read\n");

    if (!ev_press)
    {
		if (filp->f_flags & O_NONBLOCK)
	  	return -EAGAIN;
		else
	    wait_event_interruptible(button_waitq, ev_press);
    }
    
    ev_press = 0;

    err = copy_to_user(buff, (const void *)key_values, min(sizeof(key_values), count));

    return err ? -EFAULT : min(sizeof(key_values), count);
}


static unsigned int s3c24xx_buttons_poll( struct file *file, struct poll_table_struct *wait)
{
    unsigned int mask = 0;
    
		printk (DEVICE_NAME"\ts3c24xx_buttons_poll\n");

    poll_wait(file, &button_waitq, wait);
    
    if (ev_press)
        mask |= POLLIN | POLLRDNORM;
        
    return mask;
}


static struct file_operations dev_fops = {
    .owner   =   THIS_MODULE,
    .open    =   s3c24xx_buttons_open,
    .release =   s3c24xx_buttons_close, 
    .read    =   s3c24xx_buttons_read,
    .poll    =   s3c24xx_buttons_poll,
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

	printk (DEVICE_NAME"\tinitialized\n");

	return ret;
}


static void __exit dev_exit(void)
{
	printk (DEVICE_NAME"\tgoodbye!\n");
	misc_deregister(&misc);
}


module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aceeca Inc.");
