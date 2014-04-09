/* linux/arch/arm/mach-s3c2440/mach-MEZ1500.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

/*
 * Based on same file of Michel Pollet
 *        by Aceeca 2009-12-31
 *  	  visit http://www.arm9.net for more information             
 */

/*
 * linux/arch/arm/mach-s3c2440/mach-MEZ1500.c
 *
 *  Copyright (c) 2008 Ramax Lo <ramaxlo@gmail.com>
 *        Based on mach-anubis.c by Ben Dooks <ben@simtec.co.uk>
 *        and modifications by SBZ <sbz@spgui.org> and
 *        Weibing <http://weibing.blogbus.com> and
 *        Michel Pollet <buserror@gmail.com>
 * 
 *  For product information, visit http://code.google.com/p/MEZ1500/
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/sysdev.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/dm9000.h>
#include <linux/mmc/host.h>
#include <linux/apm-emulation.h>


#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <mach/regs-gpio.h>
#include <mach/regs-lcd.h>
#include <mach/leds-gpio.h>
#include <mach/idle.h>
#include <mach/fb.h>
#include <mach/mach-MEZ1500_funcs.h>


#include <plat/regs-serial.h>
#include <plat/iic.h>
#include <plat/s3c2410.h>
#include <plat/s3c2440.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/nand.h>
#include <plat/pm.h>
#include <plat/mci.h>

#include <sound/s3c24xx_uda134x.h>


//-----------------------------------------------------------------------------
// 20140408 SV: Added
#include <linux/spi/spi.h>
#include <mach/spi.h>
static struct spi_board_info s3c2410_spi0_board[] = {  
	[0] = {  
	.modalias = "spidev",  
	.bus_num      = 0,   
	.chip_select  = 0,  
	.irq 					= IRQ_EINT9,
	.max_speed_hz	= 500*1000,  
	},  
};

static struct s3c2410_spi_info s3c2410_spi0_platdata = {  
	.pin_cs		= S3C2410_GPD(0),  
	.num_cs 	= 1,  
	.bus_num 	= 0,  
};
//-----------------------------------------------------------------------------

static struct map_desc MEZ1500_iodesc[] __initdata = {
};

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg MEZ1500_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	}
};

/* LCD driver info */

#if defined(CONFIG_FB_S3C2410_N240320)

#define LCD_WIDTH 240
#define LCD_HEIGHT 320
#define LCD_PIXCLOCK 100000

// 2011-01-18 SV: Adjusted for hitachi LCD
#define LCD_RIGHT_MARGIN 4
#define LCD_LEFT_MARGIN  14
#define LCD_HSYNC_LEN 5

#define LCD_UPPER_MARGIN 4
#define LCD_LOWER_MARGIN 5
#define LCD_VSYNC_LEN 1
/*
#define LCD_RIGHT_MARGIN 36
#define LCD_LEFT_MARGIN 19
#define LCD_HSYNC_LEN 5

#define LCD_UPPER_MARGIN 1
#define LCD_LOWER_MARGIN 5
#define LCD_VSYNC_LEN 1
*/
#elif defined(CONFIG_FB_S3C2410_N480272)

#define LCD_WIDTH 480
#define LCD_HEIGHT 272
#define LCD_PIXCLOCK 100000

#define LCD_RIGHT_MARGIN 36
#define LCD_LEFT_MARGIN 19
#define LCD_HSYNC_LEN 5

#define LCD_UPPER_MARGIN 1
#define LCD_LOWER_MARGIN 5
#define LCD_VSYNC_LEN 1

#elif defined(CONFIG_FB_S3C2410_TFT640480)
#define LCD_WIDTH 640
#define LCD_HEIGHT 480
#define LCD_PIXCLOCK 40000

#define LCD_RIGHT_MARGIN 67 
#define LCD_LEFT_MARGIN 40
#define LCD_HSYNC_LEN 31

#define LCD_UPPER_MARGIN 5
#define LCD_LOWER_MARGIN 25
#define LCD_VSYNC_LEN 1

#elif defined(CONFIG_FB_S3C2410_T240320)
#define LCD_WIDTH 240
#define LCD_HEIGHT 320
#define LCD_PIXCLOCK 170000
#define LCD_RIGHT_MARGIN 25
#define LCD_LEFT_MARGIN 0
#define LCD_HSYNC_LEN 4
#define LCD_UPPER_MARGIN 1
#define LCD_LOWER_MARGIN 4
#define LCD_VSYNC_LEN 1
#define LCD_CON5 (S3C2410_LCDCON5_FRM565 | S3C2410_LCDCON5_INVVDEN | S3C2410_LCDCON5_INVVFRAME | S3C2410_LCDCON5_INVVLINE | S3C2410_LCDCON5_INVVCLK | S3C2410_LCDCON5_HWSWP ) 

#elif defined(CONFIG_FB_S3C2410_X240320)
#define LCD_WIDTH 240
#define LCD_HEIGHT 320
#define LCD_PIXCLOCK 170000
#define LCD_RIGHT_MARGIN 25
#define LCD_LEFT_MARGIN 0
#define LCD_HSYNC_LEN 4
#define LCD_UPPER_MARGIN 0
#define LCD_LOWER_MARGIN 4
#define LCD_VSYNC_LEN 9
#define LCD_CON5 (S3C2410_LCDCON5_FRM565 | S3C2410_LCDCON5_INVVDEN | S3C2410_LCDCON5_INVVFRAME | S3C2410_LCDCON5_INVVLINE | S3C2410_LCDCON5_INVVCLK | S3C2410_LCDCON5_HWSWP ) 

#elif defined(CONFIG_FB_S3C2410_TFT800480)
#define LCD_WIDTH 800
#define LCD_HEIGHT 480
#define LCD_PIXCLOCK 40000

#define LCD_RIGHT_MARGIN 67
#define LCD_LEFT_MARGIN 40
#define LCD_HSYNC_LEN 31

#define LCD_UPPER_MARGIN 25
#define LCD_LOWER_MARGIN 5
#define LCD_VSYNC_LEN 1

#elif defined(CONFIG_FB_S3C2410_VGA1024768)
#define LCD_WIDTH 1024
#define LCD_HEIGHT 768
#define LCD_PIXCLOCK 80000

#define LCD_RIGHT_MARGIN 15
#define LCD_LEFT_MARGIN 199
#define LCD_HSYNC_LEN 15

#define LCD_UPPER_MARGIN 1
#define LCD_LOWER_MARGIN 1
#define LCD_VSYNC_LEN 1
#define LCD_CON5 (S3C2410_LCDCON5_FRM565 | S3C2410_LCDCON5_HWSWP)

#endif

#if defined (LCD_WIDTH)

	static struct s3c2410fb_display MEZ1500_lcd_cfg __initdata = {
	
	#if !defined (LCD_CON5)
		.lcdcon5	= S3C2410_LCDCON5_FRM565 |
				  S3C2410_LCDCON5_INVVLINE |
				  S3C2410_LCDCON5_INVVFRAME |
				  S3C2410_LCDCON5_PWREN |
				  S3C2410_LCDCON5_HWSWP,
	#else
		.lcdcon5	= LCD_CON5,
	#endif
	
		.type		= S3C2410_LCDCON1_TFT,
	
		.width		= LCD_WIDTH,
		.height		= LCD_HEIGHT,
	
		.pixclock	= LCD_PIXCLOCK,
		.xres		= LCD_WIDTH,
		.yres		= LCD_HEIGHT,
		.bpp		= 16,
		.left_margin	= LCD_LEFT_MARGIN + 1,
		.right_margin	= LCD_RIGHT_MARGIN + 1,
		.hsync_len	= LCD_HSYNC_LEN + 1,
		.upper_margin	= LCD_UPPER_MARGIN + 1,
		.lower_margin	= LCD_LOWER_MARGIN + 1,
		.vsync_len	= LCD_VSYNC_LEN + 1,
	};
	
	
	static struct s3c2410fb_mach_info MEZ1500_fb_info __initdata = {
		.displays	= &MEZ1500_lcd_cfg,
		.num_displays	= 1,
		.default_display = 0,
		
/*	
// 20140329 SV: set up by loader
	// 2011-01-18 SV: Adjusted for MEZ1500Rev2c
		.gpccon =       0xaaa000a8,
		.gpccon_mask =  0xfff000fc,
		.gpcup =        0x0000fc0e,
		.gpcup_mask =   0xffffffff,
	
		.gpdcon =       0xaaa0aaa0,
		.gpdcon_mask =  0xfff0fff0,
		.gpdup =        0x0000faff,
		.gpdup_mask =   0xffffffff,
*/		
		
		.lpcsel		= 0xf82,
	};

#endif







static struct s3c24xx_uda134x_platform_data s3c24xx_uda134x_data = {
	.l3_clk = S3C2410_GPB(4),
	.l3_data = S3C2410_GPB(3),
	
// 2011-01-18 SV: Adjusted for MEZ1500Rev2c		
//	.l3_mode = S3C2410_GPB(2),
	.l3_mode = S3C2410_GPA(12),

	.model = UDA134X_UDA1341,
};

static struct platform_device s3c24xx_uda134x = {
	.name = "s3c24xx_uda134x",
	.dev = {
		.platform_data    = &s3c24xx_uda134x_data,
	}
};

// 2011-01-22 SV: Adjusted for MEZ1500Rev2c
static struct mtd_partition friendly_arm_default_nand_part[] = {
//64M
/*	 [0] = { .name    = "vivi",
                 .size    = 0x00020000,
                 .offset = 0X00000000,
         },
         [1] = {   .name    = "kernel_linux2.6.27",
                 .offset = 0x00050000,
                 .size    = 0x00200000,
         },
         [2] = { .name    = "root_yaffs2",
                 .offset =0x00250000,
                 .size    =0x03dac000,
         }
*/
//128M
	[0] = {
		.name	= "supervivi",
		.size	= 0x00060000,
		.offset	= 0,
	},
	[1] = {
		.name	= "Kernel",
		.offset = 0x00060000,
		.size	= 0x00200000,
	},
	[2] = {
		.name	= "root",
		.offset = 0x00260000,
		.size	= 1024 * 1024 * 1024, //64U * 1024 * 1024 - 0x00260000,
	},
	[3] = {
		.name	= "nand",
		.offset = 0x00000000,
		.size	= 1024 * 1024 * 1024, //64U * 1024 * 1024 - 0x00260000,
	}
};

/*
static struct mtd_partition friendly_arm_default_nand_part[] = {
	[0] = {
		.name	= "supervivi",
		.size	= 0x00040000,
		.offset	= 0,
	},
	[1] = {
		.name	= "param",
		.offset = 0x00040000,
		.size	= 0x00020000,
	},
	[2] = {
		.name	= "Kernel",
		.offset = 0x00060000,
		.size	= 0x00500000,
	},
	[3] = {
		.name	= "root",
		.offset = 0x00560000,
		.size	= 1024 * 1024 * 1024, //
	},
	[4] = {
		.name	= "nand",
		.offset = 0x00000000,
		.size	= 1024 * 1024 * 1024, //
	}
};
*/

static struct s3c2410_nand_set friendly_arm_nand_sets[] = {
	[0] = {
		.name		= "NAND",
		.nr_chips	= 1,
		.nr_partitions	= ARRAY_SIZE(friendly_arm_default_nand_part),
		.partitions	= friendly_arm_default_nand_part,
	},
};

/* choose a set of timings which should suit most 512Mbit
 * chips and beyond.
*/

static struct s3c2410_platform_nand friendly_arm_nand_info = {
	.tacls		= 20,
	.twrph0		= 60,
	.twrph1		= 20,
	.nr_sets	= ARRAY_SIZE(friendly_arm_nand_sets),
	.sets		= friendly_arm_nand_sets,
	.ignore_unset_ecc = 1,
};

/* DM9000AEP 10/100 ethernet controller */
#define MACH_MEZ1500_DM9K_BASE (S3C2410_CS4 + 0x300)

static struct resource MEZ1500_dm9k_resource[] = {
        [0] = {
                .start = MACH_MEZ1500_DM9K_BASE,
                .end   = MACH_MEZ1500_DM9K_BASE + 3,
                .flags = IORESOURCE_MEM
        },
        [1] = {
                .start = MACH_MEZ1500_DM9K_BASE + 4,
                .end   = MACH_MEZ1500_DM9K_BASE + 7,
                .flags = IORESOURCE_MEM
        },
        [2] = {
                .start = IRQ_EINT7,
                .end   = IRQ_EINT7,
                .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
        }
};

/*
 *  * The DM9000 has no eeprom, and it's MAC address is set by
 *   * the bootloader before starting the kernel.
 *    */
static struct dm9000_plat_data MEZ1500_dm9k_pdata = {
        .flags          = (DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM),
};

static struct platform_device MEZ1500_device_eth = {
        .name           = "dm9000",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(MEZ1500_dm9k_resource),
        .resource       = MEZ1500_dm9k_resource,
        .dev            = {
                .platform_data  = &MEZ1500_dm9k_pdata,
        },
};

/* MMC/SD  */

static struct s3c24xx_mci_pdata MEZ1500_mmc_cfg = {
   .gpio_detect   = S3C2410_GPG(8),
   .gpio_wprotect = S3C2410_GPH(8),
   .set_power     = NULL,
   .ocr_avail     = MMC_VDD_32_33|MMC_VDD_33_34,
};


/* devices we initialise */

static struct platform_device *MEZ1500_devices[] __initdata = {
	&s3c_device_usb,
	&s3c_device_rtc,
	&s3c_device_lcd,
	&s3c_device_wdt,
	&s3c_device_i2c0,
	&s3c_device_iis,
	&MEZ1500_device_eth,
	&s3c24xx_uda134x,
	&s3c_device_nand,
	&s3c_device_sdi,
	
//-----------------------------------------------------------------------------
// 20140408 SV: Added
	&s3c_device_spi0,
//-----------------------------------------------------------------------------	
	
	&s3c_device_usbgadget,
};

static void __init MEZ1500_map_io(void)
{
	s3c24xx_init_io(MEZ1500_iodesc, ARRAY_SIZE(MEZ1500_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(MEZ1500_uartcfgs, ARRAY_SIZE(MEZ1500_uartcfgs));
}

static void __init MEZ1500_machine_init(void)
{
#if defined (LCD_WIDTH)
	s3c24xx_fb_set_platdata(&MEZ1500_fb_info);
#endif
	s3c_i2c0_set_platdata(NULL);

// 2011-01-18 SV: Commented out
//	s3c2410_gpio_cfgpin(S3C2410_GPC(0), S3C2410_GPC0_LEND);

	s3c_device_nand.dev.platform_data = &friendly_arm_nand_info;
	s3c_device_sdi.dev.platform_data = &MEZ1500_mmc_cfg;
	
	
//-----------------------------------------------------------------------------
// 20140408 SV: Added
	s3c_device_spi0.dev.platform_data=&s3c2410_spi0_platdata; 
	spi_register_board_info(s3c2410_spi0_board,ARRAY_SIZE(s3c2410_spi0_board));	
//-----------------------------------------------------------------------------	
	
	
	platform_add_devices(MEZ1500_devices, ARRAY_SIZE(MEZ1500_devices));
	
// 2011-04-15 SV: Added
	mez1500_init();
	
	s3c_pm_init();
	
}

MACHINE_START(MEZ1500, "Aceeca MEZ1500 development board")
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,

	.init_irq	= s3c24xx_init_irq,
	.map_io		= MEZ1500_map_io,
	.init_machine	= MEZ1500_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END

