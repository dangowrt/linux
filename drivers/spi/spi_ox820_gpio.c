/* linux/drivers/spi/spi_ox820_gpio.c
 *
 * Copyright (c) 2010 John Larkworthy
 * Copyright (c) 2010 PLX Technology Inc
 *
 * OX820 GPIO based SPI driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <linux/autoconf.h>
#include <asm/io.h>
#include <mach/hardware.h>

#define OX820_GPIO_OUTPUT     1
#define OX820_GPIO_INPUT      0

struct ox820_spigpio_info {
	unsigned long		 pin_clk;
	unsigned long		 pin_mosi;
	unsigned long		 pin_miso;

	int			 num_chipselect;
	int			 bus_num;

	void (*chip_select)(struct ox820_spigpio_info *spi, int cs);
};

struct ox820_spigpio {
	struct spi_bitbang		 bitbang;

	struct ox820_spigpio_info	*info;
	struct platform_device		*dev;
};

static void ox820_gpio_setpin(int pin, int state) {
	if (pin < 32) {
		writel( (1<<pin),(state==0 ? GPIO_A_OUTPUT_CLEAR: GPIO_A_OUTPUT_SET));
	} else {
		writel((1<<(pin-32)),(state==0 ? GPIO_B_OUTPUT_CLEAR: GPIO_B_OUTPUT_SET));		
	}
	
}
static int ox820_gpio_getpin(int pin)
{
	if(pin < 32) return (1 & (readl(GPIO_A_DATA) >>pin));
	else return (1 & (readl(GPIO_B_DATA) >> (pin-32)));

}
static void ox820_gpio_cfgpin(int pin, int direction) {
	 ;
	
	if (pin < 32) {
		writel((1<<pin), direction == OX820_GPIO_OUTPUT ? GPIO_A_OUTPUT_ENABLE_SET : GPIO_A_OUTPUT_ENABLE_CLEAR);
	} else {
		writel((1<<(pin-32)), direction == OX820_GPIO_OUTPUT ? GPIO_B_OUTPUT_ENABLE_SET : GPIO_B_OUTPUT_ENABLE_CLEAR);
	}
	if(direction == OX820_GPIO_INPUT) {
		if (pin < 32) writel( readl(GPIO_A_INPUT_DEBOUNCE_ENABLE) & ~(1<<pin) , GPIO_A_INPUT_DEBOUNCE_ENABLE);
		else writel( readl(GPIO_B_INPUT_DEBOUNCE_ENABLE) & ~(1<<(pin-32)) , GPIO_B_INPUT_DEBOUNCE_ENABLE);
	}
}

static inline struct ox820_spigpio *spidev_to_sg(struct spi_device *spi)
{
	return spi_master_get_devdata(spi->master);
}

static inline void setsck(struct spi_device *dev, int on)
{
	ox820_gpio_setpin(CONFIG_OXNAS_SPI_CLK, on ? 1 : 0);
}

static inline void setmosi(struct spi_device *dev, int on)
{
	ox820_gpio_setpin(CONFIG_OXNAS_SPI_MOSI, on ? 1 : 0);
}

static inline u32 getmiso(struct spi_device *dev)
{
	return ox820_gpio_getpin(CONFIG_OXNAS_SPI_MISO) ? 1 : 0;
}

#define spidelay(x) ndelay(x)

#ifdef CONFIG_SPI_DEBUG
#define DEBUG_OX(...) printk(KERN_INFO __VA_ARGS__) 
#else
#define DEBUG_OX(...)
#endif

#define	EXPAND_BITBANG_TXRX
#include <linux/spi/spi_bitbang.h>


static u32 ox820_spigpio_txrx_mode0(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, word, bits);
}

static u32 ox820_spigpio_txrx_mode1(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 0, word, bits);
}

static u32 ox820_spigpio_txrx_mode2(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 1, word, bits);
}

static u32 ox820_spigpio_txrx_mode3(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 1, word, bits);
}


static void ox820_spigpio_chipselect(struct spi_device *dev, int value)
{

	DEBUG_OX( "spi chip select called:%d\n", value);
	ox820_gpio_setpin(CONFIG_OXNAS_SPI_CS, (value==0? 1 : 0) );
	DEBUG_OX("GPIO A STATUS 0x%08x\n", readl(GPIO_A_OUTPUT_VALUE));
	DEBUG_OX("GPIO B STATUS 0x%08x\n", readl(GPIO_B_OUTPUT_VALUE));
}


static int ox820_spigpio_probe(struct platform_device *dev)
{
	struct ox820_spigpio_info *info;
	struct spi_master	*master;
	struct ox820_spigpio  *sp;
	int ret;

	DEBUG_OX( "ox820 spi driver probe called\n");
	master = spi_alloc_master(&dev->dev, sizeof(struct ox820_spigpio));
	if (master == NULL) {
		dev_err(&dev->dev, "failed to allocate spi master\n");
		ret = -ENOMEM;
		goto err;
	}

	sp = spi_master_get_devdata(master);

	platform_set_drvdata(dev, sp);

	/* copy in the plkatform data */
	info = sp->info = dev->dev.platform_data;

	/* setup spi bitbang adaptor */
	sp->bitbang.master = spi_master_get(master);
	sp->bitbang.master->bus_num = 0;
	sp->bitbang.master->num_chipselect = 1;
	sp->bitbang.chipselect = ox820_spigpio_chipselect;

	sp->bitbang.txrx_word[SPI_MODE_0] = ox820_spigpio_txrx_mode0;
	sp->bitbang.txrx_word[SPI_MODE_1] = ox820_spigpio_txrx_mode1;
	sp->bitbang.txrx_word[SPI_MODE_2] = ox820_spigpio_txrx_mode2;
	sp->bitbang.txrx_word[SPI_MODE_3] = ox820_spigpio_txrx_mode3;

	/* set state of spi pins, always assume that the clock is
	 * available, but do check the MOSI and MISO. */
	ox820_gpio_setpin(CONFIG_OXNAS_SPI_CLK, 0);
	ox820_gpio_cfgpin(CONFIG_OXNAS_SPI_CLK, OX820_GPIO_OUTPUT);

	ox820_gpio_setpin(CONFIG_OXNAS_SPI_MOSI, 0);
	ox820_gpio_cfgpin(CONFIG_OXNAS_SPI_MOSI, OX820_GPIO_OUTPUT);

	ox820_gpio_cfgpin(CONFIG_OXNAS_SPI_MISO, OX820_GPIO_INPUT);
	ox820_gpio_cfgpin(CONFIG_OXNAS_SPI_CS, OX820_GPIO_OUTPUT);

	ret = spi_bitbang_start(&sp->bitbang);
	if (ret)
		goto err_no_bitbang;

	DEBUG_OX( "ox820 spi driver probe successful\n");
	DEBUG_OX("configuration gpio a output 0x%08x\n", readl(GPIO_A_OUTPUT_ENABLE));
	DEBUG_OX("configuration mfa sec sel 0x%08x\n", readl(SYS_CTRL_SECONDARY_SEL));
	DEBUG_OX("configuration mfa ter sel 0x%08x\n", readl(SYS_CTRL_TERTIARY_SEL));
	DEBUG_OX("configuration mfa quad sel 0x%08x\n", readl(SYS_CTRL_QUATERNARY_SEL));
	DEBUG_OX("configuration mfa debug sel 0x%08x\n", readl(SYS_CTRL_DEBUG_SEL));
	DEBUG_OX("configuration mfa alt sel 0x%08x\n", readl(SYS_CTRL_ALTERNATIVE_SEL));

	DEBUG_OX("configureation gpio output enable 0x%08x\n", readl(GPIO_A_OUTPUT_ENABLE));
	DEBUG_OX("configuration gpio output state 0x%08x\n", readl(GPIO_A_OUTPUT_VALUE));
	DEBUG_OX("configuration gpio data state 0x%08x\n", readl(GPIO_A_DATA));
	DEBUG_OX("configuration gpio debounce 0x%08x\n", readl(GPIO_A_INPUT_DEBOUNCE_ENABLE));

	return 0;

 err_no_bitbang:
 	DEBUG_OX("what went wrong? chipselect %p, master %p, setup %p\n",
 		sp->bitbang.chipselect, sp->bitbang.master, sp->bitbang.master->setup);
	spi_master_put(sp->bitbang.master);
 err:
 	DEBUG_OX( "ox820 spi driver probe failed\n");

	return ret;

}

static int ox820_spigpio_remove(struct platform_device *dev)
{
	struct ox820_spigpio *sp = platform_get_drvdata(dev);

	spi_bitbang_stop(&sp->bitbang);
	spi_master_put(sp->bitbang.master);

	return 0;
}

/* all gpio should be held over suspend/resume, so we should
 * not need to deal with this
*/

#define ox820_spigpio_suspend NULL
#define ox820_spigpio_resume NULL

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:spi_ox820_gpio");

static struct platform_driver ox820_spigpio_drv = {
	.probe		= ox820_spigpio_probe,
        .remove		= ox820_spigpio_remove,
        .suspend	= ox820_spigpio_suspend,
        .resume		= ox820_spigpio_resume,
        .driver		= {
		.name	= "spi_ox820_gpio",
		.owner	= THIS_MODULE,
        },
};

static int __init ox820_spigpio_init(void)
{
	unsigned long mask = (1UL << CONFIG_OXNAS_SPI_CLK | 1UL << CONFIG_OXNAS_SPI_CS | 1UL << CONFIG_OXNAS_SPI_MISO | 1UL << CONFIG_OXNAS_SPI_MOSI);

	DEBUG_OX( "ox820 spi driver intialised\n");
	/* Disable primary, secondary and teriary GPIO functions on SPI pins. */
	writel(readl(SYS_CTRL_SECONDARY_SEL)   & ~mask, SYS_CTRL_SECONDARY_SEL);
    	writel(readl(SYS_CTRL_TERTIARY_SEL)    & ~mask, SYS_CTRL_TERTIARY_SEL);
    	writel(readl(SYS_CTRL_QUATERNARY_SEL)  & ~mask, SYS_CTRL_QUATERNARY_SEL);
    	writel(readl(SYS_CTRL_DEBUG_SEL)       & ~mask, SYS_CTRL_DEBUG_SEL);
    	writel(readl(SYS_CTRL_ALTERNATIVE_SEL) & ~mask, SYS_CTRL_ALTERNATIVE_SEL);
    	writel(readl(GPIO_A_INPUT_DEBOUNCE_ENABLE) & ~mask, GPIO_A_INPUT_DEBOUNCE_ENABLE);
	
        return platform_driver_register(&ox820_spigpio_drv);
}

static void __exit ox820_spigpio_exit(void)
{
        platform_driver_unregister(&ox820_spigpio_drv);
}

module_init(ox820_spigpio_init);
module_exit(ox820_spigpio_exit);

MODULE_DESCRIPTION("OX820 SPI Driver");
MODULE_AUTHOR("John Larkworthy jlarkworthy@plxtech.com");
MODULE_LICENSE("GPL");
