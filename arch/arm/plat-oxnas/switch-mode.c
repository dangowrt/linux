/*
 * linux/arch/arm/plat-oxnas/switch-mode.c
 *
 * Copyright (C) 2010 PLX Technology Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <mach/hardware.h>

MODULE_LICENSE("GPL v2");

#if (CONFIG_OXNAS_MODE_SWITCH_GPIO < SYS_CTRL_NUM_PINS) 
#define SWITCH_NUM          CONFIG_OXNAS_MODE_SWITCH_GPIO
#define IRQ_NUM             GPIOA_INTERRUPT
#define INT_STATUS_REG      GPIO_A_INTERRUPT_EVENT
#define SWITCH_CLR_OE_REG   GPIO_A_OUTPUT_ENABLE_CLEAR
#define DEBOUNCE_REG        GPIO_A_INPUT_DEBOUNCE_ENABLE
#define LEVEL_INT_REG       GPIO_A_LEVEL_INTERRUPT_ENABLE
#define FALLING_INT_REG     GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE
#define RISING_INT_REG      GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE
#define DATA_REG            GPIO_A_DATA
#define RISING_EDGE_DET	    GPIO_A_RISING_EDGE_DETECT
#define FALLING_EDGE_DET    GPIO_A_FALLING_EDGE_DETECT
#else
#define SWITCH_NUM          (CONFIG_OXNAS_MODE_SWITCH_GPIO - SYS_CTRL_NUM_PINS)
#define IRQ_NUM             GPIOB_INTERRUPT
#define INT_STATUS_REG      GPIO_B_INTERRUPT_EVENT
#define SWITCH_CLR_OE_REG   GPIO_B_OUTPUT_ENABLE_CLEAR
#define DEBOUNCE_REG        GPIO_B_INPUT_DEBOUNCE_ENABLE
#define LEVEL_INT_REG       GPIO_B_LEVEL_INTERRUPT_ENABLE
#define FALLING_INT_REG     GPIO_B_FALLING_EDGE_ACTIVE_LOW_ENABLE
#define RISING_INT_REG      GPIO_B_RISING_EDGE_ACTIVE_HIGH_ENABLE
#define DATA_REG            GPIO_B_DATA
#define RISING_EDGE_DET     GPIO_B_RISING_EDGE_DETECT
#define FALLING_EDGE_DET    GPIO_B_FALLING_EDGE_DETECT
#endif

#define SWITCH_MASK         (1UL << (SWITCH_NUM))

extern spinlock_t oxnas_gpio_spinlock;

//#define DEBUG

#ifdef DEBUG
#define db_print(...) printk(__VA_ARGS__)
#else
#define db_print(...) 
#endif 

static char state;

static void work_handler(struct work_struct * not_used);
DECLARE_WORK(mode_switch_hotplug_work, work_handler);

/** Have to use active low level interupt generation, as otherwise might miss
 *  interrupts that arrive concurrently with a PCI interrupt, as PCI interrupts
 *  are generated via GPIO pins and std PCI drivers will not know that there
 *  may be other pending GPIO interrupt sources waiting to be serviced and will
 *  simply return IRQ_HANDLED if they see themselves as having generated the
 *  interrupt, thus preventing later chained handlers from being called
 */
static irqreturn_t int_handler(int irq, void* dev_id)
{
	int status = IRQ_NONE;
	unsigned int int_status = readl((volatile unsigned long *)INT_STATUS_REG);
	

	/* Is the interrupt for us? */
	if (int_status & SWITCH_MASK) {
		
		db_print(KERN_INFO "interrupt detected 0x%08x\n", int_status);
		spin_lock(&oxnas_gpio_spinlock);
		/* disable both interupts */
		writel(readl(RISING_INT_REG) & ~SWITCH_MASK, RISING_INT_REG);
		writel(readl(FALLING_INT_REG) & ~SWITCH_MASK, FALLING_INT_REG);
		
		/* clear edge latches */
		writel(SWITCH_MASK, RISING_EDGE_DET);
		writel(SWITCH_MASK, FALLING_EDGE_DET);
		
		if (readl(DATA_REG) & SWITCH_MASK ) {
			db_print(KERN_INFO "switch released sig high\n");
			/* enable the switch GPIO falling line interrupt */
			writel(readl(FALLING_INT_REG) | SWITCH_MASK, FALLING_INT_REG);
			state=1;
		} else {
			db_print(KERN_INFO "switch pressed sig low\n");
			/* enable the mode switch GPIO riseing line interrupt */
			writel(readl(RISING_INT_REG) | SWITCH_MASK, RISING_INT_REG);
			state=0;
		}
		spin_unlock(&oxnas_gpio_spinlock);
		schedule_work(&mode_switch_hotplug_work);
		
		/* Only mark interrupt as serviced if no other unmasked GPIO interrupts
		are pending */
		if (!readl((volatile unsigned long *)INT_STATUS_REG)) {
			status = IRQ_HANDLED;
		}
	}

	return status;
}

/*
 * Device driver object
 */
typedef struct mode_switch_driver_s {
	/** sysfs dir tree root for mode switch driver */
	struct kset   *kset;
	struct kobject  kobject;
} mode_switch_driver_t;

static mode_switch_driver_t mode_switch_driver;

static void work_handler(struct work_struct * not_used) {
	kobject_uevent(&mode_switch_driver.kobject, state ? KOBJ_OFFLINE:KOBJ_ONLINE);
}
 
static struct kobj_type ktype_mode_switch = {
	.release = 0,
	.sysfs_ops = 0,
	.default_attrs = 0,
};

static int mode_switch_hotplug_filter(struct kset* kset, struct kobject* kobj) {
	return get_ktype(kobj) == &ktype_mode_switch;
}

static const char* mode_switch_hotplug_name(struct kset* kset, struct kobject* kobj) {
	return "oxnas_mode_switch";
}

static struct kset_uevent_ops mode_switch_uevent_ops = {
	.filter = mode_switch_hotplug_filter,
	.name   = mode_switch_hotplug_name,
	.uevent = NULL,
};

static int __init mode_switch_init(void)
{
	int err = 0;
	unsigned long flags;
	unsigned int input_stat;

	/* Prepare the sysfs interface for use */
	mode_switch_driver.kset = kset_create_and_add("mode_switch", &mode_switch_uevent_ops, kernel_kobj);
	if (!mode_switch_driver.kset) {
		printk(KERN_ERR "mode_switch_init() Failed to create kset\n");
		return -ENOMEM;
	}

	mode_switch_driver.kobject.kset = mode_switch_driver.kset;
	err = kobject_init_and_add(&mode_switch_driver.kobject,
		&ktype_mode_switch, NULL, "%d", 0);
	if (err) {
		printk(KERN_ERR "mode_switch_init() Failed to add kobject\n");
		kset_unregister(mode_switch_driver.kset);
		kobject_put(&mode_switch_driver.kobject);
		return -EINVAL;
	}

	/* Install a shared interrupt handler on the appropriate GPIO bank's
	   interrupt line */
	if (request_irq(IRQ_NUM, int_handler, IRQF_SHARED, "Switch mode", &mode_switch_driver)) {
		printk(KERN_ERR "Switch mode: cannot register IRQ %d\n", IRQ_NUM);
		return -EIO;
	}

	spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
	/* Disable primary, secondary and teriary GPIO functions on switch lines */

#if (CONFIG_OXNAS_MODE_SWITCH_GPIO < SYS_CTRL_NUM_PINS)
    	writel(readl(SYS_CTRL_SECONDARY_SEL)   & ~(SWITCH_MASK), SYS_CTRL_SECONDARY_SEL);
    	writel(readl(SYS_CTRL_TERTIARY_SEL)    & ~(SWITCH_MASK), SYS_CTRL_TERTIARY_SEL);
    	writel(readl(SYS_CTRL_QUATERNARY_SEL)  & ~(SWITCH_MASK), SYS_CTRL_QUATERNARY_SEL);
    	writel(readl(SYS_CTRL_DEBUG_SEL)       & ~(SWITCH_MASK), SYS_CTRL_DEBUG_SEL);
    	writel(readl(SYS_CTRL_ALTERNATIVE_SEL) & ~(SWITCH_MASK), SYS_CTRL_ALTERNATIVE_SEL);
#else
    	writel(readl(SEC_CTRL_SECONDARY_SEL)   & ~(SWITCH_MASK), SEC_CTRL_SECONDARY_SEL);
    	writel(readl(SEC_CTRL_TERTIARY_SEL)    & ~(SWITCH_MASK), SEC_CTRL_TERTIARY_SEL);
    	writel(readl(SEC_CTRL_QUATERNARY_SEL)  & ~(SWITCH_MASK), SEC_CTRL_QUATERNARY_SEL);
    	writel(readl(SEC_CTRL_DEBUG_SEL)       & ~(SWITCH_MASK), SEC_CTRL_DEBUG_SEL);
    	writel(readl(SEC_CTRL_ALTERNATIVE_SEL) & ~(SWITCH_MASK), SEC_CTRL_ALTERNATIVE_SEL);
#endif


	/* Enable GPIO input on switch line */
	writel(SWITCH_MASK, SWITCH_CLR_OE_REG);
	
	/* ensure interupts not pending in latches */
	writel(SWITCH_MASK, RISING_EDGE_DET);
	writel(SWITCH_MASK, FALLING_EDGE_DET);

	/* Set up the mode switch GPIO line for debounced interrupt */
	writel(readl(DEBOUNCE_REG)    | SWITCH_MASK, DEBOUNCE_REG);	

	/* read gpio input status */
	input_stat = (readl(DATA_REG) & SWITCH_MASK );
	if (input_stat) {
		state = 1;
		/* Set up the mode switch GPIO line for active low*/
		writel(readl(FALLING_INT_REG) | SWITCH_MASK, FALLING_INT_REG);
	} else {
		state = 0;
		/* Set up the mode switch GPIO line for active hogh*/
		writel(readl(RISING_INT_REG) | SWITCH_MASK, RISING_INT_REG);	
	}
	
	spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

	schedule_work(&mode_switch_hotplug_work);
	
	printk(KERN_INFO "Switch Mode driver registered\n");
	return 0;
}

static void __exit mode_switch_exit(void)
{
	unsigned long flags;

	kobject_put(&mode_switch_driver.kobject);
	kset_unregister(mode_switch_driver.kset);


	/* Disable interrupt generation by the mode switch GPIO line */
	spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
	writel(readl(FALLING_INT_REG) & ~SWITCH_MASK, FALLING_INT_REG);
	spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

	/* Remove the handler for the shared interrupt line */
	free_irq(IRQ_NUM, &mode_switch_driver);
}

/** 
 * macros to register intiialisation and exit functions with kernal
 */
module_init(mode_switch_init);
module_exit(mode_switch_exit);
