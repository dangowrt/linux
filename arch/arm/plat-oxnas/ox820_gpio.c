 /* Copyright (C) 2010 PLX Technology Inc
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

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>

#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/delay.h>

#define DRIVER_NAME "gpio"

static char mac_adr[6];
static DEFINE_MUTEX(ox820_gpio_lock);

static char *INPUT_MASK = "0:0";
module_param(INPUT_MASK, charp, S_IRUGO|S_IWUSR);

static char *OUTPUT_MASK = "0:0";
module_param(OUTPUT_MASK, charp, S_IRUGO|S_IWUSR);

static u_int32_t input_a = 0;
static u_int32_t input_b = 0;
static u_int32_t output_a = 0;
static u_int32_t output_b = 0;

//#define DEBUG 
#ifdef DEBUG
#define db_print(...) printk(__VA_ARGS__)
#else
#define db_print(...)
#endif

static int gpio_setup(void)
{

	db_print("INPUT_MASK = %s\n", INPUT_MASK);
	db_print("OUTPUT_MASK = %s\n", OUTPUT_MASK);
	
	if (sscanf(INPUT_MASK, "%x:%x", &input_a, &input_b) != 2)
		return -EINVAL;

	if (sscanf(OUTPUT_MASK, "%x:%x", &output_a, &output_b) != 2)
		return -EINVAL;

	db_print("INPUT A = 0x%08x\n", input_a);
	db_print("INPUT B = 0x%08x\n", input_b);
	db_print("OUTPUT A = 0x%08x\n", output_a);
	db_print("OUTPUT B = 0x%08x\n", output_b);
	
	if (mutex_lock_interruptible(&ox820_gpio_lock))
                return -ERESTARTSYS;

	/* Disable primary, secondary and teriary GPIO functions */
	writel(readl(SYS_CTRL_SECONDARY_SEL)   & ~(input_a | output_a), SYS_CTRL_SECONDARY_SEL);
	writel(readl(SYS_CTRL_TERTIARY_SEL)    & ~(input_a | output_a), SYS_CTRL_TERTIARY_SEL);
	writel(readl(SYS_CTRL_QUATERNARY_SEL)  & ~(input_a | output_a), SYS_CTRL_QUATERNARY_SEL);
	writel(readl(SYS_CTRL_DEBUG_SEL)       & ~(input_a | output_a), SYS_CTRL_DEBUG_SEL);
	writel(readl(SYS_CTRL_ALTERNATIVE_SEL) & ~(input_a | output_a), SYS_CTRL_ALTERNATIVE_SEL);
	// Setup 2nd input source
	writel(readl(SEC_CTRL_SECONDARY_SEL)   & ~(input_b | output_b), SEC_CTRL_SECONDARY_SEL);
	writel(readl(SEC_CTRL_TERTIARY_SEL)    & ~(input_b | output_b), SEC_CTRL_TERTIARY_SEL);
	writel(readl(SEC_CTRL_QUATERNARY_SEL)  & ~(input_b | output_b), SEC_CTRL_QUATERNARY_SEL);
	writel(readl(SEC_CTRL_DEBUG_SEL)       & ~(input_b | output_b), SEC_CTRL_DEBUG_SEL);
	writel(readl(SEC_CTRL_ALTERNATIVE_SEL) & ~(input_b | output_b), SEC_CTRL_ALTERNATIVE_SEL);

	/* Enable GPIO input  */
	/* Enable GPIO input  */
	writel((input_a), GPIO_A_OUTPUT_ENABLE_CLEAR);
	writel((input_b), GPIO_B_OUTPUT_ENABLE_CLEAR);
	
	/* Enable GPIO output */
	writel(output_a, GPIO_A_OUTPUT_CLEAR);
	writel(output_a, GPIO_A_OUTPUT_ENABLE_SET);
	writel(output_b, GPIO_B_OUTPUT_CLEAR);
	writel(output_b, GPIO_B_OUTPUT_ENABLE_SET);

	mutex_unlock(&ox820_gpio_lock);

	return 0;
}

/*
 *  ox820_gpio_read - Read OTP pages
 *
 */
static ssize_t ox820_gpio_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	uint32_t val;
	int i, len = 0;
	char contents[300];
	char *p = contents, *p_tail = contents + 300;
	

//	db_print("%s():\n", __FUNCTION__);

	if (mutex_lock_interruptible(&ox820_gpio_lock))
		return -ERESTARTSYS;

	/* Read GPIO_A */
	val = readl(GPIO_A_DATA);

	p += snprintf(p, p_tail - p, "INPUT A:(MSB)");	
//	db_print("Data read back = 0x%08x\n", val);
	for (i = 31; i >= 0; i--) {
		if (input_a & (0x1UL << i)) {
			p += snprintf(p, p_tail - p, "%s", ((val >> i) & 0x1UL) ? "1" : "0");
		} else {
			p += snprintf(p, p_tail - p, ".");
		}
		if ( i % 4 == 0 )
			p += snprintf(p, p_tail - p, " | ");
	}

	p += snprintf(p, p_tail - p, "\nINPUT B:(MSB)");
	/* Read GPIO_B */
	val = readl(GPIO_B_DATA) ;

//	db_print("Data read back = 0x%08x\n", val);
	for (i = 31; i >=0; i--) {
		if (input_b & (0x1UL << i)) {
			p += snprintf(p, p_tail - p, "%s", ((val >> i) & 0x1UL) ? "1" : "0");
		} else {	
			p += snprintf(p, p_tail - p, ".");
		}
		if ( i % 4 == 0 )
                        p += snprintf(p, p_tail - p, " | ");
	}

	p += snprintf(p, p_tail - p, "\n");

	len = (p - contents) - *pos;

	if (len < 0)
		len = 0;

	count = (count >= len) ? len : count;

	if (copy_to_user(buf, contents + *pos, count)) {
		mutex_unlock(&ox820_gpio_lock);
                return -EFAULT;
        }

	*pos += count;		
	mutex_unlock(&ox820_gpio_lock);

	return count;

}

/*
 *  ox820_gpio_write - Write OTP pages
 *  .
 */
static ssize_t ox820_gpio_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos)
{
	char data[128];
	int i;
	unsigned int gpio_num, val;
	char gpio_ch;

	db_print("%s():\n", __FUNCTION__);

	if (count > sizeof(data)-1) {
        	return -EINVAL;
	}

	if (mutex_lock_interruptible(&ox820_gpio_lock))
		return -ERESTARTSYS;

	if (copy_from_user(data, buf, count)) {
		return -EFAULT;
	}
	
	db_print("Data write = %d\n", count);

#ifdef DEBUG
	for (i=0; i < count; i++)
		db_print("Data write = %x\n", data[i]);
#endif

	if (sscanf(data, "%c:%u:%u", &gpio_ch, &gpio_num, &val) != 3)
		return -EINVAL;

	db_print("GPIO A/B = %d\n", gpio_ch);
	db_print("GPIO write = %d\n", gpio_num);
	db_print("GPIO value = %d\n", val);

	if ((gpio_ch == 0x61) || (gpio_ch == 0x41)) {
		db_print("Output GPIO A\n");
		if (output_a & (0x1UL << gpio_num)) {
			val ? writel((0x1UL << gpio_num), GPIO_A_OUTPUT_SET) : writel((0x1UL << gpio_num), GPIO_A_OUTPUT_CLEAR);
		} else {
			printk(KERN_INFO "Wrong GPIO A number!!\n");
		}
	} else if ((gpio_ch == 0x62) || (gpio_ch == 0x42)) {
		db_print("Output GPIO B\n");
		if (output_b & (0x1UL << gpio_num)) {
			val ? writel((0x1UL << gpio_num), GPIO_B_OUTPUT_SET) : writel((0x1UL << gpio_num), GPIO_B_OUTPUT_CLEAR);
		} else {
			printk(KERN_INFO "Wrong GPIO B number!!\n");
		}
	} else {
		printk(KERN_INFO "Wrong GPIO channel!!\n");
		return -EINVAL;
	}

	mutex_unlock(&ox820_gpio_lock);

	*pos = count;

	return count;
}

static struct file_operations ox820_gpio_fops = {
	.owner    = THIS_MODULE,
	.read     = ox820_gpio_read,
	.write    = ox820_gpio_write,
};

static struct miscdevice ox820_gpio_misc_device = {
	.minor    = MISC_DYNAMIC_MINOR,
	.name     = DRIVER_NAME,
	.fops     = &ox820_gpio_fops,
};

/*
 *  ox820_gpio_init - Initialize module
 *
 */
static int __init ox820_gpio_init(void)
{
	int ret;
	struct proc_dir_entry *ent;

	printk("%s():\n", __FUNCTION__);

	ret = misc_register(&ox820_gpio_misc_device);
	if (ret) {
		printk(KERN_INFO "unable to register a misc device\n");
		return ret;
	}

	printk(KERN_INFO "OX820 GPIO initialized\n");

	ret = gpio_setup();
	if (ret) {
		printk(KERN_INFO "failedto setup ox820 GPIO\n");
		return ret;
	}
	return 0;
}

/*
 *  ox820_gpio_exit - Deinitialize module
 *  
 */
static void __exit ox820_gpio_exit(void)
{
	printk("%s():\n", __FUNCTION__);
	misc_deregister(&ox820_gpio_misc_device);
	remove_proc_entry("ox820_gpio", NULL);
}

module_init(ox820_gpio_init);
module_exit(ox820_gpio_exit);

MODULE_DESCRIPTION("OX820 GPIO Test driver");
MODULE_LICENSE("GPL");

