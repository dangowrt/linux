/*******************************************************************************
GPL License Option

If you received this File from somewhere, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the General
Public License Version 2, June 1991 (the "GPL License"), a copy of which is
available along with the File in the license.txt file or by writing to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
on the worldwide web at http://www.gnu.org/licenses/gpl.txt.

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
DISCLAIMED.  The GPL License provides additional details about this warranty
disclaimer.
*******************************************************************************/
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/time.h>			// -> [J.Chiang], 2011/06/18 - For setting minuteswest values.
#include <asm/uaccess.h>
#include <asm/ioctl.h>
#include <mach/hardware.h>
#include <mach/rps-irq.h>
#include "../mach-ox820/btns_dev.h"
#include "btns_driver.h"
#include <linux/delay.h>


// -> [Walker Chen], 2010/08/12 - golbal variables, "arch/arm/mach-ox820/rps-timer.c"
static u32 pwr_long_push_time = 0;
static u32 fctry_long_push_time = 0;
static u32 HD1_BTN_long_push_time = 0;
static u32 HD2_BTN_long_push_time = 0;

extern unsigned long system_B_brightness ;
extern unsigned long system_R_brightness ;
extern unsigned long HD1_B_brightness ;
extern unsigned long HD1_R_brightness ;
extern unsigned long HD2_B_brightness ;
extern unsigned long HD2_R_brightness ;
extern unsigned long USB_brightness ;

extern unsigned int breathing_led_on;

extern int sys_ready;
extern int sys_error;
extern int sata1_err;
extern int sata2_err;
// <- End.

// -> [J.Chiang], 2012/05/09 - Added for supporting surprise removing HDDs
int surprise_remove = 0;
EXPORT_SYMBOL(surprise_remove);
// <- End.

/* waiting Q */
wait_queue_head_t btns_waitq;

/*
 * common debug for all
 */
// -> [J.Chiang], Modified for debugging
#undef DEBUG
//#define DEBUG
extern struct timezone sys_tz;		// -> [J.Chiang], 2011/06/18 - For setting minuteswest values.
// <- End.
#ifdef DEBUG
#define dprintk   printk
#else
#define dprintk(a...)
#endif

/* At GPP initialization, this strucure is filled with what
 * operation will be monitored for each button (Push and/or
 * Release) */
BTN_OP 	btn_op_cfg[CONFIG_GPP_MAX_PINS] = {BTN_NO_OP};

/* At GPP initialization, this strucure is filled with what
 * operation will be monitored for each button (Push and/or
 * Release) */
u32 	gpp_default_val_cfg[CONFIG_GPP_MAX_PINS] = {-1};

/* This structures monitors how many time each button was
 * Push/Released since the last time it was sampled */
BTN		btns_status[CONFIG_GPP_MAX_PINS];

u32		is_opend = 0;
u32		gpp_changed = 0;
u32		gpp_changed_id = -1;

static irqreturn_t
mv_btns_handler(int irq , void *dev_id)
{
	u32 gpp = (u32) irq - OX820_GPIO_IRQ_START;
	u32 gppVal, gppValB;
	BTN_OP btn_op;

	/* get gpp real val */
	gppVal = readl((volatile unsigned long *)GPIO_A_DATA);
	gppValB = readl((volatile unsigned long *)GPIO_B_DATA);
	// -> [J.Chiang], 2010/06/04 - Added for passing GPP value because it will be clear after interrupt is served.
	btns_status[gpp].gppVal=gppVal;
	// <- End.

	dprintk("Gpp value was changed: ");
        if (btn_op_cfg[gpp] != BTN_NO_OP) {
                dprintk("gpp %d has changed. now it's %x -> %x \n",gpp, gppVal, gppVal & (1 << gpp));

		gppVal &= 1 << gpp;
		/* Count button Pushes/Releases
		 * mv_gpp_value_real_get == gpp_default_val_cfg[gpp] --> Button push,
		 * else --> Button release
		 */
		btn_op = (gppVal == gpp_default_val_cfg[gpp]) ? BTN_PUSH : BTN_RELEASE;

		if(btn_op == BTN_RELEASE)
		{
			dprintk("button (of gpp %d) was released \n",gpp);

			btns_status[gpp].btn_release_cntr++;
			// -> [J.Chiang], 2012/05/09 - Added for supporting surprise removing HDDs
			if (( gpp == 28) || ( gpp ==29 ))
				surprise_remove = 0;
			// <- End.
		} else {
			dprintk("button (of gpp %d) was pressed \n",gpp);

			btns_status[gpp].btn_push_cntr++;
			// -> [J.Chiang], 2012/05/09 - Added for supporting surprise removing HDDs
			if ( gpp == 28)
				dprintk("GPIO_B_GROUP=0x%x\n", gppValB);
			if ((( gpp == 28 ) && ( gppValB & 0x01 )) || (( gpp == 29 ) && ( gppValB & 0x02 )))
				surprise_remove = 1;
			// <- End.
		}

		/* Check if current botton operation should be monitored */
		if(btn_op_cfg[gpp] == btn_op || btn_op_cfg[gpp] == BTN_CHANGE)
		{
			gpp_changed = 1;
			gpp_changed_id = gpp;
			wake_up_interruptible(&btns_waitq);
		}
	}

	return IRQ_HANDLED;
}

static int
btnsdev_ioctl(
        struct inode *inode,
        struct file *filp,
        unsigned int cmd,
        unsigned long arg)
{
	unsigned int btn_id;
	BTN btn_sts;
	BTN_PTR user_btn_sts_ptr;
	unsigned int error = 0;
	int i;
	u32 gppVal, gpp;


        dprintk("%s()\n", __FUNCTION__);

        switch (cmd) {
        case CIOCWAIT_P:
			/* Haim - Is the condition here correct? */
        	    	error = wait_event_interruptible(btns_waitq, gpp_changed);
			/* Reset Wait Q condition */
			gpp_changed = 0;

			if(error < 0)
				dprintk("%s(CIOCWAIT_P) - got interrupted\n", __FUNCTION__);

			/* Set information for user*/
			btn_sts.btn_id = gpp_changed_id;
			btn_sts.btn_push_cntr   =btns_status[gpp_changed_id].btn_push_cntr;
			btn_sts.btn_release_cntr=btns_status[gpp_changed_id].btn_release_cntr;
			// -> [J.Chiang], 2010/06/04 - Added for passing GPP value because it will be
			//	clear after interrupt is served.
			btn_sts.gppVal=btns_status[gpp_changed_id].gppVal;
			// <- End.

			dprintk("Button ID %d was pressed %d and released %d\n",gpp_changed_id,
				btns_status[gpp_changed_id].btn_push_cntr,btns_status[gpp_changed_id].btn_release_cntr);

			user_btn_sts_ptr = &(((BTNS_STS_PTR)arg)->btns[0]);
			// -> debug
			dprintk("get identify id=0x%x\n", user_btn_sts_ptr->btn_id);
			// <- End.
			if ( copy_to_user((void*)user_btn_sts_ptr, &btn_sts,  sizeof(BTN)) ) {
        	                dprintk("%s(CIOCWAIT_P) - bad copy\n", __FUNCTION__);
        	                error = EFAULT;
        	        }
			// -> debug
			dprintk("get button id=0x%x\n", user_btn_sts_ptr->btn_id);
			// <- End.

			/* Reset changed button operations counters*/
			btns_status[gpp_changed_id].btn_push_cntr = 0;
			btns_status[gpp_changed_id].btn_release_cntr = 0;

            break;

		case CIOCNOWAIT_P:
			/* Eventhough we don't monitor for a button status change, we need to
 				reset the indication of a change in case it happend */
			gpp_changed = 0;

			dprintk("There are %d buttons to be checked\n", ((BTNS_STS_PTR)arg)->btns_number);

			/* Set information for user*/
			for (i=0; i<((BTNS_STS_PTR)arg)->btns_number; i++)
			{
				btn_id = ((BTNS_STS_PTR)arg)->btns[i].btn_id;

				/* initialize temp strucure which will be copied to user */
				btn_sts.btn_id = btn_id;
				btn_sts.btn_push_cntr = btns_status[btn_id].btn_push_cntr;
				btn_sts.btn_release_cntr = btns_status[btn_id].btn_release_cntr;

				/* Reset button's operations counters*/
				btns_status[btn_id].btn_push_cntr = 0;
				btns_status[btn_id].btn_release_cntr = 0;

				/* Copy temp structure to user */
				user_btn_sts_ptr = &(((BTNS_STS_PTR)arg)->btns[i]);

				if ( copy_to_user((void*)user_btn_sts_ptr, &btn_sts,  sizeof(BTN)) ) {
					dprintk("%s(CIOCNOWAIT_P) - bad copy\n", __FUNCTION__);
					error = EFAULT;
				}
			}

			break;
		// -> [J.Chiang], 2009/04/30 -Added a additional service to read GPIO data
		// [Walker Chen], 2010/08/12 Added for LED STOP/START blinking when system is ready/not ready
		case CIOCREADY_P:
			sys_ready=1;
			sys_error=0;
			dprintk("Set System Ready...\n");
			break;

		case CIOCBUSY_P:
			sys_ready=0;
			sys_error=0;
			dprintk("Set System Not Ready...\n");
			break;

		case CIOCERROR_P:
			sys_error=1;
			dprintk("Set System Error...\n");
			break;

		case CIOCDEBUG_P:
			sys_ready=3;
			dprintk("Set System Debug...\n");
			break;

		case CIOCGPIO_P:
			user_btn_sts_ptr = &(((BTNS_STS_PTR)arg)->btns[0]);
			gpp = user_btn_sts_ptr->btn_id;
			dprintk("There is %d bpio to be checked\n", user_btn_sts_ptr->btn_id);

			/* get information for GPIO */
			gppVal = readl((volatile unsigned long *)GPIO_A_DATA);
			user_btn_sts_ptr->btn_push_cntr = gppVal;
			break;

		// -> [Walker Chen], 2010/08/12 - Added for LED blinking when system prepare shutdown.
		case CIOCPOWEROFF_P:
			gppVal = readl((volatile unsigned long *)GPIO_A_DATA);
			if ( gppVal & PWB_BTN ){ // high active
				pwr_long_push_time=	jiffies + (HZ*2); // set long push 2 sec
				dprintk("PWB record pushtime~ \n");
			}else{
				if(time_after(jiffies,pwr_long_push_time)){	//Long push valild
					/* debug mode */
					if ( sys_ready > 1 ){
						*(u32 *)arg = 0; //any key pressed was not valild
						break;
					}
					/* normal mode */
					dprintk("PWB Long push is valild~ \n");
					*(u32 *)arg = 1;
					dprintk("Set System Not Ready...\n");
					sys_ready=0;
				}else{
					dprintk("PWB:Long push is not valild~ \n");
					*(u32 *)arg = 0;
				}
			}
			break;
		// <- End.

		// -> [Walker Chen], 2010/08/27 - Added for FACTORY RESET BTN.
		case CIOFCTRYRST_P:
			gppVal = readl((volatile unsigned long *)GPIO_A_DATA);
			if ((gppVal & FACTORY_RESET_BTN ) == 0){ // low active
				fctry_long_push_time=	jiffies + (HZ *3);	// set long push 3 sec
				dprintk("FRB record pushtime~ \n");
			}else{
				if(time_after(jiffies,fctry_long_push_time)){ //Long push valild
					/* debug mode */
					if ( sys_ready > 1 ){
						*(u32 *)arg = 0; 	//any key pressed was not valild
						break;
					}
					/* normal mode */
					dprintk("FRB:Long push is valild~ \n");
					*(u32 *)arg = 1;
					dprintk("Set System Not Ready...\n");
					sys_ready=0;
				}else{
					dprintk("FRB:Long push is not valild~ \n");
					*(u32 *)arg = 0;
				}
			}
		    break;
		// <- End.

		// -> [Walker Chen], 2012/04/24 - Added for HDD hot-swap
		#ifdef CONFIG_BOARD_D20
		case CIOCHD1_BTN_p:
			gppVal = readl((volatile unsigned long *)GPIO_A_DATA);
			if ( (gppVal & HD1_BTN) == 0 ){ // low active
				HD1_BTN_long_push_time = jiffies + ( HZ * 2 ); // set long push 2 sec
			}else{
				if( time_after( jiffies , HD1_BTN_long_push_time ) ){ //Long push valild
					/* debug mode */
					if ( sys_ready > 1 ){
						*(u32 *)arg = 0; 	//any key pressed was not valild
						break;
					}
					/* normal mode */
					dprintk("HD1_BTN Long push is valild~ \n");
					*(u32 *)arg = 1;
				}else{
					dprintk("HD1_BTN Long push is not valild~ \n");
					*(u32 *)arg = 0;
				}
			}
			break;

		case CIOCHD2_BTN_p:
			gppVal = readl((volatile unsigned long *)GPIO_A_DATA);
			if ( (gppVal & HD2_BTN) == 0 ){ // low active
				HD2_BTN_long_push_time = jiffies + ( HZ * 2 ); // set long push 2 sec
			}else{
				if( time_after( jiffies , HD2_BTN_long_push_time ) ){ //Long push valild
					/* debug mode */
					if ( sys_ready > 1 ){
						*(u32 *)arg = 0; //any key pressed was not valild
						break;
					}
					/* normal mode */
					dprintk("HD2_BTN Long push is valild~ \n");
					*(u32 *)arg = 1;
				}else{
					dprintk("HD2_BTN Long push is not valild~ \n");
					*(u32 *)arg = 0;
				}
			}
			break;

		case CIOCPWR_HD1_p:
			if ( *(volatile u32 *)arg ){
				dprintk("HDD1 PWR ON...\n");
				*(volatile u32*)GPIO_B_OUTPUT_ENABLE 	|=  ( 1UL << 0 );	// Set GB0 as output
				*(volatile u32*)GPIO_B_PWM0_VALUE		= 	0xFF;			// Set GB0 to be hight
			}else{
				dprintk("HDD1 PWR OFF...\n");
				*(volatile u32*)GPIO_B_OUTPUT_ENABLE 	|=  ( 1UL << 0 );	// Set GB0 as output
				*(volatile u32*)GPIO_B_PWM0_VALUE  		= 	0;				// Set GB0 to be low
			}
			break;

		case CIOCPWR_HD2_p:
			if ( *(volatile u32 *)arg ){
				dprintk("HDD2 PWR ON...\n");
				*(volatile u32*)GPIO_B_OUTPUT_ENABLE 	|=  ( 1UL << 0 );	// Set GB1 as output
				*(volatile u32*)GPIO_B_PWM1_VALUE		= 	0xFF;			// Set GB1 to be hight
			}else{
				dprintk("HDD2 PWR OFF...\n");
				*(volatile u32*)GPIO_B_OUTPUT_ENABLE 	|=  ( 1UL << 0 );	// Set GB1 as output
				*(volatile u32*)GPIO_B_PWM1_VALUE  		= 	0;				// Set GB1 to be low
			}
			break;
		case CIOCERR_HD1_p:
			if ( *(volatile u32 *)arg  ){
				dprintk("set HDD1 ERROR...\n");
				sata1_err = *(volatile u32 *)arg;
			}else{
				dprintk("clr HDD1 ERROR...\n");
				sata1_err = 0;
			}
			break;
		case CIOCERR_HD2_p:
			if ( *(volatile u32 *)arg  ){
				dprintk("set HDD2 ERROR...\n");
				sata2_err = *(volatile u32 *)arg;
			}else{
				dprintk("clr HDD2 ERROR...\n");
				sata2_err = 0;
			}
			break;
		#endif //CONFIG_BOARD_D20
		// <- End.


		#ifdef CONFIG_BOARD_D20
		case CIOCLEDS_p:
			system_B_brightness = ( *(volatile u32 *)arg&0x01 )? 0xff:0x0;
			system_R_brightness = ( *(volatile u32 *)arg&0x02 )? 0xff:0x0;
			HD1_B_brightness  	= ( *(volatile u32 *)arg&0x04 )? 0xff:0x0;
			HD1_R_brightness  	= ( *(volatile u32 *)arg&0x08 )? 0xff:0x0;
			HD2_B_brightness  	= ( *(volatile u32 *)arg&0x10 )? 0xff:0x0;
			HD2_R_brightness 	= ( *(volatile u32 *)arg&0x20 )? 0xff:0x0;
			USB_brightness 	 	= ( *(volatile u32 *)arg&0x40 )? 0xff:0x0;
			break;

		case CIOCFAN_p:
			if ( *(volatile u32 *)arg ){
				dprintk("FAN ON value=0x%x...\n", *(volatile u32 *)arg);
				*(volatile u32*)GPIO_A_OUTPUT_ENABLE 	|=  ( 1UL << 0 );	// Set G2 as output
				//*(volatile u32*)GPIO_A_PWM2_VALUE		= 	0xFF;			// Set G2 to be hight
				*(volatile u32*)GPIO_A_PWM2_VALUE		= *(volatile u32 *)arg;	// Set G2 to be hight
			}else{
				dprintk("FAN OFF...\n");
				*(volatile u32*)GPIO_A_OUTPUT_ENABLE 	|=  ( 1UL << 0 );	// Set G0 as output
				*(volatile u32*)GPIO_A_PWM2_VALUE  		= 	0;				// Set G0 to be low
				breathing_led_on = 1; // turn breathing led				
			}
			break;
		case CIOCBUZZ_p:
			*(volatile u32*)GPIO_B_CLOCK_DIVIDER  =   *(volatile u32 *) arg >>8; //set GPIO_B_PWM_CLK
			if ( *(volatile u32 *)arg & 0x1){
				*(volatile u32*)GPIO_B_PWM11_VALUE = 0x7F;	//set Buzzer ON (duty=50%)
			}else{
				*(volatile u32*)GPIO_B_PWM11_VALUE = 0;		//set Buzzer OFF (duty=0%)
			}
			break;
		// -> [J.Chiang], 2011/06/17 - Added for setting MinutesWest
		case CIOCSETTZ_p:
			sys_tz.tz_minuteswest = *(volatile u32 *) arg;	// set minuteswest
			//printk("minuteswest= %d\n", *(volatile u32 *) arg);
			break;
		// <- End.

		case CIOCBRTHLED_p:
			if ( *(volatile u32 *)arg ){
				dprintk("Breathing_led ON...\n");
				breathing_led_on = 1; 
			}else{
				dprintk("Breathing_led OFF...\n");
				breathing_led_on = 0; 
			}
			break;
		#endif //CONFIG_BOARD_D20


    	default:
    	    dprintk("%s(unknown ioctl 0x%x)\n", __FUNCTION__, cmd);
    	    error = EINVAL;
    	    break;
    	}
    	return error;
}


/*
 * btn_gpp_init
 * initialize on button's GPP and registers its IRQ
 *
 */
static int
btn_gpp_init(unsigned int gpp, unsigned int default_gpp_val, BTN_OP btn_op, char* btn_name)
{
	u32	bitMask;

	bitMask = 1 << gpp;

	writel(readl(SYS_CTRL_SECONDARY_SEL)   & ~bitMask, SYS_CTRL_SECONDARY_SEL);
	writel(readl(SYS_CTRL_TERTIARY_SEL)    & ~bitMask, SYS_CTRL_TERTIARY_SEL);
	writel(readl(SYS_CTRL_QUATERNARY_SEL)  & ~bitMask, SYS_CTRL_QUATERNARY_SEL);
	writel(readl(SYS_CTRL_DEBUG_SEL)       & ~bitMask, SYS_CTRL_DEBUG_SEL);
	writel(readl(SYS_CTRL_ALTERNATIVE_SEL) & ~bitMask, SYS_CTRL_ALTERNATIVE_SEL);

	/* Enable GPIO input on switch line */
	writel(bitMask, GPIO_A_OUTPUT_ENABLE_CLEAR);

	/* Set up the GPIO line for both eadges trigger, debounced interrupt */
	writel(readl(GPIO_A_INPUT_DEBOUNCE_ENABLE) | bitMask, GPIO_A_INPUT_DEBOUNCE_ENABLE);
	writel(readl(GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE) | bitMask, GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE);
	writel(readl(GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE) | bitMask, GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE);

	/* Set which button operation should be monitored */
	btn_op_cfg[gpp] = btn_op;

	/* Set GPP default value*/
	gpp_default_val_cfg[gpp] = default_gpp_val;

	/* Register IRQ */
	if( request_irq( OX820_GPIO_IRQ_START + gpp, mv_btns_handler,
		IRQF_DISABLED, btn_name, NULL ) )
	{
		printk( KERN_ERR "btnsdev:  can't get irq for button %s (GPP %d)\n",btn_name,gpp );
		return -1;
	}

	return 0;
}

static int
btnsdev_open(struct inode *inode, struct file *filp)
{
        dprintk("%s()\n", __FUNCTION__);

	if(!is_opend) {
		is_opend = 1;

		/* Reset button operations counters*/
		memset(&btns_status,0,CONFIG_GPP_MAX_PINS);
	}

        return(0);
}

static int
btnsdev_release(struct inode *inode, struct file *filp)
{
        dprintk("%s()\n", __FUNCTION__);
        return(0);
}


static struct file_operations btnsdev_fops = {
        .open = btnsdev_open,
        .release = btnsdev_release,
        .ioctl = btnsdev_ioctl,
};

static struct miscdevice btnsdev = {
        .minor = BTNSDEV_MINOR,
        .name = "btns",
        .fops = &btnsdev_fops,
};


static int
btns_probe(struct platform_device *pdev)
{
        struct btns_platform_data *btns_data = pdev->dev.platform_data;
        int ret, i;

	dprintk("%s\n", __FUNCTION__);
	printk(KERN_NOTICE "Aton Buttons Driver Load\n");

        for (i = 0; i < btns_data->btns_num; i++) {
		ret = btn_gpp_init(btns_data->btns_data_arr[i].gpp_id, btns_data->btns_data_arr[i].default_gpp_val,
					btns_data->btns_data_arr[i].btn_op, btns_data->btns_data_arr[i].btn_name);
		if (ret != 0) {
			return ret;
		}
        }

	/* Clear interrupt status registers */
	writel(0xffffffff, GPIO_A_RISING_EDGE_DETECT);
	writel(0xffffffff, GPIO_A_FALLING_EDGE_DETECT);
	writel(0xffffffff, GPIO_A_INTERRUPT_EVENT);

	writel(readl(RPSA_IRQ_ENABLE) | 1UL << DIRECT_RPS_GPIOA_INTERRUPT, RPSA_IRQ_ENABLE);
        return 0;
}


static struct platform_driver btns_driver = {
	.probe  	= btns_probe,
	.driver		= {
        .name		= ATON_BTNS_NAME,
	},
};


static int __init
btnsdev_init(void)
{
	int rc;

	dprintk("%s\n", __FUNCTION__);

	/* Initialize Wait Q*/
	init_waitqueue_head(&btns_waitq);

	/* Register btns device */
	if (misc_register(&btnsdev))
        {
            printk(KERN_ERR "btnsdev: registration of /dev/btnsdev failed\n");
            return -1;
        }

	/* Register platform driver*/
	rc = platform_driver_register(&btns_driver);
	if (rc) {
		printk(KERN_ERR "btnsdev: registration of platform driver failed\n");
		return rc;
	}

        return 0;
}

static void __exit
btnsdev_exit(void)
{
	dprintk("%s() should never be called.\n", __FUNCTION__);
}

module_init(btnsdev_init);
module_exit(btnsdev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ronen Shitrit & Haim Boot");
MODULE_DESCRIPTION("PH: Buttons press handling.");


