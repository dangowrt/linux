/********************************************************************************
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
#include <linux/miscdevice.h>
#include "btns_dev.h"
#include "../plat-oxnas/btns_driver.h"

/*
 * common debug for all
 */
// -> [J.Chiang], 2009/04/29 - Modified for debugging
#undef DEBUG
//#define DEBUG
// <- End.

#ifdef DEBUG
#define dprintk   printk
#else
#define dprintk(a...)
#endif

static struct btns_platform_data 	btns_data;
static struct btn_data			btn_data_aton[] = {
	[0] = {
		.gpp_id		= PWB_GPP,
		.default_gpp_val= PWB_BTN,
		.btn_op		= BTN_CHANGE,
		.btn_name	= "PWB",
	},
	[1] = {
		.gpp_id		= FACTORY_RESET,
		.default_gpp_val= FACTORY_RESET_BTN,
		.btn_op		= BTN_CHANGE,
		.btn_name	= "FRB",
	},
	[2] = {
		.gpp_id		= HD1_GPP,
		.default_gpp_val= HD1_BTN,
		.btn_op		= BTN_CHANGE,
		.btn_name	= "HD1B",
	},
	[3] = {
		.gpp_id		= HD2_GPP,
		.default_gpp_val= HD2_BTN,
		.btn_op		= BTN_CHANGE,
		.btn_name	= "HD2B",
	},
	[4] = {
		.gpp_id		= DET_HD1_GPP,
		.default_gpp_val= DET_HD1,
		.btn_op		= BTN_CHANGE,
		.btn_name	= "DET_HD1B",
	},
	[5] = {
		.gpp_id		= DET_HD2_GPP,
		.default_gpp_val= DET_HD2,
		.btn_op		= BTN_CHANGE,
		.btn_name	= "DET_HD2B",
	},	
};

static struct platform_device btns_device = {
	.name           = ATON_BTNS_NAME,
	.id             = 0,
	.num_resources  = 0,
	.dev = {
		.platform_data  = &btns_data,
	},
};

static int btns_init_data(struct platform_device *pdev)
{
	dprintk("%s - AtonNAS button init...\n", __FUNCTION__);
	btns_data.btns_data_arr = btn_data_aton;
	btns_data.btns_num = (btns_data.btns_data_arr == NULL) ? 0 : ARRAY_SIZE(btn_data_aton);

	if(btns_data.btns_num)
		dprintk("%s - Number of configured buttons: %d\n", __FUNCTION__ ,btns_data.btns_num);

	return 0;
}

static int __init   mv_btns_init(void)
{
	int                       status;

	printk(KERN_NOTICE "Aton-Buttons Device Load\n");

	/* Initialize btns related structures and data*/
	status = btns_init_data(&btns_device);
	if (status) {
            printk("Can't initialize Buttons Data, status=%d\n", status);
            return status;
        }

	/* register device */
	status = platform_device_register(&btns_device);
        if (status) {
            printk("Can't register Buttons Device, status=%d\n", status);
            return status;
        }

    	return 0;
}

subsys_initcall(mv_btns_init);

