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

#ifndef _BTNS_DEV_H_
#define _BTNS_DEV_H_

// -> [Walker Chen], 2012/04/23 - Added for D20
//#ifdef CONFIG_BOARD_D20
#define PWB_GPP 	10
#define PWB_BTN 	(1 << PWB_GPP)		// high active
#define FACTORY_RESET 11
#define FACTORY_RESET_BTN ( 1<< FACTORY_RESET ) // low active

#define HD1_GPP 	5
#define HD1_BTN 	(1 << HD1_GPP)		// low active
#define HD2_GPP 	6
#define HD2_BTN 	(1 << HD2_GPP)		// low active
#define DET_HD1_GPP 28
#define DET_HD1 	(1 << DET_HD1_GPP)	// low active
#define DET_HD2_GPP 29
#define DET_HD2 	(1 << DET_HD2_GPP)	// low active
//#endif //CONFIG_BOARD_D20
// <- End.

struct btns_platform_data {
        unsigned int    btns_num;
        struct btn_data *btns_data_arr;
};

#endif /* _BTNS_DEV_H_ */

